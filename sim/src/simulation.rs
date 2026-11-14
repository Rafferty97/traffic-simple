use arrayvec::ArrayVec;
use glam::DVec2;
use slotmap::SlotMap;

use crate::debug::{AccelSource, DebugEvent, DebugSink, NoopSink};
use crate::kinematics;
use crate::math::wrap_angle;
use crate::models::acceleration;
use crate::models::lateral::{self, LaneChange};
use crate::network::{LaneId, Side, SimNetwork};
use crate::occupancy::{self, Occupant};
use crate::vehicle::{PATH_LEN, Placement, Vehicle, VehicleId, VehicleSpec};

/// Longest substep in seconds. Longer frames are split.
const MAX_SUBSTEP: f64 = 1.0 / 60.0;
/// Period of the longitudinal model update in seconds.
const ACCEL_PERIOD: f64 = 1.0 / 20.0;

// The acceleration clock fires at most once per substep.
const _: () = assert!(MAX_SUBSTEP <= ACCEL_PERIOD);

/// The traffic simulation.
pub struct Simulation<D: DebugSink = NoopSink> {
    network: SimNetwork,
    vehicles: SlotMap<VehicleId, Vehicle>,
    /// Each lane's occupants sorted by `s`, indexed by lane.
    occupants: Vec<Vec<Occupant>>,
    /// Vehicles being held to a stop. Few, so a list beats a flag on every vehicle.
    held: Vec<VehicleId>,
    /// Time since the last longitudinal model update, in seconds.
    accel_clock: f64,
    /// Scratch space for the longitudinal update's decisions, reused between updates.
    decisions: Vec<(VehicleId, f64)>,
    /// Scratch space for vehicles leaving the network, reused between substeps.
    leavers: Vec<VehicleId>,
    /// Scratch space for vehicles that have finished leaving an old lane.
    left_sideways: Vec<VehicleId>,
    sink: D,
}

/// Why [`Simulation::change_lane`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LaneChangeError {
    #[error("no such vehicle")]
    NoVehicle,
    #[error("a lane change is already in progress")]
    InProgress,
    #[error("no lane on that side")]
    NoNeighbor,
    #[error("changing into that lane isn't permitted")]
    NotPermitted,
}

impl<D: DebugSink> Simulation<D> {
    pub fn new(network: SimNetwork, sink: D) -> Self {
        let lane_count = network.lanes().count();
        Self {
            network,
            vehicles: SlotMap::with_key(),
            occupants: vec![Vec::new(); lane_count],
            held: Vec::new(),
            accel_clock: 0.0,
            decisions: Vec::new(),
            leavers: Vec::new(),
            left_sideways: Vec::new(),
            sink,
        }
    }

    pub fn network(&self) -> &SimNetwork {
        &self.network
    }

    pub fn sink(&self) -> &D {
        &self.sink
    }

    pub fn sink_mut(&mut self) -> &mut D {
        &mut self.sink
    }

    /// Inserts a vehicle on the centerline of a lane, facing along it.
    ///
    /// It starts with zero acceleration, which it holds until the next longitudinal model update.
    pub fn insert(&mut self, spec: VehicleSpec, at: Placement) -> VehicleId {
        let lane = self.network.lane(at.lane);
        let lane_pos = lane.lane_pos(at.s, 0.0);
        let mut path = ArrayVec::new();
        path.push(at.lane);
        extend_path(&self.network, &mut path);
        let vehicle = Vehicle {
            spec,
            pos: lane.lane_to_world(lane_pos),
            heading: wrap_angle(lane.tangent_at(lane_pos).to_angle()),
            speed: at.speed,
            accel: 0.0,
            steer: 0.0,
            path,
            lane_pos,
            old_side: None,
            lane_change: None,
        };
        let id = self.vehicles.insert(vehicle);

        let occupant = Occupant::new(lane, id, &self.vehicles[id], lane_pos);
        let list = &mut self.occupants[at.lane.0 as usize];
        let index = list.partition_point(|o| o.s <= occupant.s);
        list.insert(index, occupant);
        id
    }

    /// Removes a vehicle. Scans every lane's occupants, so it is not for hot paths.
    pub fn remove(&mut self, id: VehicleId) -> Option<Vehicle> {
        for list in &mut self.occupants {
            list.retain(|o| o.id != id);
        }
        self.held.retain(|&h| h != id);
        self.vehicles.remove(id)
    }

    pub fn get(&self, id: VehicleId) -> Option<&Vehicle> {
        self.vehicles.get(id)
    }

    pub fn vehicles(&self) -> impl Iterator<Item = (VehicleId, &Vehicle)> {
        self.vehicles.iter()
    }

    /// A lane's occupants, sorted by `s`.
    pub fn occupants(&self, lane: LaneId) -> &[Occupant] {
        &self.occupants[lane.0 as usize]
    }

    /// The vehicle whose body is nearest to `point`, if any is within `tolerance`.
    ///
    /// If the point is inside several bodies, the one whose center is nearest wins.
    pub fn vehicle_at(&self, point: DVec2, tolerance: f64) -> Option<VehicleId> {
        self.vehicles
            .iter()
            .map(|(id, v)| (id, v.distance_to(point), v.center().distance(point)))
            .filter(|&(_, dist, _)| dist <= tolerance)
            .min_by(|a, b| a.1.total_cmp(&b.1).then(a.2.total_cmp(&b.2)))
            .map(|(id, ..)| id)
    }

    /// Makes a vehicle brake to a stop and wait, until [`Simulation::release`].
    pub fn hold(&mut self, id: VehicleId) {
        if self.vehicles.contains_key(id) && !self.held.contains(&id) {
            self.held.push(id);
        }
    }

    /// Lets a held vehicle go again.
    pub fn release(&mut self, id: VehicleId) {
        self.held.retain(|&h| h != id);
    }

    pub fn is_held(&self, id: VehicleId) -> bool {
        self.held.contains(&id)
    }

    /// Changes a vehicle into the lane beside it on `side`, if permitted.
    ///
    /// The vehicle's lane switches at once and it joins the new lane's occupants.
    /// Its pose doesn't move: it is now off-center on its new lane, and path
    /// tracking steers it onto the centerline along a cubic. It stays on its old
    /// lane's occupants until its rear has left that lane.
    pub fn change_lane(&mut self, id: VehicleId, side: Side) -> Result<(), LaneChangeError> {
        let vehicle = self.vehicles.get_mut(id).ok_or(LaneChangeError::NoVehicle)?;
        if vehicle.lane_change.is_some() {
            return Err(LaneChangeError::InProgress);
        }
        let current = self.network.lane(vehicle.path[0]);
        let neighbor = current.adj(side).ok_or(LaneChangeError::NoNeighbor)?;
        if !neighbor.can_change {
            return Err(LaneChangeError::NotPermitted);
        }

        // Adjacent lanes start and end together, so the same fraction along the
        // new lane is a good hint.
        let target = self.network.lane(neighbor.lane);
        let fraction = vehicle.lane_pos.s / current.length();
        let hint = target.lane_pos(fraction * target.length(), 0.0);
        let lane_pos = target.world_to_lane_near(vehicle.pos, hint);

        // The cubic starts where pure pursuit is aiming on the old path, with that
        // path's offset and slope relative to the new lane, so the target point
        // carries on smoothly instead of jumping.
        let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
        let aim = point_along_path(&self.network, &vehicle.path, ahead, 0.0);
        let beyond = point_along_path(&self.network, &vehicle.path, ahead + 1.0, 0.0);
        let start = target.world_to_lane_near(aim, lane_pos);
        let next = target.world_to_lane_near(beyond, start);
        let slope = (next.t - start.t) / (next.s - start.s);

        vehicle.path.clear();
        vehicle.path.push(neighbor.lane);
        extend_path(&self.network, &mut vehicle.path);
        vehicle.lane_pos = lane_pos;
        vehicle.old_side = Some(side.opposite());
        vehicle.lane_change = Some(LaneChange::new(start.s, start.t, slope, vehicle.speed));

        let occupant = Occupant::new(target, id, &self.vehicles[id], lane_pos);
        let list = &mut self.occupants[neighbor.lane.0 as usize];
        let index = list.partition_point(|o| o.s <= occupant.s);
        list.insert(index, occupant);
        Ok(())
    }

    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        let n = (dt / MAX_SUBSTEP).ceil().max(1.0);
        let h = dt / n;
        for _ in 0..n as u32 {
            self.substep(h);
        }
    }

    fn substep(&mut self, dt: f64) {
        self.accel_clock += dt;
        if self.accel_clock >= ACCEL_PERIOD {
            self.accel_clock -= ACCEL_PERIOD;
            self.update_accels();
        }
        self.update_motion(dt);
        self.update_occupancy();
    }

    /// Longitudinal model: sets each vehicle's held acceleration.
    ///
    /// The acceleration is the minimum over every term the model considers; each
    /// term is recorded to the debug sink as it is computed. Decisions are computed
    /// from a read-only view of the state, then applied.
    fn update_accels(&mut self) {
        let Self { network, vehicles, occupants, decisions, sink, .. } = self;
        sink.record(|| DebugEvent::RecomputeAccels);
        decisions.clear();
        for (id, vehicle) in vehicles.iter() {
            let lane = network.lane(vehicle.path[0]);
            let desired_speed = vehicle.spec.max_speed.min(lane.speed_limit());
            let mut accel = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed);
            let source = AccelSource::FreeRoad { desired_speed };
            sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel });

            let mut react = |leader: VehicleId, gap: f64| {
                let leader_speed = vehicles[leader].speed;
                let term =
                    acceleration::interaction(&vehicle.spec, vehicle.speed, gap, leader_speed);
                let source = AccelSource::Leader { id: leader, gap };
                sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                accel = accel.min(term);
            };

            let axle_s = vehicle.lane_pos.s;
            if let Some((leader, gap)) =
                occupancy::find_leader(network, occupants, id, vehicle, &vehicle.path, axle_s)
            {
                react(leader, gap);
            }

            // Still straddling after a lane change: also search the old side, along
            // the neighbors of the vehicle's path, from its position on the old lane.
            if let Some(side) = vehicle.old_side {
                let old_path: ArrayVec<LaneId, PATH_LEN> = vehicle
                    .path
                    .iter()
                    .map_while(|&lane| network.lane(lane).adj(side).map(|n| n.lane))
                    .collect();
                if let Some(&first) = old_path.first() {
                    let old = network.lane(first);
                    // Adjacent lanes start and end together, so the same fraction
                    // along the old lane is a good hint.
                    let fraction = vehicle.lane_pos.s / lane.length();
                    let hint = old.lane_pos(fraction * old.length(), 0.0);
                    let axle_s = old.world_to_lane_near(vehicle.pos, hint).s;
                    if let Some((leader, gap)) =
                        occupancy::find_leader(network, occupants, id, vehicle, &old_path, axle_s)
                    {
                        react(leader, gap);
                    }
                }
            }
            decisions.push((id, accel));
        }

        for &(id, accel) in &self.decisions {
            self.vehicles[id].accel = accel;
        }
        for &id in &self.held {
            let vehicle = &mut self.vehicles[id];
            let term = -vehicle.spec.comfy_decel;
            let source = AccelSource::Hold;
            self.sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
            vehicle.accel = vehicle.accel.min(term);
        }
    }

    /// Path tracking and integration: steers, moves, then advances each vehicle
    /// onto its next lane once its rear axle passes the end of its current one.
    ///
    /// During a lane change, the target path is offset from the centerline along
    /// the change's cubic, and the change ends where the cubic does.
    fn update_motion(&mut self, dt: f64) {
        let Self { network, vehicles, occupants, sink, .. } = self;
        sink.record(|| DebugEvent::RecomputeMotion);
        for (id, vehicle) in vehicles.iter_mut() {
            let wheelbase = vehicle.spec.wheelbase();

            let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
            let offset = vehicle.lane_change.map_or(0.0, |change| change.offset_at(ahead));
            let target = point_along_path(network, &vehicle.path, ahead, offset);
            vehicle.steer = lateral::steer_toward(vehicle.pos, vehicle.heading, wheelbase, target);
            sink.record(|| DebugEvent::Lookahead { vehicle: id, point: target });

            // Trapezoidal distance is exact for constant acceleration, unless the
            // vehicle stopped during the substep.
            let speed = (vehicle.speed + vehicle.accel * dt).max(0.0);
            let dist = 0.5 * (vehicle.speed + speed) * dt;
            vehicle.speed = speed;
            (vehicle.pos, vehicle.heading) =
                kinematics::advance(vehicle.pos, vehicle.heading, vehicle.steer, wheelbase, dist);

            let lane = network.lane(vehicle.path[0]);
            vehicle.lane_pos = lane.world_to_lane_near(vehicle.pos, vehicle.lane_pos);
            while vehicle.path.len() > 1 {
                let length = network.lane(vehicle.path[0]).length();
                if vehicle.lane_pos.s < length {
                    break;
                }
                vehicle.path.remove(0);
                extend_path(network, &mut vehicle.path);
                let next = network.lane(vehicle.path[0]);
                let hint = next.lane_pos(vehicle.lane_pos.s - length, 0.0);
                vehicle.lane_pos = next.world_to_lane_near(vehicle.pos, hint);
                if let Some(change) = &mut vehicle.lane_change {
                    change.s0 -= length;
                }
                // Joins unsorted; `update_occupancy` sets its `s` and sorts.
                let occupant = Occupant::new(next, id, vehicle, vehicle.lane_pos);
                occupants[vehicle.path[0].0 as usize].push(occupant);
                // Still straddling after a lane change: also join the old side's
                // next lane, so followers there keep seeing it.
                if let Some(old) = vehicle.old_side.and_then(|side| next.adj(side)) {
                    let old_lane = network.lane(old.lane);
                    let hint = old_lane.lane_pos(vehicle.lane_pos.s, 0.0);
                    let occupant = Occupant::new(old_lane, id, vehicle, hint);
                    occupants[old.lane.0 as usize].push(occupant);
                }
            }

            if vehicle.lane_change.is_some_and(|change| vehicle.lane_pos.s >= change.end()) {
                vehicle.lane_change = None;
            }
        }
    }

    /// Refreshes the occupant lists and removes vehicles that have left the network.
    fn update_occupancy(&mut self) {
        let Self { network, vehicles, occupants, leavers, left_sideways, .. } = self;
        occupancy::update(network, vehicles, occupants, leavers, left_sideways);
        for id in self.left_sideways.drain(..) {
            if let Some(vehicle) = self.vehicles.get_mut(id) {
                vehicle.old_side = None;
            }
        }
        for id in self.leavers.drain(..) {
            self.vehicles.remove(id);
            self.held.retain(|&h| h != id);
        }
    }
}

/// Fills `path` up to capacity by following each lane's first successor.
fn extend_path(network: &SimNetwork, path: &mut ArrayVec<LaneId, PATH_LEN>) {
    while !path.is_full() {
        let last = path[path.len() - 1];
        match network.lane(last).successors().first() {
            Some(&next) => path.push(next),
            None => break,
        }
    }
}

/// The point `s` meters along a path from the start of its first lane, `t` meters
/// left of the centerline. Past the end of the path, the last lane's final
/// segment is extended.
fn point_along_path(network: &SimNetwork, path: &[LaneId], mut s: f64, t: f64) -> DVec2 {
    for (i, &id) in path.iter().enumerate() {
        let lane = network.lane(id);
        if s < lane.length() || i == path.len() - 1 {
            return lane.lane_to_world(lane.lane_pos(s, t));
        }
        s -= lane.length();
    }
    unreachable!("a vehicle's path always holds its current lane")
}
