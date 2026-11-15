use arrayvec::ArrayVec;
use glam::DVec2;
use slotmap::SlotMap;

use crate::debug::{AccelSource, DebugEvent, DebugSink, LaneChangeEval, NoopSink};
use crate::kinematics;
use crate::math::wrap_angle;
use crate::models::acceleration;
use crate::models::lane_change::{self, Outcome};
use crate::models::lateral::{self, LaneChange};
use crate::network::{LaneId, Side, SimNetwork};
use crate::occupancy::{self, Course, Ghosts, Occupant};
use crate::vehicle::{Intent, PATH_LEN, Placement, Response, Vehicle, VehicleId, VehicleSpec};

/// Longest substep in seconds. Longer frames are split.
const MAX_SUBSTEP: f64 = 1.0 / 60.0;
/// Period of the longitudinal model update in seconds.
const ACCEL_PERIOD: f64 = 1.0 / 20.0;

/// The longest vehicle the alongside check allows for, in meters.
const MAX_VEHICLE_LENGTH: f64 = 20.0;

/// How often each vehicle makes its lane-change decision, in seconds. Decisions
/// are staggered across vehicles.
const DECISION_PERIOD: f64 = 0.2;

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
    /// Scratch space for vehicles that lost a record on a lane they had left.
    departed: Vec<VehicleId>,
    /// Simulated time, in seconds.
    time: f64,
    /// Where staggered lane-change decisions resume: a position in the
    /// vehicles' iteration order.
    decision_cursor: usize,
    /// Decisions owed but not yet made: grows by `n · dt / DECISION_PERIOD`.
    decision_budget: f64,
    /// Scratch space for lane-change decisions, reused between substeps.
    lane_decisions: Vec<(VehicleId, LaneDecision)>,
    /// Scratch space for followers' responses to intents, reused between updates.
    responses: Vec<(VehicleId, Role, Response)>,
    sink: D,
}

/// What a vehicle's lane-change decision asks for.
#[derive(Clone, Copy, Debug)]
enum LaneDecision {
    Nothing,
    Indicate(Side),
    Cancel,
    Change(Side),
}

/// Which follower a response comes from.
#[derive(Clone, Copy, Debug)]
enum Role {
    NewFollower,
    OldFollower,
}

/// Why [`Simulation::try_insert`] refused.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum InsertError {
    #[error("no safe room at that position")]
    NoRoom,
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
            departed: Vec::new(),
            time: 0.0,
            decision_cursor: 0,
            decision_budget: 0.0,
            lane_decisions: Vec::new(),
            responses: Vec::new(),
            sink,
        }
    }

    pub fn network(&self) -> &SimNetwork {
        &self.network
    }

    /// Simulated time since the simulation was created, in seconds.
    pub fn time(&self) -> f64 {
        self.time
    }

    pub fn sink(&self) -> &D {
        &self.sink
    }

    pub fn sink_mut(&mut self) -> &mut D {
        &mut self.sink
    }

    /// Inserts a vehicle on the centerline of a lane, facing along it, without
    /// checking for room. See [`Simulation::try_insert`] for a checked insert.
    ///
    /// It starts with zero acceleration, which it holds until the next longitudinal model update.
    pub fn insert(&mut self, spec: VehicleSpec, at: Placement) -> VehicleId {
        let vehicle = self.build_vehicle(spec, at);
        self.register(vehicle)
    }

    /// Inserts a vehicle as [`Simulation::insert`] does, but only where there is
    /// safe room for it.
    ///
    /// Its speed is `at.speed`, capped at its new leader's speed. It is refused if
    /// it would overlap another vehicle, or if it or its new follower would have to
    /// brake harder than [`SAFE_BRAKING`](crate::SAFE_BRAKING)
    /// because of it. Its follower is the nearest vehicle behind it on its lane, or
    /// at the end of a lane leading into it.
    pub fn try_insert(
        &mut self,
        spec: VehicleSpec,
        at: Placement,
    ) -> Result<VehicleId, InsertError> {
        let mut vehicle = self.build_vehicle(spec, at);
        let lane = self.network.lane(at.lane);
        // A key that matches no vehicle, since this one isn't inserted yet.
        let id = VehicleId::default();
        let rear = Occupant::new(lane, id, &vehicle, vehicle.lane_pos).s;
        let front = rear + spec.length;

        let list = &self.occupants[at.lane.0 as usize];
        if list.iter().any(|o| o.s >= rear && o.s <= front) {
            return Err(InsertError::NoRoom);
        }

        let axle_s = vehicle.lane_pos.s;
        let path = vehicle.path.clone();
        let leader = occupancy::find_leader(
            &self.network,
            &self.occupants,
            id,
            &vehicle,
            &path,
            axle_s,
            Course::of(&vehicle),
            Ghosts::Lead,
            None,
            |_| {},
        );
        if let Some((leader, gap)) = leader.leader {
            let leader_speed = self.vehicles[leader].speed;
            vehicle.speed = vehicle.speed.min(leader_speed);
            let accel = acceleration::interaction(&spec, vehicle.speed, gap, leader_speed);
            if gap <= 0.0 || accel < -acceleration::SAFE_BRAKING {
                return Err(InsertError::NoRoom);
            }
        }

        let follower = find_follower(
            &self.network,
            &self.occupants,
            &self.vehicles,
            at.lane,
            rear,
        );
        if let Some((follower, gap)) = follower {
            let f = &self.vehicles[follower];
            let accel = acceleration::interaction(&f.spec, f.speed, gap, vehicle.speed);
            if gap <= 0.0 || accel < -acceleration::SAFE_BRAKING {
                return Err(InsertError::NoRoom);
            }
        }

        Ok(self.register(vehicle))
    }

    /// A vehicle placed on a lane's centerline, facing along it, not yet in the simulation.
    fn build_vehicle(&self, spec: VehicleSpec, at: Placement) -> Vehicle {
        let lane = self.network.lane(at.lane);
        let lane_pos = lane.lane_pos(at.s, 0.0);
        let mut path = ArrayVec::new();
        path.push(at.lane);
        extend_path(&self.network, &mut path);
        Vehicle {
            spec,
            pos: lane.lane_to_world(lane_pos),
            heading: wrap_angle(lane.tangent_at(lane_pos).to_angle()),
            speed: at.speed,
            accel: 0.0,
            curvature: 0.0,
            path,
            lane_pos,
            old_side: None,
            lane_change: None,
            intent: None,
        }
    }

    /// Adds a built vehicle to the simulation and to its lane's occupants.
    fn register(&mut self, vehicle: Vehicle) -> VehicleId {
        let lane_id = vehicle.path[0];
        let lane_pos = vehicle.lane_pos;
        let id = self.vehicles.insert(vehicle);
        let lane = self.network.lane(lane_id);
        let occupant = Occupant::new(lane, id, &self.vehicles[id], lane_pos);
        let list = &mut self.occupants[lane_id.0 as usize];
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
        self.sink.record(|| DebugEvent::VehicleRemoved { vehicle: id });
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
    ///
    /// If the vehicle was indicating a change to this side, its ghost becomes its
    /// record on the new lane. An intent to the other side is cancelled.
    pub fn change_lane(&mut self, id: VehicleId, side: Side) -> Result<(), LaneChangeError> {
        let indicating = self.vehicles.get(id).and_then(|v| v.intent).map(|i| i.side);
        if indicating.is_some_and(|s| s != side) {
            self.cancel_intent(id);
        }
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
        let aim = self.network.point_along(&vehicle.path, ahead, 0.0);
        let beyond = self.network.point_along(&vehicle.path, ahead + 1.0, 0.0);
        let start = target.world_to_lane_near(aim, lane_pos);
        let next = target.world_to_lane_near(beyond, start);
        let slope = (next.t - start.t) / (next.s - start.s);

        vehicle.path.clear();
        vehicle.path.push(neighbor.lane);
        extend_path(&self.network, &mut vehicle.path);
        vehicle.lane_pos = lane_pos;
        vehicle.old_side = Some(side.opposite());
        vehicle.lane_change = Some(LaneChange::new(start.s, start.t, slope, vehicle.speed));
        vehicle.intent = None;

        let list = &mut self.occupants[neighbor.lane.0 as usize];
        if let Some(ghost) = list.iter_mut().find(|o| o.id == id && o.ghost) {
            ghost.ghost = false;
        } else {
            let occupant = Occupant::new(target, id, &self.vehicles[id], lane_pos);
            let index = list.partition_point(|o| o.s <= occupant.s);
            list.insert(index, occupant);
        }
        Ok(())
    }

    /// Starts a vehicle indicating a change to `side`: it gets a ghost record in
    /// the target lane's occupants.
    fn indicate(&mut self, id: VehicleId, side: Side) {
        let vehicle = &self.vehicles[id];
        let current = self.network.lane(vehicle.path[0]);
        let Some(neighbor) = current.adj(side) else {
            return;
        };
        let target = self.network.lane(neighbor.lane);
        let hint = target.lane_pos(vehicle.lane_pos.s / current.length() * target.length(), 0.0);
        let ghost = Occupant::ghost(target, id, vehicle, hint);
        let list = &mut self.occupants[neighbor.lane.0 as usize];
        let index = list.partition_point(|o| o.s <= ghost.s);
        list.insert(index, ghost);
        self.vehicles[id].intent = Some(Intent {
            side,
            since: self.time,
            new_follower: None,
            old_follower: None,
        });
    }

    /// Stops a vehicle indicating, removing its ghost.
    fn cancel_intent(&mut self, id: VehicleId) {
        let Some(vehicle) = self.vehicles.get_mut(id) else {
            return;
        };
        let Some(intent) = vehicle.intent.take() else {
            return;
        };
        if let Some(neighbor) = self.network.lane(vehicle.path[0]).adj(intent.side) {
            remove_ghost(&mut self.occupants, neighbor.lane, id);
        }
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
        self.time += dt;
        self.decision_budget += self.vehicles.len() as f64 * dt / DECISION_PERIOD;
        let count = self.decision_budget as usize;
        if count > 0 {
            self.decision_budget -= count as f64;
            self.update_decisions(count);
        }
        self.accel_clock += dt;
        if self.accel_clock >= ACCEL_PERIOD {
            self.accel_clock -= ACCEL_PERIOD;
            self.update_accels();
        }
        self.update_motion(dt);
        self.update_occupancy();
    }

    /// Lane-change decisions for the next `count` vehicles, resuming where the last
    /// substep left off and wrapping around. Decisions are computed from a
    /// read-only view of the state, then applied in order.
    fn update_decisions(&mut self, count: usize) {
        let Self { network, vehicles, occupants, lane_decisions, sink, .. } = self;
        let n = vehicles.len();
        if n == 0 {
            return;
        }
        let count = count.min(n);
        let start = self.decision_cursor % n;
        lane_decisions.clear();
        for (id, vehicle) in vehicles.iter().chain(vehicles.iter()).skip(start).take(count) {
            sink.record(|| DebugEvent::RecomputeDecision { vehicle: id });
            let decision = decide(network, occupants, vehicles, sink, self.time, id, vehicle);
            lane_decisions.push((id, decision));
        }
        self.decision_cursor = (start + count) % n;

        for i in 0..self.lane_decisions.len() {
            let (id, decision) = self.lane_decisions[i];
            match decision {
                LaneDecision::Nothing => {}
                LaneDecision::Indicate(side) => self.indicate(id, side),
                LaneDecision::Cancel => self.cancel_intent(id),
                LaneDecision::Change(side) => {
                    // Already checked: the decision only asks for permitted changes.
                    let _ = self.change_lane(id, side);
                }
            }
        }
    }

    /// Longitudinal model: sets each vehicle's held acceleration.
    ///
    /// The acceleration is the minimum over every term the model considers; each
    /// term is recorded to the debug sink as it is computed. Decisions are computed
    /// from a read-only view of the state, then applied.
    ///
    /// The same searches answer intents. A vehicle whose nearest record ahead is a
    /// ghost is that vehicle's would-be new follower, and one whose leader is
    /// indicating is its old follower; each reports its acceleration now and if the
    /// change went ahead.
    fn update_accels(&mut self) {
        let Self { network, vehicles, occupants, decisions, responses, sink, .. } = self;
        sink.record(|| DebugEvent::RecomputeAccels);
        decisions.clear();
        responses.clear();
        for (id, vehicle) in vehicles.iter() {
            let lane = network.lane(vehicle.path[0]);
            let desired_speed = vehicle.spec.desired_speed(lane.speed_limit());
            let free = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed);
            let source = AccelSource::FreeRoad { desired_speed };
            sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: free });
            let mut accel = free;

            // Takes the sink rather than capturing it, so the searches can record too.
            let mut react = |sink: &mut D, leader: VehicleId, gap: f64| {
                let leader_speed = vehicles[leader].speed;
                let term =
                    acceleration::interaction(&vehicle.spec, vehicle.speed, gap, leader_speed);
                let source = AccelSource::Leader { id: leader, gap };
                sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                accel = accel.min(term);
                term
            };

            let axle_s = vehicle.lane_pos.s;
            let path = &vehicle.path;
            let search = occupancy::find_leader(
                network,
                occupants,
                id,
                vehicle,
                path,
                axle_s,
                Course::of(vehicle),
                Ghosts::Report,
                None,
                |check| sink.record(|| DebugEvent::PassCheck { vehicle: id, check }),
            );
            if let Some((leader, gap)) = search.leader {
                react(sink, leader, gap);
            }
            let mut old_side_term = f64::INFINITY;

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
                    let search = occupancy::find_leader(
                        network,
                        occupants,
                        id,
                        vehicle,
                        &old_path,
                        axle_s,
                        Course::of(vehicle),
                        Ghosts::Report,
                        None,
                        |check| sink.record(|| DebugEvent::PassCheck { vehicle: id, check }),
                    );
                    if let Some((leader, gap)) = search.leader {
                        old_side_term = react(sink, leader, gap);
                    }
                }
            }
            decisions.push((id, accel));

            // The would-be new follower of an indicating vehicle.
            if let Some((owner, gap)) = search.ghost {
                let owner_speed = vehicles[owner].speed;
                let term =
                    acceleration::interaction(&vehicle.spec, vehicle.speed, gap, owner_speed);
                let after = accel.min(term);
                responses.push((
                    owner,
                    Role::NewFollower,
                    Response { follower: id, before: accel, after },
                ));
            }
            // The old follower of an indicating leader: what if it weren't there?
            if let Some((leader, _)) = search.leader
                && vehicles[leader].intent.is_some()
            {
                let without = occupancy::find_leader(
                    network,
                    occupants,
                    id,
                    vehicle,
                    path,
                    axle_s,
                    Course::of(vehicle),
                    Ghosts::Report,
                    Some(leader),
                    |_| {},
                );
                let mut after = free.min(old_side_term);
                if let Some((next, gap)) = without.leader {
                    let next_speed = vehicles[next].speed;
                    let term =
                        acceleration::interaction(&vehicle.spec, vehicle.speed, gap, next_speed);
                    after = after.min(term);
                }
                responses.push((
                    leader,
                    Role::OldFollower,
                    Response { follower: id, before: accel, after },
                ));
            }
        }

        for &(id, accel) in &self.decisions {
            self.vehicles[id].accel = accel;
        }
        for vehicle in self.vehicles.values_mut() {
            if let Some(intent) = &mut vehicle.intent {
                intent.new_follower = None;
                intent.old_follower = None;
            }
        }
        for &(owner, role, response) in &self.responses {
            let Some(intent) = self.vehicles.get_mut(owner).and_then(|v| v.intent.as_mut()) else {
                continue;
            };
            match role {
                Role::NewFollower => intent.new_follower = Some(response),
                Role::OldFollower => intent.old_follower = Some(response),
            }
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
    /// A vehicle whose rear axle passes the end of a lane with no successors has
    /// left the network and is removed.
    ///
    /// During a lane change, the target path is offset from the centerline along
    /// the change's cubic, and the change ends where the cubic does.
    fn update_motion(&mut self, dt: f64) {
        let Self { network, vehicles, occupants, leavers, sink, .. } = self;
        sink.record(|| DebugEvent::RecomputeMotion);
        for (id, vehicle) in vehicles.iter_mut() {
            let wheelbase = vehicle.spec.wheelbase();

            let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
            let offset = vehicle.lane_change.map_or(0.0, |change| change.offset_at(ahead));
            let target = network.point_along(&vehicle.path, ahead, offset);
            vehicle.curvature =
                lateral::curvature_toward(vehicle.pos, vehicle.heading, wheelbase, target);
            sink.record(|| DebugEvent::Lookahead { vehicle: id, point: target });

            // Trapezoidal distance is exact for constant acceleration, unless the
            // vehicle stopped during the substep.
            let speed = (vehicle.speed + vehicle.accel * dt).max(0.0);
            let dist = 0.5 * (vehicle.speed + speed) * dt;
            vehicle.speed = speed;
            (vehicle.pos, vehicle.heading) =
                kinematics::advance(vehicle.pos, vehicle.heading, vehicle.curvature, dist);

            let lane = network.lane(vehicle.path[0]);
            vehicle.lane_pos = lane.world_to_lane_near(vehicle.pos, vehicle.lane_pos);
            while vehicle.path.len() > 1 {
                let length = network.lane(vehicle.path[0]).length();
                if vehicle.lane_pos.s < length {
                    break;
                }
                let previous = vehicle.path[0];
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
                // Indicating: the ghost moves to the new lane's neighbor, or the
                // intent is cancelled if there's none to change into.
                if let Some(intent) = vehicle.intent {
                    if let Some(old) = network.lane(previous).adj(intent.side) {
                        remove_ghost(occupants, old.lane, id);
                    }
                    match next.adj(intent.side).filter(|n| n.can_change) {
                        Some(n) => {
                            let target = network.lane(n.lane);
                            let hint = target.lane_pos(vehicle.lane_pos.s, 0.0);
                            occupants[n.lane.0 as usize]
                                .push(Occupant::ghost(target, id, vehicle, hint));
                        }
                        None => vehicle.intent = None,
                    }
                }
            }

            // Past the end with nowhere to advance to: it has left the network.
            let lane = network.lane(vehicle.path[0]);
            if vehicle.lane_pos.s >= lane.length() && lane.successors().is_empty() {
                leavers.push(id);
            }

            if vehicle.lane_change.is_some_and(|change| vehicle.lane_pos.s >= change.end()) {
                vehicle.lane_change = None;
            }
        }
        // Through `remove`, so ghosts and any other records go too.
        let leavers = std::mem::take(&mut self.leavers);
        for &id in &leavers {
            self.remove(id);
        }
        self.leavers = leavers;
        self.leavers.clear();
    }

    /// Refreshes the occupant lists, and clears the old side of vehicles that have
    /// finished leaving it.
    fn update_occupancy(&mut self) {
        let Self { network, vehicles, occupants, departed, .. } = self;
        occupancy::update(network, vehicles, occupants, departed);
        // A vehicle straddles while it still has a record on the lane beside the
        // one it follows, on its old side. Losing a record elsewhere, such as at
        // the end of the lane before, leaves it straddling.
        for id in departed.drain(..) {
            let vehicle = &mut vehicles[id];
            let Some(side) = vehicle.old_side else {
                continue;
            };
            let straddling = network.lane(vehicle.path[0]).adj(side).is_some_and(|old| {
                occupants[old.lane.0 as usize].iter().any(|o| o.id == id && !o.ghost)
            });
            if !straddling {
                vehicle.old_side = None;
            }
        }
    }
}

/// The nearest vehicle behind `s` on `lane`, or failing that the last vehicle on a
/// lane leading into it, and the gap from its front to `s`. For insertion only:
/// it scans every lane for predecessors.
fn find_follower(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    lane: LaneId,
    s: f64,
) -> Option<(VehicleId, f64)> {
    let gap = |o: &Occupant, offset: f64| (o.id, s + offset - (o.s + vehicles[o.id].spec.length));
    let list = &occupants[lane.0 as usize];
    let behind = list.partition_point(|o| o.s < s);
    if behind > 0 {
        return Some(gap(&list[behind - 1], 0.0));
    }
    network
        .lanes()
        .filter(|(_, l)| l.successors().contains(&lane))
        .filter_map(|(id, l)| occupants[id.0 as usize].last().map(|o| gap(o, l.length())))
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

/// Removes a vehicle's ghost from a lane's occupants.
fn remove_ghost(occupants: &mut [Vec<Occupant>], lane: LaneId, id: VehicleId) {
    occupants[lane.0 as usize].retain(|o| !(o.id == id && o.ghost));
}

/// A vehicle's lane-change decision, under MOBIL.
///
/// - Not indicating: it weighs up each permitted side from its own gain alone,
///   and starts indicating to the better one if that is worth it and safe.
/// - Indicating: it cancels once its own gain is no longer worth it. Otherwise,
///   with its followers' responses, it changes once MOBIL's net gain is worth it,
///   it is safe and it has indicated long enough, and until then keeps
///   indicating, waiting for a gap.
///
/// A vehicle partway through a change, or still straddling, doesn't decide.
fn decide<D: DebugSink>(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    sink: &mut D,
    time: f64,
    id: VehicleId,
    vehicle: &Vehicle,
) -> LaneDecision {
    if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
        return match vehicle.intent {
            Some(_) => LaneDecision::Cancel,
            None => LaneDecision::Nothing,
        };
    }
    let lane = network.lane(vehicle.path[0]);
    let now = accel_along(
        network,
        occupants,
        vehicles,
        id,
        vehicle,
        &vehicle.path,
        vehicle.lane_pos.s,
        Course::of(vehicle),
        Ghosts::Report,
    );
    let mut record = |eval: LaneChangeEval| {
        sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
    };

    let Some(intent) = vehicle.intent else {
        // Weigh up both sides, then indicate to the better one if any.
        let mut evals = [Side::Left, Side::Right].map(|side| {
            let target = lane.adj(side).filter(|n| n.can_change)?;
            let (own_after, axle_s) =
                accel_in(network, occupants, vehicles, id, vehicle, target.lane);
            let own_gain = own_after - now;
            let safe = lane_change::safe(own_after)
                && !alongside(
                    network,
                    occupants,
                    vehicles,
                    target.lane,
                    id,
                    vehicle,
                    axle_s,
                );
            let outcome = match () {
                _ if !safe => Outcome::Unsafe,
                _ if !lane_change::worth_it(own_gain) => Outcome::NotWorthIt,
                _ => Outcome::Indicate,
            };
            let incentive = own_gain;
            let (new_follower, old_follower) = (None, None);
            Some(LaneChangeEval {
                side,
                own_after,
                own_gain,
                new_follower,
                old_follower,
                incentive,
                safe,
                outcome,
            })
        });
        if let [Some(left), Some(right)] = &mut evals
            && left.outcome == Outcome::Indicate
            && right.outcome == Outcome::Indicate
        {
            let worse = if left.own_gain >= right.own_gain {
                right
            } else {
                left
            };
            worse.outcome = Outcome::OtherSide;
        }
        let mut decision = LaneDecision::Nothing;
        for eval in evals.into_iter().flatten() {
            if eval.outcome == Outcome::Indicate {
                decision = LaneDecision::Indicate(eval.side);
            }
            record(eval);
        }
        return decision;
    };

    let Some(target) = lane.adj(intent.side).filter(|n| n.can_change) else {
        return LaneDecision::Cancel;
    };
    let (own_after, axle_s) = accel_in(network, occupants, vehicles, id, vehicle, target.lane);
    let own_gain = own_after - now;
    let new_follower = intent.new_follower.map(|r| r.change());
    let old_follower = intent.old_follower.map(|r| r.change());
    let incentive = lane_change::incentive(
        own_gain,
        new_follower.unwrap_or(0.0),
        old_follower.unwrap_or(0.0),
    );
    let indicated = time - intent.since;
    let follower_safe = match intent.new_follower {
        Some(response) => lane_change::safe(response.after),
        // No reply: safe only if no vehicle is behind the ghost, and replies
        // have had time to arrive.
        None => {
            indicated >= 2.0 * ACCEL_PERIOD
                && !anyone_behind_ghost(network, occupants, target.lane, id)
        }
    };
    let safe = lane_change::safe(own_after)
        && follower_safe
        && !alongside(
            network,
            occupants,
            vehicles,
            target.lane,
            id,
            vehicle,
            axle_s,
        );
    // Cancelling is decided on the driver's own gain, like starting, but with a
    // lower bar so the indicator doesn't flicker. Politeness and safety only hold
    // back the change.
    let outcome = match () {
        _ if !lane_change::still_worth_it(own_gain) => Outcome::Cancel,
        _ if lane_change::worth_it(incentive)
            && safe
            && indicated >= lane_change::MIN_INDICATING =>
        {
            Outcome::Change
        }
        _ => Outcome::Wait,
    };
    record(LaneChangeEval {
        side: intent.side,
        own_after,
        own_gain,
        new_follower,
        old_follower,
        incentive,
        safe,
        outcome,
    });
    match outcome {
        Outcome::Cancel => LaneDecision::Cancel,
        Outcome::Change => LaneDecision::Change(intent.side),
        _ => LaneDecision::Nothing,
    }
}

/// Whether a vehicle, not a ghost, is behind a vehicle's ghost: before it in the
/// lane's occupants or on a lane leading into it. Only a vehicle can reply to a
/// ghost, so ghosts behind don't count; counting them would deadlock a lane that
/// several vehicles indicate into, each waiting on the others' ghosts.
fn anyone_behind_ghost(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    lane: LaneId,
    id: VehicleId,
) -> bool {
    let list = &occupants[lane.0 as usize];
    let Some(ghost) = list.iter().position(|o| o.id == id && o.ghost) else {
        return false;
    };
    let vehicle = |o: &Occupant| !o.ghost && o.id != id;
    list[..ghost].iter().any(vehicle)
        || network
            .lane(lane)
            .predecessors()
            .iter()
            .any(|p| occupants[p.0 as usize].iter().any(vehicle))
}

/// A vehicle's acceleration under the longitudinal model from its rear axle at
/// `axle_s` on the first of `lanes`, following `course` across them, against the
/// free road and its leader there.
#[allow(clippy::too_many_arguments)]
fn accel_along(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    id: VehicleId,
    vehicle: &Vehicle,
    lanes: &[LaneId],
    axle_s: f64,
    course: Course<'_>,
    ghosts: Ghosts,
) -> f64 {
    let desired = vehicle.spec.desired_speed(network.lane(lanes[0]).speed_limit());
    let mut accel = acceleration::free_road(&vehicle.spec, vehicle.speed, desired);
    let search = occupancy::find_leader(
        network,
        occupants,
        id,
        vehicle,
        lanes,
        axle_s,
        course,
        ghosts,
        None,
        |_| {},
    );
    if let Some((leader, gap)) = search.leader {
        let leader_speed = vehicles[leader].speed;
        accel = accel.min(acceleration::interaction(
            &vehicle.spec,
            vehicle.speed,
            gap,
            leader_speed,
        ));
    }
    accel
}

/// A vehicle's acceleration if it were on the neighboring lane `target`, level
/// with where it is now, and its rear axle's `s` there. Other vehicles' ghosts
/// there count as leaders.
fn accel_in(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    id: VehicleId,
    vehicle: &Vehicle,
    target: LaneId,
) -> (f64, f64) {
    let current = network.lane(vehicle.path[0]);
    let lane = network.lane(target);
    // Adjacent lanes start and end together, so the same fraction is a good hint.
    let hint = lane.lane_pos(vehicle.lane_pos.s / current.length() * lane.length(), 0.0);
    let projected = lane.world_to_lane_near(vehicle.pos, hint);
    let axle_s = projected.s;
    let mut path = ArrayVec::<LaneId, PATH_LEN>::new();
    path.push(target);
    extend_path(network, &mut path);
    // Changing, it would steer from where it is across onto the target lane's
    // centerline along a cubic, as `change_lane` sets up.
    let change = LaneChange::new(axle_s, projected.t, 0.0, vehicle.speed);
    let accel = accel_along(
        network,
        occupants,
        vehicles,
        id,
        vehicle,
        &path,
        axle_s,
        Course::new(&path, Some(change)),
        Ghosts::Lead,
    );
    (accel, axle_s)
}

/// Whether another vehicle, or another vehicle's ghost, is alongside: whether any
/// record overlaps the stretch the vehicle would take up on `lane`, from its rear
/// to its front, with its rear axle at `axle_s`.
///
/// Leader search looks ahead of the rear axle and a follower's search ahead of
/// its own, so a vehicle directly alongside is seen by neither. This closes that
/// gap: MOBIL's requirement that both new gaps be positive.
///
/// A record is on the lane its rear axle is on, so near either end of `lane` the
/// stretch also overlaps records on the lanes before and after it: a vehicle
/// whose front has crossed into `lane` but whose rear axle hasn't, or whose rear
/// has crossed out of it. It reads only the records next to that stretch, in
/// `lane`'s sorted list and those of its predecessors and successors.
fn alongside(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    lane: LaneId,
    id: VehicleId,
    vehicle: &Vehicle,
    axle_s: f64,
) -> bool {
    let rear = axle_s - vehicle.spec.rear_axle;
    let front = rear + vehicle.spec.length;
    // Whether a record in `list` overlaps [rear, front], shifted by `offset` into
    // that list's own `s`.
    let overlaps = |list: &[Occupant], offset: f64| {
        let (rear, front) = (rear + offset, front + offset);
        // Records start at their rear, so only those starting before our front can
        // overlap; of those, walk back while their rear is within a vehicle length.
        let end = list.partition_point(|o| o.s < front);
        list[..end]
            .iter()
            .rev()
            .filter(|o| o.id != id)
            .take_while(|o| o.s > rear - MAX_VEHICLE_LENGTH)
            .any(|o| o.s + vehicles[o.id].spec.length > rear)
    };
    let here = network.lane(lane);
    let list = |l: &LaneId| &occupants[l.0 as usize][..];
    overlaps(list(&lane), 0.0)
        || here.predecessors().iter().any(|p| overlaps(list(p), network.lane(*p).length()))
        || here.successors().iter().any(|n| overlaps(list(n), -here.length()))
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
