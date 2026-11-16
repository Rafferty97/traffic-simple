use arrayvec::ArrayVec;
use glam::DVec2;
use slotmap::{Key, SlotMap};

use crate::debug::{AccelSource, DebugEvent, DebugSink, NoopSink};
use crate::kinematics;
use crate::lane::LanePos;
use crate::math::wrap_angle;
use crate::models::acceleration;
use crate::models::lane_change::{self, Decision, Effect, LaneChangeEval, Prospect};
use crate::models::lateral::{self, LaneChange};
use crate::network::{LaneId, Side, SimNetwork};
use crate::occupancy::{self, Course, Ghosts, Lateral, Occupant, find_follower};
use crate::vehicle::{Intent, PATH_LEN, Placement, Vehicle, VehicleId, VehicleSpec};

/// Longest substep in seconds. Longer frames are split.
const MAX_SUBSTEP: f64 = 1.0 / 60.0;
/// Period of the longitudinal model update in seconds.
const ACCEL_PERIOD: f64 = 1.0 / 20.0;

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
    /// Vehicles with a speed imposed on them, and the most each may go, in m/s: 0
    /// holds one to a stop. Few, so a list beats a field on every vehicle.
    caps: Vec<(VehicleId, f64)>,
    /// Time since the last longitudinal model update, in seconds.
    accel_clock: f64,
    /// Scratch space for the longitudinal update's decisions, reused between
    /// updates: each vehicle's acceleration, and whose ghost it is making room for.
    decisions: Vec<(VehicleId, f64, Option<VehicleId>)>,
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
    /// Scratch space for lane-change decisions, reused between substeps: each
    /// deciding vehicle's lane-change state afterwards, and what it decided.
    lane_decisions: Vec<(VehicleId, lane_change::State, Decision)>,
    sink: D,
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

/// A simulation's whole state at one instant, without its debug sink: a freeze
/// frame. [`Simulation::restore`] turns it back into a simulation, exactly as it
/// was. With the `serde` feature, it serializes, for saving to a file.
///
/// A restored simulation continues bit for bit like the original, through steps,
/// holds, lane changes and removals, until it inserts a vehicle into a slot freed
/// before the snapshot. The vehicle store's free slots come back in a different
/// order, so that vehicle can get a different id from the one it would have had,
/// and the update order can differ from then on.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Snapshot {
    network: SimNetwork,
    vehicles: SlotMap<VehicleId, Vehicle>,
    occupants: Vec<Vec<Occupant>>,
    caps: Vec<(VehicleId, f64)>,
    accel_clock: f64,
    time: f64,
    decision_cursor: usize,
    decision_budget: f64,
}

impl<D: DebugSink> Simulation<D> {
    /// A freeze frame of the whole state, network included.
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            network: self.network.clone(),
            vehicles: self.vehicles.clone(),
            occupants: self.occupants.clone(),
            caps: self.caps.clone(),
            accel_clock: self.accel_clock,
            time: self.time,
            decision_cursor: self.decision_cursor,
            decision_budget: self.decision_budget,
        }
    }

    /// A simulation exactly as it was when `snapshot` was taken, with `sink` for
    /// its debug events. The sink starts empty, so its events refill as each
    /// update runs.
    pub fn restore(snapshot: Snapshot, sink: D) -> Self {
        let Snapshot {
            network,
            vehicles,
            occupants,
            caps,
            accel_clock,
            time,
            decision_cursor,
            decision_budget,
        } = snapshot;
        Self {
            network,
            vehicles,
            occupants,
            caps,
            accel_clock,
            time,
            decision_cursor,
            decision_budget,
            // Scratch space, cleared before every use.
            decisions: Vec::new(),
            leavers: Vec::new(),
            departed: Vec::new(),
            lane_decisions: Vec::new(),
            sink,
        }
    }

    pub fn new(network: SimNetwork, sink: D) -> Self {
        let lane_count = network.lanes().count();
        Self {
            network,
            vehicles: SlotMap::with_key(),
            occupants: vec![Vec::new(); lane_count],
            caps: Vec::new(),
            accel_clock: 0.0,
            decisions: Vec::new(),
            leavers: Vec::new(),
            departed: Vec::new(),
            time: 0.0,
            decision_cursor: 0,
            decision_budget: 0.0,
            lane_decisions: Vec::new(),
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
        // A key that matches no vehicle, since this one isn't inserted yet.
        let id = VehicleId::null();
        // Leader and follower split the lane at its rear, so any vehicle alongside
        // is one or the other, with no gap, and refuses the insertion.
        let axle_s = vehicle.lane_pos.s;
        let rear = axle_s - spec.rear_axle;
        let path = vehicle.path.clone();
        let leader = occupancy::find_leader(
            &self.network,
            &self.occupants,
            &self.vehicles,
            id,
            &vehicle,
            &path,
            axle_s,
            None,
            occupancy::horizon(&self.network, &vehicle, at.lane),
            Course::of(&vehicle),
            Ghosts::Lead,
            None,
            |_| {},
        );
        // No faster than the nearest leader, and braking gently enough for all.
        if let Some(&(nearest, _)) = leader.leaders.first() {
            vehicle.speed = vehicle.speed.min(self.vehicles[nearest].speed);
        }
        for &(leader, gap) in &leader.leaders {
            let leader_speed = self.vehicles[leader].speed;
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
            id,
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
            lc: lane_change::State::default(),
        }
    }

    /// Adds a built vehicle to the simulation and to its lane's occupants.
    fn register(&mut self, vehicle: Vehicle) -> VehicleId {
        let lane_id = vehicle.path[0];
        let lane_pos = vehicle.lane_pos;
        let id = self.vehicles.insert(vehicle);
        let occupant = Occupant::new(&self.network, lane_id, id, &self.vehicles[id], lane_pos);
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
        self.caps.retain(|&(c, _)| c != id);
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

    /// Imposes a maximum speed on a vehicle, in m/s, until
    /// [`Simulation::release`], replacing any it had. Above it, the vehicle brakes
    /// at its comfortable deceleration; nearing it from below, it eases up to it.
    /// A cap of 0 brakes it to a stop and holds it there.
    pub fn cap_speed(&mut self, id: VehicleId, max_speed: f64) {
        if !self.vehicles.contains_key(id) {
            return;
        }
        match self.caps.iter_mut().find(|(c, _)| *c == id) {
            Some(cap) => cap.1 = max_speed.max(0.0),
            None => self.caps.push((id, max_speed.max(0.0))),
        }
    }

    /// The maximum speed imposed on a vehicle, if any.
    pub fn speed_cap(&self, id: VehicleId) -> Option<f64> {
        self.caps.iter().find(|(c, _)| *c == id).map(|&(_, max)| max)
    }

    /// Makes a vehicle brake to a stop and wait, until [`Simulation::release`]: a
    /// speed cap of 0.
    pub fn hold(&mut self, id: VehicleId) {
        self.cap_speed(id, 0.0);
    }

    /// Lifts any speed cap on a vehicle, held or not.
    pub fn release(&mut self, id: VehicleId) {
        self.caps.retain(|&(c, _)| c != id);
    }

    /// Whether a vehicle is held to a stop: capped at 0.
    pub fn is_held(&self, id: VehicleId) -> bool {
        self.speed_cap(id) == Some(0.0)
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
        let neighbor = self
            .network
            .lane(vehicle.path[0])
            .adj(side)
            .ok_or(LaneChangeError::NoNeighbor)?;
        if !neighbor.can_change {
            return Err(LaneChangeError::NotPermitted);
        }

        let (lane_pos, change) = plan_change(&self.network, vehicle, neighbor.lane);
        let old_lane = vehicle.path[0];
        vehicle.path.clear();
        vehicle.path.push(neighbor.lane);
        extend_path(&self.network, &mut vehicle.path);
        vehicle.lane_pos = lane_pos;
        vehicle.old_side = Some(side.opposite());
        vehicle.lane_change = Some(change);
        vehicle.intent = None;

        // Its record on the lane it is leaving is now diverging. Set here rather
        // than left to the next occupancy update, so searches before then see it.
        let old = &mut self.occupants[old_lane.0 as usize];
        if let Some(record) = old.iter_mut().find(|o| o.id == id && !o.ghost) {
            record.lateral = Lateral::Diverging(side);
        }

        let list = &mut self.occupants[neighbor.lane.0 as usize];
        if let Some(ghost) = list.iter_mut().find(|o| o.id == id && o.ghost) {
            ghost.ghost = false;
        } else {
            let occupant = Occupant::new(
                &self.network,
                neighbor.lane,
                id,
                &self.vehicles[id],
                lane_pos,
            );
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
        let ghost = Occupant::ghost(&self.network, neighbor.lane, id, vehicle, hint);
        let list = &mut self.occupants[neighbor.lane.0 as usize];
        let index = list.partition_point(|o| o.s <= ghost.s);
        list.insert(index, ghost);
        self.vehicles[id].intent = Some(Intent { side, since: self.time });
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
            let mut state = vehicle.lc;
            let Some((decision, evals)) = decide(
                network, occupants, vehicles, self.time, id, vehicle, &mut state,
            ) else {
                continue;
            };
            for eval in evals.into_iter().flatten() {
                sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
            }
            lane_decisions.push((id, state, decision));
        }
        self.decision_cursor = (start + count) % n;

        for i in 0..self.lane_decisions.len() {
            let (id, state, decision) = self.lane_decisions[i];
            self.vehicles[id].lc = state;
            match decision {
                Decision::Nothing => {}
                Decision::Indicate(side) => self.indicate(id, side),
                Decision::Cancel => self.cancel_intent(id),
                // Already checked: the decision only asks for permitted changes.
                Decision::Change(side) => drop(self.change_lane(id, side)),
            }
        }
    }

    /// Longitudinal model: sets each vehicle's held acceleration (see
    /// [`longitudinal`]). Decisions are computed from a read-only view of the
    /// state, then applied.
    ///
    /// Vehicles are visited lane by lane, from the front of each lane back, at
    /// their record on the lane they follow, so each one's search there starts at
    /// its own record rather than looking for it. SUMO plans each lane's vehicles
    /// from the front back in the same way.
    fn update_accels(&mut self) {
        let Self { network, vehicles, occupants, decisions, sink, .. } = self;
        sink.record(|| DebugEvent::RecomputeAccels);
        decisions.clear();
        for (index, list) in occupants.iter().enumerate() {
            let lane = LaneId(index as u32);
            let half_length = 0.5 * network.lane(lane).length();
            for (i, o) in list.iter().enumerate().rev() {
                if o.ghost {
                    continue;
                }
                let vehicle = &vehicles[o.id];
                // On a loop, a vehicle can briefly have a second record on the lane
                // it follows, lingering at its end, about a lane's length from its
                // rear.
                let rear = vehicle.lane_pos.s - vehicle.spec.rear_axle;
                if vehicle.path[0] != lane || (o.s - rear).abs() > half_length {
                    continue;
                }
                let (accel, yielding) =
                    longitudinal(network, occupants, vehicles, sink, o.id, vehicle, i);
                decisions.push((o.id, accel, yielding));
            }
        }
        debug_assert_eq!(
            decisions.len(),
            vehicles.len(),
            "each vehicle has one record on the lane it follows"
        );

        for &(id, accel, yielding) in &self.decisions {
            self.vehicles[id].accel = accel;
            self.vehicles[id].lc.set_yielding(yielding);
        }
        for &(id, max_speed) in &self.caps {
            let vehicle = &mut self.vehicles[id];
            let term = acceleration::capped(&vehicle.spec, vehicle.speed, max_speed);
            let source = AccelSource::Cap { max_speed };
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
                let occupant =
                    Occupant::new(network, vehicle.path[0], id, vehicle, vehicle.lane_pos);
                occupants[vehicle.path[0].0 as usize].push(occupant);
                // Still straddling after a lane change: also join the old side's
                // next lane, so followers there keep seeing it.
                if let Some(old) = vehicle.old_side.and_then(|side| next.adj(side)) {
                    let old_lane = network.lane(old.lane);
                    let hint = old_lane.lane_pos(vehicle.lane_pos.s, 0.0);
                    let occupant = Occupant::new(network, old.lane, id, vehicle, hint);
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
                                .push(Occupant::ghost(network, n.lane, id, vehicle, hint));
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
        // one it follows, on its old side, or, just after a section boundary, on
        // the lane beside the one before: at a lane drop that is all there is.
        // Losing a record elsewhere, such as at the end of the lane before, leaves
        // it straddling.
        for id in departed.drain(..) {
            let vehicle = &mut vehicles[id];
            let Some(side) = vehicle.old_side else {
                continue;
            };
            let on = |lane: LaneId| {
                network.lane(lane).adj(side).is_some_and(|old| {
                    occupants[old.lane.0 as usize].iter().any(|o| o.id == id && !o.ghost)
                })
            };
            let following = vehicle.path[0];
            let straddling =
                on(following) || network.lane(following).predecessors().iter().any(|&p| on(p));
            if !straddling {
                vehicle.old_side = None;
            }
        }
    }
}

/// A vehicle's acceleration under the longitudinal model, and whose ghost it is
/// making room for, if anyone's. `own` is the index of its record in the
/// occupants of the lane it follows.
///
/// The acceleration is the minimum over every term the model considers: the free
/// road, each leader along its path and, while straddling, on its old side, the
/// end of its path, and the ghost it makes room for. Each term is recorded to the
/// debug sink as it is computed.
#[allow(clippy::too_many_arguments)]
fn longitudinal<D: DebugSink>(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    own: usize,
) -> (f64, Option<VehicleId>) {
    let lane = network.lane(vehicle.path[0]);
    let desired_speed = vehicle.spec.desired_speed(lane.speed_limit());
    let free = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed);
    let source = AccelSource::FreeRoad { desired_speed };
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: free });
    let mut accel = free;
    let horizon = occupancy::horizon(network, vehicle, vehicle.path[0]);

    // Takes the sink rather than capturing it, so the searches can record too.
    let mut react = |sink: &mut D, leader: VehicleId, gap: f64| {
        let leader_speed = vehicles[leader].speed;
        let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, leader_speed);
        let source = AccelSource::Leader { id: leader, gap };
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        accel = accel.min(term);
    };

    let axle_s = vehicle.lane_pos.s;
    let search = occupancy::find_leader(
        network,
        occupants,
        vehicles,
        id,
        vehicle,
        &vehicle.path,
        axle_s,
        Some(own),
        horizon,
        Course::of(vehicle),
        Ghosts::Pass,
        None,
        |check| sink.record(|| DebugEvent::PassCheck { vehicle: id, check }),
    );
    for &(leader, gap) in &search.leaders {
        react(sink, leader, gap);
    }

    // The end of the lanes it knows it will take, as a stop line.
    let end_term = path_end(network, &vehicle.path, axle_s, &vehicle.spec)
        .filter(|&gap| gap <= horizon)
        .map(|gap| {
            let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, 0.0);
            let source = AccelSource::PathEnd { gap };
            sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
            term
        });

    // Making room for a vehicle indicating into its lane ahead, as the lane-change
    // model chooses, by what following its ghost would take.
    let ghost = search.ghost.map(|(owner, gap)| {
        let owner_speed = vehicles[owner].speed;
        let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, owner_speed);
        (owner, gap, term)
    });
    let yielding = lane_change::make_room(
        &vehicle.lc,
        &vehicle.spec,
        ghost.map(|(owner, _, term)| (owner, term)),
    );
    let yield_term = ghost.filter(|_| yielding.is_some()).map(|(owner, gap, term)| {
        let source = AccelSource::Yield { id: owner, gap };
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        term
    });

    // Still straddling partway through a lane change: also search the old side,
    // along the neighbors of the vehicle's path, from its position on the old
    // lane. Once the change is complete, nothing on the old lanes can be in its
    // way.
    if let Some(side) = vehicle.old_side
        && vehicle.lane_change.is_some()
    {
        let old_path: ArrayVec<LaneId, PATH_LEN> = vehicle
            .path
            .iter()
            .map_while(|&lane| network.lane(lane).adj(side).map(|n| n.lane))
            .collect();
        if let Some(&first) = old_path.first() {
            let old = network.lane(first);
            // Adjacent lanes start and end together, so the same fraction along the
            // old lane is a good hint.
            let fraction = vehicle.lane_pos.s / lane.length();
            let hint = old.lane_pos(fraction * old.length(), 0.0);
            let axle_s = old.world_to_lane_near(vehicle.pos, hint).s;
            let search = occupancy::find_leader(
                network,
                occupants,
                vehicles,
                id,
                vehicle,
                &old_path,
                axle_s,
                None,
                horizon,
                Course::of(vehicle),
                Ghosts::Pass,
                None,
                |check| sink.record(|| DebugEvent::PassCheck { vehicle: id, check }),
            );
            for &(leader, gap) in &search.leaders {
                react(sink, leader, gap);
            }
        }
    }
    for term in [end_term, yield_term].into_iter().flatten() {
        accel = accel.min(term);
    }
    (accel, yielding)
}

/// The gap from a vehicle's nose to the end of `lanes`, the lanes it knows it
/// will take, with its rear axle at `axle_s` on the first, if it must stop there.
/// Past the end of its path it doesn't know where it is going, so it treats the
/// end as a stop line, unless it leaves the network there. A routing model will
/// say where a vehicle leaves; until then, it leaves wherever its path's last
/// lane leads nowhere.
fn path_end(
    network: &SimNetwork,
    lanes: &[LaneId],
    axle_s: f64,
    spec: &VehicleSpec,
) -> Option<f64> {
    let last = network.lane(*lanes.last()?);
    if last.successors().is_empty() {
        return None;
    }
    let length: f64 = lanes.iter().map(|&lane| network.lane(lane).length()).sum();
    Some(length - (axle_s + spec.length - spec.rear_axle))
}

/// Removes a vehicle's ghost from a lane's occupants.
fn remove_ghost(occupants: &mut [Vec<Occupant>], lane: LaneId, id: VehicleId) {
    occupants[lane.0 as usize].retain(|o| !(o.id == id && o.ghost));
}

/// A vehicle's lane-change decision, by the lane-change model, updating its
/// model `state`: what it decided, and how it weighed up each side. `None` if it
/// is partway through a change, or still straddling, when it doesn't decide.
///
/// The model asks what a change to each side would do. Measuring that is the
/// simulation's part: its own acceleration there, and the followers it would
/// affect, found by searching back from it.
fn decide(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    time: f64,
    id: VehicleId,
    vehicle: &Vehicle,
    state: &mut lane_change::State,
) -> Option<(Decision, [Option<LaneChangeEval>; 2])> {
    if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
        return None;
    }
    let lane = network.lane(vehicle.path[0]);
    // Each lane is split where its leader search splits it, at its rear, so nobody
    // slips between leader and follower: a vehicle alongside is one or the other,
    // with no gap, and makes the change unsafe.
    let rear = |axle_s: f64| axle_s - vehicle.spec.rear_axle;
    // Its acceleration now, and its old follower's, behind it in its own lane, are
    // the same for either side, so they are measured once, when first needed. The
    // old follower's is as now, and with this vehicle gone.
    let mut own_now = None;
    let mut old_follower = None;
    let prospect = |side: Side| {
        let target = lane.adj(side).filter(|n| n.can_change)?;
        let own_now = *own_now.get_or_insert_with(|| {
            accel_along(
                network,
                occupants,
                vehicles,
                id,
                vehicle,
                &vehicle.path,
                vehicle.lane_pos.s,
                Course::of(vehicle),
                Ghosts::Pass,
                None,
            )
        });
        let old_follower = *old_follower.get_or_insert_with(|| {
            let own_lane = vehicle.path[0];
            find_follower(
                network,
                occupants,
                vehicles,
                own_lane,
                rear(vehicle.lane_pos.s),
                id,
            )
            .map(|(follower, _)| {
                let f = &vehicles[follower];
                let without = accel_along(
                    network,
                    occupants,
                    vehicles,
                    follower,
                    f,
                    &f.path,
                    f.lane_pos.s,
                    Course::of(f),
                    Ghosts::Pass,
                    Some(id),
                );
                Effect { before: f.accel, after: without, speed: f.speed }
            })
        });
        let (own_after, axle_s) = accel_in(network, occupants, vehicles, id, vehicle, target.lane);
        // The new follower, behind it in the target lane: as now, but with this
        // vehicle as a leader too.
        let new_follower =
            find_follower(network, occupants, vehicles, target.lane, rear(axle_s), id).map(
                |(follower, gap)| {
                    let f = &vehicles[follower];
                    let cut_in = acceleration::interaction(&f.spec, f.speed, gap, vehicle.speed);
                    Effect { before: f.accel, after: f.accel.min(cut_in), speed: f.speed }
                },
            );
        Some(Prospect { own_now, own_after, new_follower, old_follower })
    };
    Some(lane_change::decide(
        state,
        vehicle.intent,
        &vehicle.spec,
        time,
        DECISION_PERIOD,
        prospect,
    ))
}

/// A vehicle's acceleration under the longitudinal model from its rear axle at
/// `axle_s` on the first of `lanes`, following `course` across them, against the
/// free road, its leaders there and the end of `lanes`, as if `skip` weren't
/// there.
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
    skip: Option<VehicleId>,
) -> f64 {
    let desired = vehicle.spec.desired_speed(network.lane(lanes[0]).speed_limit());
    let mut accel = acceleration::free_road(&vehicle.spec, vehicle.speed, desired);
    let horizon = occupancy::horizon(network, vehicle, lanes[0]);
    let search = occupancy::find_leader(
        network,
        occupants,
        vehicles,
        id,
        vehicle,
        lanes,
        axle_s,
        None,
        horizon,
        course,
        ghosts,
        skip,
        |_| {},
    );
    for &(leader, gap) in &search.leaders {
        let leader_speed = vehicles[leader].speed;
        accel = accel.min(acceleration::interaction(
            &vehicle.spec,
            vehicle.speed,
            gap,
            leader_speed,
        ));
    }
    if let Some(gap) = path_end(network, lanes, axle_s, &vehicle.spec).filter(|&g| g <= horizon) {
        accel = accel.min(acceleration::interaction(
            &vehicle.spec,
            vehicle.speed,
            gap,
            0.0,
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
    // Changing, it would follow the same cubic `change_lane` would give it.
    let (projected, change) = plan_change(network, vehicle, target);
    let axle_s = projected.s;
    let mut path = ArrayVec::<LaneId, PATH_LEN>::new();
    path.push(target);
    extend_path(network, &mut path);
    let accel = accel_along(
        network,
        occupants,
        vehicles,
        id,
        vehicle,
        &path,
        axle_s,
        Course::new(&path, Some(change), axle_s),
        Ghosts::Lead,
        None,
    );
    (accel, axle_s)
}

/// How a vehicle not already changing lanes would change into `target`, a lane
/// beside the one it is on: its rear axle's position on `target`, and the cubic
/// its path would follow onto `target`'s centerline. [`Simulation::change_lane`]
/// carries this out, and the lane-change decision weighs it up, so the two agree.
///
/// The cubic starts where pure pursuit is aiming on the vehicle's current path,
/// with that path's offset and slope relative to `target`, so the target point
/// carries on smoothly instead of jumping.
fn plan_change(network: &SimNetwork, vehicle: &Vehicle, target: LaneId) -> (LanePos, LaneChange) {
    let current = network.lane(vehicle.path[0]);
    let lane = network.lane(target);
    // Adjacent lanes start and end together, so the same fraction is a good hint.
    let hint = lane.lane_pos(vehicle.lane_pos.s / current.length() * lane.length(), 0.0);
    let lane_pos = lane.world_to_lane_near(vehicle.pos, hint);
    let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
    let aim = network.point_along(&vehicle.path, ahead, 0.0);
    let beyond = network.point_along(&vehicle.path, ahead + 1.0, 0.0);
    let start = lane.world_to_lane_near(aim, lane_pos);
    let next = lane.world_to_lane_near(beyond, start);
    let slope = (next.t - start.t) / (next.s - start.s);
    (
        lane_pos,
        LaneChange::new(start.s, start.t, slope, vehicle.speed),
    )
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

#[cfg(test)]
mod test {
    //! Snapshots: each sets up one instant and checks what the searches and the
    //! lane-change decision make of it, without stepping.

    use glam::DVec2;

    use super::*;
    use crate::debug::DebugEvent;
    use crate::lane::Lane;
    use crate::models::lane_change::Outcome;

    const LIMIT: f64 = 14.0;
    const WIDTH: f64 = 3.5;

    fn car() -> VehicleSpec {
        VehicleSpec {
            length: 4.6,
            width: 1.9,
            max_speed: 30.0,
            speed_factor: 1.0,
            max_accel: 1.0,
            comfy_decel: 1.5,
            time_headway: 1.0,
            min_gap: 1.5,
            rear_axle: 1.0,
            front_axle: 3.7,
            courtesy: 1.0,
            politeness: 0.2,
        }
    }

    /// A straight road along x in sections, each `lanes` lanes wide, lane 0 on
    /// the left. Section `k`'s lane `i` is `LaneId(k * lanes + i)` and runs on to
    /// section `k + 1`'s lane `i`.
    fn road(sections: &[(f64, f64)], lanes: u32) -> SimNetwork {
        let mut all = Vec::new();
        for &(x0, x1) in sections {
            for i in 0..lanes {
                let y = -WIDTH * i as f64;
                all.push(Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT).unwrap());
            }
        }
        let mut network = SimNetwork::new(all);
        for k in 0..sections.len() as u32 {
            for i in 1..lanes {
                network.set_adjacent(LaneId(k * lanes + i - 1), LaneId(k * lanes + i), true, true);
            }
            if k > 0 {
                for i in 0..lanes {
                    network.connect(LaneId((k - 1) * lanes + i), LaneId(k * lanes + i));
                }
            }
        }
        network.validate().unwrap();
        network
    }

    /// Two lanes side by side, 500 m long.
    fn two_lanes() -> Simulation {
        Simulation::new(road(&[(0.0, 500.0)], 2), NoopSink)
    }

    fn at(lane: u32, s: f64, speed: f64) -> Placement {
        Placement { lane: LaneId(lane), s, speed }
    }

    /// A vehicle's rear, where both searches split its lane.
    fn rear(sim: &Simulation, id: VehicleId) -> f64 {
        let v = &sim.vehicles[id];
        v.lane_pos.s - v.spec.rear_axle
    }

    fn follower(sim: &Simulation, lane: u32, s: f64, skip: VehicleId) -> Option<(VehicleId, f64)> {
        find_follower(
            &sim.network,
            &sim.occupants,
            &sim.vehicles,
            LaneId(lane),
            s,
            skip,
        )
    }

    fn leader(sim: &Simulation, id: VehicleId) -> Option<(VehicleId, f64)> {
        let v = &sim.vehicles[id];
        occupancy::find_leader(
            &sim.network,
            &sim.occupants,
            &sim.vehicles,
            id,
            v,
            &v.path,
            v.lane_pos.s,
            None,
            occupancy::horizon(&sim.network, v, v.path[0]),
            Course::of(v),
            Ghosts::Pass,
            None,
            |_| {},
        )
        .leaders
        .first()
        .copied()
    }

    /// What `id` decides now, its lane-change state afterwards, and what it
    /// weighed up. Partway through a change, it decides nothing and weighs up
    /// nothing.
    fn decision(
        sim: &Simulation,
        id: VehicleId,
    ) -> (Decision, lane_change::State, Vec<LaneChangeEval>) {
        let (net, occ, veh) = (&sim.network, &sim.occupants, &sim.vehicles);
        let mut state = veh[id].lc;
        match decide(net, occ, veh, sim.time, id, &veh[id], &mut state) {
            Some((decision, evals)) => (decision, state, evals.into_iter().flatten().collect()),
            None => (Decision::Nothing, state, Vec::new()),
        }
    }

    /// What `id` decides to do now, and what it weighed up.
    fn decide_now(sim: &Simulation, id: VehicleId) -> (Decision, Vec<LaneChangeEval>) {
        let (decision, _, evals) = decision(sim, id);
        (decision, evals)
    }

    /// A car on lane 1 at 300 m that has been indicating left for 2 s, with
    /// advantage built up for it, behind a car held stopped 20 m ahead, so
    /// changing is well worth it.
    fn indicating_behind_a_stopped_car(sim: &mut Simulation) -> VehicleId {
        let blocker = sim.insert(car(), at(1, 320.0, 0.0));
        sim.hold(blocker);
        let id = sim.insert(car(), at(1, 300.0, 5.0));
        sim.indicate(id, Side::Left);
        sim.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        sim.time = 2.0;
        id
    }

    // Following.

    #[test]
    fn the_follower_is_the_nearest_vehicle_behind() {
        let mut sim = two_lanes();
        let far = sim.insert(car(), at(0, 20.0, LIMIT));
        let near = sim.insert(car(), at(0, 40.0, LIMIT));
        let _ahead = sim.insert(car(), at(0, 80.0, 0.0));
        let (found, gap) = follower(&sim, 0, 60.0, VehicleId::null()).unwrap();
        assert_eq!(found, near);
        // From its front, 3.6 m ahead of its rear axle at 40, to 60.
        assert!((gap - 16.4).abs() < 1e-9, "gap {gap}");
        assert_ne!(found, far);
    }

    #[test]
    fn the_follower_search_passes_over_ghosts_and_the_searcher() {
        let mut sim = two_lanes();
        let behind = sim.insert(car(), at(0, 20.0, LIMIT));
        let searcher = sim.insert(car(), at(0, 45.0, 0.0));
        // Indicating from lane 1 into lane 0, between the two.
        let indicating = sim.insert(car(), at(1, 35.0, 0.0));
        sim.indicate(indicating, Side::Left);
        let found = follower(&sim, 0, 60.0, searcher).map(|(id, _)| id);
        assert_eq!(found, Some(behind));
    }

    #[test]
    fn the_follower_search_counts_a_vehicle_moving_out_of_the_lane() {
        // Changing out of lane 0 into lane 1, it is still partly in the way.
        let mut sim = two_lanes();
        let leaving = sim.insert(car(), at(0, 30.0, LIMIT));
        sim.change_lane(leaving, Side::Right).unwrap();
        let record = sim.occupants[0].iter().find(|o| o.id == leaving).unwrap();
        assert_eq!(record.lateral, Lateral::Diverging(Side::Right));
        let found = follower(&sim, 0, 60.0, VehicleId::null()).map(|(id, _)| id);
        assert_eq!(found, Some(leaving));
    }

    #[test]
    fn the_follower_search_continues_onto_the_lanes_leading_in() {
        // Two sections of two lanes: lane 0 leads into lane 2.
        let mut sim = Simulation::new(road(&[(0.0, 100.0), (100.0, 200.0)], 2), NoopSink);
        let behind = sim.insert(car(), at(0, 80.0, LIMIT));
        let (found, gap) = follower(&sim, 2, 10.0, VehicleId::null()).unwrap();
        assert_eq!(found, behind);
        // Its front is at 83.6 on lane 0, 16.4 m before lane 2 starts.
        assert!((gap - 26.4).abs() < 1e-9, "gap {gap}");
    }

    #[test]
    fn the_follower_search_stops_at_the_followers_horizon() {
        // Only as far back as the follower's own leader search would reach.
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(0, 50.0, LIMIT));
        let horizon = occupancy::horizon(&sim.network, &sim.vehicles[id], LaneId(0));
        let front = 50.0 + 3.6;
        assert!(follower(&sim, 0, front + horizon - 0.1, VehicleId::null()).is_some());
        assert!(follower(&sim, 0, front + horizon + 0.1, VehicleId::null()).is_none());
        // Stopped, it looks only a few metres ahead.
        sim.vehicles[id].speed = 0.0;
        assert!(follower(&sim, 0, front + 10.0, VehicleId::null()).is_none());
    }

    #[test]
    fn the_follower_search_wraps_round_a_loop() {
        // A lane that is its own successor: the vehicle at 970, its front at
        // 973.6, is 76.4 m behind a point at 50, round the seam.
        let lane = Lane::new(&[DVec2::ZERO, DVec2::new(1000.0, 0.0)], WIDTH, LIMIT).unwrap();
        let mut network = SimNetwork::new(vec![lane]);
        network.connect(LaneId(0), LaneId(0));
        let mut sim = Simulation::new(network, NoopSink);
        let behind = sim.insert(car(), at(0, 970.0, LIMIT));
        let (found, gap) = follower(&sim, 0, 50.0, VehicleId::null()).unwrap();
        assert_eq!(found, behind);
        assert!((gap - 76.4).abs() < 1e-9, "gap {gap}");
    }

    #[test]
    fn on_lanes_leading_in_only_vehicles_headed_this_way_count() {
        // Lane 0 splits into lane 1, straight on, and lane 2, its first successor.
        let straight = |x0: f64, x1: f64| {
            Lane::new(&[DVec2::new(x0, 0.0), DVec2::new(x1, 0.0)], WIDTH, LIMIT).unwrap()
        };
        let mut network = SimNetwork::new(vec![
            straight(0.0, 100.0),
            straight(100.0, 200.0),
            straight(100.0, 200.0),
        ]);
        network.connect(LaneId(0), LaneId(2));
        network.connect(LaneId(0), LaneId(1));
        let mut sim = Simulation::new(network, NoopSink);
        // Its path follows the first successor, lane 2, so it isn't lane 1's follower.
        let turning = sim.insert(car(), at(0, 80.0, LIMIT));
        assert_eq!(sim.vehicles[turning].path[1], LaneId(2));
        assert!(follower(&sim, 1, 10.0, VehicleId::null()).is_none());
        assert_eq!(
            follower(&sim, 2, 10.0, VehicleId::null()).map(|(id, _)| id),
            Some(turning)
        );
    }

    #[test]
    fn the_leader_and_follower_searches_meet_at_the_rear() {
        // A car level with another in the same lane, at every offset within two
        // car lengths: exactly one search finds it, and if their bodies overlap,
        // with a gap of zero or less.
        for tenths in -92..=92 {
            let mut sim = two_lanes();
            let id = sim.insert(car(), at(0, 100.0, 0.0));
            let s = 100.0 + tenths as f64 / 10.0;
            let other = sim.insert(car(), at(0, s, 0.0));
            let ahead = leader(&sim, id).filter(|&(l, _)| l == other);
            let behind = follower(&sim, 0, rear(&sim, id), id).filter(|&(f, _)| f == other);
            assert!(
                ahead.is_some() != behind.is_some(),
                "at {s}: {ahead:?} {behind:?}"
            );
            let gap = ahead.or(behind).unwrap().1;
            let overlap = (s - 100.0).abs() < car().length;
            assert_eq!(gap <= 0.0, overlap, "at {s}: gap {gap}");
        }
    }

    // Records.

    #[test]
    fn a_lane_change_marks_the_record_it_leaves_diverging_at_once() {
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 100.0, 10.0));
        sim.change_lane(id, Side::Left).unwrap();
        let lateral = |lane: usize| {
            sim.occupants[lane].iter().find(|o| o.id == id && !o.ghost).map(|o| o.lateral)
        };
        assert_eq!(
            lateral(1),
            Some(Lateral::Diverging(Side::Left)),
            "the lane left"
        );
        assert_eq!(lateral(0), Some(Lateral::InLane), "the lane moved into");
        assert_eq!(sim.vehicles[id].old_side, Some(Side::Right));
    }

    #[test]
    fn a_ghost_is_in_the_lane_it_indicates_into() {
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 100.0, 10.0));
        sim.indicate(id, Side::Left);
        let ghost = sim.occupants[0].iter().find(|o| o.id == id).unwrap();
        assert!(ghost.ghost);
        assert_eq!(ghost.lateral, Lateral::InLane);
        let own = sim.occupants[1].iter().find(|o| o.id == id).unwrap();
        assert_eq!(own.lateral, Lateral::InLane);
    }

    // Deciding.

    #[test]
    fn it_changes_when_the_gap_is_clear_and_it_has_indicated_long_enough() {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
        // Nobody behind in either lane.
        assert_eq!((evals[0].new_follower, evals[0].old_follower), (None, None));
    }

    #[test]
    fn it_waits_until_it_has_indicated_long_enough() {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        sim.time = 0.5 * lane_change::MIN_INDICATING;
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Nothing);
        assert_eq!(evals[0].outcome, Outcome::Wait);
        assert!(evals[0].safe);
    }

    #[test]
    fn it_wont_cut_off_a_fast_car_close_behind_in_the_target_lane() {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        sim.insert(car(), at(0, 290.0, 14.0));
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Nothing);
        let eval = evals[0];
        assert!(!eval.safe);
        assert!(
            eval.new_follower.unwrap() < -SAFE_BRAKING_FOR_TESTS,
            "{eval:?}"
        );
    }

    const SAFE_BRAKING_FOR_TESTS: f64 = acceleration::SAFE_BRAKING;

    #[test]
    fn a_car_far_behind_in_the_target_lane_is_no_follower() {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        // 245 m back, beyond the leader horizon, even coming fast.
        sim.insert(car(), at(0, 50.0, 14.0));
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
        assert_eq!(evals[0].new_follower, None);
    }

    #[test]
    fn it_never_changes_into_a_car_alongside() {
        // Whichever rear is ahead, the car beside it is its leader or its new
        // follower there, with no gap.
        for tenths in -45..=45 {
            let mut sim = two_lanes();
            let id = indicating_behind_a_stopped_car(&mut sim);
            let s = 300.0 + tenths as f64 / 10.0;
            sim.insert(car(), at(0, s, 5.0));
            let (decision, evals) = decide_now(&sim, id);
            // Ahead of it, the car beside makes changing worse than staying, so it
            // cancels; either way it doesn't change.
            assert_ne!(decision, Decision::Change(Side::Left), "beside at {s}");
            assert!(!evals[0].safe, "beside at {s}");
        }
    }

    #[test]
    fn its_old_follower_gains_when_it_leaves() {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        // Close behind it on lane 1, held back by it now.
        let behind = sim.insert(car(), at(1, 290.0, 5.0));
        sim.vehicles[behind].accel = -1.0;
        let (_, evals) = decide_now(&sim, id);
        let gain = evals[0].old_follower.expect("an old follower");
        assert!(gain > 0.0, "gain {gain}");
    }

    #[test]
    fn another_cars_ghost_ahead_in_the_target_lane_is_a_leader() {
        // Weighing up changing left, a ghost just ahead in lane 0 counts: changing
        // would put it right behind a car about to arrive there.
        let mut sim = two_lanes();
        let blocker = sim.insert(car(), at(1, 220.0, 0.0));
        sim.hold(blocker);
        let id = sim.insert(car(), at(1, 200.0, 5.0));
        let other = sim.insert(car(), at(1, 206.0, 0.0));
        sim.indicate(other, Side::Left);
        // With advantage built up, so only safety can hold it back.
        sim.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE; 2]);
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Nothing);
        let left = evals.iter().find(|e| e.side == Side::Left).unwrap();
        assert_eq!(left.outcome, Outcome::Unsafe, "{left:?}");
    }

    #[test]
    fn it_cancels_once_changing_no_longer_helps_it() {
        // Indicating left, but lane 0 now has a stopped car just ahead and lane 1
        // is clear.
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 200.0, 10.0));
        sim.indicate(id, Side::Left);
        let stopped = sim.insert(car(), at(0, 215.0, 0.0));
        sim.hold(stopped);
        sim.time = 2.0;
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Cancel, "{evals:?}");
    }

    #[test]
    fn it_doesnt_decide_partway_through_a_change() {
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 200.0, 10.0));
        sim.change_lane(id, Side::Left).unwrap();
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Nothing);
        assert!(evals.is_empty());
    }

    // Crossing into the next lane section. These step only motion and occupancy,
    // at constant speed, so no decision or longitudinal update interferes.

    /// Steps `id` along at its speed until its rear axle crosses into `lane`.
    fn advance_onto<D: DebugSink>(sim: &mut Simulation<D>, id: VehicleId, lane: u32) {
        for _ in 0..600 {
            if sim.vehicles[id].path[0] == LaneId(lane) {
                return;
            }
            sim.update_motion(1.0 / 60.0);
            sim.update_occupancy();
        }
        panic!("never reached lane {lane}");
    }

    /// Lanes holding a record of `id`, and whether each is its ghost.
    fn records<D: DebugSink>(sim: &Simulation<D>, id: VehicleId) -> Vec<(u32, bool, Lateral)> {
        let lanes = sim.occupants.iter().enumerate();
        lanes
            .flat_map(|(i, list)| {
                let mine = list.iter().filter(move |o| o.id == id);
                mine.map(move |o| (i as u32, o.ghost, o.lateral))
            })
            .collect()
    }

    /// Two sections of two lanes, from 0 to 100 m and 100 to 200 m: lanes 0 and 1,
    /// then 2 and 3, with 0 and 2 on the left.
    fn two_sections() -> SimNetwork {
        road(&[(0.0, 100.0), (100.0, 200.0)], 2)
    }

    #[test]
    fn an_indicating_car_takes_its_ghost_into_the_next_section() {
        let mut sim = Simulation::new(two_sections(), NoopSink);
        let id = sim.insert(car(), at(1, 97.0, 5.0));
        sim.indicate(id, Side::Left);
        assert!(records(&sim, id).contains(&(0, true, Lateral::InLane)));
        advance_onto(&mut sim, id, 3);
        let mine = records(&sim, id);
        assert!(
            mine.contains(&(2, true, Lateral::InLane)),
            "no ghost beside lane 3: {mine:?}"
        );
        assert!(
            !mine.iter().any(|r| r.0 == 0 && r.1),
            "ghost left behind: {mine:?}"
        );
        assert_eq!(sim.vehicles[id].intent.map(|i| i.side), Some(Side::Left));
    }

    #[test]
    fn an_intent_ends_where_the_next_section_has_no_neighbour_there() {
        // Lane 1, on the right, ends at 100 m: lane 0 runs on as lane 2, alone.
        let straight = |x0: f64, x1: f64, y: f64| {
            Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT).unwrap()
        };
        let lanes = vec![
            straight(0.0, 100.0, 0.0),
            straight(0.0, 100.0, -WIDTH),
            straight(100.0, 200.0, 0.0),
        ];
        let mut network = SimNetwork::new(lanes);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network.connect(LaneId(0), LaneId(2));
        network.validate().unwrap();
        let mut sim = Simulation::new(network, NoopSink);
        let id = sim.insert(car(), at(0, 97.0, 5.0));
        sim.indicate(id, Side::Right);
        advance_onto(&mut sim, id, 2);
        assert!(sim.vehicles[id].intent.is_none());
        assert!(
            !records(&sim, id).iter().any(|r| r.1),
            "a ghost is left: {:?}",
            records(&sim, id)
        );
    }

    /// Records each vehicle's leaders, from the last longitudinal update.
    #[derive(Default)]
    struct Leaders(Vec<(VehicleId, VehicleId)>);

    impl DebugSink for Leaders {
        fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
            match event() {
                DebugEvent::RecomputeAccels => self.0.clear(),
                DebugEvent::AccelTerm {
                    vehicle, source: AccelSource::Leader { id, .. }, ..
                } => self.0.push((vehicle, id)),
                _ => {}
            }
        }
    }

    #[test]
    fn a_straddling_car_carries_its_old_side_into_the_next_section() {
        let mut sim = Simulation::new(two_sections(), Leaders::default());
        let id = sim.insert(car(), at(1, 96.0, 5.0));
        sim.change_lane(id, Side::Left).unwrap();
        // Held on the old side's next lane, within reach of the change.
        let held = sim.insert(car(), at(3, 10.0, 0.0));
        sim.hold(held);
        advance_onto(&mut sim, id, 2);
        let mine = records(&sim, id);
        // It joined the old side's next lane, diverging from it.
        assert!(
            mine.contains(&(3, false, Lateral::Diverging(Side::Left))),
            "{mine:?}"
        );
        // Its rear is still on the old lane before, which it is leaving too.
        assert!(
            mine.contains(&(1, false, Lateral::Diverging(Side::Left))),
            "{mine:?}"
        );
        assert_eq!(sim.vehicles[id].old_side, Some(Side::Right));
        // Its old-side search runs on along the next section, to the held car.
        sim.update_accels();
        assert!(sim.sink.0.contains(&(id, held)), "leaders {:?}", sim.sink.0);
    }

    #[test]
    fn at_a_lane_drop_the_old_side_clears_only_when_the_last_old_lane_record_goes() {
        // Lane 1, on the right, ends at 100 m; lane 0 runs on as lane 2.
        let straight = |x0: f64, x1: f64, y: f64| {
            Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT).unwrap()
        };
        let lanes = vec![
            straight(0.0, 100.0, 0.0),
            straight(0.0, 100.0, -WIDTH),
            straight(100.0, 200.0, 0.0),
        ];
        let mut network = SimNetwork::new(lanes);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network.connect(LaneId(0), LaneId(2));
        let mut sim = Simulation::new(network, NoopSink);
        let id = sim.insert(car(), at(1, 97.0, 5.0));
        sim.change_lane(id, Side::Left).unwrap();
        // Through the boundary and on: it straddles while it has any record on a
        // lane it has left, and only then.
        for _ in 0..120 {
            sim.update_motion(1.0 / 60.0);
            sim.update_occupancy();
            let v = &sim.vehicles[id];
            let leaving = records(&sim, id).iter().any(|r| !r.1 && LaneId(r.0) != v.path[0]);
            let on_old_lane = records(&sim, id).iter().any(|r| r.0 == 1);
            assert!(
                !on_old_lane || v.old_side.is_some(),
                "cleared while still on lane 1"
            );
            if !leaving {
                assert_eq!(v.old_side, None, "still straddling with nothing left");
            }
        }
        assert_eq!(sim.vehicles[id].path[0], LaneId(2));
    }

    // Building up advantage.

    /// A car on lane 1 at 300 m behind a car held stopped `gap` metres ahead, not
    /// yet indicating, with an empty lane 0 to change into.
    fn behind_a_stopped_car(gap: f64) -> (Simulation, VehicleId) {
        let mut sim = two_lanes();
        let blocker = sim.insert(car(), at(1, 300.0 + gap, 0.0));
        sim.hold(blocker);
        let id = sim.insert(car(), at(1, 300.0, 5.0));
        (sim, id)
    }

    #[test]
    fn it_doesnt_indicate_until_advantage_has_built_up() {
        let (mut sim, id) = behind_a_stopped_car(20.0);
        let (action, evals) = decide_now(&sim, id);
        assert_eq!(action, Decision::Nothing);
        let left = evals.iter().find(|e| e.side == Side::Left).unwrap();
        assert_eq!(left.outcome, Outcome::NotWorthIt);
        assert!(
            left.advantage > 0.0 && left.advantage < lane_change::INTENT,
            "{left:?}"
        );
        // Once enough has built up, it indicates.
        sim.vehicles[id].lc.set_advantage([lane_change::INTENT, 0.0]);
        assert_eq!(decide_now(&sim, id).0, Decision::Indicate(Side::Left));
    }

    #[test]
    fn a_decision_builds_up_advantage_from_the_net_gain() {
        let (sim, id) = behind_a_stopped_car(20.0);
        let (_, state, evals) = decision(&sim, id);
        let left = evals.iter().find(|e| e.side == Side::Left).unwrap();
        let expected = lane_change::accumulate(0.0, left.incentive, DECISION_PERIOD);
        assert_eq!(state.advantage()[lane_change::index(Side::Left)], expected);
        assert_eq!(left.advantage, expected);
        // No lane to the right: nothing builds up there.
        assert_eq!(state.advantage()[lane_change::index(Side::Right)], 0.0);
    }

    #[test]
    fn it_wont_indicate_into_an_unsafe_gap_however_much_has_built_up() {
        // A fast car close behind in lane 0: changing would cut it off.
        let (mut sim, id) = behind_a_stopped_car(20.0);
        sim.insert(car(), at(0, 290.0, 14.0));
        sim.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        let (action, evals) = decide_now(&sim, id);
        assert_eq!(action, Decision::Nothing);
        assert_eq!(evals[0].outcome, Outcome::Unsafe);
    }

    #[test]
    fn advantage_fades_while_changing_isnt_worth_it() {
        // Nothing ahead: no gain in either lane, so what had built up fades.
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 300.0, 5.0));
        sim.vehicles[id].lc.set_advantage([0.2, 0.0]);
        let (_, state, _) = decision(&sim, id);
        let faded = state.advantage()[lane_change::index(Side::Left)];
        assert!(faded < 0.2 && faded > 0.0, "{faded}");
    }

    #[test]
    fn a_change_starts_its_advantage_afresh() {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        sim.decision_budget = 0.0;
        // Its own decision tick, at the cursor.
        sim.decision_cursor = sim.vehicles.iter().position(|(v, _)| v == id).unwrap();
        sim.update_decisions(1);
        assert_eq!(
            sim.vehicles[id].path[0],
            LaneId(0),
            "it should have changed"
        );
        assert_eq!(sim.vehicles[id].lc.advantage(), [0.0; 2]);
    }

    // Frustration and cooperation.

    /// Behind a stopped car with advantage built up for changing left, and a car
    /// in lane 0 coming up 27 m behind its rear at `speed`, so changing would make
    /// it brake: about 2.1 m/s² at 12 m/s, more than a calm driver accepts.
    fn wanting_to_change_with_a_follower_at(speed: f64) -> (Simulation, VehicleId) {
        let (mut sim, id) = behind_a_stopped_car(20.0);
        // Its front 27 m behind the changer's rear, at 299.
        sim.insert(car(), at(0, 299.0 - 27.0 - 3.6, speed));
        sim.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        (sim, id)
    }

    #[test]
    fn frustration_builds_while_blocked_and_fades_once_not() {
        let (sim, id) = wanting_to_change_with_a_follower_at(12.0);
        let (action, blocked, _) = decision(&sim, id);
        assert_eq!(action, Decision::Nothing);
        assert!(
            (blocked.frustration() - DECISION_PERIOD).abs() < 1e-12,
            "{blocked:?}"
        );
        // Nothing ahead to want to change for: what had built up fades.
        let mut sim2 = two_lanes();
        let free = sim2.insert(car(), at(1, 300.0, 5.0));
        sim2.vehicles[free].lc.set_frustration(4.0);
        let (_, calm, _) = decision(&sim2, free);
        assert!(calm.frustration() < 4.0, "{calm:?}");
    }

    #[test]
    fn a_frustrated_driver_accepts_a_tighter_gap() {
        // Calm, it won't make the follower brake about 2.1 m/s²; frustrated, it will.
        let (mut sim, id) = wanting_to_change_with_a_follower_at(12.0);
        let (_, calm) = decide_now(&sim, id);
        assert!(!calm[0].safe, "{:?}", calm[0]);
        assert_eq!(calm[0].accepted_braking, car().comfy_decel);
        sim.vehicles[id].lc.set_frustration(0.99 * lane_change::FULLY_FRUSTRATED);
        let (action, frustrated) = decide_now(&sim, id);
        assert!(frustrated[0].safe, "{:?}", frustrated[0]);
        assert_eq!(action, Decision::Indicate(Side::Left));
    }

    #[test]
    fn fully_frustrated_it_asks_moving_traffic_to_make_room() {
        // No gap at all: a car close behind in lane 0, fast enough that even full
        // frustration won't accept it.
        let (mut sim, id) = behind_a_stopped_car(20.0);
        sim.insert(car(), at(0, 300.0 - 6.0, 14.0));
        sim.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        sim.vehicles[id].lc.set_frustration(lane_change::FULLY_FRUSTRATED);
        let (action, evals) = decide_now(&sim, id);
        assert!(!evals[0].safe);
        assert_eq!(evals[0].outcome, Outcome::Request);
        assert_eq!(action, Decision::Indicate(Side::Left));
    }

    #[test]
    fn it_doesnt_ask_a_stopped_queue_to_make_room() {
        let (mut sim, id) = behind_a_stopped_car(20.0);
        sim.insert(car(), at(0, 300.0 - 2.0, 0.0));
        sim.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        sim.vehicles[id].lc.set_frustration(lane_change::FULLY_FRUSTRATED);
        let (action, evals) = decide_now(&sim, id);
        assert_eq!(evals[0].outcome, Outcome::Unsafe);
        assert_eq!(action, Decision::Nothing);
    }

    #[test]
    fn its_own_politeness_weighs_its_followers() {
        // Cutting in front of a car would cost it; an impolite driver doesn't care.
        let (mut sim, id) = wanting_to_change_with_a_follower_at(12.0);
        let follower = sim.vehicles.iter().find(|(_, v)| v.path[0] == LaneId(0)).unwrap().0;
        sim.vehicles[follower].accel = 0.5;
        let incentive = |sim: &Simulation| decide_now(sim, id).1[0].incentive;
        let polite = incentive(&sim);
        sim.vehicles[id].spec.politeness = 0.0;
        assert!(
            incentive(&sim) > polite,
            "impolite {} vs polite {polite}",
            incentive(&sim)
        );
    }

    /// A car indicating left from lane 1 at 300 m, at 10 m/s, and one in lane 0
    /// `behind` metres back, at 10 m/s, with the given courtesy.
    fn behind_an_indicator(behind: f64, courtesy: f64) -> (Simulation, VehicleId, VehicleId) {
        let mut sim = two_lanes();
        let indicating = sim.insert(car(), at(1, 300.0, 10.0));
        sim.indicate(indicating, Side::Left);
        let spec = VehicleSpec { courtesy, ..car() };
        let follower = sim.insert(spec, at(0, 300.0 - behind, 10.0));
        (sim, indicating, follower)
    }

    #[test]
    fn a_courteous_follower_makes_room_for_an_indicating_car() {
        let (mut sim, indicating, follower) = behind_an_indicator(15.0, 2.0);
        sim.update_accels();
        assert_eq!(sim.vehicles[follower].lc.yielding(), Some(indicating));
        assert!(sim.vehicles[follower].accel < 0.0, "it should ease off");
    }

    #[test]
    fn a_follower_drives_on_if_making_room_would_take_more_than_its_courtesy() {
        // Making room would take braking harder than 0.2 m/s².
        let (mut sim, _, follower) = behind_an_indicator(15.0, 0.2);
        sim.update_accels();
        assert_eq!(sim.vehicles[follower].lc.yielding(), None);
        // One with no courtesy never does, however easy.
        let (mut sim, _, follower) = behind_an_indicator(80.0, 0.0);
        sim.update_accels();
        assert_eq!(sim.vehicles[follower].lc.yielding(), None);
    }

    #[test]
    fn a_follower_keeps_making_room_once_it_has_chosen_to() {
        // Having chosen to, it carries on even where it wouldn't start: here its
        // courtesy alone wouldn't stretch to it.
        let (mut sim, indicating, follower) = behind_an_indicator(15.0, 0.2);
        sim.vehicles[follower].lc.set_yielding(Some(indicating));
        sim.update_accels();
        assert_eq!(sim.vehicles[follower].lc.yielding(), Some(indicating));
        // Once the indicator is gone, there is nothing to make room for.
        sim.cancel_intent(indicating);
        sim.update_accels();
        assert_eq!(sim.vehicles[follower].lc.yielding(), None);
    }

    // Several leaders.

    /// Every leader `id` reacts to now, nearest first.
    fn leaders(sim: &Simulation, id: VehicleId) -> Vec<VehicleId> {
        let v = &sim.vehicles[id];
        let search = occupancy::find_leader(
            &sim.network,
            &sim.occupants,
            &sim.vehicles,
            id,
            v,
            &v.path,
            v.lane_pos.s,
            None,
            occupancy::horizon(&sim.network, v, v.path[0]),
            Course::of(v),
            Ghosts::Pass,
            None,
            |_| {},
        );
        search.leaders.iter().map(|&(leader, _)| leader).collect()
    }

    #[test]
    fn a_car_sees_a_diverging_car_its_leader_has_passed() {
        // C, changing left out of lane 1, held with its right rear corner 1.6 m
        // left of lane 1's centerline. B, in lane 1 behind it, clears that by
        // 0.65 m and passes; A, wider, behind B, wouldn't clear it by enough.
        let mut sim = two_lanes();
        let c = sim.insert(car(), at(1, 60.0, 0.0));
        sim.change_lane(c, Side::Left).unwrap();
        let lane_1 = sim.network.lane(LaneId(1));
        let pos = lane_1.lane_to_world(lane_1.lane_pos(60.0, 2.55));
        let lane_0 = sim.network.lane(LaneId(0));
        let on_0 = lane_0.world_to_lane_near(pos, lane_0.lane_pos(60.0, 0.0));
        let v = &mut sim.vehicles[c];
        (v.pos, v.heading, v.lane_pos) = (pos, 0.0, on_0);
        sim.update_occupancy();
        let record = sim.occupants[1].iter().find(|o| o.id == c).unwrap();
        assert_eq!(record.lateral, Lateral::Diverging(Side::Left));
        let b = sim.insert(car(), at(1, 45.0, 10.0));
        let wide = VehicleSpec { width: 2.6, ..car() };
        let a = sim.insert(wide, at(1, 30.0, 10.0));
        assert_eq!(leaders(&sim, b), [], "B passes C");
        // A reacts to B, and to C past it, rather than finding C only once B is by.
        assert_eq!(leaders(&sim, a), [b, c]);
    }

    #[test]
    fn a_leader_turning_off_doesnt_hide_the_queue_beyond() {
        // Lane 0 splits: lane 2 bears away, lane 1 runs straight on. B turns onto
        // lane 2; A goes straight on, where a car is held stopped.
        let lane = |points: &[DVec2]| Lane::new(points, WIDTH, LIMIT).unwrap();
        let lanes = vec![
            lane(&[DVec2::ZERO, DVec2::new(100.0, 0.0)]),
            lane(&[DVec2::new(100.0, 0.0), DVec2::new(200.0, 0.0)]),
            lane(&[
                DVec2::new(100.0, 0.0),
                DVec2::new(150.0, 5.0),
                DVec2::new(200.0, 15.0),
            ]),
        ];
        let mut network = SimNetwork::new(lanes);
        network.connect(LaneId(0), LaneId(2));
        network.connect(LaneId(0), LaneId(1));
        let mut sim = Simulation::new(network, NoopSink);
        let queued = sim.insert(car(), at(1, 20.0, 0.0));
        sim.hold(queued);
        let b = sim.insert(car(), at(0, 80.0, 10.0));
        assert_eq!(sim.vehicles[b].path[1], LaneId(2));
        let a = sim.insert(car(), at(0, 50.0, 10.0));
        let path = &mut sim.vehicles[a].path;
        path.clear();
        path.extend([LaneId(0), LaneId(1)]);
        assert_eq!(leaders(&sim, a), [b, queued]);
    }

    #[test]
    fn the_search_stops_at_the_second_car_in_the_lane() {
        // What constrains B, the first car ahead, reaches A through B: A needn't
        // look past the next car in the lane, D, at all.
        let mut sim = two_lanes();
        let a = sim.insert(car(), at(0, 20.0, 10.0));
        let b = sim.insert(car(), at(0, 40.0, 10.0));
        let _d = sim.insert(car(), at(0, 60.0, 10.0));
        let stopped = sim.insert(car(), at(0, 120.0, 0.0));
        sim.hold(stopped);
        assert_eq!(leaders(&sim, a), [b]);
    }

    // The end of the known path.

    /// Records every acceleration term, from the last longitudinal update.
    #[derive(Default)]
    struct Terms(Vec<(VehicleId, AccelSource, f64)>);

    impl DebugSink for Terms {
        fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
            match event() {
                DebugEvent::RecomputeAccels => self.0.clear(),
                DebugEvent::AccelTerm { vehicle, source, accel } => {
                    self.0.push((vehicle, source, accel))
                }
                _ => {}
            }
        }
    }

    /// One lane in `sections` sections of 20 m each, and a car at 5 m on the first
    /// at 10 m/s, after a longitudinal update: the gap to the end of its path, if it
    /// reacted to one, and that term.
    fn path_end_term(sections: usize) -> Option<(f64, f64)> {
        let sections: Vec<_> =
            (0..sections).map(|k| (20.0 * k as f64, 20.0 * (k + 1) as f64)).collect();
        let mut sim = Simulation::new(road(&sections, 1), Terms::default());
        let id = sim.insert(car(), at(0, 5.0, 10.0));
        sim.update_accels();
        assert_eq!(sim.vehicles[id].path.len(), PATH_LEN);
        sim.sink.0.iter().find_map(|&(_, source, accel)| match source {
            AccelSource::PathEnd { gap } => Some((gap, accel)),
            _ => None,
        })
    }

    #[test]
    fn the_end_of_a_known_path_is_a_stop_line() {
        // Its path holds the first four sections, and the road goes on beyond: it
        // doesn't know where, so it is ready to stop at 80 m, its front at 8.6 m.
        let (gap, accel) = path_end_term(6).expect("no stop line at the path's end");
        assert!((gap - 71.4).abs() < 1e-9, "gap {gap}");
        assert_eq!(accel, acceleration::interaction(&car(), 10.0, gap, 0.0));
    }

    #[test]
    fn a_path_ending_where_the_network_does_is_an_exit() {
        // The same, but the road ends with the path: it leaves there.
        assert_eq!(path_end_term(4), None);
    }
}
