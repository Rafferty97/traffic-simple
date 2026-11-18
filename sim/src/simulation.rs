use std::cmp::Ordering;
use std::ops::ControlFlow;

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
use crate::models::routing::Route;
use crate::network::{Conflict, ConflictKind, JunctionId, LaneId, Side, SimNetwork};
use crate::occupancy::{
    self, Ahead, Along, Found, Lateral, Occupant, find_follower, find_leaders, for_each_leader,
};
use crate::vehicle::{Intent, OldFollower, Placement, Vehicle, VehicleId, VehicleSpec};

/// Longest substep in seconds. Longer frames are split.
const MAX_SUBSTEP: f64 = 1.0 / 60.0;
/// Period of the longitudinal model update in seconds.
const ACCEL_PERIOD: f64 = 1.0 / 20.0;

/// How often each vehicle makes its lane-change decision, in seconds. Decisions
/// are staggered across vehicles.
const DECISION_PERIOD: f64 = 0.2;

// The acceleration clock fires at most once per substep.
const _: () = assert!(MAX_SUBSTEP <= ACCEL_PERIOD);

/// The state the searches read: the network, the vehicles and where they are.
/// Kept apart from the rest of the simulation, so that an update can lend it to
/// the searches while writing elsewhere, such as to the debug sink.
pub(crate) struct SimState {
    pub network: SimNetwork,
    pub vehicles: SlotMap<VehicleId, Vehicle>,
    /// Each lane's occupants sorted by `s`, indexed by lane.
    pub occupants: Vec<Vec<Occupant>>,
    /// At least as far as any vehicle's path has reached beyond its front, in
    /// meters: a running maximum, raised wherever a path is extended. It bounds
    /// how far back a follower search goes.
    pub max_reach: f64,
    /// Each junction's queue, by junction.
    pub queues: Vec<Queue>,
}

/// A junction's queue: the vehicles committed there, which can no longer stop
/// short of their conflicts, in the order they committed, and the lanes that
/// vehicles not yet committed may be approaching it along.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Queue {
    /// Each holds back every vehicle committed after it.
    pub committed: Vec<VehicleId>,
    /// Every lane on the way of a vehicle that has arrived, from the lane it
    /// was on to the first of the junction's lanes with a conflict on its path,
    /// that lane included, sorted and without repeats. The junction's scans
    /// for vehicles to commit start from its own lanes here and follow only
    /// lanes here (see [`commit_along`]). Vehicles only add lanes; the scans
    /// drop those they find no one on.
    pub approaching: Vec<LaneId>,
    /// The vehicles not yet committed that the last scans stopped at, each
    /// headed onto the lane scanned, so the first on its way: the committed
    /// vehicles hold these back too (see [`junction_terms`]). Those behind them
    /// follow them. Refilled every substep.
    pub fronts: Vec<VehicleId>,
}

/// The traffic simulation.
pub struct Simulation<D: DebugSink = NoopSink> {
    state: SimState,
    /// Vehicles with a speed imposed on them, and the most each may go, in m/s: 0
    /// holds one to a stop. Few, so a list beats a field on every vehicle.
    caps: Vec<(VehicleId, f64)>,
    /// Time since the last longitudinal model update, in seconds.
    accel_clock: f64,
    /// Scratch space for the longitudinal update's decisions, reused between
    /// updates.
    decisions: Vec<AccelUpdate>,
    /// Scratch space for the longitudinal update: each vehicle held up by a
    /// leader, by that leader, before the leader keeps the one it holds up most.
    held: Vec<(VehicleId, OldFollower)>,
    /// Scratch space for vehicles leaving the network, reused between substeps.
    leavers: Vec<VehicleId>,
    /// Scratch space for vehicles whose path grew or that entered a junction in
    /// a substep, which may arrive at junctions; reused between substeps.
    arrivals: Vec<VehicleId>,
    /// Scratch space for a junction's scans: the lanes they found no one on, or
    /// beyond, and those they found someone on, or beyond. Reused.
    scan_empty: Vec<LaneId>,
    scan_kept: Vec<LaneId>,
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

/// What the longitudinal update decided for a vehicle, from one of its records.
#[derive(Clone, Copy, Debug)]
enum AccelUpdate {
    /// From its record on the lane it follows: its acceleration, whose ghost it is
    /// making room for, and the leader whose term is the lowest there, if any.
    Own {
        id: VehicleId,
        accel: f64,
        yielding: Option<VehicleId>,
        binding: Option<Binding>,
    },
    /// From its record on its old lane, while it straddles it: the most its old
    /// side allows.
    OldSide { id: VehicleId, accel: f64 },
    /// From a junction whose queue it is in: the most the committed vehicles
    /// ahead of it there allow.
    Junction { id: VehicleId, accel: f64 },
}

/// The leader whose term was the lowest in a vehicle's search along its path.
#[derive(Clone, Copy, Debug)]
struct Binding {
    leader: VehicleId,
    /// The bumper-to-bumper gap to it.
    gap: f64,
    /// Its term.
    term: f64,
    /// The lowest of every other term from that search: the free road, other
    /// leaders, the end of the path and the ghost it makes room for.
    rest: f64,
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
    queues: Vec<Queue>,
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
            network: self.state.network.clone(),
            vehicles: self.state.vehicles.clone(),
            occupants: self.state.occupants.clone(),
            queues: self.state.queues.clone(),
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
            queues,
            caps,
            accel_clock,
            time,
            decision_cursor,
            decision_budget,
        } = snapshot;
        // Derived from the paths, so not part of the snapshot.
        let max_reach =
            vehicles.values().map(|v| reach_beyond_front(&network, v)).fold(0.0, f64::max);
        Self {
            state: SimState { network, vehicles, occupants, max_reach, queues },
            caps,
            accel_clock,
            time,
            decision_cursor,
            decision_budget,
            // Scratch space, cleared before every use.
            decisions: Vec::new(),
            held: Vec::new(),
            leavers: Vec::new(),
            arrivals: Vec::new(),
            scan_empty: Vec::new(),
            scan_kept: Vec::new(),
            departed: Vec::new(),
            lane_decisions: Vec::new(),
            sink,
        }
    }

    pub fn new(network: SimNetwork, sink: D) -> Self {
        let lane_count = network.lanes().count();
        let junction_count = network.junctions().count();
        Self {
            state: SimState {
                network,
                vehicles: SlotMap::with_key(),
                occupants: vec![Vec::new(); lane_count],
                max_reach: 0.0,
                queues: vec![Queue::default(); junction_count],
            },
            caps: Vec::new(),
            accel_clock: 0.0,
            decisions: Vec::new(),
            held: Vec::new(),
            leavers: Vec::new(),
            arrivals: Vec::new(),
            scan_empty: Vec::new(),
            scan_kept: Vec::new(),
            departed: Vec::new(),
            time: 0.0,
            decision_cursor: 0,
            decision_budget: 0.0,
            lane_decisions: Vec::new(),
            sink,
        }
    }

    pub fn network(&self) -> &SimNetwork {
        &self.state.network
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
        // Leader and follower split the lane at its rear, so any vehicle alongside
        // is one or the other, with no gap, and refuses the insertion.
        let rear = vehicle.lane_pos.s - spec.rear_axle;
        // No faster than the nearest leader, and braking gently enough for all,
        // ghosts and the stop line at the end of its path included. The search
        // goes nearest first, so the speed is capped before anything is checked,
        // and it stops at the first that refuses.
        let mut speed = vehicle.speed;
        let mut nearest = true;
        find_leaders(&self.state, &vehicle, Along::Inserting, |found| {
            let Found::Ahead(ahead, gap) = found else {
                return ControlFlow::Continue(());
            };
            let leader_speed = speed_of(&self.state.vehicles, ahead);
            if nearest && ahead != Ahead::PathEnd {
                speed = speed.min(leader_speed);
            }
            nearest = false;
            let accel = acceleration::interaction(&spec, speed, gap, leader_speed);
            match gap <= 0.0 || accel < -acceleration::SAFE_BRAKING {
                true => ControlFlow::Break(InsertError::NoRoom),
                false => ControlFlow::Continue(()),
            }
        })
        .continue_ok()?;
        vehicle.speed = speed;

        // A key that matches no vehicle, since this one isn't inserted yet.
        let follower = find_follower(&self.state, at.lane, rear, VehicleId::null());
        if let Some((follower, gap)) = follower {
            let f = &self.state.vehicles[follower];
            let accel = acceleration::interaction(&f.spec, f.speed, gap, vehicle.speed);
            if gap <= 0.0 || accel < -acceleration::SAFE_BRAKING {
                return Err(InsertError::NoRoom);
            }
        }

        Ok(self.register(vehicle))
    }

    /// A vehicle placed on a lane's centerline, facing along it, not yet in the simulation.
    fn build_vehicle(&self, spec: VehicleSpec, at: Placement) -> Vehicle {
        let lane = self.state.network.lane(at.lane);
        let lane_pos = lane.geometry().lane_pos(at.s, 0.0);
        let mut vehicle = Vehicle {
            spec,
            pos: lane.geometry().lane_to_world(lane_pos),
            heading: wrap_angle(lane.geometry().tangent_at(lane_pos).to_angle()),
            speed: at.speed,
            accel: 0.0,
            curvature: 0.0,
            path: vec![at.lane],
            lane_pos,
            route: at.route,
            old_side: None,
            lane_change: None,
            intent: None,
            lc: lane_change::State::default(),
            old_follower: None,
        };
        let needed = reach(&vehicle, at.s, None);
        extend_path(
            &self.state.network,
            &mut vehicle.path,
            &mut vehicle.route,
            at.s,
            needed,
        );
        vehicle
    }

    /// Adds a built vehicle to the simulation and to its lane's occupants.
    fn register(&mut self, vehicle: Vehicle) -> VehicleId {
        self.state.max_reach =
            self.state.max_reach.max(reach_beyond_front(&self.state.network, &vehicle));
        let lane_id = vehicle.path[0];
        let lane_pos = vehicle.lane_pos;
        let id = self.state.vehicles.insert(vehicle);
        let occupant = Occupant::new(
            &self.state.network,
            lane_id,
            id,
            &self.state.vehicles[id],
            lane_pos,
            Lateral::InLane,
        );
        let list = &mut self.state.occupants[lane_id.0 as usize];
        let index = list.partition_point(|o| o.s <= occupant.s);
        list.insert(index, occupant);
        let SimState { network, vehicles, occupants, queues, .. } = &mut self.state;
        arrive(network, occupants, queues, id, &vehicles[id]);
        id
    }

    /// Removes a vehicle. Scans every lane's occupants, so it is not for hot paths.
    pub fn remove(&mut self, id: VehicleId) -> Option<Vehicle> {
        for list in &mut self.state.occupants {
            list.retain(|o| o.id != id);
        }
        for queue in &mut self.state.queues {
            queue.committed.retain(|&q| q != id);
            queue.fronts.retain(|&q| q != id);
        }
        self.caps.retain(|&(c, _)| c != id);
        self.sink.record(|| DebugEvent::VehicleRemoved { vehicle: id });
        self.state.vehicles.remove(id)
    }

    pub fn get(&self, id: VehicleId) -> Option<&Vehicle> {
        self.state.vehicles.get(id)
    }

    pub fn vehicles(&self) -> impl Iterator<Item = (VehicleId, &Vehicle)> {
        self.state.vehicles.iter()
    }

    /// A lane's occupants, sorted by `s`.
    pub fn occupants(&self, lane: LaneId) -> &[Occupant] {
        &self.state.occupants[lane.0 as usize]
    }

    /// The vehicles committed at a junction, in the order they committed.
    pub fn queue(&self, junction: JunctionId) -> &[VehicleId] {
        &self.state.queues[junction.0 as usize].committed
    }

    /// The lanes of a junction that vehicles not yet committed may be
    /// approaching, in the order they were first approached.
    pub fn approaching(&self, junction: JunctionId) -> &[LaneId] {
        &self.state.queues[junction.0 as usize].approaching
    }

    /// The vehicle whose body is nearest to `point`, if any is within `tolerance`.
    ///
    /// If the point is inside several bodies, the one whose center is nearest wins.
    pub fn vehicle_at(&self, point: DVec2, tolerance: f64) -> Option<VehicleId> {
        self.state
            .vehicles
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
        if !self.state.vehicles.contains_key(id) {
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
        let indicating = self.state.vehicles.get(id).and_then(|v| v.intent).map(|i| i.side);
        if indicating.is_some_and(|s| s != side) {
            self.cancel_intent(id);
        }
        let vehicle = self.state.vehicles.get_mut(id).ok_or(LaneChangeError::NoVehicle)?;
        if vehicle.lane_change.is_some() {
            return Err(LaneChangeError::InProgress);
        }
        let neighbor = self
            .state
            .network
            .lane(vehicle.path[0])
            .adj(side)
            .ok_or(LaneChangeError::NoNeighbor)?;
        if !neighbor.can_change {
            return Err(LaneChangeError::NotPermitted);
        }

        let (lane_pos, change) = plan_change(&self.state.network, vehicle, neighbor.lane);
        let old_lane = vehicle.path[0];
        // It keeps its place in the queues it is in: those of junctions no longer
        // on its path drop it once it is clear of them, and its new path may make
        // it arrive at others.
        vehicle.path.clear();
        vehicle.path.push(neighbor.lane);
        vehicle.lane_pos = lane_pos;
        vehicle.old_side = Some(side.opposite());
        vehicle.lane_change = Some(change);
        vehicle.intent = None;
        let needed = reach(vehicle, lane_pos.s, Some(change));
        extend_path(
            &self.state.network,
            &mut vehicle.path,
            &mut vehicle.route,
            lane_pos.s,
            needed,
        );
        let beyond_front = reach_beyond_front(&self.state.network, vehicle);
        self.state.max_reach = self.state.max_reach.max(beyond_front);

        // Its record on the lane it is leaving is now diverging. Set here rather
        // than left to the next occupancy update, so searches before then see it.
        let old = &mut self.state.occupants[old_lane.0 as usize];
        if let Some(record) = old.iter_mut().find(|o| o.id == id && !o.ghost) {
            record.lateral = Lateral::Diverging(side);
        }

        let list = &mut self.state.occupants[neighbor.lane.0 as usize];
        if let Some(ghost) = list.iter_mut().find(|o| o.id == id && o.ghost) {
            ghost.ghost = false;
        } else {
            let occupant = Occupant::new(
                &self.state.network,
                neighbor.lane,
                id,
                &self.state.vehicles[id],
                lane_pos,
                Lateral::InLane,
            );
            let index = list.partition_point(|o| o.s <= occupant.s);
            list.insert(index, occupant);
        }
        let SimState { network, vehicles, occupants, queues, .. } = &mut self.state;
        arrive(network, occupants, queues, id, &vehicles[id]);
        Ok(())
    }

    /// Starts a vehicle indicating a change to `side`: it gets a ghost record in
    /// the target lane's occupants.
    fn indicate(&mut self, id: VehicleId, side: Side) {
        let vehicle = &self.state.vehicles[id];
        let current = self.state.network.lane(vehicle.path[0]);
        let Some(neighbor) = current.adj(side) else {
            return;
        };
        let target = self.state.network.lane(neighbor.lane);
        let hint = target.geometry().lane_pos(
            vehicle.lane_pos.s / current.geometry().length() * target.geometry().length(),
            0.0,
        );
        let ghost = Occupant::ghost(&self.state.network, neighbor.lane, id, vehicle, hint);
        let list = &mut self.state.occupants[neighbor.lane.0 as usize];
        let index = list.partition_point(|o| o.s <= ghost.s);
        list.insert(index, ghost);
        self.state.vehicles[id].intent = Some(Intent { side, since: self.time });
    }

    /// Stops a vehicle indicating, removing its ghost.
    fn cancel_intent(&mut self, id: VehicleId) {
        let Some(vehicle) = self.state.vehicles.get_mut(id) else {
            return;
        };
        let Some(intent) = vehicle.intent.take() else {
            return;
        };
        if let Some(neighbor) = self.state.network.lane(vehicle.path[0]).adj(intent.side) {
            remove_ghost(&mut self.state.occupants, neighbor.lane, id);
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
        self.decision_budget += self.state.vehicles.len() as f64 * dt / DECISION_PERIOD;
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
        self.update_queues();
    }

    /// Lane-change decisions for the next `count` vehicles, resuming where the last
    /// substep left off and wrapping around. Decisions are computed from a
    /// read-only view of the state, then applied in order.
    fn update_decisions(&mut self, count: usize) {
        let Self { state, lane_decisions, caps, sink, .. } = self;
        let vehicles = &state.vehicles;
        let n = vehicles.len();
        if n == 0 {
            return;
        }
        let count = count.min(n);
        let start = self.decision_cursor % n;
        lane_decisions.clear();
        for (id, vehicle) in vehicles.iter().chain(vehicles.iter()).skip(start).take(count) {
            sink.record(|| DebugEvent::RecomputeDecision { vehicle: id });
            let mut lc = vehicle.lc;
            let Some((decision, evals)) = decide(state, caps, self.time, id, vehicle, &mut lc)
            else {
                continue;
            };
            for eval in evals.into_iter().flatten() {
                sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
            }
            lane_decisions.push((id, lc, decision));
        }
        self.decision_cursor = (start + count) % n;

        for i in 0..self.lane_decisions.len() {
            let (id, state, decision) = self.lane_decisions[i];
            self.state.vehicles[id].lc = state;
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
        let Self { state, decisions, sink, .. } = self;
        let SimState { network, vehicles, occupants, .. } = &*state;
        sink.record(|| DebugEvent::RecomputeAccels);
        decisions.clear();
        for (index, list) in occupants.iter().enumerate() {
            let lane = LaneId(index as u32);
            let half_length = 0.5 * network.lane(lane).geometry().length();
            for (i, o) in list.iter().enumerate().rev() {
                if o.ghost {
                    continue;
                }
                let vehicle = &vehicles[o.id];
                if vehicle.path[0] == lane {
                    // On a loop, a vehicle can briefly have a second record on the
                    // lane it follows, lingering at its end, about a lane's length
                    // from its rear.
                    let rear = vehicle.lane_pos.s - vehicle.spec.rear_axle;
                    if (o.s - rear).abs() > half_length {
                        continue;
                    }
                    decisions.push(longitudinal(state, sink, o.id, vehicle, i));
                } else if let Some(side) = straddling(network, vehicle, lane) {
                    let accel = old_side(state, sink, o.id, vehicle, side, i);
                    decisions.push(AccelUpdate::OldSide { id: o.id, accel });
                }
            }
        }
        debug_assert_eq!(
            decisions.iter().filter(|d| matches!(d, AccelUpdate::Own { .. })).count(),
            vehicles.len(),
            "each vehicle has one record on the lane it follows"
        );
        for (j, queue) in state.queues.iter().enumerate() {
            junction_terms(state, sink, JunctionId(j as u32), queue, decisions);
        }

        // What its old side and its speed cap allow first, then its own update,
        // so that whether a leader binds it is judged against all of them.
        for update in &self.decisions {
            if let AccelUpdate::Own { id, yielding, .. } = *update {
                let vehicle = &mut self.state.vehicles[id];
                vehicle.accel = f64::INFINITY;
                vehicle.lc.set_yielding(yielding);
                vehicle.old_follower = None;
            }
        }
        for update in &self.decisions {
            if let AccelUpdate::OldSide { id, accel } | AccelUpdate::Junction { id, accel } =
                *update
            {
                let vehicle = &mut self.state.vehicles[id];
                vehicle.accel = vehicle.accel.min(accel);
            }
        }
        for &(id, max_speed) in &self.caps {
            let vehicle = &mut self.state.vehicles[id];
            let term = acceleration::capped(&vehicle.spec, vehicle.speed, max_speed);
            let source = AccelSource::Cap { max_speed };
            self.sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
            vehicle.accel = vehicle.accel.min(term);
        }
        self.held.clear();
        for update in &self.decisions {
            if let AccelUpdate::Own { id, accel, binding, .. } = *update {
                let vehicle = &mut self.state.vehicles[id];
                let others = vehicle.accel;
                vehicle.accel = accel.min(others);
                // Held up by that leader: without it, every other term would be left.
                if let Some(b) = binding {
                    let without = b.rest.min(others);
                    if b.term <= without {
                        let follower = OldFollower { id, gap: b.gap, without };
                        self.held.push((b.leader, follower));
                    }
                }
            }
        }
        // Each leader keeps the one it holds up most: the most it would gain.
        let vehicles = &mut self.state.vehicles;
        for &(leader, follower) in &self.held {
            let gain = |f: &OldFollower| f.without - vehicles[f.id].accel;
            let better =
                vehicles[leader].old_follower.is_none_or(|kept| gain(&follower) > gain(&kept));
            if better {
                vehicles[leader].old_follower = Some(follower);
            }
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
        let Self {
            state: SimState { network, vehicles, occupants, max_reach, .. },
            leavers,
            arrivals,
            sink,
            ..
        } = self;
        sink.record(|| DebugEvent::RecomputeMotion);
        for (id, vehicle) in vehicles.iter_mut() {
            let wheelbase = vehicle.spec.wheelbase();

            let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
            let offset = vehicle.lane_change.map_or(0.0, |change| change.offset_at(ahead));
            let target = network.point_along(vehicle.path.iter().copied(), ahead, offset);
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
            vehicle.lane_pos = lane.geometry().world_to_lane_near(vehicle.pos, vehicle.lane_pos);
            // Its path growing, or its entering a junction, may make it arrive at one.
            let mut arrives = false;
            while vehicle.path.len() > 1 {
                let length = network.lane(vehicle.path[0]).geometry().length();
                if vehicle.lane_pos.s < length {
                    break;
                }
                let previous = vehicle.path.remove(0);
                let next = network.lane(vehicle.path[0]);
                arrives |= next.junction().is_some();
                let hint = next.geometry().lane_pos(vehicle.lane_pos.s - length, 0.0);
                vehicle.lane_pos = next.geometry().world_to_lane_near(vehicle.pos, hint);
                if let Some(change) = &mut vehicle.lane_change {
                    change.s0 -= length;
                }
                // Lanes can be shorter than a substep's travel, so it may advance
                // again: it needs to know where it goes beyond this one.
                let s = vehicle.lane_pos.s;
                let needed = reach(vehicle, s, vehicle.lane_change);
                let lanes = vehicle.path.len();
                extend_path(network, &mut vehicle.path, &mut vehicle.route, s, needed);
                arrives |= vehicle.path.len() > lanes;
                // It joins every lane the ones it is leaving lead to, as it would
                // the lane it leaves by changing lanes: it overlaps them where they
                // part, and each record is dropped once it is clear of that lane.
                // Joins unsorted; `update_occupancy` sets its `s` and sorts.
                let followed = vehicle.path[0];
                let s = vehicle.lane_pos.s;
                let mut join = |lane: LaneId, lateral: Lateral| {
                    let hint = match lane == followed {
                        true => vehicle.lane_pos,
                        false => network.lane(lane).geometry().lane_pos(s, 0.0),
                    };
                    let occupant = Occupant::new(network, lane, id, vehicle, hint, lateral);
                    occupants[lane.0 as usize].push(occupant);
                };
                join(followed, Lateral::InLane);
                // Successors are ordered left to right, so the others lie to the
                // left of the one it follows, or to the right, and it is moving
                // out of them toward it.
                let successors = network.lane(previous).successors();
                if let Some(taken) = successors.iter().position(|&lane| lane == followed) {
                    for (i, &lane) in successors.iter().enumerate() {
                        match i.cmp(&taken) {
                            Ordering::Less => join(lane, Lateral::Diverging(Side::Right)),
                            Ordering::Equal => {}
                            Ordering::Greater => join(lane, Lateral::Diverging(Side::Left)),
                        }
                    }
                }
                // Still straddling after a lane change: also join the lanes its old
                // lane leads to, all on its old side, so followers there keep seeing
                // it, except those the lane it is leaving leads to as well.
                let old = vehicle.old_side.and_then(|side| network.lane(previous).adj(side));
                if let (Some(side), Some(old)) = (vehicle.old_side, old) {
                    for &lane in network.lane(old.lane).successors() {
                        if !successors.contains(&lane) {
                            join(lane, Lateral::Diverging(side.opposite()));
                        }
                    }
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
                            let hint = target.geometry().lane_pos(vehicle.lane_pos.s, 0.0);
                            occupants[n.lane.0 as usize]
                                .push(Occupant::ghost(network, n.lane, id, vehicle, hint));
                        }
                        None => vehicle.intent = None,
                    }
                }
            }

            // Far enough ahead to know where it is going, at its new speed.
            let s = vehicle.lane_pos.s;
            let needed = reach(vehicle, s, vehicle.lane_change);
            let lanes = vehicle.path.len();
            let covered = extend_path(network, &mut vehicle.path, &mut vehicle.route, s, needed);
            arrives |= vehicle.path.len() > lanes;
            if arrives {
                arrivals.push(id);
            }
            let nose = vehicle.spec.length - vehicle.spec.rear_axle;
            *max_reach = max_reach.max(covered - nose);

            // Past the end with nowhere to advance to: it has left the network.
            let lane = network.lane(vehicle.path[0]);
            if vehicle.lane_pos.s >= lane.geometry().length() && lane.successors().is_empty() {
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

    /// Brings the junctions' queues up to date after a substep's motion and
    /// occupancy: vehicles whose path grew or that entered a junction arrive
    /// where nothing holds them back (see [`arrive`]), committed vehicles clear
    /// of a junction leave its queue, those that can plainly stop are released
    /// (see [`may_release`]), and each lane approached is scanned for vehicles
    /// to commit (see [`commit_along`]).
    fn update_queues(&mut self) {
        let Self {
            state: SimState { network, vehicles, occupants, queues, .. },
            arrivals,
            scan_empty,
            scan_kept,
            ..
        } = self;
        // Some may have left the network since.
        for &id in arrivals.iter() {
            if let Some(vehicle) = vehicles.get(id) {
                arrive(network, occupants, queues, id, vehicle);
            }
        }
        arrivals.clear();
        for (j, queue) in queues.iter_mut().enumerate() {
            let junction = JunctionId(j as u32);
            // Headed into it, or still on it.
            queue.committed.retain(|&v| {
                headed(network, &vehicles[v], junction) || entered(network, occupants, junction, v)
            });
            // From the back, so a lane's vehicles are released back to front in
            // one pass. A released vehicle's way in is marked again, as the
            // scans passed it over while it was committed.
            for k in (0..queue.committed.len()).rev() {
                if may_release(network, vehicles, occupants, junction, &queue.committed, k) {
                    let id = queue.committed.remove(k);
                    mark_way(
                        network,
                        &mut queue.approaching,
                        &vehicles[id].path,
                        junction,
                    );
                }
            }
            queue.fronts.clear();
            if queue.approaching.is_empty() {
                continue;
            }
            let Queue { committed, approaching, fronts } = queue;
            let mut scan = Scan {
                network,
                vehicles,
                occupants,
                junction,
                approaching,
                empty: scan_empty,
                kept: scan_kept,
                fronts,
            };
            scan.empty.clear();
            scan.kept.clear();
            for &lane in scan.approaching {
                if network.lane(lane).junction() == Some(junction) {
                    commit_along(&mut scan, lane, committed);
                }
            }
            // A lane stays while some scan found someone on it or beyond, or none
            // reached it, so whoever is on it is behind someone found.
            approaching.retain(|lane| !scan_empty.contains(lane) || scan_kept.contains(lane));
        }
    }

    /// Refreshes the occupant lists, and clears the old side of vehicles that have
    /// finished leaving it.
    fn update_occupancy(&mut self) {
        let Self {
            state: SimState { network, vehicles, occupants, .. },
            departed,
            ..
        } = self;
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
/// making room for, if anyone's. `own` is the index of its record in the occupants of the lane it
/// follows.
///
/// The acceleration is the minimum over every term the model considers: the free
/// road, each leader along its path, the end of its path, and the ghost it makes
/// room for. While it straddles, its old side adds more (see [`old_side`]). Each
/// term is recorded to the debug sink as it is computed.
fn longitudinal<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    own: usize,
) -> AccelUpdate {
    let lane = vehicle.path[0];
    let desired_speed = vehicle.spec.desired_speed(state.network.lane(lane).speed_limit());
    let free = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed);
    let source = AccelSource::FreeRoad { desired_speed };
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: free });
    // The lowest of every term but the binding leader's, and that leader.
    let mut rest = free;
    let mut binding: Option<Binding> = None;

    // The nearest ghost, if it comes before anything else ahead.
    let mut ghost = None;
    let mut first = true;
    for_each_leader(state, vehicle, Along::Path(own), |found| {
        if let Found::Ahead(ahead, gap) = found {
            if let (true, Ahead::Ghost(owner)) = (first, ahead) {
                ghost = Some((owner, gap));
            }
            first = false;
        }
        let Some(term) = react(&state.vehicles, sink, id, vehicle, found) else {
            return;
        };
        match found {
            // On a loop shorter than its horizon it can find itself ahead, which
            // holds nobody up.
            Found::Ahead(Ahead::Vehicle(leader), gap) if leader != id => {
                if let Some(b) = binding.filter(|b| b.term <= term) {
                    binding = Some(b);
                    rest = rest.min(term);
                } else {
                    if let Some(b) = binding {
                        rest = rest.min(b.term);
                    }
                    binding = Some(Binding { leader, gap, term, rest: 0.0 });
                }
            }
            _ => rest = rest.min(term),
        }
    });

    // Making room for a vehicle indicating into its lane ahead, as the lane-change
    // model chooses, by what following its ghost would take.
    let ghost = ghost.map(|(owner, gap)| {
        let owner_speed = state.vehicles[owner].speed;
        let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, owner_speed);
        (owner, gap, term)
    });
    let yielding = lane_change::make_room(
        &vehicle.lc,
        &vehicle.spec,
        ghost.map(|(owner, _, term)| (owner, term)),
    );
    if let Some((owner, gap, term)) = ghost.filter(|_| yielding.is_some()) {
        let source = AccelSource::Yield { id: owner, gap };
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        rest = rest.min(term);
    }
    let binding = binding.map(|b| Binding { rest, ..b });
    let accel = binding.map_or(rest, |b| b.term.min(rest));
    AccelUpdate::Own { id, accel, yielding, binding }
}

/// Makes a vehicle arrive at each junction on its path that nothing holds it
/// back from: up to and including the first junction it hasn't physically
/// entered (see [`entered`]). Once barriers exist, one it hasn't committed to
/// will hold it back too. At each, the lanes of its path up to the first of the
/// junction's lanes with a conflict, that lane included, join those it is
/// approached along, so the junction's scans find it.
fn arrive(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    queues: &mut [Queue],
    id: VehicleId,
    vehicle: &Vehicle,
) {
    let mut last = None;
    for &lane in &vehicle.path {
        let Some(j) = network.lane(lane).junction() else {
            continue;
        };
        if last == Some(j) {
            continue;
        }
        if let Some(previous) = last
            && !entered(network, occupants, previous, id)
        {
            return;
        }
        mark_way(
            network,
            &mut queues[j.0 as usize].approaching,
            &vehicle.path,
            j,
        );
        last = Some(j);
    }
}

/// Adds the lanes of `path` up to the first of `junction`'s lanes on it with a
/// conflict, that lane included, to `approaching`, kept sorted.
fn mark_way(
    network: &SimNetwork,
    approaching: &mut Vec<LaneId>,
    path: &[LaneId],
    junction: JunctionId,
) {
    let into = |&lane: &LaneId| {
        network.lane(lane).junction() == Some(junction) && conflict_start(network, lane).is_some()
    };
    let Some(k) = path.iter().position(into) else {
        return;
    };
    for &lane in &path[..=k] {
        if let Err(i) = approaching.binary_search_by_key(&lane.0, |l| l.0) {
            approaching.insert(i, lane);
        }
    }
}

/// What a junction's scans for vehicles to commit read, and where they note
/// which lanes they found anyone on.
struct Scan<'a> {
    network: &'a SimNetwork,
    vehicles: &'a SlotMap<VehicleId, Vehicle>,
    occupants: &'a [Vec<Occupant>],
    junction: JunctionId,
    /// The lanes vehicles approach the junction along, sorted.
    approaching: &'a [LaneId],
    /// Lanes a scan found no one on, nor beyond.
    empty: &'a mut Vec<LaneId>,
    /// Lanes a scan found someone on, or beyond.
    kept: &'a mut Vec<LaneId>,
    /// The vehicles not yet committed the scans stopped at, headed onto the
    /// lane scanned.
    fronts: &'a mut Vec<VehicleId>,
}

/// How many lanes a scan back from a junction's lane holds at once: that lane
/// and those it has come back through.
const SCAN_DEPTH: usize = 32;

/// Commits the vehicles approaching `lane`, one of the scan's junction's lanes
/// with a conflict, that must commit now, nearest first, noting on the way
/// which lanes it found anyone on.
///
/// It scans back from the start of the lane's conflict, a lane being in at most
/// one junction and so having at most one conflict zone: along its occupants,
/// then the lanes leading into it, branch by branch, following only lanes
/// vehicles approach the junction along. On each branch it passes over ghosts,
/// committed vehicles and vehicles whose path has left the way to `lane` and
/// goes nowhere in the junction. At the first other one:
///
/// - one headed onto `lane` commits if it must (see [`must_commit`]) and the
///   scan goes on behind it; if it needn't yet, the scan stops there, since
///   nobody behind it may commit before it;
/// - one headed elsewhere in the junction, or whose path ends before it could
///   leave the way to `lane`, stops the scan: it may yet go in ahead of those
///   behind it, and another lane's scan, or a later one, commits it.
///
/// So vehicles commit in their order along each lane. IDM keeps a follower able
/// to stop behind its leader only roughly: one closing fast on a slow leader
/// could otherwise commit first.
fn commit_along(scan: &mut Scan, lane: LaneId, committed: &mut Vec<VehicleId>) {
    let Some(start) = conflict_start(scan.network, lane) else {
        return;
    };
    let mut way = ArrayVec::<LaneId, SCAN_DEPTH>::new();
    scan_back(scan, lane, lane, start, &mut way, committed);
}

/// [`commit_along`]'s scan of `on`, from `s` back, having come back through
/// `way` from `target`. Returns whether it stopped at a vehicle that may yet
/// approach `target`, on `on` or beyond, and notes `on` as kept or empty.
fn scan_back(
    scan: &mut Scan,
    target: LaneId,
    on: LaneId,
    s: f64,
    way: &mut ArrayVec<LaneId, SCAN_DEPTH>,
    committed: &mut Vec<VehicleId>,
) -> bool {
    // So long a way back is unheard of: stop, as if at someone, to be safe.
    if way.try_push(on).is_err() {
        scan.kept.push(on);
        return true;
    }
    let stopped = stops_on(scan, target, on, s, way, committed) || {
        let mut stopped = false;
        for &p in scan.network.lane(on).predecessors() {
            if scan.approaching.binary_search_by_key(&p.0, |l| l.0).is_ok() {
                let length = scan.network.lane(p).geometry().length();
                stopped |= scan_back(scan, target, p, length, way, committed);
            }
        }
        stopped
    };
    way.pop();
    match stopped {
        true => scan.kept.push(on),
        false => scan.empty.push(on),
    }
    stopped
}

/// Whether [`scan_back`] stops on `on`, scanning its occupants from `s` back
/// and committing those that must as it goes. A vehicle headed onto `target`
/// that it stops at is noted as a front.
fn stops_on(
    scan: &mut Scan,
    target: LaneId,
    on: LaneId,
    s: f64,
    way: &[LaneId],
    committed: &mut Vec<VehicleId>,
) -> bool {
    let list = &scan.occupants[on.0 as usize];
    let end = list.partition_point(|o| o.s < s);
    for o in list[..end].iter().rev() {
        if o.ghost || committed.contains(&o.id) {
            continue;
        }
        let vehicle = &scan.vehicles[o.id];
        if vehicle.path.contains(&target) {
            if must_commit(scan.network, scan.junction, vehicle) {
                committed.push(o.id);
                continue;
            }
            if !scan.fronts.contains(&o.id) {
                scan.fronts.push(o.id);
            }
            return true;
        }
        let into = |&l: &LaneId| scan.network.lane(l).junction() == Some(scan.junction);
        let undecided = vehicle.path.last().is_some_and(|last| way.contains(last));
        if vehicle.path.iter().any(into) || undecided {
            return true;
        }
    }
    false
}

/// Whether a vehicle has physically entered a junction: it has a record, not a
/// ghost, on one of its lanes.
fn entered(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    id: VehicleId,
) -> bool {
    let lanes = network.junction(junction).lanes();
    lanes
        .iter()
        .any(|&lane| occupants[lane.0 as usize].iter().any(|o| o.id == id && !o.ghost))
}

/// How far a vehicle needs to stop from its speed, braking at `SAFE_BRAKING`:
/// whether it can stop, for committing to a junction and being released.
fn stopping_distance(vehicle: &Vehicle) -> f64 {
    vehicle.speed * vehicle.speed / (2.0 * acceleration::SAFE_BRAKING)
}

/// Where a lane's first conflict with the rest of its junction starts along it,
/// if it has one.
fn conflict_start(network: &SimNetwork, lane: LaneId) -> Option<f64> {
    let j = network.lane(lane).junction()?;
    let lanes = network.junction(j).lanes();
    let conflicts = lanes.iter().filter_map(|&theirs| network.conflict_between(lane, theirs));
    conflicts.map(|c| c.mine.start).reduce(f64::min)
}

/// How far a vehicle's front is from the start of the first conflict on its
/// path in a junction, negative once past it, if its path has one there.
fn conflict_distance(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> Option<f64> {
    let front = vehicle.lane_pos.s - vehicle.spec.rear_axle + vehicle.spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for &lane in &vehicle.path {
        if network.lane(lane).junction() == Some(junction)
            && let Some(conflict) = conflict_start(network, lane)
        {
            return Some(start + conflict);
        }
        start += network.lane(lane).geometry().length();
    }
    None
}

/// Whether a vehicle approaching a junction must commit there: it can no
/// longer stop, braking at `SAFE_BRAKING`, before the first conflict on its
/// path in the junction, or its front is past the start of it already. One
/// with no conflict on its path never needs to.
///
/// Vehicles at the front of the ways in already slow for committed vehicles in
/// their way, so committing late costs no harder braking, and keeps the order
/// vehicles commit in close to the order they reach their conflicts.
fn must_commit(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> bool {
    conflict_distance(network, junction, vehicle)
        .is_some_and(|distance| distance <= stopping_distance(vehicle))
}

/// Whether the vehicle committed at `index` in a junction's committed list may
/// be released: it could now stop, at `SAFE_BRAKING`, a car length short of its
/// conflict, so it plainly can stop, and no vehicle that committed after it is
/// behind it on its way in. The car length is the hysteresis against
/// [`must_commit`], and keeps a vehicle held at the line committed.
///
/// A vehicle committed while it couldn't stop may since have been stopped,
/// by the junction itself, with the margin it has up to `MAX_BRAKING`, or by
/// traffic ahead. Released, it no longer holds back those committed after it,
/// which breaks any cycle of waiting it was part of. Releasing from the back of
/// each lane keeps vehicles committed in their order along it.
fn may_release(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    committed: &[VehicleId],
    index: usize,
) -> bool {
    let id = committed[index];
    let vehicle = &vehicles[id];
    let Some(distance) = conflict_distance(network, junction, vehicle) else {
        return false;
    };
    if stopping_distance(vehicle) > distance - vehicle.spec.length {
        return false;
    }
    let lane = vehicle.path[0];
    let records = &occupants[lane.0 as usize];
    let record = |id: VehicleId| records.iter().find(|o| o.id == id && !o.ghost);
    let rear = record(id).map_or(vehicle.lane_pos.s - vehicle.spec.rear_axle, |o| o.s);
    // Behind it on its lane, or on its way onto it.
    let behind = |&later: &VehicleId| match record(later) {
        Some(o) => o.s < rear,
        None => vehicles[later].path.contains(&lane),
    };
    !committed[index + 1..].iter().any(behind)
}

/// Whether a change into `target` that a vehicle started now might not be
/// over, its whole body in the new lane, before it reached a conflict: the
/// start of a conflict lies within the change's reach (as [`room_to_change`]
/// measures it) on some way on from `target`. Its route has yet to choose past
/// `target`, so every way counts.
///
/// Such a change isn't allowed. A vehicle may commit partway through a change,
/// but must not be stopped at a conflict partway through one: there, its search
/// of its old lane could wait on a vehicle that waits on it, and both would be
/// too near the conflict to be released (see [`may_release`]).
fn conflict_during_change(network: &SimNetwork, vehicle: &Vehicle, target: LaneId) -> bool {
    // Neighbors start and end together, so the same fraction along is level.
    let own = network.lane(vehicle.path[0]).geometry().length();
    let theirs = network.lane(target).geometry().length();
    let reach = change_reach(vehicle);
    // Each lane to look at, with how far its start is from the rear axle.
    let mut ways = ArrayVec::<(LaneId, f64), 16>::new();
    ways.push((target, -vehicle.lane_pos.s * theirs / own));
    while let Some((lane, start)) = ways.pop() {
        if start >= reach {
            continue;
        }
        if conflict_start(network, lane).is_some_and(|conflict| start + conflict < reach) {
            return true;
        }
        let end = start + network.lane(lane).geometry().length();
        for &next in network.lane(lane).successors() {
            // So many ways within a change's reach is unheard of: refuse, to be safe.
            if ways.try_push((next, end)).is_err() {
                return true;
            }
        }
    }
    false
}

/// Whether a vehicle's path takes it onto any of a junction's lanes.
fn headed(network: &SimNetwork, vehicle: &Vehicle, junction: JunctionId) -> bool {
    vehicle.path.iter().any(|&lane| network.lane(lane).junction() == Some(junction))
}

/// The terms the vehicles committed at a junction impose on each one committed
/// after them, and on the vehicles at the front of the ways in, not yet
/// committed, at most one per follower, the lowest, each recorded to the debug
/// sink.
///
/// For each pair, ahead and behind, every conflict between a lane of the
/// follower's path in the junction and a lane of the junction's counts:
///
/// - **Crossing:** until the one ahead has cleared the overlap on its lane, the
///   follower stops short of the overlap on its own, as if at a stopped vehicle
///   there. A follower whose front is already past the start of the overlap
///   isn't held back.
/// - **Merging:** the follower follows the one ahead as if it were already on
///   its own lane, both measured back from the end of their lanes, where they
///   meet.
///
/// O(c·(c + f)) in the numbers committed and at the fronts, which are small.
fn junction_terms<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    queue: &Queue,
    decisions: &mut Vec<AccelUpdate>,
) {
    let committed = &queue.committed;
    let followers = committed.iter().enumerate().map(|(k, &id)| (id, &committed[..k]));
    // The vehicles at the front of the ways in, not yet committed, are behind
    // every committed one.
    let fronts = queue.fronts.iter().map(|&id| (id, &committed[..]));
    for (id, ahead) in followers.chain(fronts) {
        let accel = junction_term(state, sink, junction, id, ahead);
        if accel < f64::INFINITY {
            decisions.push(AccelUpdate::Junction { id, accel });
        }
    }
}

/// The lowest of the terms the vehicles `ahead` of `id` in a junction impose on
/// it, each recorded to the debug sink: infinite if none does.
fn junction_term<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    id: VehicleId,
    ahead: &[VehicleId],
) -> f64 {
    let network = &state.network;
    let lanes = network.junction(junction).lanes();
    let follower = &state.vehicles[id];
    let spec = &follower.spec;
    let mut accel = f64::INFINITY;
    let front = follower.lane_pos.s - spec.rear_axle + spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for &mine in &follower.path {
        let length = network.lane(mine).geometry().length();
        if network.lane(mine).junction() == Some(junction) {
            for &theirs in lanes {
                let Some(conflict) = network.conflict_between(mine, theirs) else {
                    continue;
                };
                for &other in ahead {
                    let Some((gap, speed)) =
                        conflict_gap(state, conflict, theirs, other, start, length)
                    else {
                        continue;
                    };
                    let term = acceleration::interaction(spec, follower.speed, gap, speed);
                    let kind = conflict.kind;
                    let source = AccelSource::Junction { id: other, kind, gap };
                    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                    accel = accel.min(term);
                }
            }
        }
        start += length;
    }
    accel
}

/// The gap a junction holds a vehicle back by for another ahead of it in the
/// queue, and the speed of what it is held back by, where the lane it will take,
/// `start` meters from its front and `length` long, conflicts with `theirs`:
/// see [`junction_terms`]. `None` if the other one doesn't hold it back there.
fn conflict_gap(
    state: &SimState,
    conflict: &Conflict,
    theirs: LaneId,
    other: VehicleId,
    start: f64,
    length: f64,
) -> Option<(f64, f64)> {
    let network = &state.network;
    let leader = &state.vehicles[other];
    // The other one's rear along `theirs`, if its path takes it there.
    let rear = leader.path.iter().position(|&l| l == theirs).map(|i| {
        let before: f64 =
            leader.path[..i].iter().map(|&l| network.lane(l).geometry().length()).sum();
        leader.lane_pos.s - leader.spec.rear_axle - before
    });
    match conflict.kind {
        ConflictKind::Cross => {
            let clear = match rear {
                Some(rear) => rear >= conflict.theirs.end,
                // Not headed there: only its body, lingering past the lane's end
                // or beside it, can be in the way.
                None => state.occupants[theirs.0 as usize]
                    .iter()
                    .find(|o| o.id == other && !o.ghost)
                    .is_none_or(|o| {
                        o.s >= conflict.theirs.end
                            || o.s + leader.spec.length <= conflict.theirs.start
                    }),
            };
            let gap = start + conflict.mine.start;
            // Past the start of the overlap, it is committed.
            (gap >= 0.0 && !clear).then_some((gap, 0.0))
        }
        ConflictKind::Merge => {
            let theirs_length = network.lane(theirs).geometry().length();
            rear.map(|rear| ((start + length) - (theirs_length - rear), leader.speed))
        }
    }
}

/// The side of `lane` a vehicle is straddling it from, if it is partway through a
/// lane change out of it: `lane` is beside the lane it follows, on its old side.
fn straddling(network: &SimNetwork, vehicle: &Vehicle, lane: LaneId) -> Option<Side> {
    let side = vehicle.old_side.filter(|_| vehicle.lane_change.is_some())?;
    let old = network.lane(vehicle.path[0]).adj(side)?;
    (old.lane == lane).then_some(side)
}

/// The most a vehicle straddling its old lane, on `side` of the lane it follows,
/// may accelerate for what is ahead of it there: a leader search from its record
/// there, `own` in the old lane's occupants, along the neighbors of its path, so it doesn't run into its old leader. Once the change is complete,
/// nothing on the old lanes can be in its way, so it isn't asked. Each term is
/// recorded to the debug sink as it is computed.
fn old_side<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    side: Side,
    own: usize,
) -> f64 {
    let mut accel = f64::INFINITY;
    for_each_leader(state, vehicle, Along::Leaving(side, own), |found| {
        if let Some(term) = react(&state.vehicles, sink, id, vehicle, found) {
            accel = accel.min(term);
        }
    });
    accel
}

/// Records what vehicle `id`'s leader search found and, for a leader or a stop
/// line, returns the term it imposes. Ghosts aren't leaders here: the nearest is
/// weighed up for making room.
fn react<D: DebugSink>(
    vehicles: &SlotMap<VehicleId, Vehicle>,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    found: Found,
) -> Option<f64> {
    let (ahead, gap) = match found {
        Found::Ahead(ahead, gap) => (ahead, gap),
        Found::Pass(check) => {
            sink.record(|| DebugEvent::PassCheck { vehicle: id, check });
            return None;
        }
    };
    let source = match ahead {
        Ahead::Vehicle(leader) => AccelSource::Leader { id: leader, gap },
        Ahead::PathEnd => AccelSource::PathEnd { gap },
        Ahead::Ghost(_) => return None,
    };
    let speed = speed_of(vehicles, ahead);
    let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, speed);
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
    Some(term)
}

/// The speed of something a leader search found: a vehicle's, a ghost's owner's,
/// or zero for a stop line.
fn speed_of(vehicles: &SlotMap<VehicleId, Vehicle>, ahead: Ahead) -> f64 {
    match ahead {
        Ahead::Vehicle(id) | Ahead::Ghost(id) => vehicles[id].speed,
        Ahead::PathEnd => 0.0,
    }
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
    state: &SimState,
    caps: &[(VehicleId, f64)],
    time: f64,
    id: VehicleId,
    vehicle: &Vehicle,
    lc: &mut lane_change::State,
) -> Option<(Decision, [Option<LaneChangeEval>; 2])> {
    let SimState { network, vehicles, .. } = state;
    if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
        return None;
    }
    let lane = network.lane(vehicle.path[0]);
    // Its acceleration now is the one it holds, from the last longitudinal
    // update. A speed cap holds it back in any lane, so it does after changing too.
    let own_now = vehicle.accel;
    let cap = caps.iter().find(|&&(capped, _)| capped == id);
    let cap = cap.map_or(f64::INFINITY, |&(_, max_speed)| {
        acceleration::capped(&vehicle.spec, vehicle.speed, max_speed)
    });
    // Its old follower's, behind it in its own lane, is the same for either side,
    // so it is measured once, when first needed: as now, and with this vehicle
    // gone, when it would react to what this one does, as Treiber's MOBIL has it.
    let mut old_follower = None;
    let prospect = |side: Side| {
        let target = lane.adj(side).filter(|n| n.can_change)?;
        if !room_to_change(network, vehicle, side)
            || conflict_during_change(network, vehicle, target.lane)
        {
            return None;
        }
        let old_follower = *old_follower.get_or_insert_with(|| {
            let held = vehicle.old_follower?;
            let f = vehicles.get(held.id)?;
            let own = occupancy::record_index(state, vehicle.path[0], id);
            let mut after = held.without;
            for_each_leader(state, vehicle, Along::Path(own), |found| {
                if let Found::Ahead(Ahead::Vehicle(leader), gap) = found {
                    let gap = held.gap + vehicle.spec.length + gap;
                    let speed = vehicles[leader].speed;
                    after = after.min(acceleration::interaction(&f.spec, f.speed, gap, speed));
                }
            });
            Some(Effect { before: f.accel, after, speed: f.speed })
        });
        // As if it were already settled in the target lane, level with where it is
        // now, as MOBIL weighs it up: not how it would get there.
        let own_after = accel_along(state, id, vehicle, Along::Into(side), true).min(cap);
        // The new follower, behind it in the target lane: as now, but with this
        // vehicle as a leader too. The lane is split at its rear there, as its
        // search there splits it.
        let rear = occupancy::axle_beside(network, vehicle, target.lane) - vehicle.spec.rear_axle;
        let new_follower = find_follower(state, target.lane, rear, id).map(|(follower, gap)| {
            let f = &vehicles[follower];
            let cut_in = acceleration::interaction(&f.spec, f.speed, gap, vehicle.speed);
            Effect { before: f.accel, after: f.accel.min(cut_in), speed: f.speed }
        });
        Some(Prospect { own_now, own_after, new_follower, old_follower })
    };
    Some(lane_change::decide(
        lc,
        vehicle.intent,
        &vehicle.spec,
        time,
        DECISION_PERIOD,
        prospect,
    ))
}

/// Whether a change to `side` that a vehicle started now would be over, its
/// whole body in the new lane, before the lanes beside its path on that side
/// run out, as they do into a junction, whose lanes have no neighbors. Changing
/// lanes through a junction would put it where the junction doesn't know it is.
/// SUMO likewise allows no lane changes on a junction's internal lanes.
fn room_to_change(network: &SimNetwork, vehicle: &Vehicle, side: Side) -> bool {
    let needed = change_reach(vehicle);
    let mut room = -vehicle.lane_pos.s;
    for &lane in &vehicle.path {
        if room >= needed {
            return true;
        }
        if network.lane(lane).adj(side).is_none() {
            return false;
        }
        room += network.lane(lane).geometry().length();
    }
    room >= needed
}

/// How far beyond its rear axle a lane change a vehicle started now reaches
/// before its whole body is in the new lane: the change starts where pure
/// pursuit aims, and the body trails the axle.
fn change_reach(vehicle: &Vehicle) -> f64 {
    lateral::lookahead_distance(vehicle.speed)
        + lateral::change_length(vehicle.speed)
        + vehicle.spec.length
}

/// A vehicle's acceleration under the longitudinal model searching `along` some
/// lanes, against the free road there, its leaders and the end of its path.
/// Ghosts count as leaders if `ghosts_lead`, as when
/// weighing up a change into their lane: a vehicle already indicating into the
/// gap goes first. Its own ghost, `id`'s, is passed over.
fn accel_along(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    along: Along,
    ghosts_lead: bool,
) -> f64 {
    let lane = along.lane(&state.network, vehicle, 0).unwrap_or(vehicle.path[0]);
    let desired = vehicle.spec.desired_speed(state.network.lane(lane).speed_limit());
    let mut accel = acceleration::free_road(&vehicle.spec, vehicle.speed, desired);
    for_each_leader(state, vehicle, along, |found| match found {
        Found::Ahead(Ahead::Ghost(owner), _) if owner == id || !ghosts_lead => {}
        Found::Ahead(ahead, gap) => {
            let speed = speed_of(&state.vehicles, ahead);
            let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, speed);
            accel = accel.min(term);
        }
        Found::Pass(_) => {}
    });
    accel
}

/// How a vehicle not already changing lanes would change into `target`, a lane
/// beside the one it is on: its rear axle's position on `target`, and the cubic
/// its path would follow onto `target`'s centerline, as [`Simulation::change_lane`]
/// carries it out.
///
/// The cubic starts where pure pursuit is aiming on the vehicle's current path,
/// with that path's offset and slope relative to `target`, so the target point
/// carries on smoothly instead of jumping.
fn plan_change(network: &SimNetwork, vehicle: &Vehicle, target: LaneId) -> (LanePos, LaneChange) {
    let current = network.lane(vehicle.path[0]);
    let lane = network.lane(target);
    // Adjacent lanes start and end together, so the same fraction is a good hint.
    let hint = lane.geometry().lane_pos(
        vehicle.lane_pos.s / current.geometry().length() * lane.geometry().length(),
        0.0,
    );
    let lane_pos = lane.geometry().world_to_lane_near(vehicle.pos, hint);
    let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
    let aim = network.point_along(vehicle.path.iter().copied(), ahead, 0.0);
    let beyond = network.point_along(vehicle.path.iter().copied(), ahead + 1.0, 0.0);
    let start = lane.geometry().world_to_lane_near(aim, lane_pos);
    let next = lane.geometry().world_to_lane_near(beyond, start);
    let slope = (next.t - start.t) / (next.s - start.s);
    (
        lane_pos,
        LaneChange::new(start.s, start.t, slope, vehicle.speed),
    )
}

/// How far along its path, beyond its rear axle at `axle_s`, a vehicle
/// needs to know where it is going: as far as its leader search looks from its
/// front, its path tracker aims, and its lane change, `change`, runs.
fn reach(vehicle: &Vehicle, axle_s: f64, change: Option<LaneChange>) -> f64 {
    let nose = vehicle.spec.length - vehicle.spec.rear_axle;
    let search = acceleration::horizon(&vehicle.spec, vehicle.speed) + nose;
    // Path tracking, and planning a lane change, aim a metre past the lookahead.
    let tracking = lateral::lookahead_distance(vehicle.speed) + 1.0;
    let change = change.map_or(0.0, |c| c.end() - axle_s);
    search.max(tracking).max(change)
}

/// Extends `path` by the lanes `route` chooses until it reaches `needed`
/// beyond the rear axle, at `axle_s` on its first lane, or reaches a lane that
/// leads nowhere. Returns how far beyond the rear axle it then reaches.
///
/// A path that ends inside a junction goes on until it is out of it, so a
/// vehicle in a junction's queue knows its way through: the junction keeps
/// others clear of the lanes it will take, and one that chose its way only as
/// it went could be committed to a conflict before another learns it is in
/// the way.
fn extend_path(
    network: &SimNetwork,
    path: &mut Vec<LaneId>,
    route: &mut Route,
    axle_s: f64,
    needed: f64,
) -> f64 {
    let mut covered =
        path.iter().map(|&lane| network.lane(lane).geometry().length()).sum::<f64>() - axle_s;
    // Bounds the lanes added to get out of a junction, should its lanes loop.
    let mut inside = 0;
    loop {
        let last = *path.last().expect("a path holds the lane it follows");
        let junction = network.lane(last).junction();
        let through = junction.is_some_and(|j| inside < network.junction(j).lanes().len());
        if covered >= needed && !through {
            break;
        }
        let Some(next) = route.choose(network.lane(last).successors()) else {
            break;
        };
        if covered >= needed {
            inside += 1;
        }
        path.push(next);
        covered += network.lane(next).geometry().length();
    }
    covered
}

/// How far a vehicle's path reaches beyond its front, in meters.
fn reach_beyond_front(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let length: f64 = vehicle.path.iter().map(|&lane| network.lane(lane).geometry().length()).sum();
    length - vehicle.lane_pos.s - (vehicle.spec.length - vehicle.spec.rear_axle)
}

#[cfg(test)]
mod test {
    //! Snapshots: each sets up one instant and checks what the searches and the
    //! lane-change decision make of it, without stepping.

    use glam::DVec2;

    use super::*;
    use crate::debug::DebugEvent;
    use crate::models::lane_change::Outcome;
    use crate::network::Lane;

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
        Placement { lane: LaneId(lane), s, speed, route: Route::default() }
    }

    /// A vehicle's rear, where both searches split its lane.
    fn rear(sim: &Simulation, id: VehicleId) -> f64 {
        let v = &sim.state.vehicles[id];
        v.lane_pos.s - v.spec.rear_axle
    }

    fn follower(sim: &Simulation, lane: u32, s: f64, skip: VehicleId) -> Option<(VehicleId, f64)> {
        find_follower(&sim.state, LaneId(lane), s, skip)
    }

    /// `id`'s search along its path, from its own record, calling `f` with each
    /// thing it finds until `f` breaks.
    fn search<B>(
        sim: &Simulation,
        id: VehicleId,
        f: impl FnMut(Found) -> ControlFlow<B>,
    ) -> ControlFlow<B> {
        let v = &sim.state.vehicles[id];
        let own = occupancy::record_index(&sim.state, v.path[0], id);
        find_leaders(&sim.state, v, Along::Path(own), f)
    }

    /// The nearest vehicle `id` reacts to, stopping the search there.
    fn leader(sim: &Simulation, id: VehicleId) -> Option<(VehicleId, f64)> {
        search(sim, id, |found| match found {
            Found::Ahead(Ahead::Vehicle(leader), gap) => ControlFlow::Break((leader, gap)),
            _ => ControlFlow::Continue(()),
        })
        .break_value()
    }

    /// What `id` decides now, its lane-change state afterwards, and what it
    /// weighed up. Partway through a change, it decides nothing and weighs up
    /// nothing.
    fn decision(
        sim: &Simulation,
        id: VehicleId,
    ) -> (Decision, lane_change::State, Vec<LaneChangeEval>) {
        let vehicle = &sim.state.vehicles[id];
        let mut lc = vehicle.lc;
        match decide(&sim.state, &sim.caps, sim.time, id, vehicle, &mut lc) {
            Some((decision, evals)) => (decision, lc, evals.into_iter().flatten().collect()),
            None => (Decision::Nothing, lc, Vec::new()),
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
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
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
        let record = sim.state.occupants[0].iter().find(|o| o.id == leaving).unwrap();
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
        let horizon = acceleration::horizon(&sim.state.vehicles[id].spec, LIMIT);
        let front = 50.0 + 3.6;
        assert!(follower(&sim, 0, front + horizon - 0.1, VehicleId::null()).is_some());
        assert!(follower(&sim, 0, front + horizon + 0.1, VehicleId::null()).is_none());
        // Stopped, it looks only a few metres ahead.
        sim.state.vehicles[id].speed = 0.0;
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
        // Lane 0 splits into lane 1, straight on, and lane 2, bearing left.
        let lane = |x0: f64, x1: f64, y1: f64| {
            Lane::new(&[DVec2::new(x0, 0.0), DVec2::new(x1, y1)], WIDTH, LIMIT).unwrap()
        };
        let mut network = SimNetwork::new(vec![
            lane(0.0, 100.0, 0.0),
            lane(100.0, 200.0, 0.0),
            lane(100.0, 200.0, 10.0),
        ]);
        network.connect(LaneId(0), LaneId(2));
        network.connect(LaneId(0), LaneId(1));
        let mut sim = Simulation::new(network, NoopSink);
        // It turns onto lane 2, so it isn't lane 1's follower.
        let turning = sim.insert(car(), at(0, 80.0, LIMIT));
        let path = &mut sim.state.vehicles[turning].path;
        path.clear();
        path.extend([LaneId(0), LaneId(2)]);
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
        // with a gap of zero or less. Moving, so both look further than that.
        for tenths in -92..=92 {
            let mut sim = two_lanes();
            let id = sim.insert(car(), at(0, 100.0, 10.0));
            let s = 100.0 + tenths as f64 / 10.0;
            let other = sim.insert(car(), at(0, s, 10.0));
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
            sim.state.occupants[lane]
                .iter()
                .find(|o| o.id == id && !o.ghost)
                .map(|o| o.lateral)
        };
        assert_eq!(
            lateral(1),
            Some(Lateral::Diverging(Side::Left)),
            "the lane left"
        );
        assert_eq!(lateral(0), Some(Lateral::InLane), "the lane moved into");
        assert_eq!(sim.state.vehicles[id].old_side, Some(Side::Right));
    }

    #[test]
    fn a_ghost_is_in_the_lane_it_indicates_into() {
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 100.0, 10.0));
        sim.indicate(id, Side::Left);
        let ghost = sim.state.occupants[0].iter().find(|o| o.id == id).unwrap();
        assert!(ghost.ghost);
        assert_eq!(ghost.lateral, Lateral::InLane);
        let own = sim.state.occupants[1].iter().find(|o| o.id == id).unwrap();
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

    /// Two lanes side by side, 500 m long, limited to 25 m/s, that run on side by
    /// side for 200 m as lanes 2 and 5. Lane 2, on the left, is in a junction,
    /// crossed 10 m along by lane 3, and runs on as lane 4; lane 5 is beside it,
    /// so changes are permitted into lane 2, as at the ramps' lane drop.
    fn two_lanes_into_a_junction() -> Simulation {
        let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], WIDTH, 25.0).unwrap();
        let p = DVec2::new;
        let mut network = SimNetwork::new(vec![
            lane(p(0.0, 0.0), p(500.0, 0.0)),
            lane(p(0.0, -WIDTH), p(500.0, -WIDTH)),
            lane(p(500.0, 0.0), p(700.0, 0.0)),
            lane(p(510.0, -40.0), p(510.0, 40.0)),
            lane(p(700.0, 0.0), p(900.0, 0.0)),
            lane(p(500.0, -WIDTH), p(700.0, -WIDTH)),
        ]);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network.set_adjacent(LaneId(2), LaneId(5), true, true);
        for (from, to) in [(0, 2), (2, 4), (1, 5)] {
            network.connect(LaneId(from), LaneId(to));
        }
        network.add_junction(&[LaneId(2), LaneId(3)]);
        Simulation::new(network, NoopSink)
    }

    /// A car on lane 1 at `s` and `speed`, indicating left for 2 s with
    /// advantage built up for it, behind a car held stopped 20 m ahead.
    fn indicating_behind_a_stopped_car_at(sim: &mut Simulation, s: f64, speed: f64) -> VehicleId {
        let blocker = sim.insert(car(), at(1, s + 20.0, 0.0));
        sim.hold(blocker);
        let id = sim.insert(car(), at(1, s, speed));
        sim.indicate(id, Side::Left);
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        sim.time = 2.0;
        id
    }

    #[test]
    fn it_wont_start_a_change_it_couldnt_finish_before_a_conflict() {
        let mut sim = two_lanes_into_a_junction();
        // At 20 m/s a change reaches 93 m beyond its rear axle, and the crossing
        // starts 78 m beyond it on the lane beside it. The lanes beside run on,
        // so there is room for the change otherwise.
        let id = indicating_behind_a_stopped_car_at(&mut sim, 430.0, 20.0);
        let (decision, evals) = decide_now(&sim, id);
        assert_ne!(decision, Decision::Change(Side::Left), "{evals:?}");
        assert!(evals.iter().all(|e| e.side != Side::Left), "{evals:?}");
    }

    #[test]
    fn it_changes_into_a_lane_leading_to_a_conflict_beyond_the_changes_reach() {
        let mut sim = two_lanes_into_a_junction();
        // The crossing starts 208 m beyond its rear axle.
        let id = indicating_behind_a_stopped_car_at(&mut sim, 300.0, 20.0);
        let (decision, evals) = decide_now(&sim, id);
        assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
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
        // Close behind it on lane 1, held back by it now, as the longitudinal update
        // records.
        let behind = sim.insert(car(), at(1, 290.0, 5.0));
        sim.update_accels();
        assert_eq!(
            sim.state.vehicles[id].old_follower.map(|f| f.id),
            Some(behind)
        );
        let (_, evals) = decide_now(&sim, id);
        let gain = evals[0].old_follower.expect("an old follower");
        assert!(gain > 0.0, "gain {gain}");
    }

    #[test]
    fn a_car_it_doesnt_hold_up_isnt_its_old_follower() {
        // Far enough back that the free road binds it, not this car.
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 300.0, 5.0));
        sim.insert(car(), at(1, 240.0, 5.0));
        sim.update_accels();
        assert!(sim.state.vehicles[id].old_follower.is_none());
    }

    #[test]
    fn a_car_its_speed_cap_holds_back_isnt_held_up_by_the_car_ahead() {
        // Close behind, but held to a stop: its cap binds, so leaving frees it of
        // nothing.
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 300.0, 5.0));
        let behind = sim.insert(car(), at(1, 290.0, 5.0));
        sim.update_accels();
        assert_eq!(
            sim.state.vehicles[id].old_follower.map(|f| f.id),
            Some(behind)
        );
        sim.hold(behind);
        sim.update_accels();
        assert!(sim.state.vehicles[id].old_follower.is_none());
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
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE; 2]);
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
            if sim.state.vehicles[id].path[0] == LaneId(lane) {
                return;
            }
            sim.update_motion(1.0 / 60.0);
            sim.update_occupancy();
        }
        panic!("never reached lane {lane}");
    }

    #[test]
    fn at_a_fork_it_joins_every_branch_and_leaves_those_it_doesnt_take() {
        // Lane 0 forks at 100 m into lane 1, straight on, and lane 2, bearing left.
        let lane = |x0: f64, x1: f64, y1: f64| {
            Lane::new(&[DVec2::new(x0, 0.0), DVec2::new(x1, y1)], WIDTH, LIMIT).unwrap()
        };
        let mut network = SimNetwork::new(vec![
            lane(0.0, 100.0, 0.0),
            lane(100.0, 200.0, 0.0),
            lane(100.0, 200.0, 20.0),
        ]);
        network.connect(LaneId(0), LaneId(1));
        network.connect(LaneId(0), LaneId(2));
        let mut sim = Simulation::new(network, NoopSink);
        let id = sim.insert(car(), at(0, 90.0, 10.0));
        // Straight on, though lane 2, on the left, is its first successor.
        let path = &mut sim.state.vehicles[id].path;
        path.clear();
        path.extend([LaneId(0), LaneId(1)]);
        advance_onto(&mut sim, id, 1);
        let mine = records(&sim, id);
        assert!(mine.contains(&(1, false, Lateral::InLane)), "{mine:?}");
        // Lane 1 lies to the right of lane 2, so it is moving out of it that way.
        assert!(
            mine.contains(&(2, false, Lateral::Diverging(Side::Right))),
            "{mine:?}"
        );
        // Clear of lane 2 once the branches have parted, it leaves it.
        for _ in 0..600 {
            sim.update_motion(1.0 / 60.0);
            sim.update_occupancy();
            if records(&sim, id).iter().all(|r| r.0 != 2) {
                break;
            }
        }
        let mine = records(&sim, id);
        assert!(mine.iter().all(|r| r.0 != 2), "{mine:?}");
        assert!(mine.contains(&(1, false, Lateral::InLane)), "{mine:?}");
    }

    /// Lanes holding a record of `id`, and whether each is its ghost.
    fn records<D: DebugSink>(sim: &Simulation<D>, id: VehicleId) -> Vec<(u32, bool, Lateral)> {
        let lanes = sim.state.occupants.iter().enumerate();
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
        assert_eq!(
            sim.state.vehicles[id].intent.map(|i| i.side),
            Some(Side::Left)
        );
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
        assert!(sim.state.vehicles[id].intent.is_none());
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
        assert_eq!(sim.state.vehicles[id].old_side, Some(Side::Right));
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
            let v = &sim.state.vehicles[id];
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
        assert_eq!(sim.state.vehicles[id].path[0], LaneId(2));
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
        sim.state.vehicles[id].lc.set_advantage([lane_change::INTENT, 0.0]);
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
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        let (action, evals) = decide_now(&sim, id);
        assert_eq!(action, Decision::Nothing);
        assert_eq!(evals[0].outcome, Outcome::Unsafe);
    }

    #[test]
    fn advantage_fades_while_changing_isnt_worth_it() {
        // Nothing ahead: no gain in either lane, so what had built up fades.
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 300.0, 5.0));
        // What it would gain is against what it holds now.
        sim.update_accels();
        sim.state.vehicles[id].lc.set_advantage([0.2, 0.0]);
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
        sim.decision_cursor = sim.state.vehicles.iter().position(|(v, _)| v == id).unwrap();
        sim.update_decisions(1);
        assert_eq!(
            sim.state.vehicles[id].path[0],
            LaneId(0),
            "it should have changed"
        );
        assert_eq!(sim.state.vehicles[id].lc.advantage(), [0.0; 2]);
    }

    // Frustration and cooperation.

    /// Behind a stopped car with advantage built up for changing left, and a car
    /// in lane 0 coming up 27 m behind its rear at `speed`, so changing would make
    /// it brake: about 2.1 m/s² at 12 m/s, more than a calm driver accepts.
    fn wanting_to_change_with_a_follower_at(speed: f64) -> (Simulation, VehicleId) {
        let (mut sim, id) = behind_a_stopped_car(20.0);
        // Its front 27 m behind the changer's rear, at 299.
        sim.insert(car(), at(0, 299.0 - 27.0 - 3.6, speed));
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
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
        sim2.state.vehicles[free].lc.set_frustration(4.0);
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
        sim.state.vehicles[id].lc.set_frustration(0.99 * lane_change::FULLY_FRUSTRATED);
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
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        sim.state.vehicles[id].lc.set_frustration(lane_change::FULLY_FRUSTRATED);
        let (action, evals) = decide_now(&sim, id);
        assert!(!evals[0].safe);
        assert_eq!(evals[0].outcome, Outcome::Request);
        assert_eq!(action, Decision::Indicate(Side::Left));
    }

    #[test]
    fn it_doesnt_ask_a_stopped_queue_to_make_room() {
        let (mut sim, id) = behind_a_stopped_car(20.0);
        sim.insert(car(), at(0, 300.0 - 2.0, 0.0));
        sim.state.vehicles[id].lc.set_advantage([lane_change::MAX_ADVANTAGE, 0.0]);
        sim.state.vehicles[id].lc.set_frustration(lane_change::FULLY_FRUSTRATED);
        let (action, evals) = decide_now(&sim, id);
        assert_eq!(evals[0].outcome, Outcome::Unsafe);
        assert_eq!(action, Decision::Nothing);
    }

    #[test]
    fn its_own_politeness_weighs_its_followers() {
        // Cutting in front of a car would cost it; an impolite driver doesn't care.
        let (mut sim, id) = wanting_to_change_with_a_follower_at(12.0);
        let follower = sim.state.vehicles.iter().find(|(_, v)| v.path[0] == LaneId(0)).unwrap().0;
        sim.state.vehicles[follower].accel = 0.5;
        let incentive = |sim: &Simulation| decide_now(sim, id).1[0].incentive;
        let polite = incentive(&sim);
        sim.state.vehicles[id].spec.politeness = 0.0;
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
        assert_eq!(sim.state.vehicles[follower].lc.yielding(), Some(indicating));
        assert!(
            sim.state.vehicles[follower].accel < 0.0,
            "it should ease off"
        );
    }

    #[test]
    fn a_follower_drives_on_if_making_room_would_take_more_than_its_courtesy() {
        // Making room would take braking harder than 0.2 m/s².
        let (mut sim, _, follower) = behind_an_indicator(15.0, 0.2);
        sim.update_accels();
        assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
        // One with no courtesy never does, however easy.
        let (mut sim, _, follower) = behind_an_indicator(80.0, 0.0);
        sim.update_accels();
        assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
    }

    #[test]
    fn a_follower_keeps_making_room_once_it_has_chosen_to() {
        // Having chosen to, it carries on even where it wouldn't start: here its
        // courtesy alone wouldn't stretch to it.
        let (mut sim, indicating, follower) = behind_an_indicator(15.0, 0.2);
        sim.state.vehicles[follower].lc.set_yielding(Some(indicating));
        sim.update_accels();
        assert_eq!(sim.state.vehicles[follower].lc.yielding(), Some(indicating));
        // Once the indicator is gone, there is nothing to make room for.
        sim.cancel_intent(indicating);
        sim.update_accels();
        assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
    }

    // Several leaders.

    /// Every leader `id` reacts to now, nearest first.
    fn leaders(sim: &Simulation, id: VehicleId) -> Vec<VehicleId> {
        let vehicle = |ahead: Ahead| match ahead {
            Ahead::Vehicle(leader) => Some(leader),
            _ => None,
        };
        ahead(sim, id).into_iter().filter_map(vehicle).collect()
    }

    /// Everything `id`'s leader search finds now, nearest first.
    fn ahead(sim: &Simulation, id: VehicleId) -> Vec<Ahead> {
        let mut found = Vec::new();
        let ControlFlow::Continue(()) = search(sim, id, |f| {
            if let Found::Ahead(ahead, _) = f {
                found.push(ahead);
            }
            ControlFlow::<!>::Continue(())
        });
        found
    }

    #[test]
    fn a_car_sees_a_diverging_car_its_leader_has_passed() {
        // C, changing left out of lane 1, held with its right rear corner 1.6 m
        // left of lane 1's centerline. B, in lane 1 behind it, clears that by
        // 0.65 m and passes; A, wider, behind B, wouldn't clear it by enough.
        let mut sim = two_lanes();
        let c = sim.insert(car(), at(1, 60.0, 0.0));
        sim.change_lane(c, Side::Left).unwrap();
        let lane_1 = sim.state.network.lane(LaneId(1));
        let pos = lane_1.geometry().lane_to_world(lane_1.geometry().lane_pos(60.0, 2.55));
        let lane_0 = sim.state.network.lane(LaneId(0));
        let on_0 = lane_0.geometry().world_to_lane_near(pos, lane_0.geometry().lane_pos(60.0, 0.0));
        let v = &mut sim.state.vehicles[c];
        (v.pos, v.heading, v.lane_pos) = (pos, 0.0, on_0);
        sim.update_occupancy();
        let record = sim.state.occupants[1].iter().find(|o| o.id == c).unwrap();
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
        let queued = sim.insert(car(), at(1, 5.0, 0.0));
        sim.hold(queued);
        let b = sim.insert(car(), at(0, 80.0, 10.0));
        let path = &mut sim.state.vehicles[b].path;
        path.clear();
        path.extend([LaneId(0), LaneId(2)]);
        let a = sim.insert(car(), at(0, 50.0, 10.0));
        let path = &mut sim.state.vehicles[a].path;
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

    #[test]
    fn a_leader_going_the_same_way_shields_the_lanes_beyond() {
        // B carries on from lane 0 onto lane 1, as A will: what is on lane 1
        // reaches A through B.
        let mut sim = Simulation::new(road(&[(0.0, 100.0), (100.0, 200.0)], 1), NoopSink);
        let a = sim.insert(car(), at(0, 50.0, 10.0));
        let b = sim.insert(car(), at(0, 80.0, 10.0));
        let stopped = sim.insert(car(), at(1, 20.0, 0.0));
        sim.hold(stopped);
        assert_eq!(sim.state.vehicles[b].path[1], LaneId(1));
        assert_eq!(leaders(&sim, a), [b]);
    }

    #[test]
    fn a_ghost_beyond_the_leader_is_still_found_and_shields_nothing() {
        // C, in lane 1, indicates into lane 0 beyond B. B shields the in-lane
        // vehicles beyond it, but not a ghost, which only partly blocks the lane.
        let mut sim = two_lanes();
        let a = sim.insert(car(), at(0, 20.0, 10.0));
        let b = sim.insert(car(), at(0, 40.0, 10.0));
        let c = sim.insert(car(), at(1, 60.0, 10.0));
        sim.indicate(c, Side::Left);
        assert_eq!(ahead(&sim, a), [Ahead::Vehicle(b), Ahead::Ghost(c)]);
    }

    #[test]
    fn a_ghost_doesnt_shield_the_lane_beyond_it() {
        // C's ghost is A's nearest record, but only partly blocks the lane, so B
        // beyond it is still a leader.
        let mut sim = two_lanes();
        let a = sim.insert(car(), at(0, 20.0, 10.0));
        let c = sim.insert(car(), at(1, 40.0, 10.0));
        sim.indicate(c, Side::Left);
        let b = sim.insert(car(), at(0, 60.0, 10.0));
        assert_eq!(ahead(&sim, a), [Ahead::Ghost(c), Ahead::Vehicle(b)]);
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
    /// at 10 m/s, knowing only the first `known` of them, after a longitudinal
    /// update: the gap to the end of its path, if it reacted to one, and that term.
    fn path_end_term(sections: usize, known: usize) -> Option<(f64, f64)> {
        let sections: Vec<_> =
            (0..sections).map(|k| (20.0 * k as f64, 20.0 * (k + 1) as f64)).collect();
        let mut sim = Simulation::new(road(&sections, 1), Terms::default());
        let id = sim.insert(car(), at(0, 5.0, 10.0));
        // As a routing model would leave it, not knowing where it goes next.
        sim.state.vehicles[id].path.truncate(known);
        sim.update_accels();
        sim.sink.0.iter().find_map(|&(_, source, accel)| match source {
            AccelSource::PathEnd { gap } => Some((gap, accel)),
            _ => None,
        })
    }

    #[test]
    fn the_end_of_a_known_path_is_a_stop_line() {
        // Its path holds the first three sections, and the road goes on beyond: it
        // doesn't know where, so it is ready to stop at 60 m, its front at 8.6 m.
        let (gap, accel) = path_end_term(6, 3).expect("no stop line at the path's end");
        assert!((gap - 51.4).abs() < 1e-9, "gap {gap}");
        assert_eq!(accel, acceleration::interaction(&car(), 10.0, gap, 0.0));
    }

    #[test]
    fn a_path_ending_where_the_network_does_is_an_exit() {
        // The same, but the road ends with the path: it leaves there.
        assert_eq!(path_end_term(3, 3), None);
    }

    // Paths over short lanes.

    /// One lane in `count` sections, each `length` long.
    fn short_sections(length: f64, count: usize) -> SimNetwork {
        let sections: Vec<_> =
            (0..count).map(|k| (length * k as f64, length * (k + 1) as f64)).collect();
        road(&sections, 1)
    }

    /// Whether a vehicle's path reaches as far as its leader search looks.
    fn covers_horizon(sim: &Simulation<impl DebugSink>, id: VehicleId) -> bool {
        let v = &sim.state.vehicles[id];
        reach_beyond_front(&sim.state.network, v) >= acceleration::horizon(&v.spec, v.speed)
    }

    #[test]
    fn a_path_reaches_past_the_horizon_however_short_the_lanes() {
        // 2 m lanes: a car held stopped 30 lanes on, its rear 55.4 m ahead, is in
        // view, and there is no end of the path short of the horizon.
        let mut sim = Simulation::new(short_sections(2.0, 200), Terms::default());
        let id = sim.insert(car(), at(0, 1.0, LIMIT));
        let stopped = sim.insert(car(), at(30, 1.0, 0.0));
        sim.hold(stopped);
        assert!(covers_horizon(&sim, id));
        assert!(
            sim.state.vehicles[id].path.len() > 40,
            "{}",
            sim.state.vehicles[id].path.len()
        );
        sim.update_accels();
        let terms: Vec<_> = sim.sink.0.iter().filter(|t| t.0 == id).map(|t| t.1).collect();
        assert!(
            terms
                .iter()
                .any(|s| matches!(s, AccelSource::Leader { id, .. } if *id == stopped)),
            "{terms:?}"
        );
        assert!(
            !terms.iter().any(|s| matches!(s, AccelSource::PathEnd { .. })),
            "{terms:?}"
        );
        // Driving on, it keeps knowing as far ahead.
        for _ in 0..60 {
            sim.update_motion(1.0 / 60.0);
            sim.update_occupancy();
            assert!(covers_horizon(&sim, id));
        }
    }

    #[test]
    fn a_vehicle_crosses_several_short_lanes_in_one_substep() {
        // 0.1 m lanes, at 14 m/s: about 0.23 m a substep.
        let mut sim = Simulation::new(short_sections(0.1, 2000), NoopSink);
        let id = sim.insert(car(), at(0, 0.05, LIMIT));
        sim.update_motion(1.0 / 60.0);
        let v = &sim.state.vehicles[id];
        assert_eq!(v.path[0], LaneId(2), "rear axle at {:?}", v.lane_pos);
        assert!((0.0..0.1).contains(&v.lane_pos.s), "{:?}", v.lane_pos);
        assert!(covers_horizon(&sim, id));
    }

    #[test]
    fn the_follower_search_goes_back_as_far_as_paths_reach() {
        // 5 m lanes: the follower's front is 41.4 m back, eight lanes before.
        let mut sim = Simulation::new(short_sections(5.0, 40), NoopSink);
        let behind = sim.insert(car(), at(0, 1.0, LIMIT));
        let (found, gap) = follower(&sim, 9, 1.0, VehicleId::null()).unwrap();
        assert_eq!(found, behind);
        assert!((gap - 41.4).abs() < 1e-9, "gap {gap}");
    }
}
