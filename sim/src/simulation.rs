mod accel;
mod state;
mod step;

use accel::AccelUpdate;
pub(crate) use state::Derived;
pub use state::{Config, SimState};

use std::ops::ControlFlow;

use glam::DVec2;
use slotmap::{Key, SlotMap};

use crate::debug::{DebugEvent, DebugSink};
use crate::following::{self, Ahead, Along, Found, Lateral, Occupant, find_leaders};
use crate::junction::{Entry, Queue, arrive};
use crate::models::acceleration;
use crate::models::lane_change;
use crate::models::routing::Route;
use crate::models::signal::{self, MovementLight};
use crate::motion::lateral::plan_change;
use crate::motion::path::{extend_path, raise_through, reach};
use crate::network::{JunctionId, LaneId, MovementId, Side, SimNetwork};
use crate::query::Prospect;
use crate::query::find_follower;
use crate::query::{LaneSpeeds, speed_of};
use crate::vehicle::{HeldUp, LaneChangeIntent, Placement, Vehicle, VehicleId, VehicleSpec};

/// The traffic simulation.
pub struct Simulation {
    state: SimState,
    derived: Derived,
    /// Scratch space for the longitudinal update's decisions, reused between
    /// updates.
    decisions: Vec<AccelUpdate>,
    /// Scratch space for the longitudinal update: each vehicle held up by a
    /// leader, by that leader, before the leader keeps the one it holds up most.
    held: Vec<(VehicleId, HeldUp)>,
    /// Scratch space for vehicles whose path grew or that entered a junction in
    /// a step, which may arrive at junctions; reused between steps.
    arrivals: Vec<VehicleId>,
    /// Scratch space for a junction's scans: the lanes they found no one on, or
    /// beyond, and those they found someone on, or beyond. Reused.
    scan_empty: Vec<LaneId>,
    scan_kept: Vec<LaneId>,
    /// Scratch space for vehicles that lost a record on a lane they had left.
    departed: Vec<VehicleId>,
    /// Scratch space for lane-change decisions, reused between steps: each
    /// vehicle's lane-change state afterwards, and what it decided.
    lane_decisions: Vec<lane_change::Decided>,
    /// Scratch space: the gaps asked for in a step, by the driver's place in
    /// `lane_decisions`.
    asks: Vec<(usize, Side)>,
    /// Scratch space: each lane's speeds, for averages over stretches ahead.
    lane_speeds: Vec<LaneSpeeds>,
    /// Scratch space: what a lane change would do, for each vehicle whose
    /// intent asks, sorted by vehicle, measured at a lane-change update for the
    /// lane-change model to judge then, and for its terms at the longitudinal
    /// update after.
    prospects: Vec<(VehicleId, Prospect)>,
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

impl Simulation {
    /// Creates a simulation of `network` with default configuration.
    pub fn new(network: SimNetwork) -> Self {
        let config = Config { dt: 1.0 / 30.0, macro_step: 3 };
        Self::with_config(network, config)
    }

    /// Creates a simulation of `network`.
    pub fn with_config(network: SimNetwork, config: Config) -> Self {
        let lane_count = network.lanes().count();
        let junction_count = network.junctions().count();
        let state = SimState {
            network,
            vehicles: SlotMap::with_key(),
            longitudinal: Default::default(),
            lane_changing: Default::default(),
            routing: Default::default(),
            occupants: vec![Vec::new(); lane_count],
            queues: vec![Queue::default(); junction_count],
            caps: Vec::new(),
            removals: Vec::new(),
            config,
            frame: 0,
            residual: 0.0,
            signals: Vec::new(),
            lights: Vec::new(),
        };
        Self::restore(state)
    }

    /// The current state of the simulation, which can be serialized and restored.
    pub fn state(&self) -> &SimState {
        &self.state
    }

    /// A simulation exactly as it was in `state`.
    pub fn restore(mut state: SimState) -> Self {
        assert!(
            state.config.dt > 0.0 && state.config.macro_step > 0,
            "{:?}",
            state.config
        );
        // What a saved network leaves out, worked out again.
        state.network.derive_conflict_starts();
        // A freeze frame from before its network had signals starts them afresh.
        let signals = state.network.signals().count();
        if state.signals.len() != signals || state.lights.len() != state.network.movement_count() {
            (state.signals, state.lights) = signal::initial(&state.network);
        }
        Self {
            derived: Derived::new(&state),
            state,
            // Scratch space, cleared before every use.
            decisions: Vec::new(),
            held: Vec::new(),
            arrivals: Vec::new(),
            scan_empty: Vec::new(),
            scan_kept: Vec::new(),
            departed: Vec::new(),
            lane_decisions: Vec::new(),
            asks: Vec::new(),
            lane_speeds: Vec::new(),
            prospects: Vec::new(),
        }
    }

    /// A signal movement's light now, and for how many steps it has shown it.
    pub fn light(&self, movement: MovementId) -> MovementLight {
        self.state.lights[movement.0 as usize]
    }

    /// The network being simulated.
    pub fn network(&self) -> &SimNetwork {
        &self.state.network
    }

    /// Simulated time since the simulation was created, in seconds.
    pub fn time(&self) -> f64 {
        self.state.time()
    }

    /// Inserts a vehicle on the centerline of a lane, facing along it, without
    /// checking for room. See [`Simulation::try_insert`] for a checked insert.
    ///
    /// It starts with zero acceleration, which it holds until the next longitudinal model update.
    pub fn insert(&mut self, spec: VehicleSpec, at: Placement) -> VehicleId {
        let built = self.build_vehicle(spec, at);
        self.register(built)
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
        let (mut vehicle, route, seed) = self.build_vehicle(spec, at);
        // Leader and follower split the lane at its rear, so any vehicle alongside
        // is one or the other, with no gap, and refuses the insertion.
        let rear = vehicle.lane_pos.s - spec.rear_axle;
        // No faster than the nearest leader, and braking gently enough for all,
        // ghosts and the stop line at the end of its path included. The search
        // goes nearest first, so the speed is capped before anything is checked,
        // and it stops at the first that refuses.
        let mut speed = vehicle.speed;
        let mut nearest = true;
        find_leaders(&self.state, &vehicle, Along::Inserting, 0.0, |found| {
            let Found::Ahead(ahead, gap, _) = found else {
                return ControlFlow::Continue(());
            };
            let leader_speed = speed_of(&self.state.vehicles, ahead);
            if nearest && !matches!(ahead, Ahead::PathEnd | Ahead::Barrier { .. }) {
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
        let follower = find_follower(
            &self.state,
            &self.derived.through,
            at.lane,
            rear,
            VehicleId::null(),
        );
        if let Some((follower, gap)) = follower {
            let f = &self.state.vehicles[follower];
            let accel = acceleration::interaction(&f.spec, f.speed, gap, vehicle.speed);
            if gap <= 0.0 || accel < -acceleration::SAFE_BRAKING {
                return Err(InsertError::NoRoom);
            }
        }

        Ok(self.register((vehicle, route, seed)))
    }

    /// A vehicle placed on a lane's centerline, facing along it, not yet in the simulation.
    fn build_vehicle(&self, spec: VehicleSpec, at: Placement) -> (Vehicle, Route, u64) {
        let lane = self.state.network.lane(at.lane);
        let lane_pos = lane.geometry().lane_pos(at.s, 0.0);
        let mut vehicle = Vehicle {
            spec,
            pos: lane.geometry().lane_to_world(lane_pos),
            dir: lane.geometry().tangent_at(lane_pos),
            speed: at.speed,
            accel: 0.0,
            curvature: 0.0,
            front_s: at.s + spec.wheelbase(),
            path: vec![at.lane],
            lane_pos,
            old_side: None,
            lane_change: None,
            intent: LaneChangeIntent::None,
            committed: 0,
            stopped_at: None,
            held_up: None,
        };
        // Its own generator, from its route's seed and where it starts, so
        // drivers don't wander in step.
        let seed = match at.route {
            Route::Random { state } => {
                state ^ (at.s.to_bits()).rotate_left(17) ^ u64::from(at.lane.0)
            }
        };
        let needed = reach(&vehicle, at.s, None);
        let mut route = at.route;
        extend_path(
            &self.state.network,
            &mut vehicle.path,
            &mut route,
            at.s,
            needed,
        );
        (vehicle, route, seed)
    }

    /// Adds a built vehicle to the simulation and to its lane's occupants.
    fn register(&mut self, built: (Vehicle, Route, u64)) -> VehicleId {
        let (vehicle, route, seed) = built;
        raise_through(
            &self.state.network,
            &mut self.derived.through,
            &vehicle.path,
        );
        let lane_id = vehicle.path[0];
        let lane_pos = vehicle.lane_pos;
        let id = self.state.vehicles.insert(vehicle);
        self.state.longitudinal.insert(id, &self.state.vehicles[id].spec, seed);
        self.state.lane_changing.insert(id);
        self.state.routing.insert(id, route);
        let occupant = Occupant::new(
            &self.state.network,
            lane_id,
            id,
            &self.state.vehicles[id],
            lane_pos,
            Lateral::InLane,
        );
        // Inserted with room to spare, not cutting in: settled.
        let occupant = Occupant { age: acceleration::SETTLED, ..occupant };
        let list = &mut self.state.occupants[lane_id.0 as usize];
        let index = list.partition_point(|o| o.s() <= occupant.s());
        list.insert(index, occupant);
        let SimState { network, vehicles, occupants, queues, .. } = &mut self.state;
        let overlaps = &self.derived.overlaps;
        following::join_overlaps(network, overlaps, occupants, id, &vehicles[id], lane_id);
        arrive(network, occupants, queues, id, &vehicles[id]);
        id
    }

    /// Queues a vehicle for removal, which the next step carries out first, as
    /// it removes vehicles that leave the network. Until then it is still in
    /// the simulation. Returns whether there is such a vehicle.
    pub fn remove(&mut self, id: VehicleId) -> bool {
        let exists = self.state.vehicles.contains_key(id);
        if exists && !self.state.removals.contains(&id) {
            self.state.removals.push(id);
        }
        exists
    }

    /// Removes the vehicles queued for removal (see [`Simulation::remove`]),
    /// each with a [`DebugEvent::VehicleRemoved`].
    fn apply_removals<S: DebugSink>(&mut self, sink: &mut S) {
        let mut removals = std::mem::take(&mut self.state.removals);
        for &id in &removals {
            self.remove_now(id, sink);
        }
        removals.clear();
        self.state.removals = removals;
    }

    /// Removes a vehicle now, and every record of it. Scans every lane's
    /// occupants, so it is not for hot paths.
    fn remove_now<S: DebugSink>(&mut self, id: VehicleId, sink: &mut S) {
        if !self.state.vehicles.contains_key(id) {
            return;
        }
        for list in &mut self.state.occupants {
            list.retain(|o| o.id != id);
        }
        for queue in &mut self.state.queues {
            queue.committed.retain(|e| e.id != id);
            queue.fronts.retain(|&q| q != id);
            queue.held.retain(|&q| q != id);
        }
        self.state.caps.retain(|&(c, _)| c != id);
        self.state.longitudinal.remove(id);
        self.state.lane_changing.remove(id);
        self.state.routing.remove(id);
        sink.record(|| DebugEvent::VehicleRemoved { vehicle: id });
        self.state.vehicles.remove(id);
    }

    /// Whether a vehicle is finishing a stop, or staying at rest until it can
    /// move off properly.
    pub fn stopping(&self, id: VehicleId) -> bool {
        self.state.vehicles.contains_key(id) && self.state.longitudinal.stopping(id)
    }

    /// How far short of the barrier it is heading for a vehicle will stop, in
    /// meters.
    pub fn line_gap(&self, id: VehicleId) -> Option<f64> {
        self.state
            .vehicles
            .contains_key(id)
            .then(|| self.state.longitudinal.line_gap(id))
    }

    /// How a vehicle chooses its way at forks, as it is now.
    pub fn route(&self, id: VehicleId) -> Option<Route> {
        self.state.routing.get(id)
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

    /// The vehicles committed at a junction, in the order they committed: an
    /// entry for each stretch of its way a vehicle was handed over for.
    pub fn queue(&self, junction: JunctionId) -> &[Entry] {
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
        match self.state.caps.iter_mut().find(|(c, _)| *c == id) {
            Some(cap) => cap.1 = max_speed.max(0.0),
            None => self.state.caps.push((id, max_speed.max(0.0))),
        }
    }

    /// The maximum speed imposed on a vehicle, if any.
    pub fn speed_cap(&self, id: VehicleId) -> Option<f64> {
        self.state.caps.iter().find(|(c, _)| *c == id).map(|&(_, max)| max)
    }

    /// Makes a vehicle brake to a stop and wait, until [`Simulation::release`]: a
    /// speed cap of 0.
    pub fn hold(&mut self, id: VehicleId) {
        self.cap_speed(id, 0.0);
    }

    /// Lifts any speed cap on a vehicle, held or not.
    pub fn release(&mut self, id: VehicleId) {
        self.state.caps.retain(|&(c, _)| c != id);
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
        let indicating = self.state.vehicles.get(id).and_then(|v| v.intent.indicating());
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
        // The front axle carries on from where it is, at the cubic's start.
        vehicle.front_s = change.s0;
        // It keeps its place in the queues it is in: those of junctions no longer
        // on its path drop it once it is clear of them, and its new path may make
        // it arrive at others.
        vehicle.path.clear();
        vehicle.path.push(neighbor.lane);
        vehicle.lane_pos = lane_pos;
        vehicle.old_side = Some(side.opposite());
        vehicle.lane_change = Some(change);
        vehicle.intent = LaneChangeIntent::None;
        // Its barriers ahead are now others.
        vehicle.committed = 0;
        vehicle.stopped_at = None;
        self.state.longitudinal.new_barrier(id, &vehicle.spec);
        let needed = reach(vehicle, lane_pos.s, Some(change));
        let route = self.state.routing.route_mut(id);
        extend_path(
            &self.state.network,
            &mut vehicle.path,
            route,
            lane_pos.s,
            needed,
        );
        raise_through(
            &self.state.network,
            &mut self.derived.through,
            &vehicle.path,
        );

        let list = &mut self.state.occupants[neighbor.lane.0 as usize];
        if let Some(ghost) = list.iter_mut().find(|o| o.id == id && o.is_ghost()) {
            ghost.lateral = Lateral::CONVERGING;
            ghost.age = 0.0;
        } else {
            let occupant = Occupant::new(
                &self.state.network,
                neighbor.lane,
                id,
                &self.state.vehicles[id],
                lane_pos,
                Lateral::CONVERGING,
            );
            let index = list.partition_point(|o| o.s() <= occupant.s());
            list.insert(index, occupant);
        }
        let SimState { network, vehicles, occupants, queues, .. } = &mut self.state;
        let overlaps = &self.derived.overlaps;
        let vehicle = &vehicles[id];
        // Its course has changed, so every other record it has is classified
        // afresh, set here rather than left to the next occupancy update, so
        // searches before then see it.
        following::reclassify(network, overlaps, occupants, id, vehicle, old_lane, false);
        following::join_overlaps(network, overlaps, occupants, id, vehicle, neighbor.lane);
        arrive(network, occupants, queues, id, vehicle);
        Ok(())
    }

    /// Starts a vehicle indicating a change to `side`: it gets a ghost record in
    /// the target lane's occupants.
    pub(crate) fn indicate(&mut self, id: VehicleId, side: Side) {
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
        let index = list.partition_point(|o| o.s() <= ghost.s());
        list.insert(index, ghost);
        // Its gap measured at its whole time headway, until the lane-change model
        // asks for the share it judges at.
        let since = self.time();
        let check = Some(1.0);
        self.state.vehicles[id].intent = LaneChangeIntent::Indicating { side, since, check };
    }

    /// Stops a vehicle indicating, removing its ghost.
    pub(crate) fn cancel_intent(&mut self, id: VehicleId) {
        let Some(vehicle) = self.state.vehicles.get_mut(id) else {
            return;
        };
        let Some(side) = vehicle.intent.indicating() else {
            return;
        };
        vehicle.intent = LaneChangeIntent::None;
        if let Some(neighbor) = self.state.network.lane(vehicle.path[0]).adj(side) {
            following::remove_ghost(&mut self.state.occupants, neighbor.lane, id);
        }
    }

    /// How the simulation steps.
    pub fn config(&self) -> Config {
        self.state.config
    }
}

#[cfg(test)]
pub(crate) mod test;
