use std::ops::ControlFlow;

use glam::DVec2;
use slotmap::{Key, SlotMap};

use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink, NoopSink};
use crate::following::{self, Overlaps};
use crate::junction::{self, Entry, Queue, arrive, junction_term};
use crate::kinematics;
use crate::lane_changing::{
    LaneSpeeds, check_gap, comfortable_stop, friction, halfway_from_rest, incentives, plan_change,
    wants_change,
};
use crate::math::wrap_angle;
use crate::merge::{self, Merges};
use crate::models::acceleration;
use crate::models::lane_change::{self, Decision};
use crate::models::lateral;
use crate::models::routing::Route;
use crate::models::signal::{self, MovementLight, SignalState};
use crate::network::{
    BarrierKind, JunctionId, LaneId, MAX_SUCCESSORS, MovementId, Side, SimNetwork,
};
use crate::occupancy::{
    self, Ahead, Along, Found, Lateral, Occupant, find_follower, find_leaders, for_each_leader,
};
use crate::path::{Through, extend_path, raise_through, reach, recompute_through};
use crate::vehicle::{Intent, OldFollower, Placement, Vehicle, VehicleId, VehicleSpec};

/// How a simulation steps, fixed when it is made.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Config {
    /// The length of a step, in seconds.
    pub step: f64,
    /// How many steps between longitudinal model updates: every third, at 20 Hz
    /// with 60 Hz steps.
    pub accel_every: u32,
    /// The most gaps for lane changes checked at once, a backstop: drivers'
    /// checks are spread out by their own cooldowns, and any beyond this wait
    /// for the next time.
    #[cfg_attr(feature = "serde", serde(default = "max_gap_checks"))]
    pub max_gap_checks: usize,
}

#[cfg(feature = "serde")]
fn max_gap_checks() -> usize {
    Config::default().max_gap_checks
}

impl Default for Config {
    fn default() -> Self {
        Self { step: 1.0 / 60.0, accel_every: 3, max_gap_checks: 64 }
    }
}

/// A simulation's whole state at one instant, network included, without its
/// debug sink or anything derived from the rest: a freeze frame.
/// [`Simulation::state`] lends it, and [`Simulation::restore`] turns it back
/// into a simulation, exactly as it was. With the `serde` feature, it
/// serializes, for saving to a file.
///
/// The searches read it, kept apart from the rest of the simulation, so that
/// an update can lend it to them while writing elsewhere, such as to the debug
/// sink.
///
/// A restored simulation continues bit for bit like the original, through steps,
/// holds, lane changes and removals, until it inserts a vehicle into a slot freed
/// before the snapshot. The vehicle store's free slots come back in a different
/// order, so that vehicle can get a different id from the one it would have had,
/// and the update order can differ from then on.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimState {
    pub(crate) network: SimNetwork,
    pub(crate) vehicles: SlotMap<VehicleId, Vehicle>,
    /// Each lane's occupants sorted by `s`, indexed by lane.
    pub(crate) occupants: Vec<Vec<Occupant>>,
    /// Each junction's queue, by junction.
    pub(crate) queues: Vec<Queue>,
    /// Vehicles with a speed imposed on them, and the most each may go, in m/s: 0
    /// holds one to a stop. Few, so a list beats a field on every vehicle.
    pub(crate) caps: Vec<(VehicleId, f64)>,
    /// How it steps.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) config: Config,
    /// Steps taken: every [`Config::accel_every`]th updates the longitudinal model.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) steps: u64,
    /// Time [`Simulation::step_variable`] was given but hasn't stepped yet, in
    /// seconds: less than a step.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) residual: f64,
    /// Simulated time, in seconds.
    pub(crate) time: f64,
    /// Each signal controller's place in its plan, by controller.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) signals: Vec<SignalState>,
    /// Every signal movement's light, by [`MovementId`].
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) lights: Vec<MovementLight>,
}

/// What the simulation derives from its state, rebuilt rather than saved.
pub(crate) struct Derived {
    /// Each lane's overlapping lanes outside junctions, from the network.
    pub overlaps: Overlaps,
    /// Which merge each lane belongs to, from the network and the overlaps.
    pub merges: Merges,
    /// How far beyond each lane's end the paths through it reach (see
    /// [`Through`]).
    pub through: Through,
}

impl Derived {
    fn new(state: &SimState) -> Self {
        let network = &state.network;
        let overlaps = following::overlaps(network);
        let merges = merge::merges(network, &overlaps);
        let mut through = Through::new();
        let paths = state.vehicles.values().map(|v| v.path.as_slice());
        recompute_through(network, &mut through, paths);
        Self { overlaps, merges, through }
    }
}

/// The traffic simulation.
pub struct Simulation<D: DebugSink = NoopSink> {
    state: SimState,
    derived: Derived,
    /// Scratch space for the longitudinal update's decisions, reused between
    /// updates.
    decisions: Vec<AccelUpdate>,
    /// Scratch space for the longitudinal update: each vehicle held up by a
    /// leader, by that leader, before the leader keeps the one it holds up most.
    held: Vec<(VehicleId, OldFollower)>,
    /// Scratch space for vehicles leaving the network, reused between steps.
    leavers: Vec<VehicleId>,
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
    lane_decisions: Vec<(VehicleId, lane_change::State, Decision)>,
    /// Scratch space: the gaps asked for in a step, by the driver's place in
    /// `lane_decisions`.
    asks: Vec<(usize, Side)>,
    /// Scratch space: each lane's speeds, for averages over stretches ahead.
    lane_speeds: Vec<LaneSpeeds>,
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
        /// How many barriers ahead it is committed to now.
        committed: u8,
        /// Whether it has just come to rest at the all-way stop line it heads for.
        arrived: bool,
    },
    /// From its record on its old lane, while it straddles it: the most its old
    /// side allows.
    OldSide { id: VehicleId, accel: f64 },
    /// From a junction whose queue it is in: the most the committed vehicles
    /// ahead of it there allow.
    Junction { id: VehicleId, accel: f64 },
    /// From a merge it is approaching: the most the vehicle before it in the
    /// merge's order allows.
    Merge { id: VehicleId, accel: f64 },
}

/// The leader whose term was the lowest in a vehicle's search along its path.
#[derive(Clone, Copy, Debug)]
struct Binding {
    leader: VehicleId,
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

impl<D: DebugSink> Simulation<D> {
    /// The whole state, network included: a freeze frame, to clone or serialize.
    pub fn state(&self) -> &SimState {
        &self.state
    }

    /// A simulation exactly as it was in `state`, with `sink` for its debug
    /// events. The sink starts empty, so its events refill as each update runs.
    pub fn restore(mut state: SimState, sink: D) -> Self {
        assert!(
            state.config.step > 0.0 && state.config.accel_every > 0,
            "{:?}",
            state.config
        );
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
            leavers: Vec::new(),
            arrivals: Vec::new(),
            scan_empty: Vec::new(),
            scan_kept: Vec::new(),
            departed: Vec::new(),
            lane_decisions: Vec::new(),
            asks: Vec::new(),
            lane_speeds: Vec::new(),
            sink,
        }
    }

    /// A simulation of `network`, stepping by [`Config::default`].
    pub fn new(network: SimNetwork, sink: D) -> Self {
        Self::with_config(network, sink, Config::default())
    }

    /// A simulation of `network`, stepping as `config` says.
    pub fn with_config(network: SimNetwork, sink: D, config: Config) -> Self {
        let lane_count = network.lanes().count();
        let junction_count = network.junctions().count();
        let state = SimState {
            network,
            vehicles: SlotMap::with_key(),
            occupants: vec![Vec::new(); lane_count],
            queues: vec![Queue::default(); junction_count],
            caps: Vec::new(),
            config,
            steps: 0,
            residual: 0.0,
            time: 0.0,
            signals: Vec::new(),
            lights: Vec::new(),
        };
        Self::restore(state, sink)
    }

    /// A signal movement's light now, and for how many steps it has shown it.
    pub fn light(&self, movement: MovementId) -> MovementLight {
        self.state.lights[movement.0 as usize]
    }

    pub fn network(&self) -> &SimNetwork {
        &self.state.network
    }

    /// Simulated time since the simulation was created, in seconds.
    pub fn time(&self) -> f64 {
        self.state.time
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
            noise: 0.0,
            // Its own, from its route's seed and where it starts, so drivers
            // don't wander in step.
            noise_rng: match at.route {
                Route::Random { state } => {
                    state ^ (at.s.to_bits()).rotate_left(17) ^ u64::from(at.lane.0)
                }
            },
            committed: 0,
            stopping: false,
            line_gap: spec.line_gap,
            stopped_at: None,
        };
        barrier::draw_line_gap(&mut vehicle);
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
        raise_through(
            &self.state.network,
            &mut self.derived.through,
            &vehicle.path,
        );
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
        // Inserted with room to spare, not cutting in: settled.
        let occupant = Occupant { age: acceleration::SETTLED, ..occupant };
        let list = &mut self.state.occupants[lane_id.0 as usize];
        let index = list.partition_point(|o| o.s() <= occupant.s());
        list.insert(index, occupant);
        let SimState { network, vehicles, occupants, queues, .. } = &mut self.state;
        let overlaps = &self.derived.overlaps;
        join_overlaps(network, overlaps, occupants, id, &vehicles[id], lane_id);
        arrive(network, occupants, queues, id, &vehicles[id]);
        id
    }

    /// Removes a vehicle. Scans every lane's occupants, so it is not for hot paths.
    pub fn remove(&mut self, id: VehicleId) -> Option<Vehicle> {
        for list in &mut self.state.occupants {
            list.retain(|o| o.id != id);
        }
        for queue in &mut self.state.queues {
            queue.committed.retain(|e| e.id != id);
            queue.fronts.retain(|&q| q != id);
        }
        self.state.caps.retain(|&(c, _)| c != id);
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
        // Its barriers ahead are now others.
        vehicle.committed = 0;
        vehicle.stopped_at = None;
        barrier::draw_line_gap(vehicle);
        let needed = reach(vehicle, lane_pos.s, Some(change));
        extend_path(
            &self.state.network,
            &mut vehicle.path,
            &mut vehicle.route,
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
        reclassify(network, overlaps, occupants, id, vehicle, old_lane, false);
        join_overlaps(network, overlaps, occupants, id, vehicle, neighbor.lane);
        arrive(network, occupants, queues, id, vehicle);
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
        let index = list.partition_point(|o| o.s() <= ghost.s());
        list.insert(index, ghost);
        self.state.vehicles[id].intent = Some(Intent { side, since: self.state.time });
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

    /// How the simulation steps.
    pub fn config(&self) -> Config {
        self.state.config
    }

    /// Advances the simulation by `dt` seconds of wall time, a frame's: as many
    /// whole steps as fit, with what is left over carried to the next call.
    /// Returns how many steps it took.
    pub fn step_variable(&mut self, dt: f64) -> u32 {
        self.state.residual += dt;
        let step = self.state.config.step;
        let mut taken = 0;
        // A step's worth, give or take rounding, so frames of exactly a step
        // take exactly one.
        while self.state.residual >= step - 1e-9 * step {
            self.state.residual -= step;
            self.step();
            taken += 1;
        }
        self.state.residual = self.state.residual.max(0.0);
        taken
    }

    /// How far into the next step the time given to [`Simulation::step_variable`]
    /// has reached, from 0 to 1: for drawing vehicles between where they were
    /// before the last step and where they are now.
    pub fn alpha(&self) -> f64 {
        (self.state.residual / self.state.config.step).clamp(0.0, 1.0)
    }

    /// Advances the simulation by one step, [`Config::step`] seconds.
    pub fn step(&mut self) {
        let dt = self.state.config.step;
        self.state.time += dt;
        let SimState { network, signals, lights, .. } = &mut self.state;
        signal::step(network, signals, lights, dt);
        // As often as the longitudinal model, which they share lane speeds with.
        let every = self.state.config.accel_every;
        if self.state.steps.is_multiple_of(u64::from(every)) {
            self.update_lane_changes(dt * f64::from(every));
        }
        self.state.steps += 1;
        if self.state.steps.is_multiple_of(u64::from(self.state.config.accel_every)) {
            self.update_accels();
        }
        self.update_motion(dt);
        self.update_occupancy(dt);
        self.update_queues();
    }

    /// Lane changes, for every vehicle (see [`lane_change`]), `dt` seconds after
    /// the last: each driver's desires from its incentives, cheaply, and a
    /// check of the gap for those whose desire has reached their threshold, up
    /// to [`Config::max_gap_checks`] at once, the rest waiting for the next
    /// time. Decisions are computed from a read-only view of the state, then
    /// applied in order.
    fn update_lane_changes(&mut self, dt: f64) {
        let Self { state, derived, lane_decisions, asks, lane_speeds, sink, .. } = self;
        let time = state.time;
        // Exact again for the follower searches the gap checks make.
        let paths = state.vehicles.values().map(|v| v.path.as_slice());
        recompute_through(&state.network, &mut derived.through, paths);
        LaneSpeeds::refresh(lane_speeds, state);
        lane_decisions.clear();
        asks.clear();
        for (id, vehicle) in &state.vehicles {
            // Partway through a change, or still straddling, it doesn't decide.
            if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
                continue;
            }
            let mut lc = vehicle.lc;
            let incentives = incentives(state, lane_speeds, id, vehicle);
            lane_change::aim(&mut lc, incentives);
            // Nothing to weigh up: nowhere to change to, and nothing to fade.
            if lc.idle() && vehicle.intent.is_none() {
                continue;
            }
            let ask = lane_change::step(&mut lc, vehicle.intent, &vehicle.spec, time, dt);
            // How it weighed up each side it didn't ask about, for the debug view.
            sink.record(|| DebugEvent::RecomputeDecision { vehicle: id });
            for side in [Side::Left, Side::Right] {
                let Some(incentive) = incentives[lane_change::index(side)] else {
                    continue;
                };
                let outcome = match ask {
                    lane_change::Ask::Check(asked) if asked == side => continue,
                    lane_change::Ask::Check(_) => lane_change::Outcome::OtherSide,
                    lane_change::Ask::Cancel => lane_change::Outcome::Cancel,
                    lane_change::Ask::Nothing if vehicle.intent.is_some() => {
                        lane_change::Outcome::Wait
                    }
                    lane_change::Ask::Nothing => lane_change::Outcome::NotWorthIt,
                };
                let eval = lane_change::unchecked(side, incentive, lc.desire(side), outcome);
                sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
            }
            let decision = match ask {
                lane_change::Ask::Cancel => Decision::Cancel,
                lane_change::Ask::Check(side) => {
                    asks.push((lane_decisions.len(), side));
                    Decision::Nothing
                }
                lane_change::Ask::Nothing => Decision::Nothing,
            };
            lane_decisions.push((id, lc, decision));
        }
        // The gaps, up to the limit: drivers' own cooldowns spread them out, so
        // this is only a backstop.
        let checks = asks.len().min(state.config.max_gap_checks);
        for &(slot, side) in &asks[..checks] {
            let (id, mut lc, _) = lane_decisions[slot];
            let vehicle = &state.vehicles[id];
            // Measured again, for the debug view: checks are few.
            let incentive = incentives(state, lane_speeds, id, vehicle)[lane_change::index(side)]
                .unwrap_or_default();
            let desire = lc.desire(side);
            let prospect = check_gap(state, &derived.through, id, vehicle, side, desire);
            let (decision, eval) = lane_change::resolve(
                &mut lc,
                vehicle.intent,
                &vehicle.spec,
                side,
                incentive,
                prospect,
            );
            sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
            lane_decisions[slot] = (id, lc, decision);
        }
        // Those beyond the limit wait for the next time.
        for &(slot, side) in &asks[checks..] {
            let (id, lc, _) = lane_decisions[slot];
            sink.record(|| {
                let vehicle = &state.vehicles[id];
                let incentive = incentives(state, lane_speeds, id, vehicle)
                    [lane_change::index(side)]
                .unwrap_or_default();
                let waiting = lane_change::Outcome::Waiting;
                let eval = lane_change::unchecked(side, incentive, lc.desire(side), waiting);
                DebugEvent::LaneChangeEval { vehicle: id, eval }
            });
        }

        for i in 0..self.lane_decisions.len() {
            let (id, state, decision) = self.lane_decisions[i];
            self.state.vehicles[id].lc = state;
            match decision {
                Decision::Nothing => {}
                Decision::Indicate(side) => self.indicate(id, side),
                Decision::Cancel => self.cancel_intent(id),
                // Already checked: only permitted changes are asked about.
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
        let Self { state, derived, decisions, sink, lane_speeds, .. } = self;
        let SimState { network, vehicles, occupants, .. } = &*state;
        sink.record(|| DebugEvent::RecomputeAccels);
        decisions.clear();
        for (index, list) in occupants.iter().enumerate() {
            let lane = LaneId(index as u32);
            let length = network.lane(lane).geometry().length();
            for (i, o) in list.iter().enumerate().rev() {
                if o.is_ghost() {
                    continue;
                }
                let vehicle = &vehicles[o.id];
                if vehicle.path[0] == lane {
                    // On a loop, a vehicle can briefly have a second record on the
                    // lane it follows, lingering at its end, about a lane's length
                    // from its rear.
                    let rear = vehicle.lane_pos.s - vehicle.spec.rear_axle;
                    if !occupancy::same_record(length, o.s(), rear) {
                        continue;
                    }
                    decisions.push(longitudinal(
                        state,
                        &derived.through,
                        lane_speeds,
                        sink,
                        o.id,
                        vehicle,
                        i,
                    ));
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
        for m in 0..derived.merges.len() {
            merge_terms(state, &derived.merges, sink, m, decisions);
        }

        let noise_step = acceleration::NoiseStep::new(
            self.state.config.step * f64::from(self.state.config.accel_every),
        );
        // What its old side and its speed cap allow first, then its own update,
        // so that whether a leader binds it is judged against all of them.
        for update in &self.decisions {
            if let AccelUpdate::Own { id, yielding, committed, arrived, .. } = *update {
                let commits = committed.saturating_sub(self.state.vehicles[id].committed);
                let released = committed < self.state.vehicles[id].committed;
                let vehicle = &mut self.state.vehicles[id];
                // Heading for another barrier now, it draws how far short it stops;
                // past one it arrived at, it has yet to arrive at the next.
                if vehicle.committed != committed {
                    if committed > vehicle.committed {
                        vehicle.stopped_at = None;
                    }
                    vehicle.committed = committed;
                    barrier::draw_line_gap(vehicle);
                }
                if arrived {
                    vehicle.stopped_at = Some(self.state.steps);
                }
                // Committed past a barrier, it may now arrive at a junction beyond,
                // or be handed over further in one; released, handed back.
                if commits > 0 || released {
                    self.arrivals.push(id);
                }
                let spread = vehicle.spec.accel_noise;
                vehicle.noise =
                    acceleration::noise(vehicle.noise, spread, noise_step, &mut vehicle.noise_rng);
                vehicle.accel = f64::INFINITY;
                vehicle.lc.set_yielding(yielding);
                vehicle.old_follower = None;
            }
        }
        for update in &self.decisions {
            if let AccelUpdate::OldSide { id, accel }
            | AccelUpdate::Junction { id, accel }
            | AccelUpdate::Merge { id, accel } = *update
            {
                let vehicle = &mut self.state.vehicles[id];
                vehicle.accel = vehicle.accel.min(accel);
            }
        }
        for &(id, max_speed) in &self.state.caps {
            let vehicle = &mut self.state.vehicles[id];
            let term = acceleration::capped(vehicle.speed, max_speed);
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
                        let follower = OldFollower { id, without };
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
        // Slow and slowing, it finishes its stop, or stays put until it can
        // move off properly.
        for update in &self.decisions {
            if let AccelUpdate::Own { id, .. } = *update {
                let vehicle = &mut vehicles[id];
                let (speed, accel) = (vehicle.speed, vehicle.accel);
                vehicle.stopping = acceleration::stopping(vehicle.stopping, speed, accel);
                if let Some(term) = vehicle.stopping.then_some(-acceleration::STOPPING_DECEL) {
                    let source = AccelSource::FinishingStop;
                    self.sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                    vehicle.accel = vehicle.accel.min(term);
                }
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
            state: SimState { network, vehicles, occupants, .. },
            derived: Derived { overlaps, merges, through },
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
            // vehicle stopped during the step.
            let speed = (vehicle.speed + vehicle.accel * dt).max(0.0);
            let dist = 0.5 * (vehicle.speed + speed) * dt;
            vehicle.speed = speed;
            (vehicle.pos, vehicle.heading) =
                kinematics::advance(vehicle.pos, vehicle.heading, vehicle.curvature, dist);

            let lane = network.lane(vehicle.path[0]);
            // The barriers its rear axle passes, from where it was along the
            // lane it follows.
            let mut from = vehicle.lane_pos.s;
            let mut passed = 0;
            vehicle.lane_pos = lane.geometry().world_to_lane_near(vehicle.pos, vehicle.lane_pos);
            // Its path growing, or its entering a junction, may make it arrive at one.
            let mut arrives = false;
            while vehicle.path.len() > 1 {
                let length = network.lane(vehicle.path[0]).geometry().length();
                if vehicle.lane_pos.s < length {
                    break;
                }
                let previous = vehicle.path.remove(0);
                if vehicle.committed > 0 {
                    passed += barrier::between(network.lane(previous), from, f64::INFINITY);
                    from = f64::NEG_INFINITY;
                }
                let next = network.lane(vehicle.path[0]);
                arrives |= next.junction().is_some();
                let hint = next.geometry().lane_pos(vehicle.lane_pos.s - length, 0.0);
                vehicle.lane_pos = next.geometry().world_to_lane_near(vehicle.pos, hint);
                if let Some(change) = &mut vehicle.lane_change {
                    change.s0 -= length;
                }
                // Lanes can be shorter than a step's travel, so it may advance
                // again: it needs to know where it goes beyond this one.
                let s = vehicle.lane_pos.s;
                let needed = reach(vehicle, s, vehicle.lane_change);
                let lanes = vehicle.path.len();
                extend_path(network, &mut vehicle.path, &mut vehicle.route, s, needed);
                arrives |= vehicle.path.len() > lanes;
                // It joins the lane it now follows, and every lane overlapping
                // it, such as the other branches where lanes part and the other
                // lanes leading into the same one, each classified from its
                // course. Joins unsorted; `update_occupancy` sets its `s` and
                // sorts.
                let followed = vehicle.path[0];
                // Carried over from the lane before, unless it enters a merge,
                // where it joins another lane's traffic.
                let carried = match merges.of(followed) {
                    Some(m) if merges.of(previous) != Some(m) => 0.0,
                    _ => occupants[previous.0 as usize]
                        .iter()
                        .filter(|o| o.is_record_of(id))
                        .map(|o| o.age)
                        .fold(0.0, f64::max),
                };
                let join = |occupants: &mut [Vec<Occupant>], lane: LaneId| {
                    let geometry = network.lane(lane).geometry();
                    let hint = match lane == followed {
                        true => vehicle.lane_pos,
                        false => geometry.world_to_lane(vehicle.pos),
                    };
                    let mut occupant =
                        Occupant::new(network, lane, id, vehicle, hint, Lateral::InLane);
                    // Already there, as one lane it joins can overlap another,
                    // and, straddling into a merge, the lane its old lane leads
                    // to overlaps the one it follows: once is enough. If that is
                    // the lane it follows, the record there is its own now.
                    let list = &mut occupants[lane.0 as usize];
                    let length = geometry.length();
                    let near = |o: &Occupant| {
                        o.is_record_of(id) && occupancy::same_record(length, o.s(), occupant.s())
                    };
                    if let Some(existing) = list.iter_mut().find(|o| near(o)) {
                        if lane == followed {
                            existing.age = carried;
                        }
                        return;
                    }
                    following::classify(network, overlaps, vehicle, lane, &mut occupant);
                    if lane == followed {
                        occupant.age = carried;
                    }
                    occupants[lane.0 as usize].push(occupant);
                };
                join(occupants, followed);
                // Still straddling after a lane change: also join the lanes its old
                // lane leads to, so followers there keep seeing it.
                let old = vehicle.old_side.and_then(|side| network.lane(previous).adj(side));
                // Every lane it enters, it also joins those overlapping it.
                let mut entered = arrayvec::ArrayVec::<LaneId, { 1 + MAX_SUCCESSORS }>::new();
                entered.push(followed);
                if let Some(old) = old {
                    for &lane in network.lane(old.lane).successors() {
                        join(occupants, lane);
                        entered.push(lane);
                    }
                }
                for &lane in &entered {
                    for p in &overlaps[lane.0 as usize] {
                        join(occupants, p.lane);
                    }
                }
                // What it knows of its course has grown: records that were
                // converging for want of it are classified again.
                reclassify(network, overlaps, occupants, id, vehicle, previous, true);
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

            if vehicle.committed > 0 {
                passed += barrier::between(network.lane(vehicle.path[0]), from, vehicle.lane_pos.s);
                vehicle.committed = vehicle.committed.saturating_sub(passed.min(255) as u8);
                if passed > 0 {
                    barrier::draw_line_gap(vehicle);
                    vehicle.stopped_at = None;
                }
            }

            // Far enough ahead to know where it is going, at its new speed.
            let s = vehicle.lane_pos.s;
            let needed = reach(vehicle, s, vehicle.lane_change);
            let lanes = vehicle.path.len();
            extend_path(network, &mut vehicle.path, &mut vehicle.route, s, needed);
            arrives |= vehicle.path.len() > lanes;
            if arrives {
                raise_through(network, through, &vehicle.path);
            }
            if arrives {
                arrivals.push(id);
            }

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

    /// Brings the junctions' queues up to date after a step's motion and
    /// occupancy: vehicles whose path grew or that entered a junction arrive
    /// where nothing holds them back (see [`arrive`]), committed vehicles clear
    /// of a junction leave its queue, those that can plainly stop are released
    /// (see [`may_release`]), and each lane approached is scanned for vehicles
    /// to commit (see [`commit_along`]).
    fn update_queues(&mut self) {
        let Self { state, arrivals, scan_empty, scan_kept, .. } = self;
        junction::update(state, arrivals, scan_empty, scan_kept);
    }

    /// Refreshes the occupant lists, and clears the old side of vehicles that have
    /// finished leaving it.
    fn update_occupancy(&mut self, dt: f64) {
        let Self {
            state: SimState { network, vehicles, occupants, .. },
            departed,
            ..
        } = self;
        occupancy::update(network, vehicles, occupants, departed, dt);
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
                    occupants[old.lane.0 as usize].iter().any(|o| o.is_record_of(id))
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
    through: &[f64],
    speeds: &[LaneSpeeds],
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    own: usize,
) -> AccelUpdate {
    let lane = vehicle.path[0];
    let desired_speed = vehicle.spec.desired_speed(state.network.lane(lane).speed_limit());
    let desired_speed = desired_speed.min(friction(state, speeds, vehicle));
    // Drivers don't hold a speed exactly: their noise wanders it.
    let free = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed) + vehicle.noise;
    let source = AccelSource::FreeRoad { desired_speed };
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: free });
    // The lowest of every term but the binding leader's, and that leader.
    let mut rest = free;
    let mut binding: Option<Binding> = None;

    // The nearest ghost, if it comes before anything else ahead.
    let mut ghost = None;
    let mut first = true;
    // Wanting to change lanes, it keeps room to get halfway through a change
    // from rest.
    let halfway = wants_change(vehicle).then(|| halfway_from_rest(&state.network, vehicle));
    // How many barriers ahead it is committed to, in order: those it is released
    // from drop out, with all after them, and once one holds it, or one beyond is
    // found out of turn behind one shielded, the rest wait.
    let mut committed = vehicle.committed;
    let mut held = false;
    let mut arrived = false;
    for_each_leader(state, vehicle, Along::Path(own), |found| {
        if let Found::Ahead(Ahead::Barrier { lane, index, rank }, gap, _) = found {
            let barrier = state.network.lane(lane).barriers()[usize::from(index)];
            let gap = barrier::gap_to(vehicle, &barrier, gap);
            if rank < u16::from(committed) {
                if !barrier::released(state, vehicle, barrier, gap) {
                    return;
                }
                committed = rank as u8;
            }
            if held || rank != u16::from(committed) {
                held = true;
                return;
            }
            let k = vehicle.path.iter().position(|&l| l == lane).unwrap_or(0);
            let (decision, yields_to) =
                barrier::decide(state, through, id, vehicle, k, barrier, gap);
            // At rest as the head on its way to an all-way stop line: it has
            // arrived there.
            if barrier.kind == BarrierKind::AllWayStop
                && vehicle.speed == 0.0
                && vehicle.stopped_at.is_none()
            {
                arrived = true;
            }
            sink.record(|| DebugEvent::Barrier {
                vehicle: id,
                lane,
                index,
                gap,
                decision,
                yields_to,
            });
            match decision {
                barrier::Decision::Commit => committed = committed.saturating_add(1),
                barrier::Decision::Pass => held = true,
                barrier::Decision::Hold(term) => {
                    held = true;
                    let source = AccelSource::Barrier { lane, index, gap };
                    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                    rest = rest.min(term);
                }
            }
            return;
        }
        if let Found::Ahead(ahead, gap, _) = found {
            if let (true, Ahead::Ghost(owner)) = (first, ahead) {
                ghost = Some((owner, gap));
            }
            first = false;
        }
        let Some(term) = react(&state.vehicles, sink, id, vehicle, found, halfway) else {
            return;
        };
        match found {
            // On a loop shorter than its horizon it can find itself ahead, which
            // holds nobody up.
            Found::Ahead(Ahead::Vehicle(leader), ..) if leader != id => {
                if let Some(b) = binding.filter(|b| b.term <= term) {
                    binding = Some(b);
                    rest = rest.min(term);
                } else {
                    if let Some(b) = binding {
                        rest = rest.min(b.term);
                    }
                    binding = Some(Binding { leader, term, rest: 0.0 });
                }
            }
            _ => rest = rest.min(term),
        }
    });

    // Making room for a vehicle indicating into its lane ahead, as the lane-change
    // model chooses, by what following its ghost would take.
    // LMRS's cooperation: only for a driver whose desire is strong enough.
    let ghost = ghost.map(|(owner, gap)| {
        let o = &state.vehicles[owner];
        let desire = o.intent.map_or(0.0, |i| o.lc.desire(i.side));
        let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, o.speed);
        (owner, gap, desire, term)
    });
    let yielding = lane_change::make_room(
        &vehicle.lc,
        &vehicle.spec,
        ghost.map(|(owner, _, desire, term)| (owner, desire, term)),
    );
    if let Some((owner, gap, _, term)) = ghost.filter(|_| yielding.is_some()) {
        let source = AccelSource::Yield { id: owner, gap };
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        rest = rest.min(term);
    }
    // LMRS's synchronisation: indicating with desire enough, it adjusts its
    // speed to the leader it would have in the target lane, no harder than
    // comfortably, with its headway shortened by its desire.
    if let Some(intent) = vehicle.intent {
        let desire = vehicle.lc.desire(intent.side);
        if desire >= lane_change::D_SYNC {
            let headway = lane_change::headway(desire);
            let _ = find_leaders(
                state,
                vehicle,
                Along::Into(intent.side),
                |found| match found {
                    Found::Ahead(Ahead::Vehicle(leader), gap, _) if leader != id => {
                        let speed = state.vehicles[leader].speed;
                        let term = acceleration::relaxed(
                            &vehicle.spec,
                            vehicle.speed,
                            gap,
                            speed,
                            headway,
                        )
                        .max(-vehicle.spec.comfy_decel);
                        let source = AccelSource::Sync { id: leader, gap };
                        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                        rest = rest.min(term);
                        ControlFlow::Break(())
                    }
                    _ => ControlFlow::Continue(()),
                },
            );
        }
    }
    let binding = binding.map(|b| Binding { rest, ..b });
    let accel = binding.map_or(rest, |b| b.term.min(rest));
    AccelUpdate::Own { id, accel, yielding, binding, committed, arrived }
}

/// Whether `id` has a record, not a ghost, on `lane` near `s` (see
/// [`occupancy::same_record`]). On a loop, a vehicle can have a second one,
/// lingering past the end, at the other end.
fn has_record_at(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    lane: LaneId,
    id: VehicleId,
    s: f64,
) -> bool {
    let length = network.lane(lane).geometry().length();
    occupants[lane.0 as usize]
        .iter()
        .any(|o| o.is_record_of(id) && occupancy::same_record(length, o.s(), s))
}

/// Joins `vehicle`, which has just joined `lane`, to every lane overlapping it
/// that it isn't on yet, each record classified and kept in order.
fn join_overlaps(
    network: &SimNetwork,
    overlaps: &Overlaps,
    occupants: &mut [Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lane: LaneId,
) {
    for p in &overlaps[lane.0 as usize] {
        let occupant = following::record(network, overlaps, p.lane, id, vehicle);
        if has_record_at(network, occupants, p.lane, id, occupant.s()) {
            continue;
        }
        let list = &mut occupants[p.lane.0 as usize];
        let index = list.partition_point(|o| o.s() <= occupant.s());
        list.insert(index, occupant);
    }
}

/// Classifies afresh `vehicle`'s records around `lane` (see
/// [`following::around`]), other than on the lane it follows: all of them, or,
/// if `converging_only`, those still converging.
fn reclassify(
    network: &SimNetwork,
    overlaps: &Overlaps,
    occupants: &mut [Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lane: LaneId,
    converging_only: bool,
) {
    for lane in following::around(network, overlaps, lane) {
        if lane == vehicle.path[0] {
            continue;
        }
        for o in occupants[lane.0 as usize].iter_mut() {
            if o.is_record_of(id) && (!converging_only || o.lateral.is_converging()) {
                following::classify(network, overlaps, vehicle, lane, o);
            }
        }
    }
}

/// The terms the vehicles committed at a junction impose on each one committed
/// after them, and on the vehicles at the front of the ways in, not yet
/// committed, at most one per follower, the lowest, each recorded to the debug
/// sink.
///
/// For each pair, ahead and behind, every conflict between a lane of the
/// follower's path in the junction and a lane of the junction's counts, crossing
/// or merging alike: see [`conflict_term`].
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
    // Each entry for the stretch it covers: from where the vehicle's entry
    // before it stops covering it.
    let followers = committed.iter().enumerate().map(|(k, e)| {
        let from = committed[..k].iter().rev().find(|b| b.id == e.id).and_then(|b| b.until);
        (e.id, &committed[..k], [from, e.until])
    });
    // The vehicles at the front of the ways in, not yet committed, are behind
    // every committed one.
    let fronts = queue.fronts.iter().map(|&id| (id, &committed[..], [None; 2]));
    for (id, ahead, stretch) in followers.chain(fronts) {
        let accel = junction_term(state, sink, junction, id, ahead, stretch);
        if accel < f64::INFINITY {
            decisions.push(AccelUpdate::Junction { id, accel });
        }
    }
}

/// Each vehicle in a merge's order takes a term from the one before it (see
/// [`merge::term`]), each recorded to the debug sink.
fn merge_terms<D: DebugSink>(
    state: &SimState,
    merges: &Merges,
    sink: &mut D,
    merge: usize,
    decisions: &mut Vec<AccelUpdate>,
) {
    let SimState { network, vehicles, occupants, .. } = state;
    let mut before: Option<merge::Member> = None;
    merge::order(network, merges, vehicles, occupants, merge, |follower| {
        let leader = before.replace(follower);
        let Some((accel, gap)) = leader.and_then(|l| merge::term(vehicles, &follower, &l)) else {
            return;
        };
        let (id, source) = (
            follower.id(),
            AccelSource::Merge { id: leader.unwrap().id(), gap },
        );
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel });
        decisions.push(AccelUpdate::Merge { id, accel });
    });
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
        if let Some(term) = react(&state.vehicles, sink, id, vehicle, found, None) {
            accel = accel.min(term);
        }
    });
    accel
}

/// Records what vehicle `id`'s leader search found and, for a leader or a stop
/// line, returns the term it imposes. Ghosts aren't leaders here: the nearest is
/// weighed up for making room. With `halfway`, the distance its rear axle needs
/// to get halfway through a lane change it wants to make, it keeps that much
/// room behind a leader before it would have to stop (see [`room_to_stop`]),
/// so it isn't boxed in: as VISSIM's minimum clearance for changing lanes from
/// a standstill, and the room drivers are taught to leave in a queue.
fn react<D: DebugSink>(
    vehicles: &SlotMap<VehicleId, Vehicle>,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    found: Found,
    halfway: Option<f64>,
) -> Option<f64> {
    let (ahead, gap, headway) = match found {
        Found::Ahead(ahead, gap, headway) => (ahead, gap, headway),
        Found::Pass(check) => {
            sink.record(|| DebugEvent::PassCheck { vehicle: id, check });
            return None;
        }
    };
    let source = match ahead {
        Ahead::Vehicle(leader) => AccelSource::Leader { id: leader, gap, headway },
        Ahead::PathEnd => AccelSource::PathEnd { gap },
        Ahead::Ghost(_) | Ahead::Barrier { .. } => return None,
    };
    let speed = speed_of(vehicles, ahead);
    // Wanting to change lanes, it keeps room behind a leader to get halfway
    // through a change before it would have to stop, so it isn't boxed in: that
    // room is the minimum gap of a second term, which IDM settles on from above,
    // so the room it keeps is never short.
    let mut term = acceleration::relaxed(&vehicle.spec, vehicle.speed, gap, speed, headway);
    if let (Some(halfway), Ahead::Vehicle(leader)) = (halfway, ahead) {
        let short = halfway - comfortable_stop(&vehicles[leader]);
        if short > 0.0 {
            // A preference, not safety: never braking harder than comfortably.
            let min_gap = vehicle.spec.min_gap + short;
            let spec = VehicleSpec { min_gap, ..vehicle.spec };
            let room = acceleration::relaxed(&spec, vehicle.speed, gap, speed, headway);
            term = term.min(room.max(-vehicle.spec.comfy_decel));
        }
    }
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
    Some(term)
}

/// The speed of something a leader search found: a vehicle's, a ghost's owner's,
/// or zero for a stop line.
pub(crate) fn speed_of(vehicles: &SlotMap<VehicleId, Vehicle>, ahead: Ahead) -> f64 {
    match ahead {
        Ahead::Vehicle(id) | Ahead::Ghost(id) => vehicles[id].speed,
        Ahead::PathEnd | Ahead::Barrier { .. } => 0.0,
    }
}

/// Removes a vehicle's ghost from a lane's occupants.
fn remove_ghost(occupants: &mut [Vec<Occupant>], lane: LaneId, id: VehicleId) {
    occupants[lane.0 as usize].retain(|o| !(o.id == id && o.is_ghost()));
}

#[cfg(test)]
mod test;
