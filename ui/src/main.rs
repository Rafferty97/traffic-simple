//! egui app with a pretty view and a debug view of the simulation.

mod glyphs;
mod replay;

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::path::PathBuf;

use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use glam::DVec2;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use replay::Recorded;
use traffic_network::{
    Arrow, MarkingStyle, RenderGeometry, RoadNetwork, Sign, Strip, compile, scenarios,
};
use traffic_sim::{
    AccelSource, BarrierDecision, BarrierKind, ConflictKind, DebugEvent, DestinationId, Driver,
    Field, LaneChangeError, LaneChangeIntent, LaneId, Leaving, Light, PassCheck, Placement, RoadId,
    Route, Side, Simulation, Vehicle, VehicleClass, VehicleId, VehicleSpec,
};

/// The hook-turn scenario's speed limit: 40 km/h, as Melbourne's CBD has.
const HOOK_SPEED_LIMIT: f64 = 40.0 / 3.6;

/// The inner lane's circumference: about 160 m across, roomy enough for
/// vehicles to accelerate, settle into gaps and change lanes.
const RING_CIRCUMFERENCE: f64 = 1000.0;
const RING_LANES: usize = 3;
const SPEED_LIMIT: f64 = 50.0 / 3.6;

/// Most spawns or despawns per simulated second, while the count is off target.
const CHANGE_RATE: f64 = 2.0;
/// The speed the inspector's slow button imposes, in m/s: 20 km/h.
const SLOW_CAP: f64 = 20.0 / 3.6;
/// How far outside a vehicle a click still selects it, in screen points.
const PICK_TOLERANCE: f32 = 6.0;

const SELECTED: Color32 = Color32::from_rgb(250, 210, 60);
/// The selected vehicle's path, translucent so the road shows through.
const PATH: Color32 = Color32::from_rgba_premultiplied(40, 90, 140, 140);
/// The colours of the vehicles the selected one relates to, in turn.
const RELATED: [Color32; 8] = [
    Color32::from_rgb(70, 140, 255),
    Color32::from_rgb(230, 60, 200),
    Color32::from_rgb(40, 200, 200),
    Color32::from_rgb(255, 130, 30),
    Color32::from_rgb(150, 90, 255),
    Color32::from_rgb(60, 200, 80),
    Color32::from_rgb(255, 90, 110),
    Color32::from_rgb(140, 110, 60),
];

/// A vehicle's number, as the UI shows it: its slot's index, unique among the
/// vehicles there at once.
fn number(id: VehicleId) -> String {
    id.index().to_string()
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Entity {
    Vehicle(VehicleId),
    Lane(LaneId),
}

/// How an acceleration term is described in the inspector, and the gap it
/// keeps, if any; the vehicle it names is [`named`]'s.
fn describe(source: AccelSource) -> (String, Option<Entity>, Option<f64>) {
    match source {
        AccelSource::FreeRoad { desired_speed } => (
            format!("target speed of {:.0} km/h", desired_speed * 3.6),
            None,
            None,
        ),
        AccelSource::Leader { id, gap, headway } if headway < 1.0 => (
            format!("leader (headway ×{headway:.2})"),
            Some(Entity::Vehicle(id)),
            Some(gap),
        ),
        AccelSource::Leader { id, gap, .. } => {
            ("leader".into(), Some(Entity::Vehicle(id)), Some(gap))
        }
        AccelSource::Cap { max_speed: 0.0 } => ("held to a stop".into(), None, None),
        AccelSource::Cap { max_speed } => {
            (format!("capped at {:.0} km/h", max_speed * 3.6), None, None)
        }
        AccelSource::FinishingStop => (
            format!(
                "finishing a stop, below {:.0} km/h",
                traffic_sim::STOPPING_SPEED * 3.6
            ),
            None,
            None,
        ),
        AccelSource::PathEnd { gap } => ("end of known path".into(), None, Some(gap)),
        AccelSource::LimitAhead { lane, limit, gap } => (
            format!("slowing to {:.0} km/h ahead", limit * 3.6),
            Some(Entity::Lane(lane)),
            Some(gap),
        ),
        AccelSource::Barrier { lane, gap, kind, .. } => {
            let kind = match kind {
                BarrierKind::Signal { .. } => "traffic light",
                BarrierKind::GiveWay => "give way",
                BarrierKind::Stop => "stop",
                BarrierKind::KeepClear { .. } => "keep clear",
            };
            (kind.into(), Some(Entity::Lane(lane)), Some(gap))
        }
        AccelSource::Junction { id, kind: ConflictKind::Cross, gap } => (
            "crossing after".into(),
            Some(Entity::Vehicle(id)),
            Some(gap),
        ),
        AccelSource::Junction { id, kind: ConflictKind::Merge, gap } => {
            ("merging after".into(), Some(Entity::Vehicle(id)), Some(gap))
        }
        AccelSource::Merge { id, gap } => (
            "merge order after".into(),
            Some(Entity::Vehicle(id)),
            Some(gap),
        ),
        AccelSource::Model { id, model, term, gap } => {
            (format!("{model}: {term}"), id.map(Entity::Vehicle), gap)
        }
    }
}

/// The vehicle an acceleration term is for, if any.
fn named(source: AccelSource) -> Option<VehicleId> {
    match source {
        AccelSource::Leader { id, .. }
        | AccelSource::Junction { id, .. }
        | AccelSource::Merge { id, .. } => Some(id),
        AccelSource::Model { id, .. } => id,
        _ => None,
    }
}

/// The vehicles `id` gives way to at its barriers and its acceleration terms
/// name, each with its colour, in that order.
fn related(debug: &DebugState, id: VehicleId) -> Vec<(VehicleId, Color32)> {
    let mut out: Vec<(VehicleId, Color32)> = Vec::new();
    let barriers = debug.barriers.iter().filter(|b| b.0 == id).map(|b| b.5);
    let terms = debug.terms(id).map(|(source, _)| named(source));
    for other in barriers.chain(terms) {
        if let Some(other) = other
            && other != id
            && !out.iter().any(|&(v, _)| v == other)
        {
            out.push((other, RELATED[out.len() % RELATED.len()]));
        }
    }
    out
}

/// What colour to draw `id`'s body: `plain` with nobody selected; otherwise
/// the selected vehicle's colour, its related vehicles' colours, or white.
fn body_colour(
    id: VehicleId,
    selected: Option<VehicleId>,
    related: &[(VehicleId, Color32)],
    plain: Color32,
) -> Color32 {
    match selected {
        None => plain,
        Some(s) if s == id => SELECTED,
        Some(_) => related.iter().find(|&&(v, _)| v == id).map_or(VEHICLE, |&(_, c)| c),
    }
}

/// Runs the UI, starting on the scenario named on the command line, by its
/// key (see [`Scenario::key`]), or on the ring.
fn main() -> eframe::Result {
    let scenario = match std::env::args().nth(1) {
        None => Scenario::Ring,
        Some(key) => match Scenario::ALL.into_iter().find(|s| s.key() == key) {
            Some(scenario) => scenario,
            None => {
                eprintln!("No scenario called {key:?}. The scenarios are:");
                for s in Scenario::ALL {
                    eprintln!("  {:<16} {}", s.key(), s.name());
                }
                std::process::exit(2);
            }
        },
    };
    eframe::run_native(
        "Traffic Simulator",
        eframe::NativeOptions::default(),
        Box::new(move |_cc| Ok(Box::new(App::new(scenario)))),
    )
}

/// A network to run, and how vehicles come and go on it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    Ring,
    /// Two two-way roads crossing, with this many lanes each way.
    Crossroads(usize),
    /// As `Crossroads`, signalised.
    Signals(usize),
    /// As `Signals`, with two lanes each way, the roads crossing at 60°.
    ObliqueSignals,
    /// As `Signals`, with two lanes each way, and a keep-clear zone on the
    /// way in from the west.
    KeepClear,
    /// As `Signals`, with two lanes each way, turning left on red.
    TurnOnRed,
    /// As `Crossroads`, an all-way stop.
    AllWayStop(usize),
    /// A crossroads, the east–west road, with the first number of lanes each
    /// way, with priority, and the other, with the second, with this sign.
    Priority([usize; 2], Sign),
    /// A T-junction, the road through, with the first number of lanes each
    /// way, with priority, and the one from the south, with the second, with
    /// this sign.
    TJunction([usize; 2], Sign),
    /// A freeze frame: a right turner and a cross-street car waiting on each
    /// other (see `traffic_network::frames::turner_cycle`).
    TurnerCycle,
    /// A freeway with an on-ramp and an off-ramp, the acceleration lane left as
    /// this says.
    Ramps(Leaving),
    /// A grid of 4 by 4 crossroads 100 m apart, of roads with two lanes each
    /// way, signalised and controlled by priority in turn.
    Grid,
    /// A Melbourne crossroads, one lane each way and room for tram tracks,
    /// signalised, its right turns hook turns.
    HookTurns,
}

impl Scenario {
    const ALL: [Scenario; 26] = [
        Scenario::Ring,
        Scenario::Crossroads(1),
        Scenario::Crossroads(2),
        Scenario::Crossroads(3),
        Scenario::Signals(1),
        Scenario::Signals(2),
        Scenario::Signals(3),
        Scenario::ObliqueSignals,
        Scenario::KeepClear,
        Scenario::TurnOnRed,
        Scenario::AllWayStop(1),
        Scenario::AllWayStop(2),
        Scenario::Priority([1, 1], Sign::GiveWay),
        Scenario::Priority([2, 1], Sign::GiveWay),
        Scenario::Priority([2, 2], Sign::GiveWay),
        Scenario::Priority([1, 1], Sign::Stop),
        Scenario::Priority([2, 1], Sign::Stop),
        Scenario::TJunction([1, 1], Sign::GiveWay),
        Scenario::TJunction([2, 1], Sign::GiveWay),
        Scenario::TJunction([1, 1], Sign::Stop),
        Scenario::TJunction([2, 1], Sign::Stop),
        Scenario::Ramps(Leaving::Zipper),
        Scenario::Ramps(Leaving::Early),
        Scenario::Grid,
        Scenario::HookTurns,
        Scenario::TurnerCycle,
    ];

    /// What to call it on the command line.
    fn key(self) -> &'static str {
        match self {
            Scenario::Ring => "ring",
            Scenario::Crossroads(1) => "crossroads-1",
            Scenario::Crossroads(2) => "crossroads-2",
            Scenario::Crossroads(_) => "crossroads-3",
            Scenario::Signals(1) => "signals-1",
            Scenario::Signals(2) => "signals-2",
            Scenario::Signals(_) => "signals-3",
            Scenario::ObliqueSignals => "oblique-signals",
            Scenario::KeepClear => "keep-clear",
            Scenario::TurnOnRed => "turn-on-red",
            Scenario::AllWayStop(1) => "all-way-1",
            Scenario::AllWayStop(_) => "all-way-2",
            Scenario::Priority([1, 1], Sign::GiveWay) => "give-way-1x1",
            Scenario::Priority([2, 1], Sign::GiveWay) => "give-way-2x1",
            Scenario::Priority(_, Sign::GiveWay) => "give-way-2x2",
            Scenario::Priority([1, 1], Sign::Stop) => "stop-1x1",
            Scenario::Priority(_, Sign::Stop) => "stop-2x1",
            Scenario::TJunction([1, 1], Sign::GiveWay) => "t-give-way-1x1",
            Scenario::TJunction(_, Sign::GiveWay) => "t-give-way-2x1",
            Scenario::TJunction([1, 1], Sign::Stop) => "t-stop-1x1",
            Scenario::TJunction(_, Sign::Stop) => "t-stop-2x1",
            Scenario::Ramps(Leaving::Zipper) => "ramps-zipper",
            Scenario::Ramps(Leaving::Early) => "ramps-early",
            Scenario::Grid => "grid",
            Scenario::HookTurns => "hook-turns",
            Scenario::TurnerCycle => "turner-cycle",
        }
    }

    fn name(self) -> &'static str {
        match self {
            Scenario::Ring => "Three-lane ring",
            Scenario::Crossroads(1) => "Crossroads of 2-lane roads",
            Scenario::Crossroads(2) => "Crossroads of 4-lane roads",
            Scenario::Crossroads(_) => "Crossroads of 6-lane roads",
            Scenario::Signals(1) => "Signalised crossroads of 2-lane roads",
            Scenario::Signals(2) => "Signalised crossroads of 4-lane roads",
            Scenario::Signals(_) => "Signalised crossroads of 6-lane roads",
            Scenario::ObliqueSignals => "Signalised 4-lane roads crossing at 60°",
            Scenario::KeepClear => "Signalised crossroads with a keep-clear zone",
            Scenario::TurnOnRed => "Signalised crossroads, turning left on red",
            Scenario::AllWayStop(1) => "All-way stop of 2-lane roads",
            Scenario::AllWayStop(_) => "All-way stop of 4-lane roads",
            Scenario::Priority([1, 1], Sign::GiveWay) => "Give-way crossroads of 2-lane roads",
            Scenario::Priority([2, 1], Sign::GiveWay) => {
                "Give-way crossroads, a 2-lane road across a 4-lane one"
            }
            Scenario::Priority(_, Sign::GiveWay) => "Give-way crossroads of 4-lane roads",
            Scenario::Priority([1, 1], Sign::Stop) => "Stop-sign crossroads of 2-lane roads",
            Scenario::Priority(_, Sign::Stop) => {
                "Stop-sign crossroads, a 2-lane road across a 4-lane one"
            }
            Scenario::TJunction([1, 1], Sign::GiveWay) => "Give-way T-junction of 2-lane roads",
            Scenario::TJunction(_, Sign::GiveWay) => {
                "Give-way T-junction, a 2-lane road into a 4-lane one"
            }
            Scenario::TJunction([1, 1], Sign::Stop) => "Stop-sign T-junction of 2-lane roads",
            Scenario::TJunction(_, Sign::Stop) => {
                "Stop-sign T-junction, a 2-lane road into a 4-lane one"
            }
            Scenario::Ramps(Leaving::Zipper) => "Freeway with ramps, zipper merge",
            Scenario::Ramps(Leaving::Early) => "Freeway with ramps, early merge",
            Scenario::Grid => "Grid of 4 by 4 crossroads of 4-lane roads, signals and priority",
            Scenario::HookTurns => "Melbourne crossroads with hook turns, 1 lane each way",
            Scenario::TurnerCycle => {
                "Freeze frame: a turner and a car that would wait on each other"
            }
        }
    }

    fn network(self) -> RoadNetwork {
        match self {
            Scenario::Ring => scenarios::ring(RING_CIRCUMFERENCE, RING_LANES, SPEED_LIMIT),
            Scenario::Crossroads(lanes) => scenarios::crossroads(lanes, SPEED_LIMIT),
            Scenario::Signals(lanes) => scenarios::signalised_crossroads(lanes, SPEED_LIMIT),
            Scenario::ObliqueSignals => {
                scenarios::signalised_oblique_crossroads(2, SPEED_LIMIT, 60f64.to_radians())
            }
            Scenario::KeepClear => scenarios::signalised_crossroads_keep_clear(2, SPEED_LIMIT),
            Scenario::TurnOnRed => scenarios::signalised_crossroads_turn_on_red(2, SPEED_LIMIT),
            Scenario::AllWayStop(lanes) => scenarios::all_way_stop_crossroads(lanes, SPEED_LIMIT),
            Scenario::Priority(lanes, sign) => {
                scenarios::priority_crossroads(lanes, SPEED_LIMIT, sign)
            }
            Scenario::TJunction(lanes, sign) => {
                scenarios::priority_t_junction(lanes, SPEED_LIMIT, sign)
            }
            Scenario::Ramps(leaving) => scenarios::ramps_leaving(100.0 / 3.6, 80.0 / 3.6, leaving),
            Scenario::Grid => scenarios::grid(4, 4, 2, 100.0, SPEED_LIMIT),
            Scenario::HookTurns => scenarios::hook_turn_crossroads(HOOK_SPEED_LIMIT),
            Scenario::TurnerCycle => unreachable!("a freeze frame is restored, not built"),
        }
    }

    /// Whether it is a freeze frame: a snapshot restored, opened paused, which
    /// the harness adds no traffic to.
    fn frame(self) -> bool {
        matches!(self, Scenario::TurnerCycle)
    }

    /// Whether the network is closed: vehicles neither enter nor leave it.
    fn closed(self) -> bool {
        matches!(self, Scenario::Ring)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    Pretty,
    Debug,
}

struct App {
    sim: Recorded,
    /// What the debug view shows of the simulation's algorithms, from the
    /// events its debug steps return.
    debug: DebugState,
    /// The last frame's debug events, before they are applied: kept to reuse.
    events: Vec<DebugEvent>,
    /// What to draw for the network.
    render: RenderGeometry,
    scenario: Scenario,
    view: View,
    paused: bool,
    /// Simulated seconds per real second.
    speed: f64,
    /// Visible region of the world, in scene coordinates (see [`to_scene`]).
    scene_rect: Rect,
    selected: Option<VehicleId>,
    /// Which debug view layers are drawn.
    layers: Layers,
    /// Why the last lane-change request for the selected vehicle was refused.
    lane_change_error: Option<LaneChangeError>,
    /// Where the last capture was saved, or why it failed.
    capture_status: Option<String>,
    harness: Harness,
    /// Each vehicle's corners before the last step, to draw it between there and
    /// where it is now (see [`Poses`]).
    poses: Poses,
    /// How long the last fixed steps took to simulate.
    step_times: StepTimes,
    /// How far the selected vehicle has gone, for its path's dashes.
    travelled: Travelled,
}

/// How far a vehicle has gone along its lanes since it was selected, kept so
/// its path's dashes stay put on the road as it moves on from one lane to the
/// next, where its distance along its lane starts afresh.
#[derive(Default)]
struct Travelled {
    /// The vehicle, and the lane it was on when last seen.
    seen: Option<(VehicleId, LaneId)>,
    /// How far along the lanes before that one it had gone, in meters.
    before: f64,
}

impl Travelled {
    /// How far `id` has gone, now that it is as `v` is: the lanes it has moved
    /// on from, and how far along the one it is on. A lane change keeps the
    /// distance, since its new lane runs beside its old one.
    fn update(&mut self, sim: &Simulation, id: VehicleId, v: &Vehicle) -> f64 {
        match self.seen {
            Some((seen, lane)) if seen == id && lane != v.lane() => {
                let old = sim.network().lane(lane);
                let beside = [Side::Left, Side::Right]
                    .into_iter()
                    .any(|side| old.adj(side).is_some_and(|n| n.lane == v.lane()));
                if !beside {
                    self.before += old.geometry().length();
                }
            }
            Some((seen, _)) if seen == id => {}
            _ => self.before = 0.0,
        }
        self.seen = Some((id, v.lane()));
        self.before + v.lane_pos().s
    }
}

/// How many fixed steps the step time is averaged over: a second of simulated
/// time at 30 steps a second.
const STEPS_AVERAGED: usize = 30;

/// The time each of the last [`STEPS_AVERAGED`] fixed steps of the simulation
/// took, in seconds, with the UI's recording debug sink. Several steps run in
/// one rendered frame at higher speeds, each counting as one, so it doesn't
/// vary with the speed; drawing doesn't count.
#[derive(Default)]
struct StepTimes {
    times: [f64; STEPS_AVERAGED],
    /// Where the next goes.
    next: usize,
    /// How many there are, up to [`STEPS_AVERAGED`].
    count: usize,
}

impl StepTimes {
    /// Records `steps` steps that took `seconds` together, each the average.
    fn push(&mut self, steps: u32, seconds: f64) {
        let each = seconds / f64::from(steps.max(1));
        for _ in 0..(steps as usize).min(STEPS_AVERAGED) {
            self.times[self.next] = each;
            self.next = (self.next + 1) % STEPS_AVERAGED;
            self.count = (self.count + 1).min(STEPS_AVERAGED);
        }
    }

    /// The average, in seconds, or none before the first step.
    fn average(&self) -> Option<f64> {
        (self.count > 0).then(|| self.times[..self.count].iter().sum::<f64>() / self.count as f64)
    }
}

/// Where vehicles were before the simulation's last fixed step, so they can be
/// drawn between there and where they are by how far into the next step the
/// frame has reached ([`Simulation::alpha`]), rather than jumping a step at a
/// time: interpolation, as in "Fix Your Timestep". Drawn a step behind.
#[derive(Default)]
struct Poses {
    before: HashMap<VehicleId, [DVec2; 4]>,
}

impl Poses {
    /// Where to draw `v`'s corners, `alpha` of the way from before the last step
    /// to now. A vehicle that wasn't there before is drawn where it is.
    fn corners(&self, id: VehicleId, v: &Vehicle, alpha: f64) -> [DVec2; 4] {
        let now = v.corners();
        match self.before.get(&id) {
            Some(before) => std::array::from_fn(|i| before[i].lerp(now[i], alpha)),
            None => now,
        }
    }
}

/// Keeps vehicles coming. On a closed network, such as the ring, it keeps a
/// target number on the road, spawning and despawning anywhere; on an open one,
/// it spawns at a flow into each lane that nothing leads into, and vehicles
/// leave where lanes lead nowhere.
struct Harness {
    /// Whether it leaves the simulation alone, as for a freeze frame.
    still: bool,
    /// Whether the network is closed: nowhere to come in or leave.
    closed: bool,
    /// How many vehicles it keeps on a closed network.
    target: usize,
    /// Vehicles per hour into each entry lane of an open network.
    flow: f64,
    /// Whether vehicles on an open network are routed to an exit, drawn at
    /// random from those they can reach, rather than choosing at random at
    /// each fork.
    to_exits: bool,
    /// Each exit of the network, the lanes leading out at the end of a road,
    /// registered as a destination.
    exits: Vec<DestinationId>,
    /// Spawns or despawns owed: grows at [`CHANGE_RATE`], or at the flow, and a
    /// refused spawn waits here.
    credit: f64,
    spawned: u32,
    despawned: u32,
    /// Draws drivers and their routes. Seeded, so every run spawns the same.
    rng: Xoshiro256PlusPlus,
}

impl Default for Harness {
    fn default() -> Self {
        Self {
            still: false,
            closed: true,
            target: 60,
            flow: 600.0,
            to_exits: true,
            exits: Vec::new(),
            credit: 0.0,
            spawned: 0,
            despawned: 0,
            rng: Xoshiro256PlusPlus::seed_from_u64(0),
        }
    }
}

impl Harness {
    /// Registers each exit of an open network as a destination: the lanes
    /// leading nowhere, grouped by road.
    fn register_exits(&mut self, sim: &mut Recorded) {
        if self.closed || self.still {
            return;
        }
        let mut roads: Vec<(RoadId, Vec<LaneId>)> = Vec::new();
        for (id, lane) in sim.network().lanes() {
            if !lane.successors().is_empty() {
                continue;
            }
            match roads.iter_mut().find(|(road, _)| *road == lane.road()) {
                Some((_, lanes)) => lanes.push(id),
                None => roads.push((lane.road(), vec![id])),
            }
        }
        self.exits = roads.iter().map(|(_, lanes)| sim.add_destination(lanes)).collect();
    }

    /// Runs after each simulation step of `dt` simulated seconds.
    fn step(&mut self, sim: &mut Recorded, selected: Option<VehicleId>, dt: f64) {
        if self.still {
            return;
        }
        match self.closed {
            true => self.keep_count(sim, selected, dt),
            false => self.keep_flowing(sim, dt),
        }
    }

    /// While the count is off target, spawns or despawns at up to
    /// [`CHANGE_RATE`], each at a random lane and point, or a random vehicle
    /// other than the `selected` one.
    fn keep_count(&mut self, sim: &mut Recorded, selected: Option<VehicleId>, dt: f64) {
        let count = sim.vehicles().count();
        if count == self.target {
            self.credit = 0.0;
            return;
        }
        // At most one owed: a backlog would arrive as a burst.
        self.credit = (self.credit + CHANGE_RATE * dt).min(1.0);
        if self.credit < 1.0 {
            return;
        }
        if count < self.target {
            let lanes = sim.network().lanes().count() as u32;
            let lane = LaneId(self.rng.random_range(0..lanes));
            let s = self.rng.random::<f64>() * sim.network().lane(lane).geometry().length();
            self.spawn(sim, lane, s);
        } else {
            // Never the one being inspected.
            let others = || sim.vehicles().map(|(id, _)| id).filter(|&id| Some(id) != selected);
            let n = others().count();
            if n > 0 {
                let id = others().nth(self.rng.random_range(0..n)).unwrap();
                sim.remove(id);
                self.credit -= 1.0;
                self.despawned += 1;
            }
        }
    }

    /// Spawns at the flow, each into a random entry lane, near its start.
    fn keep_flowing(&mut self, sim: &mut Recorded, dt: f64) {
        let entries: Vec<LaneId> = sim
            .network()
            .lanes()
            .filter(|(_, lane)| lane.predecessors().is_empty())
            .map(|(id, _)| id)
            .collect();
        if entries.is_empty() {
            return;
        }
        let rate = self.flow / 3600.0 * entries.len() as f64;
        // A few owed at most: a long backlog would arrive as a burst.
        self.credit = (self.credit + rate * dt).min(entries.len() as f64);
        if self.credit >= 1.0 {
            let lane = entries[self.rng.random_range(0..entries.len())];
            self.spawn(sim, lane, 5.0);
        }
    }

    /// Tries to spawn a driver at `s` on `lane`, at its desired speed, taking
    /// one from the credit if it fits.
    fn spawn(&mut self, sim: &mut Recorded, lane: LaneId, s: f64) {
        // Drivers vary in how hard they'll brake to let someone in, in how much
        // they weigh others when changing lanes, and in how readily they change:
        // mostly cautious, about one in ten above 0.7.
        let car = demo_car();
        let mut driver = Driver {
            speed_factor: self.speed_factor(),
            courtesy: 2.0 * self.rng.random::<f64>(),
            politeness: 0.1 + 0.3 * self.rng.random::<f64>(),
            assertiveness: self.rng.random::<f64>().powi(3),
            ..Driver::default()
        };
        let limit = sim.network().lane(lane).speed_limit();
        let seed = self.rng.random();
        driver.seed = seed;
        // To an exit it can reach from where it starts, if it is to go to one.
        let reachable: Vec<_> = match self.to_exits {
            true => self
                .exits
                .iter()
                .filter(|&&d| sim.time_to(d, VehicleClass::Car, lane).is_some())
                .collect(),
            false => Vec::new(),
        };
        let route = match reachable.len() {
            0 => Route::random(seed),
            n => Route::to(*reachable[self.rng.random_range(0..n)]),
        };
        let at = Placement { lane, s, speed: driver.desired_speed(&car, limit), route };
        // Refused where there is no safe room: the credit waits for another try.
        if sim.try_insert(car, driver, at).is_ok() {
            self.credit -= 1.0;
            self.spawned += 1;
        }
    }

    /// A speed factor from a normal distribution with mean 1 and deviation 0.05,
    /// clamped to [0.85, 1.15]: nearly every driver on a 50 km/h road wants 45
    /// to 55 km/h. SUMO's default deviation, 0.1, spreads them wider.
    fn speed_factor(&mut self) -> f64 {
        // Box-Muller on two uniform samples, u in (0, 1] so its log is finite.
        let u = 1.0 - self.rng.random::<f64>();
        let v = self.rng.random::<f64>();
        let normal = (-2.0 * u.ln()).sqrt() * (TAU * v).cos();
        (1.0 + 0.05 * normal).clamp(0.85, 1.15)
    }

    fn controls(&mut self, ui: &mut egui::Ui, on_road: usize) {
        ui.horizontal(|ui| {
            if self.closed {
                ui.add(egui::Slider::new(&mut self.target, 0..=300).text("target vehicles"));
            } else {
                let slider = egui::Slider::new(&mut self.flow, 0.0..=1800.0);
                ui.add(slider.text("vehicles per hour per entry lane"));
                ui.checkbox(&mut self.to_exits, "routed to exits");
            }
            ui.separator();
            ui.label(format!(
                "spawned {}, despawned {}, on the road {on_road}",
                self.spawned, self.despawned
            ));
        });
    }
}

impl App {
    fn new(scenario: Scenario) -> Self {
        let (sim, render, _) = new_sim(scenario);
        let mut app = Self {
            sim,
            debug: DebugState::default(),
            events: Vec::new(),
            render,
            scenario,
            view: View::Pretty,
            paused: false,
            speed: 1.0,
            scene_rect: Rect::NOTHING,
            selected: None,
            layers: Layers::default(),
            lane_change_error: None,
            capture_status: None,
            harness: Harness::default(),
            poses: Poses::default(),
            step_times: StepTimes::default(),
            travelled: Travelled::default(),
        };
        app.load(scenario);
        app
    }

    /// Starts the scenario afresh, with the harness's settings kept, and fits
    /// the view to it.
    fn load(&mut self, scenario: Scenario) {
        let selected;
        (self.sim, self.render, selected) = new_sim(scenario);
        self.debug = DebugState::default();
        self.scenario = scenario;
        self.selected = selected;
        // A freeze frame opens paused, to look at before stepping on.
        if scenario.frame() {
            self.paused = true;
        }
        self.capture_status = None;
        self.poses = Poses::default();
        let Harness { target, flow, to_exits, .. } = self.harness;
        self.harness = Harness {
            still: scenario.frame(),
            closed: scenario.closed(),
            target,
            flow,
            to_exits,
            ..Harness::default()
        };
        self.harness.register_exits(&mut self.sim);
        let road_points = self.render.roads.iter().flat_map(|r| r.left.iter().chain(&r.right));
        let points: Vec<Pos2> = road_points.map(|&p| to_scene(p)).collect();
        self.scene_rect = Rect::from_points(&points).expand(10.0);
    }

    fn step(&mut self, dt: f64) {
        let before: HashMap<_, _> = self.sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
        let started = std::time::Instant::now();
        let steps = self.sim.step_variable_debug(dt, &mut self.events);
        for event in self.events.drain(..) {
            self.debug.apply(event);
        }
        if steps > 0 {
            self.step_times.push(steps, started.elapsed().as_secs_f64());
            self.poses.before = before;
        }
        self.harness.step(&mut self.sim, self.selected, dt);
    }

    /// Saves a replay of the session so far to `captures/` in the workspace,
    /// named by the wall-clock time, and returns its path.
    fn capture(&self) -> std::io::Result<PathBuf> {
        let dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../captures"));
        std::fs::create_dir_all(&dir)?;
        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH);
        let stamp = now.map_or(0, |d| d.as_secs());
        let path = dir.join(format!("capture-{stamp}.replay"));
        std::fs::write(&path, self.sim.capture(self.selected))?;
        Ok(path.canonicalize().unwrap_or(path))
    }

    /// Saves a freeze frame of the simulation as it is now, with the selected
    /// vehicle, to `captures/` in the workspace, named by the wall-clock time, and
    /// returns its path. `sim/tests/common/frame.rs` loads it back.
    fn freeze_frame(&self) -> std::io::Result<PathBuf> {
        let dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../captures"));
        std::fs::create_dir_all(&dir)?;
        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH);
        let stamp = now.map_or(0, |d| d.as_secs());
        let path = dir.join(format!("frame-{stamp}.json"));
        let frame = serde_json::json!({
            "selected": self.selected,
            "snapshot": self.sim.state(),
        });
        std::fs::write(&path, serde_json::to_string_pretty(&frame)?)?;
        Ok(path.canonicalize().unwrap_or(path))
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.view, View::Pretty, "Pretty");
            ui.selectable_value(&mut self.view, View::Debug, "Debug");
            ui.separator();
            let label = if self.paused { "Play" } else { "Pause" };
            if ui.button(label).clicked() {
                self.paused = !self.paused;
            }
            if ui.add_enabled(self.paused, egui::Button::new("Step")).clicked() {
                self.step(self.sim.config().dt);
            }
            for (speed, label) in [
                (0.1, "0.1×"),
                (0.5, "0.5×"),
                (1.0, "1×"),
                (2.0, "2×"),
                (5.0, "5×"),
                (10.0, "10×"),
                (20.0, "20×"),
                (100.0, "100×"),
            ] {
                ui.selectable_value(&mut self.speed, speed, label);
            }
            ui.separator();
            // The average time a fixed step takes, and its share of the time it
            // simulates.
            if let Some(average) = self.step_times.average() {
                let step = self.sim.config().dt;
                ui.monospace(format!(
                    "step {:.2} ms ({:.0}% of 1/{:.0} s)",
                    average * 1000.0,
                    average / step * 100.0,
                    1.0 / step
                ))
                .on_hover_text(
                    "The last 30 fixed steps' average time to simulate, with the UI's \
                     debug recording, whatever the speed; drawing isn't counted",
                );
            }
            ui.separator();
            let mut scenario = self.scenario;
            egui::ComboBox::from_id_salt("scenario").selected_text(scenario.name()).show_ui(
                ui,
                |ui| {
                    for option in Scenario::ALL {
                        ui.selectable_value(&mut scenario, option, option.name());
                    }
                },
            );
            if scenario != self.scenario || ui.button("Reset").clicked() {
                self.load(scenario);
            }
            if ui
                .button("Capture")
                .on_hover_text("Save a replay of this session, for a test case")
                .clicked()
            {
                self.capture_status = Some(match self.capture() {
                    Ok(path) => format!("Saved {}", path.display()),
                    Err(error) => format!("Capture failed: {error}"),
                });
            }
            if ui
                .button("Freeze frame")
                .on_hover_text("Save the simulation exactly as it is now, for a test case")
                .clicked()
            {
                self.capture_status = Some(match self.freeze_frame() {
                    Ok(path) => format!("Saved {}", path.display()),
                    Err(error) => format!("Freeze frame failed: {error}"),
                });
            }
        });
        if let Some(status) = &self.capture_status {
            ui.small(status);
        }
        let on_road = self.sim.vehicles().count();
        self.harness.controls(ui, on_road);
    }

    fn inspector(&mut self, ui: &mut egui::Ui, id: VehicleId) {
        let Some(v) = self.sim.get(id) else {
            ui.label("Vehicle has left the network.");
            return;
        };
        let pos = v.lane_pos();
        egui::Grid::new("inspector").num_columns(2).striped(true).show(ui, |ui| {
            ui.label("id");
            ui.label(egui::RichText::new(number(id)).monospace().color(SELECTED));
            fill_row(ui);
            ui.end_row();
            let mut row = |name: &str, value: String| {
                ui.label(name);
                ui.monospace(value);
                ui.end_row();
            };
            row("lane", format!("{}", v.lane().0));
            let neighbor = |side| match self.sim.network().lane(v.lane()).adj(side) {
                Some(n) if n.can_change => format!("{}", n.lane.0),
                Some(n) => format!("{} (no change)", n.lane.0),
                None => "none".into(),
            };
            row("left", neighbor(Side::Left));
            row("right", neighbor(Side::Right));
            let side = |side: Option<Side>| match side {
                Some(Side::Left) => "left".to_string(),
                Some(Side::Right) => "right".to_string(),
                None => "none".to_string(),
            };
            row("old side", side(v.old_side()));
            match v.lane_change() {
                Some(change) => {
                    let done = (pos.s - change.s0) / change.length;
                    row(
                        "changing",
                        format!("{:.0}% of {:.0} m", 100.0 * done, change.length),
                    )
                }
                None => row("changing", "no".into()),
            }
            match v.intent() {
                LaneChangeIntent::Indicating { side: to, since, .. } => {
                    let since = self.sim.time() - since;
                    row("indicating", format!("{} for {since:.1} s", side(Some(to))));
                }
                LaneChangeIntent::Desire { side: to, check } => {
                    let checking = if check.is_some() {
                        ", checking the gap"
                    } else {
                        ""
                    };
                    row(
                        "indicating",
                        format!("no, wants {}{checking}", side(Some(to))),
                    );
                }
                LaneChangeIntent::None => row("indicating", "no".into()),
            }
            let path: Vec<_> = v.path().iter().map(|l| l.0.to_string()).collect();
            row("path", path.join(" → "));
            match self.sim.route(id) {
                Some(Route::Destination(destination)) => {
                    let lanes: Vec<_> =
                        self.sim.destination(destination).iter().map(|l| l.0.to_string()).collect();
                    row("route", format!("to lanes {}", lanes.join(", ")));
                    let way = match self.sim.time_to(destination, v.spec().class, v.lane()) {
                        Some((time, changes)) => {
                            format!("{time:.0} s from this lane's start, {changes} lane changes")
                        }
                        None => "none from here".into(),
                    };
                    row("way", way);
                }
                _ => row("route", "random".into()),
            }
            row("s", format!("{:.2} m", pos.s));
            row("t", format!("{:+.3} m", pos.t));
            row(
                "speed",
                format!("{:.1} km/h ({:.2} m/s)", v.speed() * 3.6, v.speed()),
            );
            let limit = self.sim.network().lane(v.lane()).speed_limit();
            row(
                "speed limit",
                format!(
                    "{:.0} km/h, wanting {:.0} km/h",
                    limit * 3.6,
                    self.sim.driver(id).map_or(limit, |d| d.desired_speed(v.spec(), limit)) * 3.6
                ),
            );
            row("accel", format!("{:+.2} m/s²", v.accel()));
            row(
                "stopping",
                if self.sim.stopping(id) { "yes" } else { "no" }.into(),
            );
            row("curvature", format!("{:+.4} /m", v.curvature()));
        });

        ui.separator();
        ui.strong("Acceleration terms");
        let related = related(&self.debug, id);
        let colour_of = |other: VehicleId| {
            related.iter().find(|&&(v, _)| v == other).map_or(SELECTED, |&(_, c)| c)
        };
        let terms: Vec<_> = self.debug.terms(id).collect();
        egui::Grid::new("accel terms").num_columns(4).striped(true).show(ui, |ui| {
            for heading in ["term", "entity", "gap", "m/s²"] {
                ui.strong(heading);
            }
            fill_row(ui);
            ui.end_row();
            for &(source, accel) in &terms {
                let (term, entity, gap) = describe(source);
                ui.label(term);
                match entity {
                    Some(Entity::Vehicle(other)) => {
                        ui.label(egui::RichText::new(number(other)).color(colour_of(other)))
                    }
                    Some(Entity::Lane(lane)) => ui.label(lane.0.to_string()),
                    None => ui.label(""),
                };
                ui.monospace(gap.map_or(String::new(), |gap| format!("{gap:.2} m")));
                let value = egui::RichText::new(format!("{accel:+.2}")).monospace();
                ui.label(match accel == v.accel() {
                    true => value.strong().color(SELECTED),
                    false => value,
                });
                ui.end_row();
            }
        });
        // The vehicle's acceleration is the minimum of its terms, so if none
        // matches, an input to the model wasn't recorded.
        if !terms.is_empty() && !terms.iter().any(|&(_, accel)| accel == v.accel()) {
            ui.colored_label(Color32::RED, "No recorded term matches the acceleration.");
        }

        // What the models last weighed up for it, a table for each model: a
        // row for each field, a column for each thing it weighed up.
        let evals = self.debug.decisions.get(&id).cloned().unwrap_or_default();
        let mut models: Vec<&'static str> = Vec::new();
        for (model, _) in &evals {
            if !models.contains(model) {
                models.push(model);
            }
        }
        for model in models {
            ui.separator();
            egui::CollapsingHeader::new(egui::RichText::new(format!("Model: {model}")).strong())
                .id_salt(("model", model))
                .default_open(false)
                .show(ui, |ui| {
                    let columns: Vec<_> =
                        evals.iter().filter(|e| e.0 == model).map(|e| &e.1).collect();
                    let mut names: Vec<&'static str> = Vec::new();
                    for &(name, _) in columns.iter().flat_map(|c| c.iter()) {
                        if !names.contains(&name) {
                            names.push(name);
                        }
                    }
                    egui::Grid::new(("model", model))
                        .num_columns(columns.len() + 1)
                        .striped(true)
                        .show(ui, |ui| {
                            for (k, name) in names.into_iter().enumerate() {
                                ui.label(name);
                                for column in &columns {
                                    let value = column.iter().find(|f| f.0 == name).map(|f| f.1);
                                    ui.label(match value {
                                        Some(Field::Num(x)) => format!("{x:+.2}"),
                                        Some(Field::Flag(b)) => if b { "yes" } else { "no" }.into(),
                                        Some(Field::Text(t)) => t.into(),
                                        Some(Field::None) | None => "–".into(),
                                    });
                                }
                                if k == 0 {
                                    fill_row(ui);
                                }
                                ui.end_row();
                            }
                        });
                });
        }

        ui.separator();
        ui.label(format!("Committed to {} barriers ahead", v.committed()));
        let decisions = self.debug.barriers.iter().filter(|b| b.0 == id);
        for &(_, lane, index, gap, decision, yields_to) in decisions {
            let what = match decision {
                BarrierDecision::Commit => "commits".to_string(),
                BarrierDecision::Pass => "passes for now".to_string(),
                BarrierDecision::Hold(accel) => format!("held, {accel:+.2} m/s²"),
            };
            let font = egui::TextStyle::Small.resolve(ui.style());
            let plain = egui::TextFormat::simple(font.clone(), ui.visuals().text_color());
            let mut text = egui::text::LayoutJob::default();
            let line = format!("Barrier {index} on lane {}, gap {gap:.1} m: {what}", lane.0);
            text.append(&line, 0.0, plain.clone());
            if let Some(other) = yields_to {
                text.append(", giving way to ", 0.0, plain);
                text.append(
                    &number(other),
                    0.0,
                    egui::TextFormat::simple(font, colour_of(other)),
                );
            }
            ui.add(egui::Label::new(text).wrap());
        }
    }

    /// The selected vehicle's controls, in two rows of equal-width buttons: lane
    /// changes, then speed caps.
    fn inspector_actions(&mut self, ui: &mut egui::Ui, id: VehicleId) {
        if let Some(error) = self.lane_change_error {
            ui.colored_label(Color32::RED, error.to_string());
        }
        let cap = self.sim.speed_cap(id);
        let gap = ui.spacing().item_spacing.x;
        let size = egui::vec2(
            (ui.available_width() - gap) / 2.0,
            ui.spacing().interact_size.y,
        );
        ui.horizontal(|ui| {
            if ui.add_sized(size, egui::Button::new("Change left")).clicked() {
                self.lane_change_error = self.sim.change_lane(id, Side::Left).err();
            }
            if ui.add_sized(size, egui::Button::new("Change right")).clicked() {
                self.lane_change_error = self.sim.change_lane(id, Side::Right).err();
            }
        });
        ui.horizontal(|ui| {
            // Each a toggle, drawn as an ordinary button when off and highlighted
            // when on; turning one on replaces the other.
            for (label, max_speed) in [("Stop", 0.0), ("20 km/h", SLOW_CAP)] {
                let on = cap == Some(max_speed);
                if ui.add_sized(size, egui::Button::new(label).selected(on)).clicked() {
                    if on {
                        self.sim.release(id);
                    } else {
                        self.sim.cap_speed(id, max_speed);
                    }
                }
            }
        });
    }

    /// Selects the vehicle under `point` (scene coordinates), or clears the selection.
    fn pick(&mut self, point: Pos2, tolerance: f64) {
        self.lane_change_error = None;
        let world = DVec2::new(point.x as f64, -point.y as f64);
        self.selected = self.sim.vehicle_at(world, tolerance);
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Space toggles pause, unless a text field is being edited. Consumed
        // before the panels are drawn, so a focused button doesn't also click.
        let typing = ui.ctx().egui_wants_keyboard_input();
        if !typing && ui.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Space)) {
            self.paused = !self.paused;
        }
        // The number keys pick a speed: 1×, 2×, 5× and 20×.
        for (key, speed) in [
            (egui::Key::Num1, 1.0),
            (egui::Key::Num2, 2.0),
            (egui::Key::Num3, 5.0),
            (egui::Key::Num4, 20.0),
            (egui::Key::Num5, 100.0),
        ] {
            if !typing && ui.input_mut(|i| i.consume_key(egui::Modifiers::NONE, key)) {
                self.speed = speed;
            }
        }

        egui::Panel::top("controls").show(ui, |ui| self.controls(ui));

        // Always shown at a fixed width, so selecting a vehicle doesn't move the scene.
        egui::Panel::right("inspector")
            .resizable(false)
            .exact_size(480.0)
            .show(ui, |ui| {
                // Content that is too wide scrolls instead of widening the panel,
                // which would shift the scene.
                if let Some(id) = self.selected {
                    // Pinned to the bottom, so the buttons stay put as the
                    // details above them grow and shrink.
                    if self.sim.get(id).is_some() {
                        egui::Panel::bottom("vehicle actions")
                            .frame(egui::Frame::new().inner_margin(egui::Margin::symmetric(0, 6)))
                            .show(ui, |ui| self.inspector_actions(ui, id));
                    }
                    egui::ScrollArea::both().auto_shrink(false).show(ui, |ui| {
                        ui.heading("Vehicle");
                        self.inspector(ui, id);
                    });
                } else {
                    // Vertical only, so the descriptions wrap.
                    egui::ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                        ui.heading("Debug view layers");
                        ui.small("Select a vehicle to show only its marks.");
                        self.layers.legend(ui);
                    });
                }
            });

        if !self.paused {
            let dt = ui.input(|i| i.stable_dt).min(0.1) as f64;
            self.step(dt * self.speed);
            ui.ctx().request_repaint();
        }

        egui::CentralPanel::default().show(ui, |ui| {
            let background = match self.view {
                View::Pretty => Color32::from_rgb(70, 110, 60),
                View::Debug => Color32::from_gray(20),
            };
            ui.painter().rect_filled(ui.max_rect(), 0.0, background);
            let scene = egui::Scene::new().zoom_range(0.1..=200.0).scroll_zooms(true);
            let related = self.selected.map(|id| related(&self.debug, id)).unwrap_or_default();
            // Wall-clock time, so a selected vehicle's path flows even while paused.
            let time = ui.input(|i| i.time);
            if self.selected.is_some() {
                ui.ctx().request_repaint();
            }
            // Where the path's dashes are: flowing on, and staying put on the
            // road as the vehicle moves.
            let selected = self.selected.and_then(|id| Some((id, self.sim.get(id)?)));
            let march = selected.map_or(0.0, |(id, v)| {
                PATH_FLOW * time - self.travelled.update(&self.sim, id, v)
            });
            let response = scene
                .show(ui, &mut self.scene_rect, |ui| match self.view {
                    View::Pretty => draw_pretty(
                        ui.painter(),
                        &self.sim,
                        &self.render,
                        self.selected,
                        &related,
                        &self.poses,
                        march,
                    ),
                    View::Debug => draw_debug(
                        ui.painter(),
                        &self.sim,
                        &self.debug,
                        self.selected,
                        &related,
                        &self.layers,
                        &self.poses,
                        march,
                    ),
                })
                .response;
            if response.clicked()
                && let Some(point) = response.hover_pos()
            {
                let to_global = ui.ctx().layer_transform_to_global(response.layer_id);
                let scale = to_global.map_or(1.0, |t| t.scaling);
                self.pick(point, (PICK_TOLERANCE / scale) as f64);
            }
        });
    }
}

fn draw_pretty(
    painter: &egui::Painter,
    sim: &Simulation,
    render: &RenderGeometry,
    selected: Option<VehicleId>,
    related: &[(VehicleId, Color32)],
    poses: &Poses,
    march: f64,
) {
    let asphalt = Color32::from_gray(60);
    for surface in &render.intersections {
        painter.add(fan_mesh(surface, asphalt));
    }
    for strip in &render.roads {
        painter.add(strip_mesh(strip, asphalt));
    }
    if let Some(id) = selected {
        draw_path(painter, sim, id, march);
    }
    let white = Stroke::new(0.15, Color32::from_gray(230));
    let yellow = Stroke::new(0.15, Color32::from_rgb(240, 200, 40));
    let stop = Stroke::new(0.3, Color32::from_gray(230));
    for marking in &render.markings {
        let points: Vec<Pos2> = marking.points.iter().map(|&p| to_scene(p)).collect();
        match marking.style {
            MarkingStyle::Edge => drop(painter.add(Shape::line(points, white))),
            MarkingStyle::Centre => drop(painter.add(Shape::line(points, yellow))),
            MarkingStyle::Stop => drop(painter.add(Shape::line(points, stop))),
            MarkingStyle::KeepClear => drop(painter.add(Shape::line(points, yellow))),
            // 3 m dashes every 9 m.
            MarkingStyle::Dashed => {
                for dash in dashes(&marking.points, 3.0, 9.0) {
                    painter.add(Shape::line(dash.into_iter().map(to_scene).collect(), white));
                }
            }
        }
    }
    painter.add(arrows_mesh(&render.arrows, Color32::from_gray(230)));
    draw_lights(painter, sim);
    for (id, v) in sim.vehicles() {
        // Where it is drawn, between steps: the indicators go with the body.
        let drawn = poses.corners(id, v, sim.alpha());
        let corners = drawn.map(to_scene).to_vec();
        let outline = if Some(id) == selected {
            Stroke::new(0.25, SELECTED)
        } else {
            Stroke::NONE
        };
        let colour = body_colour(id, selected, related, VEHICLE);
        painter.add(Shape::convex_polygon(corners, colour, outline));
        draw_indicators(painter, sim, v, drawn);
    }
}

/// Widens the last column of a grid to the edge of the panel, called in its
/// last cell, so the table and its stripes span the panel's width. Inside a
/// scroll area that scrolls sideways, the width available is only as wide as
/// the content was, so it reaches to the visible edge instead.
fn fill_row(ui: &mut egui::Ui) {
    let width = ui.clip_rect().right() - ui.cursor().left();
    ui.allocate_space(egui::vec2(width.max(0.0), 0.0));
}

/// How fast the dashes of a selected vehicle's path flow along it, in meters
/// a second of wall-clock time, and the lengths of its dashes and gaps.
const PATH_FLOW: f64 = 2.5;
const PATH_DASH: f32 = 3.0;
const PATH_GAP: f32 = 1.0;
/// How wide a selected vehicle's path is drawn, in meters.
const PATH_WIDTH: f32 = 1.0;

/// A vehicle's path, along its lanes' centerlines from its rear axle to the
/// end of the last lane it knows it will take, as a thick dashed line, its
/// first dash starting `march` meters along it, as far as the dashes have
/// flowed less how far the vehicle has gone, so they flow slowly the way it
/// goes and stay put on the road as it moves.
fn draw_path(painter: &egui::Painter, sim: &Simulation, id: VehicleId, march: f64) {
    let Some(v) = sim.get(id) else {
        return;
    };
    // From its rear axle to its front axle, then along its front axle's course:
    // its lanes' centerlines, offset along the quintic of a lane change in
    // progress, so the line runs smoothly onto its new lane.
    let front = v.front_s();
    let change = v.lane_change();
    let mut points = vec![
        to_scene(v.pos()),
        to_scene(v.pos() + v.dir() * v.spec().wheelbase()),
    ];
    // How far along its path each lane starts.
    let mut base = 0.0;
    let mut along = Vec::new();
    for &lane in v.path() {
        let geometry = sim.network().lane(lane).geometry();
        // Its vertices, and every meter through a lane change, ahead of the front axle.
        along.clear();
        let mut s = 0.0;
        let mut last: Option<DVec2> = None;
        for p in geometry.centerline() {
            s += last.map_or(0.0, |q| p.distance(q));
            last = Some(p);
            along.push(s);
        }
        if let Some(c) = change {
            let (from, to) = (
                (c.s0 - base).max(0.0),
                (c.end() - base).min(geometry.length()),
            );
            along.extend((0..).map(|k| from + k as f64).take_while(|&s| s < to));
        }
        along.retain(|&s| base + s > front);
        along.sort_by(f64::total_cmp);
        for &s in &along {
            let t = change.map_or(0.0, |c| c.offset_at(base + s));
            points.push(to_scene(geometry.lane_to_world(geometry.lane_pos(s, t))));
        }
        base += geometry.length();
    }
    let period = f64::from(PATH_DASH + PATH_GAP);
    let offset = march.rem_euclid(period) as f32;
    let stroke = Stroke::new(PATH_WIDTH, PATH);
    painter.extend(dashed_path(&points, stroke, PATH_DASH, PATH_GAP, offset));
}

/// A dashed line along `points`, its dashes `dash` long and `gap` apart, one
/// starting `offset` along it, the one before cut short where the line starts.
/// Each dash is one stroked path, bending with the line and joined at its
/// vertices, rather than a straight segment for each piece of it, which leaves
/// notches on the outside of a bend when the line is thick.
fn dashed_path(points: &[Pos2], stroke: Stroke, dash: f32, gap: f32, offset: f32) -> Vec<Shape> {
    let period = dash + gap;
    let mut shapes = Vec::new();
    // The dash being drawn, and where along the line it starts.
    let mut current = Vec::new();
    let mut start = offset.rem_euclid(period) - period;
    // How far along the line the piece from `a` starts.
    let mut along = 0.0;
    for piece in points.windows(2) {
        let (a, b) = (piece[0], piece[1]);
        let length = a.distance(b);
        if length == 0.0 {
            continue;
        }
        let at = |d: f32| a + (b - a) * ((d - along) / length);
        let end = along + length;
        loop {
            if current.is_empty() {
                if start + dash <= along {
                    start += period;
                    continue;
                }
                if start >= end {
                    break;
                }
                current.push(at(start.max(along)));
            }
            if start + dash <= end {
                current.push(at(start + dash));
                shapes.push(Shape::line(std::mem::take(&mut current), stroke));
                start += period;
            } else {
                current.push(b);
                break;
            }
        }
        along = end;
    }
    if current.len() >= 2 {
        shapes.push(Shape::line(current, stroke));
    }
    shapes
}

/// How far in from the body's front and rear ends the indicators sit, in
/// meters, and their radius.
const INDICATOR_INSET: f64 = 0.4;
const INDICATOR_RADIUS: f32 = 0.28;

/// Blinking indicators on the side a vehicle is indicating to, or changing
/// toward, at about 1.5 Hz, on its body as drawn, `corners`: rear left, rear
/// right, front right, front left.
fn draw_indicators(painter: &egui::Painter, sim: &Simulation, v: &Vehicle, corners: [DVec2; 4]) {
    let side = v
        .intent()
        .indicating()
        .or(v.lane_change().and(v.old_side()).map(Side::opposite));
    let Some(side) = side else {
        return;
    };
    if (sim.time() * 1.5).fract() >= 0.5 {
        return;
    }
    let [rear_left, rear_right, front_right, front_left] = corners;
    let (rear, front) = match side {
        Side::Left => (rear_left, front_left),
        Side::Right => (rear_right, front_right),
    };
    // A little in from each end, along that side.
    let along = (front - rear).normalize_or_zero() * INDICATOR_INSET;
    let color = Color32::from_rgb(255, 170, 0);
    for p in [rear + along, front - along] {
        painter.circle_filled(to_scene(p), INDICATOR_RADIUS, color);
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_debug(
    painter: &egui::Painter,
    sim: &Simulation,
    debug: &DebugState,
    selected: Option<VehicleId>,
    related: &[(VehicleId, Color32)],
    layers: &Layers,
    poses: &Poses,
    march: f64,
) {
    // With a vehicle selected, only its own marks are drawn; otherwise everyone's.
    // Vehicle bodies and the network are always drawn.
    let shown = |id: VehicleId| selected.is_none_or(|s| s == id);
    if let Some(id) = selected.filter(|_| layers.on(Layer::Path)) {
        draw_path(painter, sim, id, march);
    }
    let thin = Stroke::new(0.08, OUTLINE);
    for (_, lane) in sim.network().lanes() {
        if layers.on(Layer::LaneOutlines) {
            let geometry = lane.geometry();
            // Its sides solid, and its ends dashed and faint, where it meets the
            // lanes before and after it.
            let outline: Vec<_> = geometry.outline().collect();
            let n = outline.len();
            let (left, right) = outline.split_at(n / 2);
            for side in [left, right] {
                painter.add(Shape::line(
                    side.iter().map(|&p| to_scene(p)).collect(),
                    thin,
                ));
            }
            let faint = Stroke::new(0.08, OUTLINE.gamma_multiply(0.5));
            for ends in [
                [outline[n - 1], outline[0]],
                [left[left.len() - 1], right[0]],
            ] {
                painter.extend(Shape::dashed_line(&ends.map(to_scene), faint, 0.4, 0.3));
            }
        }

        if layers.on(Layer::Centerlines) {
            let centerline: Vec<_> = lane.geometry().centerline().map(to_scene).collect();
            for &p in &centerline {
                painter.circle_filled(p, 0.1, VERTEX);
            }
            painter.add(Shape::line(centerline, Stroke::new(0.04, CENTERLINE)));
        }

        if layers.on(Layer::Ticks) {
            let geometry = lane.geometry();
            for i in 0..=(geometry.length() / 10.0) as u32 {
                let s = 10.0 * i as f64;
                let pos = geometry.lane_pos(s, 0.0);
                let left = geometry.half_width(pos, Side::Left);
                let right = -geometry.half_width(pos, Side::Right);
                let ends = [left, right]
                    .map(|t| to_scene(geometry.lane_to_world(geometry.lane_pos(s, t))));
                painter.line_segment(ends, thin);
            }
        }

        // Every 20 m, from the centerline to each side's edge.
        if layers.on(Layer::Neighbors) {
            for side in [Side::Left, Side::Right] {
                let Some(n) = lane.adj(side) else { continue };
                let color = if n.can_change {
                    MAY_CHANGE
                } else {
                    MAY_NOT_CHANGE
                };
                let geometry = lane.geometry();
                let sign = if side == Side::Left { 1.0 } else { -1.0 };
                for i in 0..(geometry.length() / 20.0) as u32 {
                    let s = 20.0 * i as f64 + 5.0;
                    // Short of the edge, so neighbors' markers don't meet.
                    let edge = 0.9 * sign * geometry.half_width(geometry.lane_pos(s, 0.0), side);
                    let ends = [0.0, edge]
                        .map(|t| to_scene(geometry.lane_to_world(geometry.lane_pos(s, t))));
                    painter.line_segment(ends, Stroke::new(0.06, color));
                }
            }
        }
    }

    // Solid, over every lane's dashed ends.
    if layers.on(Layer::Barriers) {
        for (_, lane) in sim.network().lanes() {
            let geometry = lane.geometry();
            for barrier in lane.barriers() {
                let colour = match barrier.kind {
                    BarrierKind::Signal { movement, .. } => match sim.light(movement).light {
                        Light::Green => LIGHT_GREEN,
                        Light::Amber => LIGHT_AMBER,
                        Light::Red => LIGHT_RED,
                    },
                    BarrierKind::GiveWay => GIVE_WAY,
                    // Giving way to nothing, only taking turns: an all-way stop.
                    BarrierKind::Stop if barrier.yields.is_none() && barrier.ties.is_some() => {
                        ALL_WAY
                    }
                    BarrierKind::Stop => STOP,
                    BarrierKind::KeepClear { .. } => KEEP_CLEAR,
                };
                let pos = geometry.lane_pos(barrier.s, 0.0);
                let left = geometry.half_width(pos, Side::Left);
                let right = -geometry.half_width(pos, Side::Right);
                let edges =
                    [left, right].map(|t| geometry.lane_to_world(geometry.lane_pos(barrier.s, t)));
                // Along its line, if it has one, from edge to edge, the edges
                // taken as running straight on from `s`; and faintly square
                // across at `s`, where it is safe to stop.
                let ends = match barrier.line {
                    Some(line) => {
                        let tangent = geometry.tangent_at(pos);
                        let along = tangent.dot(line.normal);
                        edges.map(|e| e - line.short_of(e) / along * tangent)
                    }
                    None => edges,
                };
                if barrier.line.is_some() {
                    let faint = Stroke::new(0.08, colour.gamma_multiply(0.5));
                    painter.line_segment(edges.map(to_scene), faint);
                }
                painter.line_segment(ends.map(to_scene), Stroke::new(0.3, colour));
            }
        }
    }

    // Every mark on a vehicle is placed on its body as drawn, between steps, so
    // they stay together.
    let drawn = |id: VehicleId, v: &Vehicle| Drawn(poses.corners(id, v, sim.alpha()));
    for (id, v) in sim.vehicles() {
        if layers.on(Layer::Vehicles) {
            let color = body_colour(id, selected, related, VEHICLE);
            let body = drawn(id, v);
            painter.add(Shape::convex_polygon(
                body.0.map(to_scene).to_vec(),
                color,
                Stroke::NONE,
            ));
            // Its axles, across the body.
            let axle = body.axle(v.spec().rear_axle);
            painter.line_segment(axle.map(to_scene), Stroke::new(0.15, AXLE));
            let axle = body.axle(v.spec().front_axle);
            painter.line_segment(axle.map(to_scene), Stroke::new(0.15, AXLE));
        }

        // From the vehicle's stored cubic, drawn on its current lane only, so it
        // stops short if it crosses a lane's end.
        if layers.on(Layer::LaneChanges)
            && shown(id)
            && let Some(change) = v.lane_change()
        {
            let lane = sim.network().lane(v.lane());
            let samples = (0..=40).map(|i| change.s0 + change.length * i as f64 / 40.0);
            let points = samples
                .take_while(|&s| s <= lane.geometry().length())
                .map(|s| {
                    to_scene(
                        lane.geometry()
                            .lane_to_world(lane.geometry().lane_pos(s, change.offset_at(s))),
                    )
                })
                .collect();
            painter.add(Shape::line(points, Stroke::new(0.08, LANE_CHANGE)));
        }
    }

    if layers.on(Layer::Occupants) {
        for (lane_id, lane) in sim.network().lanes() {
            for occupant in sim.occupants(lane_id).iter().filter(|o| shown(o.id)) {
                let p = lane.geometry().lane_to_world(lane.geometry().lane_pos(occupant.s(), 0.0));
                let color = if occupant.is_ghost() { GHOST } else { OCCUPANT };
                painter.circle_filled(to_scene(p), 0.25, color);
            }
        }
    }

    // From the front bumper to the leader's rear bumper.
    if layers.on(Layer::Leaders) {
        for &(id, source, accel) in debug.accel_terms.iter().filter(|t| shown(t.0)) {
            let AccelSource::Leader { id: leader, .. } = source else {
                continue;
            };
            let (Some(v), Some(l)) = (sim.get(id), sim.get(leader)) else {
                continue;
            };
            let color = if accel == v.accel() {
                LEADER_BINDING
            } else {
                LEADER_OTHER
            };
            painter.line_segment(
                [
                    to_scene(drawn(id, v).front()),
                    to_scene(drawn(leader, l).rear()),
                ],
                Stroke::new(0.08, color),
            );
        }
    }

    // Dashed, from the front bumper to the rear bumper of the vehicle before it in
    // a merge's order, which it keeps behind as if on its own lane.
    if layers.on(Layer::Leaders) {
        for &(id, source, accel) in debug.accel_terms.iter().filter(|t| shown(t.0)) {
            let AccelSource::Merge { id: leader, .. } = source else {
                continue;
            };
            let (Some(v), Some(l)) = (sim.get(id), sim.get(leader)) else {
                continue;
            };
            let color = if accel == v.accel() {
                LEADER_BINDING
            } else {
                LEADER_OTHER
            };
            let ends = [
                to_scene(drawn(id, v).front()),
                to_scene(drawn(leader, l).rear()),
            ];
            painter.extend(Shape::dashed_line(
                &ends,
                Stroke::new(0.08, color),
                0.6,
                0.4,
            ));
        }
    }

    // In world space, as the check is made: the follower's predicted body as its
    // front reaches the other vehicle's rear, that vehicle's nearer rear corner,
    // and the gap between them across the follower's heading.
    if layers.on(Layer::PassChecks) {
        for &(id, check) in debug.pass_checks.iter().filter(|c| shown(c.0)) {
            let Some(v) = sim.get(id) else { continue };
            let (u, n) = (check.tangent, check.tangent.perp());
            let (back, ahead) = (-v.spec().rear_axle, v.spec().length - v.spec().rear_axle);
            let half = 0.5 * v.spec().width;
            let body = [(back, half), (ahead, half), (ahead, -half), (back, -half)]
                .map(|(x, y)| to_scene(check.pos + u * x + n * y))
                .to_vec();
            painter.add(Shape::closed_line(body, Stroke::new(0.08, PASS_BODY)));
            painter.circle_filled(to_scene(check.corner), 0.25, PASS_CORNER);
            // From the corner straight across to the body's near edge, extended
            // ahead: a negative gap is an overlap, drawn the same way in red.
            let along = (check.corner - check.pos).dot(u);
            let edge = match check.side {
                Side::Left => -half,
                Side::Right => half,
            };
            let foot = check.pos + u * along + n * edge;
            let color = if check.passed { PASS_OK } else { PASS_BLOCKED };
            painter.line_segment(
                [to_scene(check.corner), to_scene(foot)],
                Stroke::new(0.1, color),
            );
            painter.line_segment(
                [
                    to_scene(drawn(id, v).front()),
                    to_scene(check.pos + u * ahead),
                ],
                Stroke::new(0.03, PASS_BODY),
            );
        }
    }
}

// Debug view colors, shared by the drawing and the legend.
const OUTLINE: Color32 = Color32::from_gray(160);
const VERTEX: Color32 = Color32::from_gray(200);
const CENTERLINE: Color32 = Color32::from_gray(110);
const MAY_CHANGE: Color32 = Color32::from_rgb(80, 200, 80);
const MAY_NOT_CHANGE: Color32 = Color32::from_rgb(220, 70, 70);
const OCCUPANT: Color32 = Color32::from_rgb(200, 160, 255);
const GHOST: Color32 = Color32::from_rgb(255, 150, 40);
/// A vehicle, with none selected: white, as every vehicle unrelated to a
/// selected one is.
const VEHICLE: Color32 = Color32::WHITE;
/// Half-transparent white.
const AXLE: Color32 = Color32::from_rgba_premultiplied(0, 0, 0, 64);
const LANE_CHANGE: Color32 = Color32::from_rgb(250, 160, 60);
const LEADER_BINDING: Color32 = Color32::from_rgb(120, 230, 120);
const LEADER_OTHER: Color32 = Color32::from_rgb(60, 110, 60);
const LIGHT_GREEN: Color32 = Color32::from_rgb(60, 220, 90);
const LIGHT_AMBER: Color32 = Color32::from_rgb(250, 190, 40);
const LIGHT_RED: Color32 = Color32::from_rgb(230, 60, 50);
const GIVE_WAY: Color32 = Color32::from_rgb(240, 240, 240);
const STOP: Color32 = Color32::from_rgb(255, 120, 190);
const ALL_WAY: Color32 = Color32::from_rgb(190, 120, 255);
const KEEP_CLEAR: Color32 = Color32::from_rgb(240, 200, 40);
const PASS_CORNER: Color32 = Color32::from_rgb(255, 120, 200);
const PASS_BODY: Color32 = Color32::from_rgb(120, 170, 255);
const PASS_OK: Color32 = Color32::from_rgb(80, 220, 80);
const PASS_BLOCKED: Color32 = Color32::from_rgb(230, 60, 60);

/// A debug view layer that can be turned on and off.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Layer {
    Path,
    LaneOutlines,
    Centerlines,
    Ticks,
    Neighbors,
    Occupants,
    Vehicles,
    LaneChanges,
    Leaders,
    PassChecks,
    Barriers,
}

impl Layer {
    const ALL: [Layer; 11] = [
        Layer::Path,
        Layer::LaneOutlines,
        Layer::Centerlines,
        Layer::Ticks,
        Layer::Neighbors,
        Layer::Occupants,
        Layer::Vehicles,
        Layer::LaneChanges,
        Layer::Leaders,
        Layer::PassChecks,
        Layer::Barriers,
    ];

    /// Its name, its colors, and what they mean, for the legend.
    fn legend(self) -> (&'static str, &'static [Color32], &'static str) {
        match self {
            Layer::Path => (
                "Path",
                &[PATH],
                "The selected vehicle's path, the lanes it knows it will take, flowing the way it goes; under everything else",
            ),
            Layer::LaneOutlines => (
                "Lane outlines",
                &[OUTLINE],
                "Each lane's edges, and its ends dashed and fainter",
            ),
            Layer::Centerlines => (
                "Centerlines",
                &[CENTERLINE, VERTEX],
                "Each lane's centerline and its vertices",
            ),
            Layer::Ticks => ("Ticks", &[OUTLINE], "Across each lane every 10 m of s"),
            Layer::Neighbors => (
                "Neighbors",
                &[MAY_CHANGE, MAY_NOT_CHANGE],
                "Links to each side's neighbor: green where changing into it is allowed, red where not",
            ),
            Layer::Occupants => (
                "Occupants",
                &[OCCUPANT, GHOST],
                "Each occupant record, on the centerline at its s; orange for the ghost of a vehicle indicating into that lane",
            ),
            Layer::Vehicles => (
                "Vehicles",
                &[VEHICLE, SELECTED],
                "Solid bodies, each with its rear axle across it; the selected vehicle in yellow, those it gives way to and its acceleration terms name each in its own colour, as the inspector names them, and the rest white",
            ),
            Layer::LaneChanges => (
                "Lane changes",
                &[LANE_CHANGE],
                "The cubic path of each lane change in progress",
            ),
            Layer::Leaders => (
                "Leaders",
                &[LEADER_BINDING, LEADER_OTHER],
                "Front bumper to each leader's rear: bright when that leader sets the acceleration",
            ),
            Layer::PassChecks => (
                "Passing checks",
                &[PASS_CORNER, PASS_BODY, PASS_OK, PASS_BLOCKED],
                "Where a follower checked whether it can pass a vehicle ahead: its predicted body as its front reaches that vehicle's rear, that vehicle's nearer rear corner, and the gap from the corner across to the follower's edge, green if wide enough to pass and red if not. Only pairs where the follower could pass are checked: a vehicle moving out of the lane, or one the follower is moving away from. A fine line runs from the follower to its predicted body",
            ),
            Layer::Barriers => (
                "Barriers",
                &[
                    LIGHT_GREEN,
                    LIGHT_AMBER,
                    LIGHT_RED,
                    GIVE_WAY,
                    STOP,
                    ALL_WAY,
                    KEEP_CLEAR,
                ],
                "Each barrier across its lane: a signal in its light's colour, a give-way line in white, a stop line in pink, an all-way stop line in violet, the start of a keep-clear zone in yellow",
            ),
        }
    }
}

/// Which debug view layers are on.
struct Layers([bool; Layer::ALL.len()]);

impl Default for Layers {
    fn default() -> Self {
        let mut layers = [false; Layer::ALL.len()];
        for layer in [
            Layer::Path,
            Layer::LaneOutlines,
            Layer::Occupants,
            Layer::Vehicles,
            Layer::LaneChanges,
            Layer::PassChecks,
            Layer::Leaders,
            Layer::Barriers,
        ] {
            layers[layer as usize] = true;
        }
        Self(layers)
    }
}

impl Layers {
    fn on(&self, layer: Layer) -> bool {
        self.0[layer as usize]
    }

    /// A checkbox per layer, with its colors and what they mean.
    fn legend(&mut self, ui: &mut egui::Ui) {
        for layer in Layer::ALL {
            let (name, colors, meaning) = layer.legend();
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.0[layer as usize], name);
                for &color in colors {
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
                    ui.painter().rect_filled(rect, 2.0, color);
                }
            });
            ui.indent(name, |ui| ui.small(meaning));
        }
    }
}

/// Something a model weighed up for a vehicle: the model, and its fields, by
/// name, in order.
type ModelEval = (&'static str, Vec<(&'static str, Field)>);

/// Accumulates the simulation's debug events. Each group is cleared by its reset
/// event, so it always holds exactly what the last update of that group used.
#[derive(Default)]
struct DebugState {
    /// The last longitudinal update's terms: vehicle, what it reacted to, acceleration.
    accel_terms: Vec<(VehicleId, AccelSource, f64)>,
    /// Each vehicle's evaluations from its models' last decisions: the model,
    /// and what it weighed up, as named fields.
    decisions: HashMap<VehicleId, Vec<ModelEval>>,
    /// The last longitudinal update's checks for passing a diverging vehicle.
    pass_checks: Vec<(VehicleId, PassCheck)>,
    /// The last longitudinal update's barrier decisions, by vehicle: the lane,
    /// the barrier's index there, the gap to it, and what it did.
    /// Each barrier decision: the vehicle, the barrier's lane and index, the
    /// gap to it, what it did, and the vehicle it gives way to, if any.
    barriers: Vec<(
        VehicleId,
        LaneId,
        u16,
        f64,
        BarrierDecision,
        Option<VehicleId>,
    )>,
}

impl DebugState {
    /// Brings what it shows up to date with an event from a debug step: each
    /// group's reset clears what that group recorded before.
    fn apply(&mut self, event: DebugEvent) {
        match event {
            DebugEvent::RecomputeAccels => {
                self.accel_terms.clear();
                self.pass_checks.clear();
                self.barriers.clear();
            }
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.accel_terms.push((vehicle, source, accel))
            }
            // Not drawn: the debug view draws both axles on each body.
            DebugEvent::RecomputeMotion | DebugEvent::FrontAxle { .. } => {}
            DebugEvent::RecomputeDecision { vehicle } => {
                self.decisions.entry(vehicle).or_default().clear()
            }
            DebugEvent::ModelEval { vehicle, model, fields } => {
                self.decisions.entry(vehicle).or_default().push((model, fields))
            }
            DebugEvent::VehicleRemoved { vehicle } => {
                self.decisions.remove(&vehicle);
            }
            DebugEvent::PassCheck { vehicle, check } => self.pass_checks.push((vehicle, check)),
            DebugEvent::Barrier { vehicle, lane, index, gap, decision, yields_to } => {
                self.barriers.push((vehicle, lane, index, gap, decision, yields_to))
            }
        }
    }
}

impl DebugState {
    /// One vehicle's terms from the last longitudinal update.
    fn terms(&self, id: VehicleId) -> impl Iterator<Item = (AccelSource, f64)> + '_ {
        self.accel_terms
            .iter()
            .filter(move |t| t.0 == id)
            .map(|&(_, source, accel)| (source, accel))
    }
}

/// A vehicle's corners where it is drawn: rear left, rear right, front right,
/// front left.
#[derive(Clone, Copy)]
struct Drawn([DVec2; 4]);

impl Drawn {
    /// Midpoint of the front bumper.
    fn front(self) -> DVec2 {
        0.5 * (self.0[2] + self.0[3])
    }

    /// Midpoint of the rear bumper.
    fn rear(self) -> DVec2 {
        0.5 * (self.0[0] + self.0[1])
    }

    /// An axle's ends, at the sides of the body.
    fn axle(self, offset: f64) -> [DVec2; 2] {
        let length = self.0[0].distance(self.0[3]);
        let ahead = if length > 0.0 { offset / length } else { 0.0 };
        [
            self.0[0].lerp(self.0[3], ahead),
            self.0[1].lerp(self.0[2], ahead),
        ]
    }
}

/// A polygon every point of which can see its centroid, as a fan of triangles
/// from there.
fn fan_mesh(points: &[DVec2], color: Color32) -> Shape {
    let mut mesh = egui::Mesh::default();
    if points.is_empty() {
        return Shape::mesh(mesh);
    }
    let centre = points.iter().sum::<DVec2>() / points.len() as f64;
    mesh.colored_vertex(to_scene(centre), color);
    for &p in points {
        mesh.colored_vertex(to_scene(p), color);
    }
    let n = points.len() as u32;
    for i in 0..n {
        mesh.add_triangle(0, 1 + i, 1 + (i + 1) % n);
    }
    Shape::mesh(mesh)
}

/// Each signal barrier's lane, from the barrier on, in its movement's light's
/// colour while green or amber, and nothing while red, as the old visualiser
/// drew them: the ways through the junction open now. A far-side turn's goes on
/// in white from its give-way line: open, but giving way; and so does one that
/// may turn on red, while red.
fn draw_lights(painter: &egui::Painter, sim: &Simulation) {
    let network = sim.network();
    // Along `lane` from `from` to `to`, its vertices between and the points at
    // either end.
    let line = |lane: &traffic_sim::Lane, from: f64, to: f64, colour: Color32| {
        let geometry = lane.geometry();
        let at = |s: f64| to_scene(geometry.lane_to_world(geometry.lane_pos(s, 0.0)));
        let between = geometry.vertex_s().zip(geometry.centerline());
        let inner = between.filter(|&(s, _)| from < s && s < to).map(|(_, p)| to_scene(p));
        let points: Vec<Pos2> = [at(from)].into_iter().chain(inner).chain([at(to)]).collect();
        painter.add(Shape::line(points, Stroke::new(0.6, colour)));
    };
    let giving_way = Color32::from_rgba_unmultiplied(240, 240, 240, 170);
    // Along `lane` from `from` on: in white from a give-way line past it, and
    // only up to a light past it, as a hook turn's where it waits, which shows
    // its own. With the colour it ends in, if it reaches the lane's end.
    let rest = |lane: &traffic_sim::Lane, from: f64, colour: Color32| {
        let end = lane.geometry().length();
        let next = lane.barriers().iter().find(|b| {
            b.s > from && matches!(b.kind, BarrierKind::GiveWay | BarrierKind::Signal { .. })
        });
        match next {
            Some(b) if b.kind == BarrierKind::GiveWay => {
                line(lane, from, b.s, colour);
                line(lane, b.s, end, giving_way);
                Some(giving_way)
            }
            Some(b) => {
                line(lane, from, b.s, colour);
                None
            }
            None => {
                line(lane, from, end, colour);
                Some(colour)
            }
        }
    };
    for (_, lane) in network.lanes() {
        for barrier in lane.barriers() {
            let BarrierKind::Signal { movement, on_red } = barrier.kind else {
                continue;
            };
            let colour = match sim.light(movement).light {
                Light::Green => Color32::from_rgba_unmultiplied(60, 220, 90, 150),
                Light::Amber => Color32::from_rgba_unmultiplied(250, 190, 40, 170),
                Light::Red if on_red.is_some() => giving_way,
                Light::Red => continue,
            };
            let Some(mut colour) = rest(lane, barrier.s, colour) else {
                continue;
            };
            // On through the rest of its connector, the lanes after it in the
            // junction, each the only way on from the last.
            let mut on = lane;
            while let [next] = on.successors()[..] {
                let next_lane = network.lane(next);
                let on_through = on.junction().is_none_or(|j| next_lane.junction() == Some(j));
                if next_lane.junction().is_none() || !on_through {
                    break;
                }
                let Some(next_colour) = rest(next_lane, 0.0, colour) else {
                    break;
                };
                colour = next_colour;
                on = next_lane;
            }
        }
    }
}

/// Turn arrows, each its glyph turned to its heading and placed at its anchor.
fn arrows_mesh(arrows: &[Arrow], color: Color32) -> Shape {
    let mut mesh = egui::Mesh::default();
    for arrow in arrows {
        let glyph = match (arrow.left, arrow.straight, arrow.right) {
            (true, false, false) => &glyphs::TL,
            (false, false, true) => &glyphs::TR,
            (true, false, true) => &glyphs::TLR,
            (true, true, false) => &glyphs::TSL,
            (false, true, true) => &glyphs::TSR,
            (true, true, true) => &glyphs::TSLR,
            (false, true, false) => &glyphs::TS,
            (false, false, false) => continue,
        };
        // The glyph's +y along the heading, and its +x to the right of it.
        let ahead = DVec2::from_angle(arrow.heading);
        let right = DVec2::new(ahead.y, -ahead.x);
        let base = mesh.vertices.len() as u32;
        for &[x, y] in glyph.vertices {
            let p = arrow.pos + x as f64 * right + y as f64 * ahead;
            mesh.colored_vertex(to_scene(p), color);
        }
        for &[a, b, c] in glyph.triangles {
            mesh.add_triangle(base + a as u32, base + b as u32, base + c as u32);
        }
    }
    Shape::mesh(mesh)
}

/// A road's surface as a mesh of quads between its edges.
fn strip_mesh(strip: &Strip, color: Color32) -> Shape {
    let mut mesh = egui::Mesh::default();
    for (&l, &r) in strip.left.iter().zip(&strip.right) {
        mesh.colored_vertex(to_scene(l), color);
        mesh.colored_vertex(to_scene(r), color);
    }
    for i in 0..strip.left.len().saturating_sub(1) as u32 {
        let (l0, r0, l1, r1) = (2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3);
        mesh.add_triangle(l0, r0, r1);
        mesh.add_triangle(l0, r1, l1);
    }
    Shape::mesh(mesh)
}

/// Dashes `on` meters long every `period` meters along a polyline, each as a
/// polyline of its own, following bends.
fn dashes(points: &[DVec2], on: f64, period: f64) -> Vec<Vec<DVec2>> {
    let mut out: Vec<Vec<DVec2>> = Vec::new();
    let mut s = 0.0;
    for w in points.windows(2) {
        let length = w[0].distance(w[1]);
        if length == 0.0 {
            continue;
        }
        let at = |d: f64| w[0] + (w[1] - w[0]) * ((d - s) / length);
        // Each dash that overlaps this segment, cut to it.
        let mut k = (s / period).floor();
        while k * period < s + length {
            let (a, b) = ((k * period).max(s), (k * period + on).min(s + length));
            if a < b {
                let continues = out.last().is_some_and(|d| d.last() == Some(&at(a)));
                if !continues {
                    out.push(vec![at(a)]);
                }
                out.last_mut().unwrap().push(at(b));
            }
            k += 1.0;
        }
        s += length;
    }
    out
}

/// World meters (y up) to scene coordinates (y down).
fn to_scene(p: DVec2) -> Pos2 {
    Pos2::new(p.x as f32, -p.y as f32)
}

/// An empty simulation of a scenario's network, and what to draw for it; the
/// harness fills it.
/// The simulation for `scenario`, what to draw for its network, and the
/// vehicle to select: built afresh, or, for a freeze frame, restored.
fn new_sim(scenario: Scenario) -> (Recorded, RenderGeometry, Option<VehicleId>) {
    if scenario == Scenario::TurnerCycle {
        let frame = traffic_network::frames::turner_cycle();
        let sim = Recorded::new(Simulation::restore(frame.state));
        return (sim, frame.render, frame.selected);
    }
    let compiled = compile(&scenario.network()).expect("every scenario compiles");
    let sim = Recorded::new(Simulation::new(compiled.sim));
    (sim, compiled.render, None)
}

fn demo_car() -> VehicleSpec {
    VehicleSpec::passenger_car()
}
