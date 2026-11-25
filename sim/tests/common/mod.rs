//! Shared helpers for tests and benchmarks.
#![allow(dead_code)]

#[cfg(feature = "serde")]
pub mod frame;

use std::f64::consts::TAU;

use glam::DVec2;
use traffic_sim::{
    Config, DebugEvent, Lane, LaneId, Placement, Route, SimNetwork, Simulation, VehicleSpec,
};

/// A road no other lane is part of, for a lane that stands alone: each call
/// gives another.
pub fn alone() -> traffic_sim::RoadId {
    thread_local! {
        static NEXT: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
    }
    NEXT.with(|next| {
        next.set(next.get() + 1);
        traffic_sim::RoadId(next.get() - 1)
    })
}

/// The length of a step in the tests' standard simulation, in seconds: 30 Hz.
pub const DT: f64 = 1.0 / 30.0;

/// The tests' standard stepping: 30 Hz steps, with the slower updates every
/// third, at 10 Hz.
pub const CONFIG: Config = Config { dt: DT, macro_step: 3 };

/// A simulation of `network` stepping by [`CONFIG`].
pub fn sim(network: SimNetwork) -> Simulation {
    Simulation::with_config(network, CONFIG)
}

/// What a test collects from a simulation's debug events, as they come.
pub trait Collect {
    fn collect(&mut self, event: DebugEvent);
}

/// A simulation stepped with its debug events, each handed to `sink` as it
/// comes. It derefs to the simulation, so it reads and changes like one.
pub struct Watched<C> {
    sim: Simulation,
    sink: C,
    events: Vec<DebugEvent>,
}

impl<C: Collect> Watched<C> {
    pub fn new(sim: Simulation, sink: C) -> Self {
        Self { sim, sink, events: Vec::new() }
    }

    /// One step, as [`Simulation::step`], collecting its events.
    pub fn step(&mut self) {
        self.sim.step_debug(&mut self.events);
        self.drain();
    }

    /// As [`Simulation::step_variable`], collecting its events.
    pub fn step_variable(&mut self, dt: f64) -> u32 {
        let steps = self.sim.step_variable_debug(dt, &mut self.events);
        self.drain();
        steps
    }

    /// What it has collected.
    pub fn sink(&self) -> &C {
        &self.sink
    }

    fn drain(&mut self) {
        for event in self.events.drain(..) {
            self.sink.collect(event);
        }
    }
}

impl<C> std::ops::Deref for Watched<C> {
    type Target = Simulation;

    fn deref(&self) -> &Simulation {
        &self.sim
    }
}

impl<C> std::ops::DerefMut for Watched<C> {
    fn deref_mut(&mut self) -> &mut Simulation {
        &mut self.sim
    }
}

/// A simulation of `network` stepping by [`CONFIG`], watched by `sink`.
pub fn watched<C: Collect>(network: SimNetwork, sink: C) -> Watched<C> {
    Watched::new(sim(network), sink)
}

/// How many standard steps take `seconds`.
pub fn steps(seconds: f64) -> u64 {
    (seconds / DT).round() as u64
}

pub fn car() -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        max_accel: 1.5,
        comfy_decel: 2.0,
        time_headway: 1.2,
        min_gap: 2.0,
        rear_axle: 1.0,
        front_axle: 3.7,
    }
}

/// The tests' driver: at the limit exactly, holding its speed without
/// dawdling, and stopping exactly a metre short of lines, so runs are
/// deterministic and easy to reason about.
pub fn driver() -> traffic_sim::Driver {
    traffic_sim::Driver {
        speed_factor: 1.0,
        accel_noise: 0.0,
        line_gap: 1.0,
        line_gap_spread: 0.0,
        courtesy: 1.0,
        politeness: 0.2,
        assertiveness: 0.0,
        seed: 0,
    }
}

/// A circular lane of the given circumference that is its own successor.
pub fn ring(circumference: f64, speed_limit: f64) -> SimNetwork {
    ring_road(circumference, 1, speed_limit)
}

/// A ring road of concentric lanes, each its own successor, with lane changes
/// allowed both ways between neighbors. Lane 0 is the innermost, with the given
/// circumference. Travel is counterclockwise, so lane `i` is on the left of lane `i + 1`.
///
/// Chords are sized for at most 1 cm of sagitta. Every lane starts and ends at
/// angle 0, at the midpoint of a chord, so there is no kink at the seam and the
/// lanes start and end together.
pub fn ring_road(circumference: f64, lanes: usize, speed_limit: f64) -> SimNetwork {
    const WIDTH: f64 = 3.5;
    let lane = |radius: f64| {
        let max_chord = (8.0 * radius * 0.01).sqrt();
        let n = (TAU * radius / max_chord).ceil() as usize;
        let vertex = |i: usize| radius * DVec2::from_angle(TAU * (i as f64 + 0.5) / n as f64);
        let seam = 0.5 * (vertex(n - 1) + vertex(0));
        let mut points = vec![seam];
        points.extend((0..n).map(vertex));
        points.push(seam);
        Lane::new(&points, WIDTH, speed_limit, alone()).unwrap()
    };
    let inner = circumference / TAU;
    let mut network = SimNetwork::new((0..lanes).map(|i| lane(inner + WIDTH * i as f64)).collect());
    for i in 0..lanes as u32 {
        network.connect(LaneId(i), LaneId(i));
        if i > 0 {
            network.set_adjacent(LaneId(i - 1), LaneId(i), true, true);
        }
    }
    network.validate().expect("ring road is a valid network");
    network
}

/// Inserts `count` cars at rest, evenly spaced around lane 0 of a ring.
pub fn fill_ring(sim: &mut Simulation, count: usize) {
    fill_lane(sim, LaneId(0), count);
}

/// Inserts `count` cars at rest, evenly spaced along a lane.
pub fn fill_lane(sim: &mut Simulation, lane: LaneId, count: usize) {
    let length = sim.network().lane(lane).geometry().length();
    for i in 0..count {
        let s = length * i as f64 / count as f64;
        sim.insert(
            car(),
            driver(),
            Placement { lane, s, speed: 0.0, route: Route::default() },
        );
    }
}

/// A collector that keeps the last longitudinal update's acceleration terms (`.0`)
/// and each vehicle's last model evaluations (`.1`), as their fields.
#[derive(Default)]
pub struct Terms(
    pub Vec<(traffic_sim::VehicleId, traffic_sim::AccelSource, f64)>,
    pub  Vec<(
        traffic_sim::VehicleId,
        Vec<(&'static str, traffic_sim::Field)>,
    )>,
);

impl Collect for Terms {
    fn collect(&mut self, event: DebugEvent) {
        match event {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.0.push((vehicle, source, accel))
            }
            DebugEvent::RecomputeDecision { vehicle } | DebugEvent::VehicleRemoved { vehicle } => {
                self.1.retain(|e| e.0 != vehicle)
            }
            DebugEvent::ModelEval { vehicle, fields, .. } => self.1.push((vehicle, fields)),
            DebugEvent::RecomputeMotion
            | DebugEvent::FrontAxle { .. }
            | DebugEvent::PassCheck { .. }
            | DebugEvent::Barrier { .. } => {}
        }
    }
}

impl Terms {
    /// The leader a vehicle reacted to in the last longitudinal update, and the gap.
    pub fn leader(&self, id: traffic_sim::VehicleId) -> Option<(traffic_sim::VehicleId, f64)> {
        self.0.iter().find_map(|&(vehicle, source, _)| match source {
            traffic_sim::AccelSource::Leader { id: leader, gap, .. } if vehicle == id => {
                Some((leader, gap))
            }
            _ => None,
        })
    }
}

/// Whether two convex bodies, given by their corners, overlap: by the separating
/// axis theorem, along the edges of each.
pub fn overlap(a: [glam::DVec2; 4], b: [glam::DVec2; 4]) -> bool {
    let separated = |axis: glam::DVec2| {
        let span = |c: [glam::DVec2; 4]| {
            c.iter()
                .map(|&q| axis.dot(q))
                .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| {
                    (lo.min(d), hi.max(d))
                })
        };
        let ((a0, a1), (b0, b1)) = (span(a), span(b));
        a1 < b0 || b1 < a0
    };
    let axes = [a[1] - a[0], a[2] - a[1], b[1] - b[0], b[2] - b[1]];
    !axes.iter().any(|&axis| separated(axis.perp()))
}

/// The vehicles in a junction's queue, an entry for each stretch it was handed
/// over for.
#[allow(dead_code)]
pub fn queued(sim: &Simulation, junction: traffic_sim::JunctionId) -> Vec<traffic_sim::VehicleId> {
    sim.queue(junction).to_vec()
}
