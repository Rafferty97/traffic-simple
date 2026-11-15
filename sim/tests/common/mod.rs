//! Shared helpers for tests and benchmarks.
#![allow(dead_code)]

#[cfg(feature = "serde")]
pub mod frame;
pub mod replay;

use std::f64::consts::TAU;

use glam::DVec2;
use traffic_sim::{Lane, LaneId, Placement, SimNetwork, Simulation, VehicleSpec};

pub fn car() -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        speed_factor: 1.0,
        max_accel: 1.5,
        comfy_decel: 2.0,
        time_headway: 1.2,
        min_gap: 2.0,
        rear_axle: 1.0,
        front_axle: 3.7,
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
        Lane::new(&points, WIDTH, speed_limit).unwrap()
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
pub fn fill_ring<D: traffic_sim::DebugSink>(sim: &mut Simulation<D>, count: usize) {
    fill_lane(sim, LaneId(0), count);
}

/// Inserts `count` cars at rest, evenly spaced along a lane.
pub fn fill_lane<D: traffic_sim::DebugSink>(sim: &mut Simulation<D>, lane: LaneId, count: usize) {
    let length = sim.network().lane(lane).length();
    for i in 0..count {
        let s = length * i as f64 / count as f64;
        sim.insert(car(), Placement { lane, s, speed: 0.0 });
    }
}

/// A sink that keeps the last longitudinal update's acceleration terms (`.0`)
/// and each vehicle's last lane-change evaluations (`.1`).
#[derive(Default)]
pub struct Terms(
    pub Vec<(traffic_sim::VehicleId, traffic_sim::AccelSource, f64)>,
    pub Vec<(traffic_sim::VehicleId, traffic_sim::LaneChangeEval)>,
);

impl traffic_sim::DebugSink for Terms {
    fn record(&mut self, event: impl FnOnce() -> traffic_sim::DebugEvent) {
        use traffic_sim::DebugEvent;
        match event() {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.0.push((vehicle, source, accel))
            }
            DebugEvent::RecomputeDecision { vehicle } | DebugEvent::VehicleRemoved { vehicle } => {
                self.1.retain(|e| e.0 != vehicle)
            }
            DebugEvent::LaneChangeEval { vehicle, eval } => self.1.push((vehicle, eval)),
            DebugEvent::RecomputeMotion
            | DebugEvent::Lookahead { .. }
            | DebugEvent::PassCheck { .. } => {}
        }
    }
}

impl Terms {
    /// The leader a vehicle reacted to in the last longitudinal update, and the gap.
    pub fn leader(&self, id: traffic_sim::VehicleId) -> Option<(traffic_sim::VehicleId, f64)> {
        self.0.iter().find_map(|&(vehicle, source, _)| match source {
            traffic_sim::AccelSource::Leader { id: leader, gap } if vehicle == id => {
                Some((leader, gap))
            }
            _ => None,
        })
    }
}
