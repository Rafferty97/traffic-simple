//! Shared helpers for tests and benchmarks.
#![allow(dead_code)]

use std::f64::consts::TAU;

use glam::DVec2;
use traffic_sim::{Lane, LaneId, Placement, SimNetwork, Simulation, VehicleSpec};

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

/// A circular lane of the given circumference that is its own successor.
///
/// Chords are sized for at most 1 cm of sagitta. The lane starts and ends at the
/// midpoint of one chord, so there is no kink at the seam.
pub fn ring(circumference: f64, speed_limit: f64) -> SimNetwork {
    let radius = circumference / TAU;
    let max_chord = (8.0 * radius * 0.01).sqrt();
    let n = (circumference / max_chord).ceil() as usize;
    let vertex = |i: usize| radius * DVec2::from_angle(TAU * i as f64 / n as f64);
    let seam = 0.5 * (vertex(n - 1) + vertex(0));
    let mut points = vec![seam];
    points.extend((0..n).map(vertex));
    points.push(seam);
    let lane = Lane::new(&points, 3.5, speed_limit).unwrap();
    let mut network = SimNetwork::new(vec![lane]);
    network.connect(LaneId(0), LaneId(0));
    network
}

/// Inserts `count` cars at rest, evenly spaced around a ring from [`ring`].
pub fn fill_ring<D: traffic_sim::DebugSink>(sim: &mut Simulation<D>, count: usize) {
    let length = sim.network().lane(LaneId(0)).length();
    for i in 0..count {
        let s = length * i as f64 / count as f64;
        sim.insert(car(), Placement { lane: LaneId(0), s, speed: 0.0 });
    }
}

/// A sink that keeps the last longitudinal update's acceleration terms.
#[derive(Default)]
pub struct Terms(pub Vec<(traffic_sim::VehicleId, traffic_sim::AccelSource, f64)>);

impl traffic_sim::DebugSink for Terms {
    fn record(&mut self, event: impl FnOnce() -> traffic_sim::DebugEvent) {
        use traffic_sim::DebugEvent;
        match event() {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.0.push((vehicle, source, accel))
            }
            DebugEvent::RecomputeMotion | DebugEvent::Lookahead { .. } => {}
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
