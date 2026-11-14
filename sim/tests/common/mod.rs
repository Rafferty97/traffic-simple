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
