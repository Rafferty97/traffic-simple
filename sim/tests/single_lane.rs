//! One vehicle drives the length of a single lane from rest.

use glam::DVec2;
use traffic_sim::{Lane, LaneId, NoopSink, Placement, SimNetwork, Simulation};

mod common;
use common::car;

const SPEED_LIMIT: f64 = 14.0;
const DT: f64 = 1.0 / 60.0;

/// 100 m straight, then `turn` radians of bend at 5 m chords, then 100 m straight.
fn lane(turn: f64) -> Lane {
    let steps = 20;
    let mut p = DVec2::new(100.0, 0.0);
    let mut pts = vec![DVec2::ZERO, p];
    for i in 0..steps {
        p += 5.0 * DVec2::from_angle(turn * (i as f64 + 0.5) / steps as f64);
        pts.push(p);
    }
    pts.push(p + 100.0 * DVec2::from_angle(turn));
    Lane::new(&pts, 3.5, SPEED_LIMIT).unwrap()
}

/// Runs a vehicle from rest at the start of the lane until it is removed.
/// Returns the time taken and the largest |t| seen.
fn drive(lane: Lane) -> (f64, f64) {
    let length = lane.length();
    let mut sim = Simulation::new(SimNetwork::new(vec![lane]), NoopSink);
    let at = Placement { lane: LaneId(0), s: 0.0, speed: 0.0 };
    let id = sim.insert(car(), at);

    let (mut time, mut max_t, mut last_s) = (0.0, 0.0f64, 0.0);
    while let Some(v) = sim.get(id) {
        let pos = v.lane_pos();
        assert!(
            v.speed() <= SPEED_LIMIT + 1e-9,
            "speed {} over the limit",
            v.speed()
        );
        assert!(pos.s >= last_s, "went backwards at s = {}", pos.s);
        if pos.s < length {
            max_t = max_t.max(pos.t.abs());
        }
        last_s = pos.s;
        sim.step(DT);
        time += DT;
        assert!(time < 120.0, "never reached the end of the lane");
    }
    (time, max_t)
}

#[test]
fn straight_lane_is_driven_end_to_end() {
    let (time, max_t) = drive(lane(0.0));
    // Lower bound: the whole lane at the speed limit.
    assert!(time > lane(0.0).length() / SPEED_LIMIT);
    assert!(max_t < 1e-9, "drifted {max_t} m off the centreline");
}

#[test]
fn bent_lane_is_tracked_closely() {
    let (_, max_t) = drive(lane(1.0));
    assert!(max_t < 0.1, "drifted {max_t} m off the centreline");
}
