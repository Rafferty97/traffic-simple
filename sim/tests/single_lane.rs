//! One vehicle drives the length of a single lane from rest.

use glam::DVec2;
use traffic_sim::{Lane, LaneId, Placement, Route, SimNetwork, Simulation};

mod common;
use common::car;

const SPEED_LIMIT: f64 = 14.0;
use common::DT;

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
    Lane::new(&pts, 3.5, SPEED_LIMIT, common::alone()).unwrap()
}

/// Runs a vehicle from rest at the start of the lane until it is removed.
/// Returns the time taken and the largest |t| seen.
fn drive(lane: Lane) -> (f64, f64) {
    let length = lane.geometry().length();
    let mut sim = common::sim(SimNetwork::new(vec![lane]));
    let at = Placement {
        lane: LaneId(0),
        s: 0.0,
        speed: 0.0,
        route: Route::default(),
        seed: 0,
    };
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
        sim.step_variable(DT);
        time += DT;
        assert!(time < 120.0, "never reached the end of the lane");
    }
    (time, max_t)
}

#[test]
fn straight_lane_is_driven_end_to_end() {
    let (time, max_t) = drive(lane(0.0));
    // Lower bound: the whole lane at the speed limit.
    assert!(time > lane(0.0).geometry().length() / SPEED_LIMIT);
    assert!(max_t < 1e-9, "drifted {max_t} m off the centerline");
}

#[test]
fn bent_lane_is_tracked_closely() {
    let (_, max_t) = drive(lane(1.0));
    assert!(max_t < 0.1, "drifted {max_t} m off the centerline");
}

#[test]
fn drivers_settle_at_their_share_of_the_limit() {
    for factor in [0.8, 1.2] {
        // Alone on a ring, so nothing gets in the way.
        let mut sim = common::sim(common::ring(230.0, SPEED_LIMIT));
        let at = Placement {
            lane: LaneId(0),
            s: 0.0,
            speed: 0.0,
            route: Route::default(),
            seed: 0,
        };
        let id = sim.insert(
            traffic_sim::VehicleSpec { speed_factor: factor, ..car() },
            at,
        );
        for _ in 0..(40.0 / DT) as usize {
            sim.step_variable(DT);
        }
        // Free-road IDM approaches the desired speed from below.
        let speed = sim.get(id).unwrap().speed();
        assert!(
            (speed - factor * SPEED_LIMIT).abs() < 0.2,
            "factor {factor}: {speed} m/s"
        );
    }
}

#[test]
fn a_capped_car_slows_to_its_cap_and_goes_on_once_released() {
    // A lone car on a long straight lane, so nothing but the cap holds it back.
    let points: Vec<_> = (0..=10).map(|i| DVec2::new(100.0 * i as f64, 0.0)).collect();
    let lane = Lane::new(&points, 3.5, SPEED_LIMIT, common::alone()).unwrap();
    let mut sim = common::sim(SimNetwork::new(vec![lane]));
    let id = sim.insert(
        car(),
        Placement {
            lane: LaneId(0),
            s: 0.0,
            speed: SPEED_LIMIT,
            route: Route::default(),
            seed: 0,
        },
    );
    let cap = 20.0 / 3.6;
    sim.cap_speed(id, cap);
    assert_eq!(sim.speed_cap(id), Some(cap));
    assert!(!sim.is_held(id));
    for _ in 0..(10.0 / DT) as usize {
        sim.step_variable(DT);
    }
    let speed = sim.get(id).unwrap().speed();
    assert!(
        (speed - cap).abs() < 0.05,
        "at {speed} m/s, capped at {cap}"
    );
    sim.release(id);
    assert_eq!(sim.speed_cap(id), None);
    for _ in 0..(5.0 / DT) as usize {
        sim.step_variable(DT);
    }
    assert!(
        sim.get(id).unwrap().speed() > cap + 2.0,
        "it should speed up again"
    );
}

#[test]
fn a_queue_comes_to_rest_at_the_minimum_gap() {
    // Two cars arrive at a stopped one. Each keeps its leader in view all the way
    // in, so none creeps closer than the minimum gap, and they come to rest there.
    let lane = Lane::new(
        &[DVec2::ZERO, DVec2::new(1000.0, 0.0)],
        3.5,
        SPEED_LIMIT,
        common::alone(),
    )
    .unwrap();
    let mut sim = common::sim(SimNetwork::new(vec![lane]));
    let spec = car();
    let at = |s, speed| Placement { lane: LaneId(0), s, speed, route: Route::default(), seed: 0 };
    let stopped = sim.insert(spec, at(500.0, 0.0));
    sim.hold(stopped);
    let a = sim.insert(spec, at(300.0, SPEED_LIMIT));
    let b = sim.insert(spec, at(250.0, SPEED_LIMIT));
    // Bumper to bumper: the cars' rear axles are a car length plus the gap apart.
    let gap = |sim: &Simulation, follower, leader| {
        let s = |id| sim.get(id).unwrap().lane_pos().s;
        s(leader) - s(follower) - spec.length
    };
    for _ in 0..(60.0 / DT) as usize {
        sim.step_variable(DT);
        for (follower, leader) in [(a, stopped), (b, a)] {
            let gap = gap(&sim, follower, leader);
            assert!(gap > spec.min_gap - 0.05, "closed to {gap} m");
        }
    }
    for (follower, leader) in [(a, stopped), (b, a)] {
        assert!(sim.get(follower).unwrap().speed() < 1e-3);
        let gap = gap(&sim, follower, leader);
        // Finishing its stop, braking firmly at walking pace, leaves it a few
        // centimetres further back than IDM alone would.
        assert!(gap < spec.min_gap + 0.2, "stopped {gap} m behind");
    }
}

#[test]
fn a_vehicle_slows_for_a_lower_limit_ahead_before_it_reaches_it() {
    // 500 m at 100 km/h, then 300 m at 50 km/h.
    let (fast, slow) = (100.0 / 3.6, 50.0 / 3.6);
    let lanes = vec![
        Lane::new(
            &[DVec2::ZERO, DVec2::new(500.0, 0.0)],
            3.5,
            fast,
            common::alone(),
        )
        .unwrap(),
        Lane::new(
            &[DVec2::new(500.0, 0.0), DVec2::new(800.0, 0.0)],
            3.5,
            slow,
            common::alone(),
        )
        .unwrap(),
    ];
    let mut network = SimNetwork::new(lanes);
    network.connect(LaneId(0), LaneId(1));
    let mut sim = common::sim(network);
    let spec = car();
    let at = Placement {
        lane: LaneId(0),
        s: 0.0,
        speed: fast,
        route: Route::default(),
        seed: 0,
    };
    let id = sim.insert(spec, at);
    let mut hardest: f64 = 0.0;
    loop {
        let v = sim.get(id).unwrap();
        let front = v.lane_pos().s + spec.length - spec.rear_axle;
        if v.lane() == LaneId(0) && front < 150.0 {
            // Far from the limit, it hasn't begun to slow.
            assert!(v.speed() > 0.95 * fast, "{} m/s at {front} m", v.speed());
        }
        if v.lane() == LaneId(1) || front >= 500.0 {
            // Down to about the limit as its front reaches the slower lane.
            let limit = spec.desired_speed(slow);
            assert!(v.speed() < limit + 1.0, "{} m/s against {limit}", v.speed());
            break;
        }
        hardest = hardest.min(v.accel());
        sim.step();
    }
    // About comfortably, never harder than twice that.
    assert!(hardest > -2.0 * spec.comfy_decel - 0.5, "{hardest} m/s²");
}
