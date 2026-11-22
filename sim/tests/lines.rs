//! Barriers with lines in world space: a vehicle stops by its nearest corner,
//! so on an oblique line a narrow one stops closer than a wide one, and none
//! crosses it.

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierKind, Lane, LaneId, Line, Movement, NoopSink, Phase, Placement, Route,
    SignalPlan, SimNetwork, Simulation, VehicleSpec,
};

mod common;
use common::car;

const LIMIT: f64 = 14.0;

/// How near its gap short of a line a vehicle stops, in meters: finishing its
/// stop, braking firmly at walking pace, leaves it a few centimetres further
/// back than IDM alone would (see `models::acceleration::stopping`).
const STOP_TOLERANCE: f64 = 0.1;
const HALF_WIDTH: f64 = 1.75;

/// A lane from the west to the origin and one on east from there, with a line
/// on the second through (20, 0) at `skew` from square, whose signal stays
/// red: its `s` where the line crosses the lane's edge furthest back, as far
/// as any vehicle on the lane could reach.
fn red_line(skew: f64) -> (SimNetwork, Line) {
    let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], 2.0 * HALF_WIDTH, LIMIT).unwrap();
    let mut net = SimNetwork::new(vec![
        lane(DVec2::new(-100.0, 0.0), DVec2::ZERO),
        lane(DVec2::ZERO, DVec2::new(100.0, 0.0)),
    ]);
    net.connect(LaneId(0), LaneId(1));
    let red = Movement { conflicts: 0b10, amber: 3.0, clearance: 1.0 };
    let green = Movement { conflicts: 0b01, ..red };
    let movement = net.add_signal(SignalPlan {
        movements: vec![red, green],
        phases: vec![Phase { green: 0b10, duration: 1e9 }],
    });
    let line = Line {
        point: DVec2::new(20.0, 0.0),
        normal: DVec2::new(-skew.cos(), skew.sin()),
    };
    let s = 20.0 - HALF_WIDTH * skew.tan().abs();
    let kind = BarrierKind::Signal { movement };
    net.add_barrier(
        LaneId(1),
        Barrier { line: Some(line), ..Barrier::new(s, kind) },
    );
    (net, line)
}

/// Where a vehicle like `spec` comes to rest for the line, from 60 m back at
/// the limit: how far its nearest corner is short of it then, and its nose's
/// x. Asserts no corner ever crosses it.
fn stop(skew: f64, spec: VehicleSpec) -> (f64, f64) {
    let (net, line) = red_line(skew);
    let mut sim = Simulation::new(net, NoopSink);
    let at = Placement {
        lane: LaneId(0),
        s: 40.0,
        speed: LIMIT,
        route: Route::random(0),
    };
    let id = sim.insert(spec, at);
    let short = |sim: &Simulation| {
        let corners = sim.get(id).unwrap().corners();
        corners.map(|c| line.short_of(c)).into_iter().fold(f64::INFINITY, f64::min)
    };
    while sim.time() < 30.0 {
        sim.step();
        assert!(
            short(&sim) > 0.0,
            "a corner crossed the line at {:.2} s",
            sim.time()
        );
    }
    let v = sim.get(id).unwrap();
    assert!(v.speed() < 0.01, "it never came to rest");
    let nose = v.pos().x + (spec.length - spec.rear_axle) * v.heading().cos();
    (short(&sim), nose)
}

/// A car with no spread in its gap to lines, so it stops exactly that short.
fn exact(width: f64) -> VehicleSpec {
    VehicleSpec { width, line_gap_spread: 0.0, ..car() }
}

#[test]
fn a_vehicle_stops_its_gap_short_of_an_oblique_line_by_its_nearest_corner() {
    let spec = exact(1.9);
    let (short, _) = stop(30f64.to_radians(), spec);
    assert!(
        (short - spec.line_gap).abs() < STOP_TOLERANCE,
        "it stopped {short:.3} m short"
    );
}

#[test]
fn on_an_oblique_line_a_narrow_vehicle_stops_closer_than_a_wide_one() {
    let skew = 30f64.to_radians();
    let (_, wide) = stop(skew, exact(1.9));
    let (_, narrow) = stop(skew, exact(1.2));
    // Its leading corner is that much nearer its nose's centre.
    let expected = (0.95 - 0.6) * skew.tan();
    let further = narrow - wide;
    assert!(
        (further - expected).abs() < 0.05,
        "{further:.3} m further, not {expected:.3} m"
    );
}

#[test]
fn a_square_line_stops_a_vehicle_where_its_position_alone_does() {
    let spec = exact(1.9);
    let (short, nose) = stop(0.0, spec);
    assert!(
        (short - spec.line_gap).abs() < STOP_TOLERANCE,
        "it stopped {short:.3} m short"
    );
    assert!(
        (nose - (20.0 - spec.line_gap)).abs() < STOP_TOLERANCE,
        "its nose is at {nose:.3}"
    );
}

#[test]
fn a_vehicle_never_stops_short_of_where_the_line_is_safe_to_reach() {
    // Skewed the other way, the line's `s` is just as far back, and the
    // vehicle still stops by the line, not there.
    let spec = exact(1.9);
    let (short, _) = stop(-30f64.to_radians(), spec);
    assert!(
        (short - spec.line_gap).abs() < STOP_TOLERANCE,
        "it stopped {short:.3} m short"
    );
}
