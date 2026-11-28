//! Traffic across joins: round a ring joined to itself, and through two roads
//! merging into one and forking again, nobody overlapping anybody.

use glam::DVec2;
use traffic_network::{Network, Path, Road, RoadId, RoadLane, kmh, simulate};
use traffic_sim::{Driver, Placement, Side, Simulation, VehicleSpec};

const LANE: RoadLane = RoadLane { offset: 0.0, left: 1.75, right: 1.75 };

fn p(x: f64, y: f64) -> DVec2 {
    DVec2::new(x, y)
}

fn one_lane(net: &mut Network, path: Path) -> RoadId {
    net.add_road(Road {
        path,
        speed_limit: kmh(60.0),
        forward: vec![LANE],
        reverse: vec![],
    })
}

fn insert(sim: &mut Simulation, road: RoadId, s: f64) {
    let at = Placement {
        lane: road.lane(0),
        s,
        speed: kmh(50.0),
        route: Default::default(),
    };
    sim.insert(VehicleSpec::PASSENGER_CAR, Driver::DEFAULT, at);
}

/// Steps `sim` for `seconds`, panicking if two bodies ever overlap.
fn run(sim: &mut Simulation, seconds: f64) {
    for step in 0..(seconds * 30.0) as usize {
        sim.step();
        let bodies: Vec<_> = sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
        for (i, (a, ca)) in bodies.iter().enumerate() {
            for (b, cb) in &bodies[i + 1..] {
                assert!(!overlap(*ca, *cb), "{a:?} and {b:?} overlap at step {step}");
            }
        }
    }
}

/// Whether two convex bodies, given by their corners, overlap: by the
/// separating axis theorem, along the edges of each.
fn overlap(a: [DVec2; 4], b: [DVec2; 4]) -> bool {
    let separated = |axis: DVec2| {
        let span = |c: [DVec2; 4]| {
            let dots = c.map(|q| axis.dot(q));
            (
                dots.into_iter().fold(f64::MAX, f64::min),
                dots.into_iter().fold(f64::MIN, f64::max),
            )
        };
        let ((a0, a1), (b0, b1)) = (span(a), span(b));
        a1 < b0 || b1 < a0
    };
    let axes = [a[1] - a[0], a[2] - a[1], b[1] - b[0], b[2] - b[1]];
    !axes.iter().any(|&axis| separated(axis.perp()))
}

#[test]
fn traffic_goes_round_a_ring_joined_to_itself() {
    // A circle of radius 80, near enough, clockwise from four cubics.
    let (r, k) = (80.0, 0.5523 * 80.0);
    let path = Path::new(p(r, 0.0))
        .cubic_to(p(r, -k), p(k, -r), p(0.0, -r))
        .cubic_to(p(-k, -r), p(-r, -k), p(-r, 0.0))
        .cubic_to(p(-r, k), p(-k, r), p(0.0, r))
        .cubic_to(p(k, r), p(r, k), p(r, 0.0));
    let mut net = Network::new(Side::Left);
    let ring = one_lane(&mut net, path);
    net.join(ring, ring);
    let mut sim = simulate(&net).unwrap();
    for i in 0..20 {
        insert(&mut sim, ring, 25.0 * i as f64);
    }
    run(&mut sim, 60.0);
    assert_eq!(sim.vehicles().count(), 20, "nobody leaves a ring");
    assert!(
        sim.vehicles().all(|(_, v)| v.speed() > 1.0),
        "and everybody keeps going"
    );
}

#[test]
fn traffic_merges_from_two_roads_and_forks_again() {
    let mut net = Network::new(Side::Left);
    // Each approach curves in from 30 m out to run alongside the other, then
    // squeezes in to meet it end on: the lanes start overlapping where the
    // squeeze starts, the merge point (see `docs/known-issues.md`, Merges
    // starting far apart).
    // North of the merged road, `side` 1, or south, -1.
    let approach = |side: f64| {
        let (out, beside) = (30.0 * side, 1.75 * side);
        Path::new(p(0.0, out)).line_to(p(100.0, out)).cubic_to(
            p(175.0, out),
            p(175.0, beside),
            p(250.0, beside),
        )
    };
    let squeeze = |side: f64| {
        let beside = 1.75 * side;
        Path::new(p(250.0, beside)).cubic_to(p(275.0, beside), p(275.0, 0.0), p(300.0, 0.0))
    };
    let north = one_lane(&mut net, approach(1.0));
    let south = one_lane(&mut net, approach(-1.0));
    let north_in = one_lane(&mut net, squeeze(1.0));
    let south_in = one_lane(&mut net, squeeze(-1.0));
    let merged = one_lane(&mut net, Path::new(p(300.0, 0.0)).line_to(p(500.0, 0.0)));
    let branch = |y| Path::new(p(500.0, 0.0)).cubic_to(p(575.0, 0.0), p(625.0, y), p(700.0, y));
    let left = one_lane(&mut net, branch(30.0));
    let right = one_lane(&mut net, branch(-30.0));
    net.join(north, north_in);
    net.join(south, south_in);
    net.join(north_in, merged);
    net.join(south_in, merged);
    net.join(merged, left);
    net.join(merged, right);
    let mut sim = simulate(&net).unwrap();
    // Arriving side by side, so they have to take turns.
    for i in 0..6 {
        insert(&mut sim, north, 10.0 + 20.0 * i as f64);
        insert(&mut sim, south, 10.0 + 20.0 * i as f64);
    }
    run(&mut sim, 90.0);
    assert_eq!(
        sim.vehicles().count(),
        0,
        "everybody gets through and leaves"
    );
}
