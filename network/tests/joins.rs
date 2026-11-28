//! Traffic across joins and where roads overlap: round a ring joined to itself,
//! through two roads merging into one and forking again, nobody overlapping
//! anybody, and across a road at another grade, as if it weren't there.

use glam::DVec2;
use traffic_network::{Compiled, Network, Path, Road, RoadId, RoadLane, Station, compile, kmh};
use traffic_sim::{Driver, Side, Simulation, VehicleSpec};

fn p(x: f64, y: f64) -> DVec2 {
    DVec2::new(x, y)
}

fn one_lane(net: &mut Network, path: Path) -> RoadId {
    net.add_road(Road {
        path,
        speed_limit: kmh(60.0),
        forward: vec![RoadLane::new(0.0, 3.5)],
        reverse: vec![],
    })
}

fn insert(sim: &mut Simulation, compiled: &Compiled, road: RoadId, s: f64) {
    let at = compiled.place(road, 0, s, kmh(50.0)).unwrap();
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
    // A whole turn clockwise, of radius 80.
    let path = Path::new(p(80.0, 0.0)).arc_to(DVec2::ZERO, -std::f64::consts::TAU);
    let mut net = Network::new(Side::Left);
    let ring = one_lane(&mut net, path);
    net.join(ring, ring);
    let compiled = compile(&net).unwrap();
    let mut sim = compiled.simulate();
    for i in 0..20 {
        insert(&mut sim, &compiled, ring, 25.0 * i as f64);
    }
    run(&mut sim, 60.0);
    assert_eq!(sim.vehicles().count(), 20, "nobody leaves a ring");
    assert!(
        sim.vehicles().all(|(_, v)| v.speed() > 1.0),
        "and everybody keeps going"
    );
}

/// Two one-lane roads 60 m apart, curving together over 150 m to meet end on
/// and merge into one, which forks again 200 m on: the roads joining one in
/// common, and the branches forking from it, are cut where their lanes start
/// or stop overlapping.
fn merging_and_forking() -> (Network, [RoadId; 4]) {
    let mut net = Network::new(Side::Left);
    // North of the merged road, `side` 1, or south, -1.
    let approach = |side: f64| Path::new(p(0.0, 30.0 * side)).line_to(p(150.0, 30.0 * side));
    let curve_in = |side: f64| {
        let out = 30.0 * side;
        Path::new(p(150.0, out)).cubic_to(p(225.0, out), p(225.0, 0.0), p(300.0, 0.0))
    };
    let branch = |side: f64| {
        let out = 30.0 * side;
        Path::new(p(500.0, 0.0)).cubic_to(p(575.0, 0.0), p(625.0, out), p(700.0, out))
    };
    let north = one_lane(&mut net, approach(1.0));
    let south = one_lane(&mut net, approach(-1.0));
    let north_in = one_lane(&mut net, curve_in(1.0));
    let south_in = one_lane(&mut net, curve_in(-1.0));
    let merged = one_lane(&mut net, Path::new(p(300.0, 0.0)).line_to(p(500.0, 0.0)));
    let left = one_lane(&mut net, branch(1.0));
    let right = one_lane(&mut net, branch(-1.0));
    net.join(north, north_in);
    net.join(south, south_in);
    net.join(north_in, merged);
    net.join(south_in, merged);
    net.join(merged, left);
    net.join(merged, right);
    (net, [north, south, north_in, left])
}

#[test]
fn roads_merging_are_cut_where_their_lanes_start_to_overlap() {
    let (net, [_, _, north_in, left]) = merging_and_forking();
    let compiled = compile(&net).unwrap();
    let sim = compiled.sim_network();
    let geometry = |lane| sim.lane(lane).geometry();
    let pieces = compiled.lanes(north_in, 0);
    assert_eq!(pieces.len(), 2, "cut once, where it starts to overlap");
    let south_in = net.road_id(3).unwrap();
    let theirs = compiled.lanes(south_in, 0);
    assert_eq!(theirs.len(), 2);
    // The first pieces are apart, and the second overlap from about their
    // starts.
    for &other in &theirs {
        assert_eq!(geometry(pieces[0]).overlap(geometry(other)), None);
    }
    let [on_mine, _] = geometry(pieces[1]).overlap(geometry(theirs[1])).unwrap();
    assert!(
        on_mine.start < 0.5,
        "overlapping from {} m in",
        on_mine.start
    );
    // The branches of the fork are cut where they part.
    let branch = compiled.lanes(left, 0);
    assert_eq!(branch.len(), 2, "cut once, where it stops overlapping");
    let right = net.road_id(6).unwrap();
    for &other in &compiled.lanes(right, 0) {
        assert_eq!(geometry(branch[1]).overlap(geometry(other)), None);
    }
}

#[test]
fn traffic_merges_from_two_roads_and_forks_again() {
    let (net, [north, south, ..]) = merging_and_forking();
    let compiled = compile(&net).unwrap();
    let mut sim = compiled.simulate();
    // Arriving side by side, so they have to take turns.
    for i in 0..6 {
        insert(&mut sim, &compiled, north, 10.0 + 20.0 * i as f64);
        insert(&mut sim, &compiled, south, 10.0 + 20.0 * i as f64);
    }
    run(&mut sim, 90.0);
    assert_eq!(
        sim.vehicles().count(),
        0,
        "everybody gets through and leaves"
    );
}

#[test]
fn traffic_crosses_a_road_at_another_grade_as_if_it_werent_there() {
    // A road east, and one north over it, as a bridge, neither joined to the
    // other or to anything in common.
    let mut net = Network::new(Side::Left);
    let east = one_lane(&mut net, Path::new(p(0.0, 0.0)).line_to(p(200.0, 0.0)));
    let north = one_lane(
        &mut net,
        Path::new(p(100.0, -100.0)).line_to(p(100.0, 100.0)),
    );
    let compiled = compile(&net).unwrap();
    assert_eq!(compiled.lanes(east, 0).len(), 1, "not cut");
    assert!(compiled.sim_network().declared_overlaps().is_empty());
    let mut sim = compiled.simulate();
    // Timed to reach the crossing together.
    insert(&mut sim, &compiled, east, 40.0);
    insert(&mut sim, &compiled, north, 40.0);
    for _ in 0..30 * 10 {
        sim.step();
        for (id, v) in sim.vehicles() {
            assert!(v.speed() > kmh(45.0), "{id:?} slowed to {} m/s", v.speed());
        }
    }
}

#[test]
fn roads_declared_to_overlap_are_cut_and_related_where_they_do() {
    // A road east, and beside it another that swerves in over its lane and
    // back out, joined to nothing in common.
    let roads = |declared: bool| {
        let mut net = Network::new(Side::Left);
        let east = one_lane(&mut net, Path::new(p(0.0, 0.0)).line_to(p(300.0, 0.0)));
        let swerve = Path::new(p(0.0, -6.0))
            .line_to(p(50.0, -6.0))
            .cubic_to(p(100.0, -6.0), p(100.0, -2.0), p(150.0, -2.0))
            .cubic_to(p(200.0, -2.0), p(200.0, -6.0), p(250.0, -6.0))
            .line_to(p(300.0, -6.0));
        let beside = one_lane(&mut net, swerve);
        if declared {
            net.overlap(east, beside);
        }
        (compile(&net).unwrap(), east, beside)
    };
    let (apart, east, _) = roads(false);
    assert_eq!(apart.lanes(east, 0).len(), 1);
    assert!(apart.sim_network().declared_overlaps().is_empty());

    let (related, east, beside) = roads(true);
    let (mine, theirs) = (related.lanes(east, 0), related.lanes(beside, 0));
    assert_eq!((mine.len(), theirs.len()), (3, 3), "cut in and out");
    let declared = related.sim_network().declared_overlaps();
    let pair = |a, b| declared.contains(&[a, b]) || declared.contains(&[b, a]);
    assert!(pair(mine[1], theirs[1]), "{declared:?}");
}

/// A two-lane road whose outer lane eases in over the inner from 150 m to
/// 230 m, the two running on as one lane: the road and the one after.
fn two_into_one(dropped: bool) -> (Network, RoadId, RoadId) {
    let mut net = Network::new(Side::Left);
    let outer = RoadLane {
        dropped,
        ..RoadLane::through(vec![
            Station::new(150.0, -5.25, 3.5),
            Station::new(230.0, -1.75, 3.5),
        ])
    };
    let two = net.add_road(Road {
        path: Path::new(p(0.0, 0.0)).line_to(p(250.0, 0.0)),
        speed_limit: kmh(60.0),
        forward: vec![RoadLane::new(-1.75, 3.5), outer],
        reverse: vec![],
    });
    let one = net.add_road(Road {
        path: Path::new(p(250.0, 0.0)).line_to(p(450.0, 0.0)),
        speed_limit: kmh(60.0),
        forward: vec![RoadLane::new(-1.75, 3.5)],
        reverse: vec![],
    });
    net.join(two, one);
    (net, two, one)
}

#[test]
fn traffic_zips_from_two_lanes_into_one_where_one_eases_in_over_the_other() {
    let (net, two, _) = two_into_one(false);
    let compiled = compile(&net).unwrap();
    let mut sim = compiled.simulate();
    // Side by side, so they have to take turns.
    for i in 0..7 {
        for lane in [0, 1] {
            let at = compiled.place(two, lane, 10.0 + 20.0 * i as f64, kmh(50.0)).unwrap();
            sim.insert(VehicleSpec::PASSENGER_CAR, Driver::DEFAULT, at);
        }
    }
    run(&mut sim, 90.0);
    assert_eq!(
        sim.vehicles().count(),
        0,
        "everybody gets through and leaves"
    );
}

#[test]
fn drivers_on_a_dropped_lane_change_out_of_it_before_its_taper() {
    let (net, two, one) = two_into_one(true);
    let compiled = compile(&net).unwrap();
    let mut sim = compiled.simulate();
    let destination = sim.add_destination(&compiled.lanes(one, 0));
    // In the lane that drops, all with room to change out of it but the
    // last, 40 m short of the taper, where a change at 50 km/h takes 55 m.
    let mut insert = |s: f64| {
        let mut at = compiled.place(two, 1, s, kmh(50.0)).unwrap();
        at.route = traffic_sim::Route::to(destination);
        sim.insert(VehicleSpec::PASSENGER_CAR, Driver::DEFAULT, at)
    };
    for i in 0..4 {
        insert(10.0 + 20.0 * i as f64);
    }
    let late = insert(110.0);
    let taper = compiled.lanes(two, 1)[1];
    let mut through = false;
    for _ in 0..30 * 40 {
        sim.step();
        for (id, v) in sim.vehicles() {
            match id == late {
                true => through |= v.lane() == taper,
                false => assert_ne!(v.lane(), taper, "{id:?} went into the taper"),
            }
        }
    }
    assert!(through, "too late to change, it goes through the taper");
    assert_eq!(
        sim.vehicles().count(),
        0,
        "everybody gets through and leaves"
    );
}
