//! Junctions: where their lanes conflict, and how their queues keep vehicles on
//! conflicting lanes apart.

use glam::DVec2;
use traffic_sim::{
    ConflictKind, JunctionId, Lane, LaneId, NoopSink, Placement, Side, SimNetwork, Simulation,
    VehicleId,
};

mod common;
use common::car;

const DT: f64 = 1.0 / 60.0;
const WIDTH: f64 = 3.5;

fn lane(points: &[DVec2]) -> Lane {
    Lane::new(points, WIDTH, 14.0).unwrap()
}

fn p(x: f64, y: f64) -> DVec2 {
    DVec2::new(x, y)
}

/// Two roads crossing at right angles at the origin. Lane 0 approaches from the
/// west and lane 1 from the south; lanes 2 and 3 cross the junction, 20 m long
/// each, centred on the origin; lanes 4 and 5 leave it. Lanes 2 and 3 are the
/// junction.
fn crossroads() -> (SimNetwork, JunctionId) {
    let mut net = SimNetwork::new(vec![
        lane(&[p(-100.0, 0.0), p(-10.0, 0.0)]),
        lane(&[p(0.0, -100.0), p(0.0, -10.0)]),
        lane(&[p(-10.0, 0.0), p(10.0, 0.0)]),
        lane(&[p(0.0, -10.0), p(0.0, 10.0)]),
        lane(&[p(10.0, 0.0), p(100.0, 0.0)]),
        lane(&[p(0.0, 10.0), p(0.0, 100.0)]),
    ]);
    for (from, to) in [(0, 2), (1, 3), (2, 4), (3, 5)] {
        net.connect(LaneId(from), LaneId(to));
    }
    let junction = net.add_junction(&[LaneId(2), LaneId(3)]);
    (net, junction)
}

/// Two neighbouring lanes, 0 on the left and 1 on the right, each into its own
/// 30 m connector, 2 and 3, which meet at the origin and both lead into lane 4.
/// The connectors are the junction.
fn merge() -> (SimNetwork, JunctionId) {
    let mut net = SimNetwork::new(vec![
        lane(&[p(-130.0, 0.0), p(-30.0, 0.0)]),
        lane(&[p(-130.0, -WIDTH), p(-30.0, -WIDTH)]),
        lane(&[p(-30.0, 0.0), p(0.0, 0.0)]),
        lane(&[p(-30.0, -WIDTH), p(0.0, 0.0)]),
        lane(&[p(0.0, 0.0), p(300.0, 0.0)]),
    ]);
    for (from, to) in [(0, 2), (1, 3), (2, 4), (3, 4)] {
        net.connect(LaneId(from), LaneId(to));
    }
    net.set_adjacent(LaneId(0), LaneId(1), true, true);
    let junction = net.add_junction(&[LaneId(2), LaneId(3)]);
    (net, junction)
}

fn at(lane: u32, s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(lane), s, speed }
}

/// Whether two convex bodies, given by their corners, overlap: by the separating
/// axis theorem, along the edges of each.
fn overlap(a: [DVec2; 4], b: [DVec2; 4]) -> bool {
    let separated = |axis: DVec2| {
        let span = |c: [DVec2; 4]| {
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

/// Steps until every vehicle has left, checking after each step that no two
/// bodies overlap. Panics if they take longer than `limit` seconds.
fn run_clear(sim: &mut Simulation, limit: f64) {
    let mut time = 0.0;
    while sim.vehicles().next().is_some() {
        sim.step(DT);
        time += DT;
        assert!(
            time < limit,
            "vehicles still in the network after {limit} s"
        );
        let bodies: Vec<(VehicleId, [DVec2; 4])> =
            sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
        for (i, &(a, body_a)) in bodies.iter().enumerate() {
            for &(b, body_b) in &bodies[i + 1..] {
                assert!(
                    !overlap(body_a, body_b),
                    "{a:?} and {b:?} overlap at {time:.2} s"
                );
            }
        }
    }
}

#[test]
fn crossing_lanes_conflict_where_their_outlines_overlap() {
    let (net, _) = crossroads();
    let conflict = net.conflict_between(LaneId(2), LaneId(3)).unwrap();
    assert_eq!(conflict.kind, ConflictKind::Cross);
    // Each lane is 3.5 m wide and crosses the other 10 m along it.
    for range in [&conflict.mine, &conflict.theirs] {
        assert!((range.start - 8.25).abs() < 1e-9, "{range:?}");
        assert!((range.end - 11.75).abs() < 1e-9, "{range:?}");
    }
    assert_eq!(net.lane(LaneId(2)).junction(), Some(JunctionId(0)));
    assert_eq!(net.lane(LaneId(0)).junction(), None);
    // Not in the junction, so not looked at.
    assert!(net.conflict_between(LaneId(4), LaneId(5)).is_none());
}

#[test]
fn lanes_into_the_same_lane_merge() {
    let (net, _) = merge();
    let conflict = net.conflict_between(LaneId(3), LaneId(2)).unwrap();
    assert_eq!(conflict.kind, ConflictKind::Merge);
    assert!(conflict.mine.end > 29.9 && conflict.theirs.end > 29.9);
}

#[test]
fn neighbours_branches_of_a_fork_and_successive_lanes_dont_conflict() {
    // Lane 0 forks into 1 and 2, which overlap where they part; lane 3 is
    // beside lane 0 and touches it along an edge; lane 4 follows lane 1.
    // The branches set off straight, so they square off where lane 3 ends.
    let mut net = SimNetwork::new(vec![
        lane(&[p(0.0, 0.0), p(20.0, 0.0)]),
        lane(&[p(20.0, 0.0), p(25.0, 0.0), p(45.0, 4.0)]),
        lane(&[p(20.0, 0.0), p(25.0, 0.0), p(45.0, -4.0)]),
        lane(&[p(0.0, -WIDTH), p(20.0, -WIDTH)]),
        lane(&[p(45.0, 4.0), p(65.0, 4.0)]),
    ]);
    net.connect(LaneId(0), LaneId(1));
    net.connect(LaneId(0), LaneId(2));
    net.connect(LaneId(1), LaneId(4));
    net.set_adjacent(LaneId(0), LaneId(3), true, true);
    net.add_junction(&[0, 1, 2, 3, 4].map(LaneId));
    for a in 0..5 {
        for b in 0..5 {
            assert!(
                net.conflict_between(LaneId(a), LaneId(b)).is_none(),
                "{a} and {b}"
            );
        }
    }
}

#[test]
#[should_panic(expected = "at most one junction")]
fn a_lane_belongs_to_at_most_one_junction() {
    let (mut net, _) = crossroads();
    net.add_junction(&[LaneId(2)]);
}

#[test]
fn the_second_to_register_waits_for_the_first_to_clear_the_crossing() {
    let (net, junction) = crossroads();
    let mut sim = Simulation::new(net, NoopSink);
    // Level with each other, 30 m out, at the same speed.
    let a = sim.insert(car(), at(0, 60.0, 10.0));
    let b = sim.insert(car(), at(1, 60.0, 10.0));
    // The overlap is 8.25 m to 11.75 m along each lane through the junction.
    let spec = car();
    let (front, rear) = (spec.length - spec.rear_axle, spec.rear_axle);
    let mut both = false;
    let mut b_slowest = f64::INFINITY;
    for _ in 0..(20.0 / DT) as usize {
        sim.step(DT);
        if sim.queue(junction) == [a, b] {
            both = true;
        }
        if let (Some(va), Some(vb)) = (sim.get(a), sim.get(b)) {
            b_slowest = b_slowest.min(vb.speed());
            let a_in = va.lane() == LaneId(0)
                || (va.lane() == LaneId(2) && va.lane_pos().s - rear < 11.75);
            let b_in = vb.lane() == LaneId(3) && vb.lane_pos().s + front > 8.25;
            assert!(
                !(a_in && b_in),
                "the second enters before the first has cleared"
            );
        }
    }
    assert!(both, "both register, the one on the lower lane first");
    assert!(b_slowest < 5.0, "the second slows to let the first through");
    assert!(
        sim.get(a).is_none() && sim.get(b).is_none(),
        "both get through"
    );
    assert!(sim.queue(junction).is_empty(), "and leave the queue");
}

#[test]
fn streams_from_both_sides_of_a_crossing_get_through_without_touching() {
    let (net, _) = crossroads();
    let mut sim = Simulation::new(net, NoopSink);
    for i in 0..4 {
        sim.insert(car(), at(0, 70.0 - 12.0 * i as f64, 8.0));
        sim.insert(car(), at(1, 75.0 - 12.0 * i as f64, 8.0));
    }
    run_clear(&mut sim, 60.0);
}

#[test]
fn streams_from_both_lanes_of_a_merge_take_turns_without_touching() {
    let (net, _) = merge();
    let mut sim = Simulation::new(net, NoopSink);
    for i in 0..5 {
        sim.insert(car(), at(0, 90.0 - 15.0 * i as f64, 10.0));
        sim.insert(car(), at(1, 85.0 - 15.0 * i as f64, 10.0));
    }
    run_clear(&mut sim, 60.0);
}

#[test]
fn a_fast_follower_that_registers_first_goes_after_the_vehicle_ahead_of_it() {
    let (net, junction) = crossroads();
    let mut sim = Simulation::new(net, NoopSink);
    // The follower is fast and sees the junction from further back than its slow
    // leader does, so it registers first.
    let leader = sim.insert(car(), at(0, 60.0, 2.0));
    let follower = sim.insert(car(), at(0, 30.0, 14.0));
    let mut alone = false;
    let mut both = false;
    for _ in 0..(10.0 / DT) as usize {
        sim.step(DT);
        let queue = sim.queue(junction);
        alone |= queue == [follower];
        if queue.contains(&leader) && queue.contains(&follower) {
            both = true;
            assert_eq!(queue, [leader, follower]);
        }
    }
    assert!(alone, "the follower registers first");
    assert!(both, "then the leader, ahead of it");
}

#[test]
fn a_vehicle_that_cuts_in_ahead_goes_first() {
    let (net, junction) = merge();
    let mut sim = Simulation::new(net, NoopSink);
    // Both register on the same update, lane 0's first, but the one on lane 1
    // is ahead, and changes into lane 0.
    let behind = sim.insert(car(), at(0, 85.0, 5.0));
    let ahead = sim.insert(car(), at(1, 92.0, 5.0));
    for _ in 0..6 {
        sim.step(DT);
    }
    assert_eq!(sim.queue(junction), [behind, ahead]);
    sim.change_lane(ahead, Side::Left).unwrap();
    for _ in 0..6 {
        sim.step(DT);
    }
    assert_eq!(sim.queue(junction), [ahead, behind]);
}

#[test]
fn it_joins_only_the_next_junction_ahead_of_its_front() {
    // A road through two junctions of one 20 m lane each, 20 m apart: lane 1
    // is the first junction and lane 3 the second.
    let mut net = SimNetwork::new(vec![
        lane(&[p(-100.0, 0.0), p(-10.0, 0.0)]),
        lane(&[p(-10.0, 0.0), p(10.0, 0.0)]),
        lane(&[p(10.0, 0.0), p(30.0, 0.0)]),
        lane(&[p(30.0, 0.0), p(50.0, 0.0)]),
        lane(&[p(50.0, 0.0), p(150.0, 0.0)]),
    ]);
    for i in 0..4 {
        net.connect(LaneId(i), LaneId(i + 1));
    }
    let first = net.add_junction(&[LaneId(1)]);
    let second = net.add_junction(&[LaneId(3)]);
    let mut sim = Simulation::new(net, NoopSink);
    // Both junctions are within its horizon from the start.
    let id = sim.insert(car(), at(0, 70.0, 14.0));
    let spec = car();
    let mut in_both = false;
    for _ in 0..(8.0 / DT) as usize {
        sim.step(DT);
        let Some(v) = sim.get(id) else {
            break;
        };
        let front = v.lane_pos().s - spec.rear_axle + spec.length;
        let in_second = sim.queue(second) == [id];
        let past_first = v.lane().0 > 1 || (v.lane() == LaneId(1) && front >= 20.0);
        assert!(
            !in_second || past_first,
            "it joins the second before its front leaves the first"
        );
        in_both |= in_second && sim.queue(first) == [id];
    }
    assert!(
        in_both,
        "its tail is still in the first as it joins the second"
    );
}
