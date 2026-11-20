//! Junctions: where their lanes conflict, and how their queues keep vehicles on
//! conflicting lanes apart.

use glam::DVec2;
use traffic_sim::{
    ConflictKind, JunctionId, Lane, LaneId, NoopSink, Placement, Route, SAFE_BRAKING, Side,
    SimNetwork, Simulation, VehicleId,
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
    Placement { lane: LaneId(lane), s, speed, route: Route::default() }
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
        sim.step_variable(DT);
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
fn the_second_to_commit_waits_for_the_first_to_clear_the_crossing() {
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
        sim.step_variable(DT);
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
    assert!(both, "both commit, the one on the lower lane first");
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

/// Steps for `seconds`, checking after each step that no two bodies overlap,
/// and returns the lowest speed `id` drops to.
fn slowest(sim: &mut Simulation, id: VehicleId, seconds: f64) -> f64 {
    let mut slowest = f64::INFINITY;
    for _ in 0..(seconds / DT) as usize {
        sim.step_variable(DT);
        if let Some(v) = sim.get(id) {
            slowest = slowest.min(v.speed());
        }
        let bodies: Vec<_> = sim.vehicles().map(|(_, v)| v.corners()).collect();
        for (i, &a) in bodies.iter().enumerate() {
            for &b in &bodies[i + 1..] {
                assert!(!overlap(a, b), "bodies overlap at {:.2} s", sim.time());
            }
        }
    }
    slowest
}

#[test]
fn a_vehicle_crossing_behind_another_times_its_arrival_rather_than_stopping() {
    let (net, _) = crossroads();
    let mut sim = Simulation::new(net, NoopSink);
    // The first will have cleared the crossing just before the second reaches
    // it, so the second, following its progress, needn't slow for it.
    sim.insert(car(), at(1, 65.0, 10.0));
    let second = sim.insert(car(), at(0, 45.0, 10.0));
    let slowest = slowest(&mut sim, second, 12.0);
    assert!(slowest > 9.5, "it slows to {slowest:.2} m/s");
}

#[test]
fn a_vehicle_merging_behind_another_follows_it_through_the_merge() {
    let (net, _) = merge();
    let mut sim = Simulation::new(net, NoopSink);
    // The first is ahead, going the same way, so the second follows it through
    // the merge, rather than waiting for it to be through.
    sim.insert(car(), at(1, 85.0, 10.0));
    let second = sim.insert(car(), at(0, 70.0, 10.0));
    let slowest = slowest(&mut sim, second, 12.0);
    assert!(slowest > 9.0, "it slows to {slowest:.2} m/s");
}

#[test]
fn a_fast_follower_commits_only_after_the_slow_vehicle_ahead_of_it() {
    let (net, junction) = crossroads();
    let mut sim = Simulation::new(net, NoopSink);
    // The follower closes fast on its slow leader, 10 m short of the crossing:
    // it needs 24.5 m to stop at safe braking, but is only 24 m from the
    // crossing, while the leader could still stop.
    let leader = sim.insert(car(), at(0, 84.0, 2.0));
    let follower = sim.insert(car(), at(0, 70.65, 14.0));
    let spec = car();
    // From its front to the start of the overlap, 8.25 m along lane 2, which
    // follows lane 0, 90 m long.
    let to_conflict = |sim: &Simulation, id| {
        let v = sim.get(id).unwrap();
        let front = v.lane_pos().s - spec.rear_axle + spec.length;
        8.25 - front + if v.lane() == LaneId(0) { 90.0 } else { 0.0 }
    };
    let mut ready_first = false;
    let mut order = Vec::new();
    for _ in 0..(10.0 / DT) as usize {
        sim.step_variable(DT);
        commit_order(&sim, junction, &mut order);
        if sim.get(leader).is_some() && !order.contains(&leader) {
            let v = sim.get(follower).unwrap();
            let stop = v.speed() * v.speed() / (2.0 * SAFE_BRAKING);
            ready_first |= stop >= to_conflict(&sim, follower);
        }
    }
    assert!(ready_first, "the follower could no longer stop first");
    assert_eq!(order, [leader, follower], "but the leader commits first");
}

/// Adds the vehicles newly committed at a junction to `order`, in the order
/// they committed.
fn commit_order(sim: &Simulation, junction: JunctionId, order: &mut Vec<VehicleId>) {
    for &id in sim.queue(junction) {
        if !order.contains(&id) {
            order.push(id);
        }
    }
}

#[test]
fn a_vehicle_that_cuts_in_ahead_commits_first() {
    let (net, junction) = merge();
    let mut sim = Simulation::new(net, NoopSink);
    // Both arrive as they are inserted, but the one on lane 1 is ahead, and
    // changes into lane 0.
    let behind = sim.insert(car(), at(0, 85.0, 5.0));
    let ahead = sim.insert(car(), at(1, 92.0, 5.0));
    assert!(sim.queue(junction).is_empty());
    assert_eq!(sim.approaching(junction), [0, 1, 2, 3].map(LaneId));
    sim.change_lane(ahead, Side::Left).unwrap();
    let mut order = Vec::new();
    for _ in 0..(10.0 / DT) as usize {
        sim.step_variable(DT);
        commit_order(&sim, junction, &mut order);
    }
    assert_eq!(order, [ahead, behind], "both commit, the one ahead first");
}

#[test]
fn a_vehicle_not_yet_committed_stops_gently_for_a_committed_one_in_its_way() {
    let (net, junction) = crossroads();
    let mut sim = Simulation::new(net, NoopSink);
    // Its front past the start of the overlap, 8.25 m along lane 3, so it
    // commits, and held there.
    let blocker = sim.insert(car(), at(3, 6.0, 0.0));
    sim.hold(blocker);
    // 85 m short of the overlap, 8.25 m along lane 2, which follows lane 0.
    let id = sim.insert(car(), at(0, 10.0, 14.0));
    let spec = car();
    let mut hardest = 0.0f64;
    for _ in 0..(20.0 / DT) as usize {
        sim.step_variable(DT);
        assert_eq!(sim.queue(junction), [blocker], "it never commits");
        let v = sim.get(id).unwrap();
        hardest = hardest.min(v.accel());
        let front = v.lane_pos().s - spec.rear_axle + spec.length;
        let to_overlap = 8.25 - front + if v.lane() == LaneId(0) { 90.0 } else { 0.0 };
        assert!(to_overlap > 0.0, "it stops short of the overlap");
    }
    assert!(sim.get(id).unwrap().speed() < 0.1, "it stops");
    assert!(
        hardest > -2.0 * spec.comfy_decel,
        "it brakes gently: {hardest:.2} m/s²"
    );
}

#[test]
fn a_committed_vehicle_held_up_short_of_its_conflict_is_released() {
    let (net, junction) = merge();
    let mut sim = Simulation::new(net, NoopSink);
    // At 14 m/s, 24.4 m short of the merge, it can no longer stop at safe
    // braking, so it commits. Then a slower car cuts in ahead of it, and it
    // brakes hard behind that one, until it could stop a car length short of
    // the merge.
    let held = sim.insert(car(), at(0, 72.0, 14.0));
    let slow = sim.insert(car(), at(1, 82.0, 6.0));
    sim.step_variable(DT);
    assert_eq!(sim.queue(junction), [held]);
    sim.change_lane(slow, Side::Left).unwrap();
    let mut released = false;
    for _ in 0..(5.0 / DT) as usize {
        sim.step_variable(DT);
        let short = sim.get(held).is_some_and(|v| v.lane() == LaneId(0));
        released |= short && !sim.queue(junction).contains(&held);
        let bodies: Vec<_> = sim.vehicles().map(|(_, v)| v.corners()).collect();
        if let [x, y] = bodies[..] {
            assert!(!overlap(x, y), "they overlap");
        }
    }
    assert!(released, "it is released short of the merge");
}

#[test]
fn it_arrives_at_the_next_junction_only_once_it_has_entered_the_one_before() {
    // A road through two junctions 20 m apart, each a 20 m lane crossed halfway
    // along: lanes 1 and 5 are the first junction, and lanes 3 and 6 the
    // second.
    let mut net = SimNetwork::new(vec![
        lane(&[p(-100.0, 0.0), p(-10.0, 0.0)]),
        lane(&[p(-10.0, 0.0), p(10.0, 0.0)]),
        lane(&[p(10.0, 0.0), p(30.0, 0.0)]),
        lane(&[p(30.0, 0.0), p(50.0, 0.0)]),
        lane(&[p(50.0, 0.0), p(150.0, 0.0)]),
        lane(&[p(0.0, -20.0), p(0.0, 20.0)]),
        lane(&[p(40.0, -20.0), p(40.0, 20.0)]),
    ]);
    for i in 0..4 {
        net.connect(LaneId(i), LaneId(i + 1));
    }
    let first = net.add_junction(&[LaneId(1), LaneId(5)]);
    let second = net.add_junction(&[LaneId(3), LaneId(6)]);
    let mut sim = Simulation::new(net, NoopSink);
    // Both junctions are on its path from the start.
    let id = sim.insert(car(), at(0, 70.0, 14.0));
    assert_eq!(sim.approaching(first), [LaneId(0), LaneId(1)]);
    assert!(sim.approaching(second).is_empty());
    let mut arrived = false;
    for _ in 0..(8.0 / DT) as usize {
        sim.step_variable(DT);
        let Some(v) = sim.get(id) else {
            break;
        };
        // On the first junction's lane or beyond: its rear axle has entered it.
        let at_second = !sim.approaching(second).is_empty() || sim.queue(second) == [id];
        assert!(
            !at_second || v.lane().0 >= 1,
            "it arrives at the second before entering the first"
        );
        arrived |= at_second && sim.queue(first) == [id];
    }
    assert!(
        arrived,
        "it arrives at the second while still committed at the first"
    );
}
