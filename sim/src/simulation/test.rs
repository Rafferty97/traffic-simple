//! Snapshots: each sets up one instant and checks what the searches and the
//! lane-change decision make of it, without stepping.

use glam::DVec2;

use super::*;
use crate::debug::DebugEvent;
use crate::models::lane_change::{LaneChangeEval, Outcome};
use crate::network::{Barrier, Lane};

const LIMIT: f64 = 14.0;
const WIDTH: f64 = 3.5;

fn car() -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        speed_factor: 1.0,
        max_accel: 1.0,
        comfy_decel: 1.5,
        time_headway: 1.0,
        min_gap: 1.5,
        rear_axle: 1.0,
        front_axle: 3.7,
        courtesy: 1.0,
        politeness: 0.2,
        assertiveness: 0.0,
        accel_noise: 0.0,
        line_gap: 1.0,
        line_gap_spread: 0.0,
    }
}

/// A straight road along x in sections, each `lanes` lanes wide, lane 0 on
/// the left. Section `k`'s lane `i` is `LaneId(k * lanes + i)` and runs on to
/// section `k + 1`'s lane `i`.
fn road(sections: &[(f64, f64)], lanes: u32) -> SimNetwork {
    let mut all = Vec::new();
    for &(x0, x1) in sections {
        for i in 0..lanes {
            let y = -WIDTH * i as f64;
            all.push(Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT).unwrap());
        }
    }
    let mut network = SimNetwork::new(all);
    for k in 0..sections.len() as u32 {
        for i in 1..lanes {
            network.set_adjacent(LaneId(k * lanes + i - 1), LaneId(k * lanes + i), true, true);
        }
        if k > 0 {
            for i in 0..lanes {
                network.connect(LaneId((k - 1) * lanes + i), LaneId(k * lanes + i));
            }
        }
    }
    network.validate().unwrap();
    network
}

/// Two lanes side by side, 500 m long.
fn two_lanes() -> Simulation {
    Simulation::new(road(&[(0.0, 500.0)], 2), NoopSink)
}

fn at(lane: u32, s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(lane), s, speed, route: Route::default() }
}

/// A vehicle's rear, where both searches split its lane.
fn rear(sim: &Simulation, id: VehicleId) -> f64 {
    let v = &sim.state.vehicles[id];
    v.lane_pos.s - v.spec.rear_axle
}

fn follower(sim: &Simulation, lane: u32, s: f64, skip: VehicleId) -> Option<(VehicleId, f64)> {
    find_follower(&sim.state, &sim.derived.through, LaneId(lane), s, skip)
}

/// `id`'s search along its path, from its own record, calling `f` with each
/// thing it finds until `f` breaks.
fn search<B>(
    sim: &Simulation,
    id: VehicleId,
    f: impl FnMut(Found) -> ControlFlow<B>,
) -> ControlFlow<B> {
    let v = &sim.state.vehicles[id];
    let own = occupancy::record_index(&sim.state, v.path[0], id);
    find_leaders(&sim.state, v, Along::Path(own), f)
}

/// The nearest vehicle `id` reacts to, stopping the search there.
fn leader(sim: &Simulation, id: VehicleId) -> Option<(VehicleId, f64)> {
    search(sim, id, |found| match found {
        Found::Ahead(Ahead::Vehicle(leader), gap, _) => ControlFlow::Break((leader, gap)),
        _ => ControlFlow::Continue(()),
    })
    .break_value()
}

/// What `id` decides in one step of its lane-change model now, with a gap
/// check if it asks for one, its lane-change state afterwards, and how it
/// weighed up each side it may change to. Partway through a change, or
/// straddling, it decides nothing and weighs up nothing.
fn decision(
    sim: &Simulation,
    id: VehicleId,
) -> (Decision, lane_change::State, Vec<LaneChangeEval>) {
    let vehicle = &sim.state.vehicles[id];
    if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
        return (Decision::Nothing, vehicle.lc, Vec::new());
    }
    let mut speeds = Vec::new();
    LaneSpeeds::refresh(&mut speeds, &sim.state);
    let incentives = incentives(&sim.state, &speeds, id, vehicle);
    let mut lc = vehicle.lc;
    let dt = sim.state.config.step * f64::from(sim.state.config.accel_every);
    lane_change::aim(&mut lc, incentives);
    let ask = lane_change::step(&mut lc, vehicle.intent, &vehicle.spec, sim.state.time, dt);
    let mut evals = Vec::new();
    let mut decision = Decision::Nothing;
    for side in [Side::Left, Side::Right] {
        let Some(incentive) = incentives[lane_change::index(side)] else {
            continue;
        };
        let eval = match ask {
            lane_change::Ask::Check(asked) if asked == side => {
                let prospect = check_gap(
                    &sim.state,
                    &sim.derived.through,
                    id,
                    vehicle,
                    side,
                    lc.desire(side),
                );
                let spec = &vehicle.spec;
                let (d, eval) = lane_change::resolve(
                    &mut lc,
                    vehicle.intent,
                    spec,
                    side,
                    incentive,
                    prospect,
                    dt,
                );
                decision = d;
                eval
            }
            lane_change::Ask::Check(_) => {
                lane_change::unchecked(side, incentive, lc.desire(side), Outcome::OtherSide)
            }
            lane_change::Ask::Cancel => {
                decision = Decision::Cancel;
                lane_change::unchecked(side, incentive, lc.desire(side), Outcome::Cancel)
            }
            lane_change::Ask::Nothing => {
                let outcome = match vehicle.intent {
                    Some(_) => Outcome::Wait,
                    None => Outcome::NotWorthIt,
                };
                lane_change::unchecked(side, incentive, lc.desire(side), outcome)
            }
        };
        evals.push(eval);
    }
    (decision, lc, evals)
}

/// What `id` decides to do now, and what it weighed up.
fn decide_now(sim: &Simulation, id: VehicleId) -> (Decision, Vec<LaneChangeEval>) {
    let (decision, _, evals) = decision(sim, id);
    (decision, evals)
}

/// A desire for a change past the threshold for a free lane change, short of
/// synchronising: it changes if the gap lets it.
const WANTING: f64 = 0.5;

/// A car on lane 1 at 300 m that has been indicating left for 2 s, with
/// desire for it, behind a car held stopped 20 m ahead, so
/// changing is well worth it.
fn indicating_behind_a_stopped_car(sim: &mut Simulation) -> VehicleId {
    let blocker = sim.insert(car(), at(1, 320.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, 300.0, 5.0));
    sim.indicate(id, Side::Left);
    sim.state.vehicles[id].lc.set_desire([WANTING, 0.0]);
    sim.state.time = 2.0;
    id
}

// Following.

#[test]
fn the_follower_is_the_nearest_vehicle_behind() {
    let mut sim = two_lanes();
    let far = sim.insert(car(), at(0, 20.0, LIMIT));
    let near = sim.insert(car(), at(0, 40.0, LIMIT));
    let _ahead = sim.insert(car(), at(0, 80.0, 0.0));
    let (found, gap) = follower(&sim, 0, 60.0, VehicleId::null()).unwrap();
    assert_eq!(found, near);
    // From its front, 3.6 m ahead of its rear axle at 40, to 60.
    assert!((gap - 16.4).abs() < 1e-9, "gap {gap}");
    assert_ne!(found, far);
}

#[test]
fn the_follower_search_passes_over_ghosts_and_the_searcher() {
    let mut sim = two_lanes();
    let behind = sim.insert(car(), at(0, 20.0, LIMIT));
    let searcher = sim.insert(car(), at(0, 45.0, 0.0));
    // Indicating from lane 1 into lane 0, between the two.
    let indicating = sim.insert(car(), at(1, 35.0, 0.0));
    sim.indicate(indicating, Side::Left);
    let found = follower(&sim, 0, 60.0, searcher).map(|(id, _)| id);
    assert_eq!(found, Some(behind));
}

#[test]
fn the_follower_search_counts_a_vehicle_moving_out_of_the_lane() {
    // Changing out of lane 0 into lane 1, it is still partly in the way.
    let mut sim = two_lanes();
    let leaving = sim.insert(car(), at(0, 30.0, LIMIT));
    sim.change_lane(leaving, Side::Right).unwrap();
    let record = sim.state.occupants[0].iter().find(|o| o.id == leaving).unwrap();
    assert_eq!(record.lateral, Lateral::Diverging(Side::Right));
    let found = follower(&sim, 0, 60.0, VehicleId::null()).map(|(id, _)| id);
    assert_eq!(found, Some(leaving));
}

#[test]
fn the_follower_search_continues_onto_the_lanes_leading_in() {
    // Two sections of two lanes: lane 0 leads into lane 2.
    let mut sim = Simulation::new(road(&[(0.0, 100.0), (100.0, 200.0)], 2), NoopSink);
    let behind = sim.insert(car(), at(0, 80.0, LIMIT));
    let (found, gap) = follower(&sim, 2, 10.0, VehicleId::null()).unwrap();
    assert_eq!(found, behind);
    // Its front is at 83.6 on lane 0, 16.4 m before lane 2 starts.
    assert!((gap - 26.4).abs() < 1e-9, "gap {gap}");
}

#[test]
fn the_follower_search_stops_at_the_followers_horizon() {
    // Only as far back as the follower's own leader search would reach.
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(0, 50.0, LIMIT));
    let horizon = acceleration::horizon(&sim.state.vehicles[id].spec, LIMIT);
    let front = 50.0 + 3.6;
    assert!(follower(&sim, 0, front + horizon - 0.1, VehicleId::null()).is_some());
    assert!(follower(&sim, 0, front + horizon + 0.1, VehicleId::null()).is_none());
    // Stopped, it looks only a few metres ahead.
    sim.state.vehicles[id].speed = 0.0;
    assert!(follower(&sim, 0, front + 10.0, VehicleId::null()).is_none());
}

#[test]
fn the_follower_search_wraps_round_a_loop() {
    // A lane that is its own successor: the vehicle at 970, its front at
    // 973.6, is 76.4 m behind a point at 50, round the seam.
    let lane = Lane::new(&[DVec2::ZERO, DVec2::new(1000.0, 0.0)], WIDTH, LIMIT).unwrap();
    let mut network = SimNetwork::new(vec![lane]);
    network.connect(LaneId(0), LaneId(0));
    let mut sim = Simulation::new(network, NoopSink);
    let behind = sim.insert(car(), at(0, 970.0, LIMIT));
    let (found, gap) = follower(&sim, 0, 50.0, VehicleId::null()).unwrap();
    assert_eq!(found, behind);
    assert!((gap - 76.4).abs() < 1e-9, "gap {gap}");
}

#[test]
fn on_lanes_leading_in_only_vehicles_headed_this_way_count() {
    // Lane 0 splits into lane 1, straight on, and lane 2, bearing left.
    let lane = |x0: f64, x1: f64, y1: f64| {
        Lane::new(&[DVec2::new(x0, 0.0), DVec2::new(x1, y1)], WIDTH, LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![
        lane(0.0, 100.0, 0.0),
        lane(100.0, 200.0, 0.0),
        lane(100.0, 200.0, 10.0),
    ]);
    network.connect(LaneId(0), LaneId(2));
    network.connect(LaneId(0), LaneId(1));
    let mut sim = Simulation::new(network, NoopSink);
    // It turns onto lane 2, so it isn't lane 1's follower.
    let turning = sim.insert(car(), at(0, 80.0, LIMIT));
    let path = &mut sim.state.vehicles[turning].path;
    path.clear();
    path.extend([LaneId(0), LaneId(2)]);
    assert!(follower(&sim, 1, 10.0, VehicleId::null()).is_none());
    assert_eq!(
        follower(&sim, 2, 10.0, VehicleId::null()).map(|(id, _)| id),
        Some(turning)
    );
}

#[test]
fn the_leader_and_follower_searches_meet_at_the_rear() {
    // A car level with another in the same lane, at every offset within two
    // car lengths: exactly one search finds it, and if their bodies overlap,
    // with a gap of zero or less. Moving, so both look further than that.
    for tenths in -92..=92 {
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(0, 100.0, 10.0));
        let s = 100.0 + tenths as f64 / 10.0;
        let other = sim.insert(car(), at(0, s, 10.0));
        let ahead = leader(&sim, id).filter(|&(l, _)| l == other);
        let behind = follower(&sim, 0, rear(&sim, id), id).filter(|&(f, _)| f == other);
        assert!(
            ahead.is_some() != behind.is_some(),
            "at {s}: {ahead:?} {behind:?}"
        );
        let gap = ahead.or(behind).unwrap().1;
        let overlap = (s - 100.0).abs() < car().length;
        assert_eq!(gap <= 0.0, overlap, "at {s}: gap {gap}");
    }
}

// Records.

#[test]
fn a_lane_change_marks_the_record_it_leaves_diverging_at_once() {
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(1, 100.0, 10.0));
    sim.change_lane(id, Side::Left).unwrap();
    let lateral = |lane: usize| {
        sim.state.occupants[lane].iter().find(|o| o.is_record_of(id)).map(|o| o.lateral)
    };
    assert_eq!(
        lateral(1),
        Some(Lateral::Diverging(Side::Left)),
        "the lane left"
    );
    assert_eq!(
        lateral(0),
        Some(Lateral::CONVERGING),
        "the lane moved into, until the change ends"
    );
    assert_eq!(sim.state.vehicles[id].old_side, Some(Side::Right));
}

#[test]
fn a_ghost_is_in_the_lane_it_indicates_into() {
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(1, 100.0, 10.0));
    sim.indicate(id, Side::Left);
    let ghost = sim.state.occupants[0].iter().find(|o| o.id == id).unwrap();
    assert_eq!(ghost.lateral, Lateral::Ghost);
    let own = sim.state.occupants[1].iter().find(|o| o.id == id).unwrap();
    assert_eq!(own.lateral, Lateral::InLane);
}

// Deciding.

#[test]
fn it_changes_when_the_gap_is_clear_and_it_has_indicated_long_enough() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
    // Nobody behind in the target lane.
    assert_eq!(evals[0].new_follower, None);
}

/// Two lanes side by side, 500 m long, limited to 25 m/s, that run on side by
/// side for 200 m as lanes 2 and 5. Lane 2, on the left, is in a junction,
/// crossed 10 m along by lane 3, and runs on as lane 4; the network makes
/// lane 5 its neighbor, as at the ramps' lane drop.
fn two_lanes_into_a_junction() -> Simulation {
    let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], WIDTH, 25.0).unwrap();
    let p = DVec2::new;
    let mut network = SimNetwork::new(vec![
        lane(p(0.0, 0.0), p(500.0, 0.0)),
        lane(p(0.0, -WIDTH), p(500.0, -WIDTH)),
        lane(p(500.0, 0.0), p(700.0, 0.0)),
        lane(p(510.0, -40.0), p(510.0, 40.0)),
        lane(p(700.0, 0.0), p(900.0, 0.0)),
        lane(p(500.0, -WIDTH), p(700.0, -WIDTH)),
    ]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    network.set_adjacent(LaneId(2), LaneId(5), true, true);
    for (from, to) in [(0, 2), (2, 4), (1, 5)] {
        network.connect(LaneId(from), LaneId(to));
    }
    network.add_junction(&[LaneId(2), LaneId(3)]);
    Simulation::new(network, NoopSink)
}

/// A taper: lane 0, from 0 to 100 m, and lane 1 starting beside it and
/// easing into it from 20 m on, both leading into lane 2, from 100 to 300 m.
/// Lanes 3 and 4 lead into 0 and 1 from 100 m back. Lane 5 comes in from the
/// side and leads into lane 2 as well, without starting with them.
fn a_taper() -> Simulation {
    let p = DVec2::new;
    let lane = |points: &[DVec2]| Lane::new(points, WIDTH, 25.0).unwrap();
    let mut network = SimNetwork::new(vec![
        lane(&[p(0.0, 0.0), p(100.0, 0.0)]),
        lane(&[p(0.0, -WIDTH), p(20.0, -WIDTH), p(100.0, 0.0)]),
        lane(&[p(100.0, 0.0), p(300.0, 0.0)]),
        lane(&[p(-100.0, 0.0), p(0.0, 0.0)]),
        lane(&[p(-100.0, -WIDTH), p(0.0, -WIDTH)]),
        lane(&[p(50.0, 50.0), p(100.0, 0.0)]),
    ]);
    network.set_adjacent(LaneId(3), LaneId(4), true, true);
    for (from, to) in [(0, 2), (1, 2), (5, 2), (3, 0), (4, 1)] {
        network.connect(LaneId(from), LaneId(to));
    }
    network.validate().unwrap();
    Simulation::new(network, NoopSink)
}

// Merges.

/// The order of the only merge, by vehicle.
fn merge_order(sim: &Simulation) -> Vec<VehicleId> {
    let mut order = Vec::new();
    let SimState { network, vehicles, occupants, .. } = &sim.state;
    merge::order(network, &sim.derived.merges, vehicles, occupants, 0, |m| {
        order.push(m.id())
    });
    order
}

#[test]
fn lanes_that_start_to_overlap_are_a_merge_and_branches_of_a_fork_are_not() {
    let sim = a_taper();
    let of = |lane: u32| sim.derived.merges.of(LaneId(lane));
    // The taper and the lane coming in from the side, all one merge.
    assert_eq!(sim.derived.merges.len(), 1);
    assert!(of(0).is_some() && of(0) == of(1) && of(1) == of(5));
    // The lanes leading in, apart, and the lane they lead into.
    assert_eq!((of(2), of(3), of(4)), (None, None, None));

    // Lane 0 forks at 100 m into lanes 1 and 2, which overlap where they part.
    let lane = |x0: f64, x1: f64, y1: f64| {
        Lane::new(&[DVec2::new(x0, 0.0), DVec2::new(x1, y1)], WIDTH, LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![
        lane(0.0, 100.0, 0.0),
        lane(100.0, 200.0, 0.0),
        lane(100.0, 200.0, 20.0),
    ]);
    network.connect(LaneId(0), LaneId(1));
    network.connect(LaneId(0), LaneId(2));
    let sim = Simulation::new(network, NoopSink);
    assert!(!sim.derived.overlaps[1].is_empty(), "the branches overlap");
    assert_eq!(sim.derived.merges.len(), 0);
}

#[test]
fn a_vehicle_joins_a_merges_order_on_the_way_and_leaves_it_once_in_the_merge() {
    let mut sim = a_taper();
    let id = sim.insert(car(), at(3, 50.0, 10.0));
    assert_eq!(merge_order(&sim), [id]);
    while sim.state.vehicles[id].path[0] == LaneId(3) {
        sim.step_variable(0.1);
        if sim.state.vehicles[id].path[0] == LaneId(3) {
            assert_eq!(merge_order(&sim), [id]);
        }
    }
    sim.step_variable(0.1);
    assert_eq!(merge_order(&sim), []);
}

#[test]
fn a_slow_vehicle_still_knows_where_it_is_going_and_joins_a_merges_order() {
    let mut sim = a_taper();
    // At rest 90 m short of the merge point: its path reaches as far as
    // 15 s at the limit, through the merge.
    let id = sim.insert(car(), at(3, 10.0, 0.0));
    assert_eq!(
        sim.state.vehicles[id].path,
        [LaneId(3), LaneId(0), LaneId(2)]
    );
    assert_eq!(merge_order(&sim), [id]);
}

#[test]
fn the_order_is_by_distance_to_the_merge_point_and_one_pulling_ahead_goes_first() {
    let mut sim = a_taper();
    let a = sim.insert(car(), at(3, 50.0, 10.0));
    let b = sim.insert(car(), at(4, 49.0, 20.0));
    sim.step_variable(0.05);
    assert_eq!(merge_order(&sim), [a, b]);
    // Faster, b pulls ahead within a few frames, and goes first at once.
    while rear(&sim, b) <= rear(&sim, a) {
        sim.step();
    }
    sim.step_variable(0.05);
    assert_eq!(merge_order(&sim), [b, a]);
}

#[test]
fn straddling_into_a_merge_it_has_one_record_on_the_lane_it_follows() {
    let mut sim = a_taper();
    // Changing left from lane 4 just before it leads into the merge: it
    // advances onto lane 0 still straddling, and joins lane 1, after its old
    // lane, and the lanes overlapping both, lane 0 among them.
    let id = sim.insert(car(), at(4, 92.0, 10.0));
    sim.change_lane(id, Side::Left).unwrap();
    while sim.state.vehicles[id].path[0] == LaneId(3) {
        sim.step();
    }
    assert_eq!(
        sim.state.vehicles[id].old_side,
        Some(Side::Right),
        "still straddling"
    );
    let own = sim.state.occupants[0].iter().filter(|o| o.is_record_of(id)).count();
    assert_eq!(own, 1);
    assert!(sim.state.occupants[1].iter().any(|o| o.is_record_of(id)));
}

#[test]
fn the_order_keeps_vehicles_on_one_lane_in_their_order_along_it() {
    let mut sim = a_taper();
    // The one behind is faster, so by estimated arrival alone it would go
    // first: 1.8 s to the merge point against 2.3 s.
    let ahead = sim.insert(car(), at(3, 85.0, 5.0));
    let behind = sim.insert(car(), at(3, 60.0, 20.0));
    sim.step_variable(0.1);
    assert_eq!(merge_order(&sim), [ahead, behind]);
}

#[test]
fn side_by_side_before_a_merge_one_drops_back_gently_and_they_merge_apart() {
    let mut sim = a_taper();
    let a = sim.insert(car(), at(3, 10.0, 15.0));
    let b = sim.insert(car(), at(4, 10.0, 15.0));
    let (mut least, mut gap) = (f64::INFINITY, f64::INFINITY);
    for _ in 0..(10.0 / crate::STEP) as u32 {
        sim.step();
        for id in [a, b] {
            least = least.min(sim.state.vehicles[id].accel);
        }
        // Once both are past the merge point, how far apart they are.
        let (va, vb) = (&sim.state.vehicles[a], &sim.state.vehicles[b]);
        if va.path[0] != LaneId(3) && vb.path[0] != LaneId(4) {
            gap = gap.min(va.pos.distance(vb.pos));
        }
    }
    assert!(
        least > -acceleration::SAFE_BRAKING,
        "braked at {least:.2} m/s²"
    );
    assert!(gap > car().length + car().min_gap, "{gap:.2} m apart");
}

#[test]
fn queues_in_both_lanes_before_a_merge_go_through_it_by_turns() {
    let mut sim = a_taper();
    // Five a lane, at rest 8 m apart, the right lane staggered 4 m back.
    let ids: Vec<_> = (0..5)
        .flat_map(|k| {
            let s = 94.0 - 8.0 * k as f64;
            [at(3, s, 0.0), at(4, s - 4.0, 0.0)]
        })
        .map(|p| sim.insert(car(), p))
        .collect();
    let waiting = |sim: &Simulation, id| {
        let lane = sim.state.vehicles.get(id).map(|v: &Vehicle| v.path[0]);
        matches!(lane, Some(LaneId(3) | LaneId(4)))
    };
    let mut entered = Vec::new();
    for _ in 0..(60.0 / crate::STEP) as u32 {
        sim.step();
        for (k, &id) in ids.iter().enumerate() {
            if !entered.contains(&k) && !waiting(&sim, id) {
                entered.push(k);
            }
        }
    }
    // All of them, one from each lane in turn: a zipper.
    assert_eq!(entered.len(), ids.len());
    let lanes: Vec<_> = entered.iter().map(|k| k % 2).collect();
    assert!(lanes.windows(2).all(|w| w[0] != w[1]), "{entered:?}");
}

#[test]
fn a_record_ages_carries_its_age_on_and_starts_afresh_on_a_change_or_a_merge() {
    // Along a road of two sections: carried over.
    let mut sim = Simulation::new(road(&[(0.0, 100.0), (100.0, 200.0)], 2), NoopSink);
    let id = sim.insert(car(), at(0, 80.0, 10.0));
    let age = |sim: &Simulation, lane: u32| {
        sim.state.occupants[lane as usize]
            .iter()
            .find(|o| o.is_record_of(id))
            .map(|o| o.age)
    };
    sim.step_variable(1.0);
    // Inserted settled, and older since.
    let before = age(&sim, 0).unwrap();
    assert!(before > acceleration::SETTLED, "{before}");
    while sim.state.vehicles[id].path[0] == LaneId(0) {
        sim.step();
    }
    assert!(age(&sim, 2).unwrap() > before, "carried over");
    // Changing lanes: afresh.
    sim.change_lane(id, Side::Right).unwrap();
    assert_eq!(age(&sim, 3), Some(0.0));

    // Into a merge: afresh.
    let mut sim = a_taper();
    let id = sim.insert(car(), at(3, 90.0, 10.0));
    sim.step_variable(1.0);
    assert_eq!(sim.state.vehicles[id].path[0], LaneId(0));
    let record = sim.state.occupants[0].iter().find(|o| o.is_record_of(id)).unwrap();
    assert!(record.age < 1.0, "{}", record.age);
}

#[test]
fn lanes_leading_into_the_same_lane_overlap_and_never_diverge() {
    let sim = a_taper();
    let partners = |lane: u32| {
        let mut found: Vec<_> = sim.derived.overlaps[lane as usize]
            .iter()
            .map(|p| (p.lane.0, p.diverge))
            .collect();
        found.sort_by_key(|p| p.0);
        found
    };
    // They merge, so never diverge.
    assert!(
        partners(0).contains(&(1, f64::INFINITY)),
        "{:?}",
        partners(0)
    );
    assert!(
        partners(1).contains(&(0, f64::INFINITY)),
        "{:?}",
        partners(1)
    );
    assert!(
        partners(5).iter().all(|p| p.1 == f64::INFINITY),
        "{:?}",
        partners(5)
    );
    // Lanes one after the other, and neighbours, only touch.
    assert_eq!(partners(3), []);
    assert_eq!(partners(2), []);
}

#[test]
fn a_vehicle_entering_a_taper_merges_into_the_other_lane_until_past_it() {
    let mut sim = a_taper();
    let id = sim.insert(car(), at(4, 95.0, 15.0));
    while sim.state.vehicles[id].path[0] != LaneId(1) {
        sim.step_variable(0.05);
    }
    let mine = records(&sim, id);
    assert!(mine.contains(&(1, false, Lateral::InLane)), "{mine:?}");
    assert!(mine.contains(&(0, false, Lateral::CONVERGING)), "{mine:?}");
    // Wholly beside lane 0 at first, but kept on it.
    for _ in 0..20 {
        sim.step_variable(0.05);
    }
    assert!(sim.state.vehicles[id].lane_pos.s < 20.0);
    assert!(records(&sim, id).contains(&(0, false, Lateral::CONVERGING)));
    // Dropped from both once past their end.
    for _ in 0..200 {
        sim.step_variable(0.05);
    }
    assert_eq!(sim.state.vehicles[id].path[0], LaneId(2));
    assert_eq!(records(&sim, id), [(2, false, Lateral::InLane)]);
}

#[test]
fn a_vehicle_inserted_in_a_taper_merges_into_the_other_lane() {
    let mut sim = a_taper();
    let id = sim.insert(car(), at(1, 30.0, 0.0));
    let mine = records(&sim, id);
    assert!(mine.contains(&(0, false, Lateral::CONVERGING)), "{mine:?}");
}

#[test]
fn a_follower_in_a_taper_stays_behind_a_vehicle_merging_in_beside_it() {
    let mut sim = a_taper();
    // Not yet overlapping lane 0 at 30 m, so a follower there could drive by.
    let merging = sim.insert(car(), at(1, 30.0, 0.0));
    sim.hold(merging);
    let follower = sim.insert(car(), at(3, 60.0, 15.0));
    for _ in 0..600 {
        sim.step_variable(0.05);
    }
    let v = &sim.state.vehicles[follower];
    assert_eq!(v.path[0], LaneId(0));
    assert!(v.speed < 0.1, "{}", v.speed);
    let theirs = sim.state.occupants[0].iter().find(|o| o.id == merging).unwrap();
    assert!(rear(&sim, follower) + v.spec.length < theirs.s());
}

/// A car on lane 1 at `s` and `speed`, indicating left for 2 s with
/// advantage built up for it, behind a car held stopped 20 m ahead.
fn indicating_behind_a_stopped_car_at(sim: &mut Simulation, s: f64, speed: f64) -> VehicleId {
    let blocker = sim.insert(car(), at(1, s + 20.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, s, speed));
    sim.indicate(id, Side::Left);
    sim.state.vehicles[id].lc.set_desire([WANTING, 0.0]);
    sim.state.time = 2.0;
    id
}

#[test]
fn it_wont_start_a_change_it_couldnt_get_halfway_through_before_stopping() {
    // Crawling up to a stopped car, 6 m bumper to bumper, it would have to
    // stop well before halfway through a change: it would be left straddling.
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), at(1, 300.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, 289.4, 1.0));
    sim.indicate(id, Side::Left);
    sim.state.vehicles[id].lc.set_desire([WANTING, 0.0]);
    sim.state.time = 2.0;
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Nothing, "{evals:?}");
    assert!(evals[0].safe && !evals[0].room, "{evals:?}");
    // With room to get halfway, it goes.
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), at(1, 300.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, 280.0, 1.0));
    sim.indicate(id, Side::Left);
    sim.state.vehicles[id].lc.set_desire([WANTING, 0.0]);
    sim.state.time = 2.0;
    assert_eq!(decide_now(&sim, id).0, Decision::Change(Side::Left));
}

#[test]
fn wanting_to_change_lanes_it_keeps_room_behind_a_stopped_leader() {
    // At rest 4 m behind a stopped car: settled, it creeps up to its
    // minimum gap; wanting to change, it holds back, room to pull out.
    let accel = |indicating: bool| {
        let mut sim = two_lanes();
        let blocker = sim.insert(car(), at(1, 300.0, 0.0));
        sim.hold(blocker);
        let id = sim.insert(car(), at(1, 291.4, 0.0));
        if indicating {
            sim.indicate(id, Side::Left);
        }
        sim.update_accels();
        sim.state.vehicles[id].accel
    };
    assert!(accel(false) > 0.0, "{}", accel(false));
    assert!(accel(true) < 0.0, "{}", accel(true));
}

#[test]
fn it_wont_start_a_change_that_would_run_into_a_junction() {
    let mut sim = two_lanes_into_a_junction();
    // At 20 m/s a change reaches 93 m beyond its rear axle, and the junction
    // starts 70 m beyond it on the lane beside it. The network gives the
    // junction's lane a neighbor, but that doesn't count.
    let id = indicating_behind_a_stopped_car_at(&mut sim, 430.0, 20.0);
    let (decision, evals) = decide_now(&sim, id);
    assert_ne!(decision, Decision::Change(Side::Left), "{evals:?}");
    // It wants to, but the gap check refuses it.
    assert!(
        evals.iter().all(|e| e.side != Side::Left || !e.safe),
        "{evals:?}"
    );
}

#[test]
fn it_changes_into_a_lane_leading_to_a_junction_beyond_the_changes_reach() {
    let mut sim = two_lanes_into_a_junction();
    // The junction starts 200 m beyond its rear axle. Slow enough to be
    // halfway through the change before it would have to stop behind the car.
    let id = indicating_behind_a_stopped_car_at(&mut sim, 300.0, 5.0);
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
}

#[test]
fn it_waits_until_it_has_indicated_long_enough() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    sim.state.time = 0.5 * lane_change::MIN_INDICATING;
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Nothing);
    assert_eq!(evals[0].outcome, Outcome::Wait);
}

#[test]
fn it_wont_cut_off_a_fast_car_close_behind_in_the_target_lane() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    sim.insert(car(), at(0, 290.0, 14.0));
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Nothing);
    let eval = evals[0];
    assert!(!eval.safe);
    assert!(
        eval.new_follower.unwrap() < -SAFE_BRAKING_FOR_TESTS,
        "{eval:?}"
    );
}

const SAFE_BRAKING_FOR_TESTS: f64 = acceleration::SAFE_BRAKING;

#[test]
fn a_car_far_behind_in_the_target_lane_is_no_follower() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    // 245 m back, beyond the leader horizon, even coming fast.
    sim.insert(car(), at(0, 50.0, 14.0));
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
    assert_eq!(evals[0].new_follower, None);
}

#[test]
fn it_never_changes_into_a_car_alongside() {
    // Whichever rear is ahead, the car beside it is its leader or its new
    // follower there, with no gap.
    for tenths in -45..=45 {
        let mut sim = two_lanes();
        let id = indicating_behind_a_stopped_car(&mut sim);
        let s = 300.0 + tenths as f64 / 10.0;
        sim.insert(car(), at(0, s, 5.0));
        let (decision, evals) = decide_now(&sim, id);
        // Ahead of it, the car beside makes changing worse than staying, so it
        // cancels; either way it doesn't change.
        assert_ne!(decision, Decision::Change(Side::Left), "beside at {s}");
        assert!(!evals[0].safe, "beside at {s}");
    }
}

#[test]
fn its_old_follower_gains_when_it_leaves() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    // Close behind it on lane 1, held back by it now, as the longitudinal update
    // records.
    let behind = sim.insert(car(), at(1, 290.0, 5.0));
    sim.update_accels();
    assert_eq!(
        sim.state.vehicles[id].old_follower.map(|f| f.id),
        Some(behind)
    );
    // Courtesy: it wants to let it by.
    let (_, evals) = decide_now(&sim, id);
    let courtesy = evals[0].incentives.courtesy;
    assert!(courtesy > 0.0, "courtesy {courtesy}");
}

#[test]
fn a_car_it_doesnt_hold_up_isnt_its_old_follower() {
    // Far enough back that the free road binds it, not this car.
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(1, 300.0, 5.0));
    sim.insert(car(), at(1, 240.0, 5.0));
    sim.update_accels();
    assert!(sim.state.vehicles[id].old_follower.is_none());
}

#[test]
fn a_car_its_speed_cap_holds_back_isnt_held_up_by_the_car_ahead() {
    // Close behind, but held to a stop: its cap binds, so leaving frees it of
    // nothing.
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(1, 300.0, 5.0));
    let behind = sim.insert(car(), at(1, 290.0, 5.0));
    sim.update_accels();
    assert_eq!(
        sim.state.vehicles[id].old_follower.map(|f| f.id),
        Some(behind)
    );
    sim.hold(behind);
    sim.update_accels();
    assert!(sim.state.vehicles[id].old_follower.is_none());
}

#[test]
fn another_cars_ghost_ahead_in_the_target_lane_is_a_leader() {
    // Weighing up changing left, a ghost just ahead in lane 0 counts: changing
    // would put it right behind a car about to arrive there.
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), at(1, 220.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, 200.0, 5.0));
    let other = sim.insert(car(), at(1, 206.0, 0.0));
    sim.indicate(other, Side::Left);
    // With desire for it, so only safety can hold it back.
    sim.state.vehicles[id].lc.set_desire([WANTING; 2]);
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Nothing);
    let left = evals.iter().find(|e| e.side == Side::Left).unwrap();
    assert_eq!(left.outcome, Outcome::Unsafe, "{left:?}");
}

#[test]
fn it_cancels_once_changing_no_longer_helps_it() {
    // Indicating left, but lane 0 now has a stopped car just ahead and lane 1
    // is clear.
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(1, 200.0, 10.0));
    sim.indicate(id, Side::Left);
    let stopped = sim.insert(car(), at(0, 215.0, 0.0));
    sim.hold(stopped);
    sim.state.time = 2.0;
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Cancel, "{evals:?}");
}

#[test]
fn it_doesnt_decide_partway_through_a_change() {
    let mut sim = two_lanes();
    let id = sim.insert(car(), at(1, 200.0, 10.0));
    sim.change_lane(id, Side::Left).unwrap();
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Nothing);
    assert!(evals.is_empty());
}

// Crossing into the next lane section. These step only motion and occupancy,
// at constant speed, so no decision or longitudinal update interferes.

/// Steps `id` along at its speed until its rear axle crosses into `lane`.
fn advance_onto<D: DebugSink>(sim: &mut Simulation<D>, id: VehicleId, lane: u32) {
    for _ in 0..600 {
        if sim.state.vehicles[id].path[0] == LaneId(lane) {
            return;
        }
        sim.update_motion(1.0 / 60.0);
        sim.update_occupancy(0.0);
    }
    panic!("never reached lane {lane}");
}

#[test]
fn at_a_fork_it_joins_every_branch_and_leaves_those_it_doesnt_take() {
    // Lane 0 forks at 100 m into lane 1, straight on, and lane 2, bearing left.
    let lane = |x0: f64, x1: f64, y1: f64| {
        Lane::new(&[DVec2::new(x0, 0.0), DVec2::new(x1, y1)], WIDTH, LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![
        lane(0.0, 100.0, 0.0),
        lane(100.0, 200.0, 0.0),
        lane(100.0, 200.0, 20.0),
    ]);
    network.connect(LaneId(0), LaneId(1));
    network.connect(LaneId(0), LaneId(2));
    let mut sim = Simulation::new(network, NoopSink);
    let id = sim.insert(car(), at(0, 90.0, 10.0));
    // Straight on, though lane 2, on the left, is its first successor.
    let path = &mut sim.state.vehicles[id].path;
    path.clear();
    path.extend([LaneId(0), LaneId(1)]);
    advance_onto(&mut sim, id, 1);
    let mine = records(&sim, id);
    assert!(mine.contains(&(1, false, Lateral::InLane)), "{mine:?}");
    // Lane 1 lies to the right of lane 2, so it is moving out of it that way.
    assert!(
        mine.contains(&(2, false, Lateral::Diverging(Side::Right))),
        "{mine:?}"
    );
    // Clear of lane 2 once the branches have parted, it leaves it.
    for _ in 0..600 {
        sim.update_motion(1.0 / 60.0);
        sim.update_occupancy(0.0);
        if records(&sim, id).iter().all(|r| r.0 != 2) {
            break;
        }
    }
    let mine = records(&sim, id);
    assert!(mine.iter().all(|r| r.0 != 2), "{mine:?}");
    assert!(mine.contains(&(1, false, Lateral::InLane)), "{mine:?}");
}

/// Lanes holding a record of `id`, whether each is its ghost, and how it
/// relates to the lane, converging records whatever their turn.
fn records<D: DebugSink>(sim: &Simulation<D>, id: VehicleId) -> Vec<(u32, bool, Lateral)> {
    let lanes = sim.state.occupants.iter().enumerate();
    lanes
        .flat_map(|(i, list)| {
            let mine = list.iter().filter(move |o| o.id == id);
            mine.map(move |o| {
                let lateral = match o.lateral {
                    Lateral::Converging { .. } => Lateral::CONVERGING,
                    other => other,
                };
                (i as u32, o.is_ghost(), lateral)
            })
        })
        .collect()
}

/// Two sections of two lanes, from 0 to 100 m and 100 to 200 m: lanes 0 and 1,
/// then 2 and 3, with 0 and 2 on the left.
fn two_sections() -> SimNetwork {
    road(&[(0.0, 100.0), (100.0, 200.0)], 2)
}

#[test]
fn an_indicating_car_takes_its_ghost_into_the_next_section() {
    let mut sim = Simulation::new(two_sections(), NoopSink);
    let id = sim.insert(car(), at(1, 97.0, 5.0));
    sim.indicate(id, Side::Left);
    assert!(records(&sim, id).contains(&(0, true, Lateral::Ghost)));
    advance_onto(&mut sim, id, 3);
    let mine = records(&sim, id);
    assert!(
        mine.contains(&(2, true, Lateral::Ghost)),
        "no ghost beside lane 3: {mine:?}"
    );
    assert!(
        !mine.iter().any(|r| r.0 == 0 && r.1),
        "ghost left behind: {mine:?}"
    );
    assert_eq!(
        sim.state.vehicles[id].intent.map(|i| i.side),
        Some(Side::Left)
    );
}

#[test]
fn an_intent_ends_where_the_next_section_has_no_neighbour_there() {
    // Lane 1, on the right, ends at 100 m: lane 0 runs on as lane 2, alone.
    let straight = |x0: f64, x1: f64, y: f64| {
        Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT).unwrap()
    };
    let lanes = vec![
        straight(0.0, 100.0, 0.0),
        straight(0.0, 100.0, -WIDTH),
        straight(100.0, 200.0, 0.0),
    ];
    let mut network = SimNetwork::new(lanes);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    network.connect(LaneId(0), LaneId(2));
    network.validate().unwrap();
    let mut sim = Simulation::new(network, NoopSink);
    let id = sim.insert(car(), at(0, 97.0, 5.0));
    sim.indicate(id, Side::Right);
    advance_onto(&mut sim, id, 2);
    assert!(sim.state.vehicles[id].intent.is_none());
    assert!(
        !records(&sim, id).iter().any(|r| r.1),
        "a ghost is left: {:?}",
        records(&sim, id)
    );
}

/// Records each vehicle's leaders, from the last longitudinal update.
#[derive(Default)]
struct Leaders(Vec<(VehicleId, VehicleId)>);

impl DebugSink for Leaders {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::AccelTerm { vehicle, source: AccelSource::Leader { id, .. }, .. } => {
                self.0.push((vehicle, id))
            }
            _ => {}
        }
    }
}

#[test]
fn a_straddling_car_carries_its_old_side_into_the_next_section() {
    let mut sim = Simulation::new(two_sections(), Leaders::default());
    let id = sim.insert(car(), at(1, 96.0, 5.0));
    sim.change_lane(id, Side::Left).unwrap();
    // Held on the old side's next lane, within reach of the change.
    let held = sim.insert(car(), at(3, 10.0, 0.0));
    sim.hold(held);
    advance_onto(&mut sim, id, 2);
    let mine = records(&sim, id);
    // It joined the old side's next lane, diverging from it.
    assert!(
        mine.contains(&(3, false, Lateral::Diverging(Side::Left))),
        "{mine:?}"
    );
    // Its rear is still on the old lane before, which it is leaving too.
    assert!(
        mine.contains(&(1, false, Lateral::Diverging(Side::Left))),
        "{mine:?}"
    );
    assert_eq!(sim.state.vehicles[id].old_side, Some(Side::Right));
    // Its old-side search runs on along the next section, to the held car.
    sim.update_accels();
    assert!(sim.sink.0.contains(&(id, held)), "leaders {:?}", sim.sink.0);
}

#[test]
fn at_a_lane_drop_the_old_side_clears_only_when_the_last_old_lane_record_goes() {
    // Lane 1, on the right, ends at 100 m; lane 0 runs on as lane 2.
    let straight = |x0: f64, x1: f64, y: f64| {
        Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT).unwrap()
    };
    let lanes = vec![
        straight(0.0, 100.0, 0.0),
        straight(0.0, 100.0, -WIDTH),
        straight(100.0, 200.0, 0.0),
    ];
    let mut network = SimNetwork::new(lanes);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    network.connect(LaneId(0), LaneId(2));
    let mut sim = Simulation::new(network, NoopSink);
    let id = sim.insert(car(), at(1, 97.0, 5.0));
    sim.change_lane(id, Side::Left).unwrap();
    // Through the boundary and on: it straddles while it has any record on a
    // lane it has left, and only then.
    for _ in 0..120 {
        sim.update_motion(1.0 / 60.0);
        sim.update_occupancy(0.0);
        let v = &sim.state.vehicles[id];
        let leaving = records(&sim, id).iter().any(|r| !r.1 && LaneId(r.0) != v.path[0]);
        let on_old_lane = records(&sim, id).iter().any(|r| r.0 == 1);
        assert!(
            !on_old_lane || v.old_side.is_some(),
            "cleared while still on lane 1"
        );
        if !leaving {
            assert_eq!(v.old_side, None, "still straddling with nothing left");
        }
    }
    assert_eq!(sim.state.vehicles[id].path[0], LaneId(2));
}

// Building up advantage.

/// A car on lane 1 at 300 m behind a car held stopped `gap` metres ahead, not
/// yet indicating, with an empty lane 0 to change into.
fn behind_a_stopped_car(gap: f64) -> (Simulation, VehicleId) {
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), at(1, 300.0 + gap, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, 300.0, 5.0));
    (sim, id)
}

#[test]
fn traffic_stopped_ahead_makes_it_want_the_free_lane_once_its_desire_builds_up() {
    let (mut sim, id) = behind_a_stopped_car(20.0);
    let (action, evals) = decide_now(&sim, id);
    assert_eq!(action, Decision::Nothing);
    let left = evals.iter().find(|e| e.side == Side::Left).unwrap();
    assert_eq!(left.outcome, Outcome::NotWorthIt);
    // Its lane stopped, the other free: as fast as it would like to go there,
    // over the gain that makes a full desire.
    let wanted = car().desired_speed(LIMIT) / lane_change::V_GAIN;
    assert!((left.incentives.speed - wanted).abs() < 1e-9, "{left:?}");
    // Smoothed: one step goes a step's share of the way.
    let step =
        sim.state.config.step * f64::from(sim.state.config.accel_every) / lane_change::DESIRE_TIME;
    assert!(
        (left.desire - wanted.min(1.0) * step).abs() < 1e-9,
        "{left:?}"
    );
    // Once its desire has built up, it checks the gap and indicates.
    sim.state.vehicles[id].lc.set_desire([WANTING, 0.0]);
    assert_eq!(decide_now(&sim, id).0, Decision::Indicate(Side::Left));
}

#[test]
fn it_wont_indicate_into_an_unsafe_gap_short_of_synchronising() {
    // A fast car close behind in lane 0: changing would cut it off.
    let (mut sim, id) = behind_a_stopped_car(20.0);
    sim.insert(car(), at(0, 290.0, 14.0));
    sim.state.vehicles[id].lc.set_desire([WANTING, 0.0]);
    let (action, state, evals) = decision(&sim, id);
    assert_eq!(action, Decision::Nothing);
    assert_eq!(evals[0].outcome, Outcome::Unsafe);
    // It waits a few steps before checking again.
    let dt = sim.state.config.step * f64::from(sim.state.config.accel_every);
    assert_eq!(state.cooldown(), lane_change::cooldown(&car(), dt));
    // Wanting it enough to synchronise, it indicates all the same, asking
    // for room.
    sim.state.vehicles[id].lc.set_desire([lane_change::D_SYNC + 0.05, 0.0]);
    let (action, evals) = decide_now(&sim, id);
    assert_eq!(evals[0].outcome, Outcome::Request);
    assert_eq!(action, Decision::Indicate(Side::Left));
}

#[test]
fn a_change_starts_its_desire_afresh() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    sim.update_lane_changes(sim.state.config.step * f64::from(sim.state.config.accel_every));
    assert_eq!(
        sim.state.vehicles[id].path[0],
        LaneId(0),
        "it should have changed"
    );
    let lc = sim.state.vehicles[id].lc;
    assert_eq!((lc.desire(Side::Left), lc.desire(Side::Right)), (0.0, 0.0));
}

#[test]
fn synchronising_it_adjusts_its_speed_to_the_target_lanes_leader() {
    // Indicating left with desire enough to synchronise, a slow car just
    // ahead in lane 0, nothing ahead in its own lane.
    let accel = |desire: f64| {
        let mut sim = two_lanes();
        let id = sim.insert(car(), at(1, 300.0, 12.0));
        sim.insert(car(), at(0, 315.0, 4.0));
        sim.indicate(id, Side::Left);
        sim.state.vehicles[id].lc.set_desire([desire, 0.0]);
        sim.update_accels();
        sim.state.vehicles[id].accel
    };
    assert!(accel(WANTING) > 0.0, "{}", accel(WANTING));
    let synchronising = accel(lane_change::D_SYNC + 0.1);
    assert!(synchronising < 0.0, "{synchronising}");
    assert!(synchronising >= -car().comfy_decel, "{synchronising}");
}

/// Two lanes side by side, as [`two_lanes`], lane 0 on the left, with the
/// lanes set as `lanes` says.
fn two_lanes_with(lanes: impl Fn(Lane) -> Lane) -> Simulation {
    let lane = |y: f64| {
        lanes(Lane::new(&[DVec2::new(0.0, y), DVec2::new(500.0, y)], WIDTH, LIMIT).unwrap())
    };
    let mut network = SimNetwork::new(vec![lane(0.0), lane(-WIDTH)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    Simulation::new(network, NoopSink)
}

#[test]
fn keeping_to_a_side_is_a_free_lane_changes_worth_of_desire_toward_it() {
    let mut sim = two_lanes_with(|lane| lane.with_keep(crate::network::Keep::Left));
    let id = sim.insert(car(), at(1, 300.0, 10.0));
    let (_, evals) = decide_now(&sim, id);
    assert_eq!(evals[0].side, Side::Left);
    assert_eq!(evals[0].incentives.keep, lane_change::D_FREE);
    // Already on the side kept to, nothing pulls it further.
    let other = sim.insert(car(), at(0, 100.0, 10.0));
    let (_, evals) = decide_now(&sim, other);
    assert_eq!(evals[0].incentives.keep, 0.0);
}

#[test]
fn a_lane_left_early_makes_drivers_want_to_leave_it_more_near_its_end() {
    // Lane 1 ends by merging into lane 0, and drivers leave it early.
    let early = Some(crate::network::Leaving::Early);
    let mut network = {
        let lane = |y: f64| Lane::new(&[DVec2::new(0.0, y), DVec2::new(500.0, y)], WIDTH, LIMIT);
        SimNetwork::new(vec![
            lane(0.0).unwrap(),
            lane(-WIDTH).unwrap().with_leaving(early),
        ])
    };
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    let mut sim = Simulation::new(network, NoopSink);
    let far = sim.insert(car(), at(1, 50.0, 10.0));
    let near = sim.insert(car(), at(1, 450.0, 10.0));
    let route = |id| decide_now(&sim, id).1[0].incentives.route;
    assert!(
        route(near) > route(far),
        "{} vs {}",
        route(near),
        route(far)
    );
    assert!(route(near) > 0.5, "{}", route(near));
    // Drivers in the lane that goes on have no reason to leave it.
    let staying = sim.insert(car(), at(0, 450.0, 10.0));
    let evals = decide_now(&sim, staying).1;
    assert_eq!(evals[0].incentives.route, 0.0);
}

/// A car indicating left from lane 1 at 300 m, at 10 m/s, and one in lane 0
/// `behind` metres back, at 10 m/s, with the given courtesy.
fn behind_an_indicator(behind: f64, courtesy: f64) -> (Simulation, VehicleId, VehicleId) {
    let mut sim = two_lanes();
    let indicating = sim.insert(car(), at(1, 300.0, 10.0));
    sim.indicate(indicating, Side::Left);
    // Wanting it enough for followers to cooperate.
    sim.state.vehicles[indicating].lc.set_desire([lane_change::D_COOP, 0.0]);
    let spec = VehicleSpec { courtesy, ..car() };
    let follower = sim.insert(spec, at(0, 300.0 - behind, 10.0));
    (sim, indicating, follower)
}

#[test]
fn a_courteous_follower_makes_room_for_an_indicating_car() {
    let (mut sim, indicating, follower) = behind_an_indicator(15.0, 2.0);
    sim.update_accels();
    assert_eq!(sim.state.vehicles[follower].lc.yielding(), Some(indicating));
    assert!(
        sim.state.vehicles[follower].accel < 0.0,
        "it should ease off"
    );
}

#[test]
fn followers_make_room_only_for_a_driver_wanting_it_enough_to_cooperate() {
    let (mut sim, indicating, follower) = behind_an_indicator(15.0, 2.0);
    sim.state.vehicles[indicating].lc.set_desire([WANTING, 0.0]);
    sim.update_accels();
    assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
}

#[test]
fn a_follower_drives_on_if_making_room_would_take_more_than_its_courtesy() {
    // Making room would take braking harder than 0.2 m/s².
    let (mut sim, _, follower) = behind_an_indicator(15.0, 0.2);
    sim.update_accels();
    assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
    // One with no courtesy never does, however easy.
    let (mut sim, _, follower) = behind_an_indicator(80.0, 0.0);
    sim.update_accels();
    assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
}

#[test]
fn a_follower_keeps_making_room_once_it_has_chosen_to() {
    // Having chosen to, it carries on even where it wouldn't start: here its
    // courtesy alone wouldn't stretch to it.
    let (mut sim, indicating, follower) = behind_an_indicator(15.0, 0.2);
    sim.state.vehicles[follower].lc.set_yielding(Some(indicating));
    sim.update_accels();
    assert_eq!(sim.state.vehicles[follower].lc.yielding(), Some(indicating));
    // Once the indicator is gone, there is nothing to make room for.
    sim.cancel_intent(indicating);
    sim.update_accels();
    assert_eq!(sim.state.vehicles[follower].lc.yielding(), None);
}

// Several leaders.

/// Every leader `id` reacts to now, nearest first.
fn leaders(sim: &Simulation, id: VehicleId) -> Vec<VehicleId> {
    let vehicle = |ahead: Ahead| match ahead {
        Ahead::Vehicle(leader) => Some(leader),
        _ => None,
    };
    ahead(sim, id).into_iter().filter_map(vehicle).collect()
}

/// Everything `id`'s leader search finds now, nearest first.
fn ahead(sim: &Simulation, id: VehicleId) -> Vec<Ahead> {
    let mut found = Vec::new();
    let ControlFlow::Continue(()) = search(sim, id, |f| {
        if let Found::Ahead(ahead, _, _) = f {
            found.push(ahead);
        }
        ControlFlow::<!>::Continue(())
    });
    found
}

#[test]
fn a_car_sees_a_diverging_car_its_leader_has_passed() {
    // C, changing left out of lane 1, held with its right rear corner 1.6 m
    // left of lane 1's centerline. B, in lane 1 behind it, clears that by
    // 0.65 m and passes; A, wider, behind B, wouldn't clear it by enough.
    let mut sim = two_lanes();
    // Fast enough for a gentle change, its tail never swinging back.
    let c = sim.insert(car(), at(1, 60.0, 3.75));
    sim.change_lane(c, Side::Left).unwrap();
    let lane_1 = sim.state.network.lane(LaneId(1));
    let pos = lane_1.geometry().lane_to_world(lane_1.geometry().lane_pos(60.0, 2.55));
    let lane_0 = sim.state.network.lane(LaneId(0));
    let on_0 = lane_0.geometry().world_to_lane_near(pos, lane_0.geometry().lane_pos(60.0, 0.0));
    let v = &mut sim.state.vehicles[c];
    (v.pos, v.heading, v.lane_pos) = (pos, 0.0, on_0);
    sim.update_occupancy(0.0);
    let record = sim.state.occupants[1].iter().find(|o| o.id == c).unwrap();
    assert_eq!(record.lateral, Lateral::Diverging(Side::Left));
    let b = sim.insert(car(), at(1, 45.0, 10.0));
    let wide = VehicleSpec { width: 2.6, ..car() };
    let a = sim.insert(wide, at(1, 30.0, 10.0));
    assert_eq!(leaders(&sim, b), [], "B passes C");
    // A reacts to B, and to C past it, rather than finding C only once B is by.
    assert_eq!(leaders(&sim, a), [b, c]);
}

#[test]
fn a_leader_turning_off_doesnt_hide_the_queue_beyond() {
    // Lane 0 splits: lane 2 bears away, lane 1 runs straight on. B turns onto
    // lane 2; A goes straight on, where a car is held stopped.
    let lane = |points: &[DVec2]| Lane::new(points, WIDTH, LIMIT).unwrap();
    let lanes = vec![
        lane(&[DVec2::ZERO, DVec2::new(100.0, 0.0)]),
        lane(&[DVec2::new(100.0, 0.0), DVec2::new(200.0, 0.0)]),
        lane(&[
            DVec2::new(100.0, 0.0),
            DVec2::new(150.0, 5.0),
            DVec2::new(200.0, 15.0),
        ]),
    ];
    let mut network = SimNetwork::new(lanes);
    network.connect(LaneId(0), LaneId(2));
    network.connect(LaneId(0), LaneId(1));
    let mut sim = Simulation::new(network, NoopSink);
    let queued = sim.insert(car(), at(1, 5.0, 0.0));
    sim.hold(queued);
    let b = sim.insert(car(), at(0, 80.0, 10.0));
    let path = &mut sim.state.vehicles[b].path;
    path.clear();
    path.extend([LaneId(0), LaneId(2)]);
    let a = sim.insert(car(), at(0, 50.0, 10.0));
    let path = &mut sim.state.vehicles[a].path;
    path.clear();
    path.extend([LaneId(0), LaneId(1)]);
    assert_eq!(leaders(&sim, a), [b, queued]);
}

#[test]
fn the_search_stops_at_the_second_car_in_the_lane() {
    // What constrains B, the first car ahead, reaches A through B: A needn't
    // look past the next car in the lane, D, at all.
    let mut sim = two_lanes();
    let a = sim.insert(car(), at(0, 20.0, 10.0));
    let b = sim.insert(car(), at(0, 40.0, 10.0));
    let _d = sim.insert(car(), at(0, 60.0, 10.0));
    let stopped = sim.insert(car(), at(0, 120.0, 0.0));
    sim.hold(stopped);
    assert_eq!(leaders(&sim, a), [b]);
}

#[test]
fn a_leader_going_the_same_way_shields_the_lanes_beyond() {
    // B carries on from lane 0 onto lane 1, as A will: what is on lane 1
    // reaches A through B.
    let mut sim = Simulation::new(road(&[(0.0, 100.0), (100.0, 200.0)], 1), NoopSink);
    let a = sim.insert(car(), at(0, 50.0, 10.0));
    let b = sim.insert(car(), at(0, 80.0, 10.0));
    let stopped = sim.insert(car(), at(1, 20.0, 0.0));
    sim.hold(stopped);
    assert_eq!(sim.state.vehicles[b].path[1], LaneId(1));
    assert_eq!(leaders(&sim, a), [b]);
}

#[test]
fn a_ghost_beyond_the_leader_is_still_found_and_shields_nothing() {
    // C, in lane 1, indicates into lane 0 beyond B. B shields the in-lane
    // vehicles beyond it, but not a ghost, which only partly blocks the lane.
    let mut sim = two_lanes();
    let a = sim.insert(car(), at(0, 20.0, 10.0));
    let b = sim.insert(car(), at(0, 40.0, 10.0));
    let c = sim.insert(car(), at(1, 60.0, 10.0));
    sim.indicate(c, Side::Left);
    assert_eq!(ahead(&sim, a), [Ahead::Vehicle(b), Ahead::Ghost(c)]);
}

#[test]
fn a_ghost_doesnt_shield_the_lane_beyond_it() {
    // C's ghost is A's nearest record, but only partly blocks the lane, so B
    // beyond it is still a leader.
    let mut sim = two_lanes();
    let a = sim.insert(car(), at(0, 20.0, 10.0));
    let c = sim.insert(car(), at(1, 40.0, 10.0));
    sim.indicate(c, Side::Left);
    let b = sim.insert(car(), at(0, 60.0, 10.0));
    assert_eq!(ahead(&sim, a), [Ahead::Ghost(c), Ahead::Vehicle(b)]);
}

// The end of the known path.

/// Records every acceleration term, from the last longitudinal update.
#[derive(Default)]
struct Terms(Vec<(VehicleId, AccelSource, f64)>);

impl DebugSink for Terms {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.0.push((vehicle, source, accel))
            }
            _ => {}
        }
    }
}

/// One lane in `sections` sections of 20 m each, and a car at 5 m on the first
/// at 10 m/s, knowing only the first `known` of them, after a longitudinal
/// update: the gap to the end of its path, if it reacted to one, and that term.
fn path_end_term(sections: usize, known: usize) -> Option<(f64, f64)> {
    let sections: Vec<_> =
        (0..sections).map(|k| (20.0 * k as f64, 20.0 * (k + 1) as f64)).collect();
    let mut sim = Simulation::new(road(&sections, 1), Terms::default());
    let id = sim.insert(car(), at(0, 5.0, 10.0));
    // As a routing model would leave it, not knowing where it goes next.
    sim.state.vehicles[id].path.truncate(known);
    sim.update_accels();
    sim.sink.0.iter().find_map(|&(_, source, accel)| match source {
        AccelSource::PathEnd { gap } => Some((gap, accel)),
        _ => None,
    })
}

#[test]
fn the_end_of_a_known_path_is_a_stop_line() {
    // Its path holds the first three sections, and the road goes on beyond: it
    // doesn't know where, so it is ready to stop at 60 m, its front at 8.6 m.
    let (gap, accel) = path_end_term(6, 3).expect("no stop line at the path's end");
    assert!((gap - 51.4).abs() < 1e-9, "gap {gap}");
    assert_eq!(accel, acceleration::interaction(&car(), 10.0, gap, 0.0));
}

#[test]
fn a_path_ending_where_the_network_does_is_an_exit() {
    // The same, but the road ends with the path: it leaves there.
    assert_eq!(path_end_term(3, 3), None);
}

// Paths over short lanes.

/// One lane in `count` sections, each `length` long.
fn short_sections(length: f64, count: usize) -> SimNetwork {
    let sections: Vec<_> =
        (0..count).map(|k| (length * k as f64, length * (k + 1) as f64)).collect();
    road(&sections, 1)
}

/// Whether a vehicle's path reaches as far as its leader search looks.
fn covers_horizon(sim: &Simulation<impl DebugSink>, id: VehicleId) -> bool {
    let v = &sim.state.vehicles[id];
    crate::path::reach_beyond_front(&sim.state.network, v)
        >= acceleration::horizon(&v.spec, v.speed)
}

#[test]
fn a_path_reaches_past_the_horizon_however_short_the_lanes() {
    // 2 m lanes: a car held stopped 30 lanes on, its rear 55.4 m ahead, is in
    // view, and there is no end of the path short of the horizon.
    let mut sim = Simulation::new(short_sections(2.0, 200), Terms::default());
    let id = sim.insert(car(), at(0, 1.0, LIMIT));
    let stopped = sim.insert(car(), at(30, 1.0, 0.0));
    sim.hold(stopped);
    assert!(covers_horizon(&sim, id));
    assert!(
        sim.state.vehicles[id].path.len() > 40,
        "{}",
        sim.state.vehicles[id].path.len()
    );
    sim.update_accels();
    let terms: Vec<_> = sim.sink.0.iter().filter(|t| t.0 == id).map(|t| t.1).collect();
    assert!(
        terms
            .iter()
            .any(|s| matches!(s, AccelSource::Leader { id, .. } if *id == stopped)),
        "{terms:?}"
    );
    assert!(
        !terms.iter().any(|s| matches!(s, AccelSource::PathEnd { .. })),
        "{terms:?}"
    );
    // Driving on, it keeps knowing as far ahead.
    for _ in 0..60 {
        sim.update_motion(1.0 / 60.0);
        sim.update_occupancy(0.0);
        assert!(covers_horizon(&sim, id));
    }
}

#[test]
fn a_vehicle_crosses_several_short_lanes_in_one_step() {
    // 0.1 m lanes, at 14 m/s: about 0.23 m a step.
    let mut sim = Simulation::new(short_sections(0.1, 2000), NoopSink);
    let id = sim.insert(car(), at(0, 0.05, LIMIT));
    sim.update_motion(1.0 / 60.0);
    let v = &sim.state.vehicles[id];
    assert_eq!(v.path[0], LaneId(2), "rear axle at {:?}", v.lane_pos);
    assert!((0.0..0.1).contains(&v.lane_pos.s), "{:?}", v.lane_pos);
    assert!(covers_horizon(&sim, id));
}

#[test]
fn the_follower_search_goes_back_as_far_as_paths_reach() {
    // 5 m lanes: the follower's front is 41.4 m back, eight lanes before.
    let mut sim = Simulation::new(short_sections(5.0, 40), NoopSink);
    let behind = sim.insert(car(), at(0, 1.0, LIMIT));
    let (found, gap) = follower(&sim, 9, 1.0, VehicleId::null()).unwrap();
    assert_eq!(found, behind);
    assert!((gap - 41.4).abs() < 1e-9, "gap {gap}");
}

#[test]
fn a_car_at_rest_short_of_a_stop_line_is_released_unless_queued_past_it() {
    // A connector from the south, starting with a stop line, crosses one from
    // the west.
    let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], WIDTH, LIMIT).unwrap();
    let p = DVec2::new;
    let mut network = SimNetwork::new(vec![
        lane(p(0.0, -100.0), p(0.0, -10.0)),
        lane(p(0.0, -10.0), p(0.0, 10.0)),
        lane(p(-10.0, 0.0), p(10.0, 0.0)),
    ]);
    network.connect(LaneId(0), LaneId(1));
    network.add_junction(&[LaneId(1), LaneId(2)]);
    let yields = Some(network.add_yields(&[LaneId(2)]));
    let line = Barrier { yields, ..Barrier::new(0.0, BarrierKind::Stop) };
    network.add_barrier(LaneId(1), line);
    let mut sim = Simulation::new(network, NoopSink);
    let id = sim.insert(car(), at(0, 80.0, 0.0));
    let vehicle = &sim.state.vehicles[id];
    assert!(barrier::released(
        &sim.state,
        id,
        vehicle,
        LaneId(1),
        line,
        5.0
    ));
    sim.state.queues[0].committed.push(Entry { id, until: None });
    let vehicle = &sim.state.vehicles[id];
    assert!(!barrier::released(
        &sim.state,
        id,
        vehicle,
        LaneId(1),
        line,
        5.0
    ));
}
