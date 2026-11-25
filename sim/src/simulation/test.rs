//! Snapshots: each sets up one instant and checks what the core's searches,
//! merges and barriers make of it, without stepping. Its helpers are shared
//! with the models' tests.

use glam::DVec2;

use super::*;
use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink, NoopSink};
use crate::merge;
use crate::network::{Barrier, BarrierKind, Lane};

/// The length of a step in the unit tests' standard simulation: 30 Hz.
pub(crate) const STEP: f64 = 1.0 / 30.0;

/// The unit tests' standard stepping: 30 Hz steps, with the slower updates
/// every third, at 10 Hz.
pub(crate) const CONFIG: Config = Config { dt: STEP, macro_step: 3 };
pub(crate) const LIMIT: f64 = 14.0;
pub(crate) const WIDTH: f64 = 3.5;

pub(crate) fn car() -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        max_accel: 1.0,
        comfy_decel: 1.5,
        time_headway: 1.0,
        min_gap: 1.5,
        rear_axle: 1.0,
        front_axle: 3.7,
        class: crate::VehicleClass::Car,
        rail: false,
    }
}

/// The tests' driver: at the limit exactly, holding its speed without
/// dawdling, and stopping exactly a metre short of lines, so runs are
/// deterministic and easy to reason about.
pub(crate) fn driver() -> crate::models::driver::Driver {
    crate::models::driver::Driver {
        speed_factor: 1.0,
        accel_noise: 0.0,
        line_gap: 1.0,
        line_gap_spread: 0.0,
        courtesy: 1.0,
        politeness: 0.2,
        assertiveness: 0.0,
        seed: 0,
    }
}

/// A road no other lane is part of, for a lane that stands alone: each call
/// gives another.
pub(crate) fn alone() -> crate::network::RoadId {
    thread_local! {
        static NEXT: std::cell::Cell<u32> = const { std::cell::Cell::new(0) };
    }
    NEXT.with(|next| {
        next.set(next.get() + 1);
        crate::network::RoadId(next.get() - 1)
    })
}

/// A straight road along x in sections, each `lanes` lanes wide, lane 0 on
/// the left. Section `k`'s lane `i` is `LaneId(k * lanes + i)` and runs on to
/// section `k + 1`'s lane `i`.
pub(crate) fn road(sections: &[(f64, f64)], lanes: u32) -> SimNetwork {
    let mut all = Vec::new();
    for &(x0, x1) in sections {
        // The lanes of a section side by side are one road.
        let road = alone();
        for i in 0..lanes {
            let y = -WIDTH * i as f64;
            all.push(
                Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, LIMIT, road).unwrap(),
            );
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

/// A simulation of `network` stepping by [`CONFIG`].
/// What the tests of the models reach inside a simulation for.
impl Simulation {
    pub(crate) fn state_mut(&mut self) -> &mut SimState {
        &mut self.state
    }

    /// How far beyond each lane's end the paths through it reach.
    pub(crate) fn through(&self) -> &[f64] {
        &self.derived.through
    }

    /// Measures the gaps drivers' intents ask for, as a lane-change update
    /// does first.
    pub(crate) fn measure_prospects(&mut self) {
        super::step::measure_prospects(&self.state, &self.derived.through, &mut self.prospects);
    }
}

pub(crate) fn new_sim(network: SimNetwork) -> Simulation {
    Simulation::with_config(network, CONFIG)
}

/// Two lanes side by side, 500 m long.
pub(crate) fn two_lanes() -> Simulation {
    new_sim(road(&[(0.0, 500.0)], 2))
}

pub(crate) fn at(lane: u32, s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(lane), s, speed, route: Route::default() }
}

/// A vehicle's rear, where both searches split its lane.
pub(crate) fn rear(sim: &Simulation, id: VehicleId) -> f64 {
    let v = &sim.state.vehicles[id];
    v.lane_pos.s - v.spec.rear_axle
}

pub(crate) fn follower(
    sim: &Simulation,
    lane: u32,
    s: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    find_follower(&sim.state, &sim.derived.through, LaneId(lane), s, skip)
}

/// `id`'s search along its path, from its own record, calling `f` with each
/// thing it finds until `f` breaks.
pub(crate) fn search<B>(
    sim: &Simulation,
    id: VehicleId,
    f: impl FnMut(Found) -> ControlFlow<B>,
) -> ControlFlow<B> {
    let v = &sim.state.vehicles[id];
    let own = crate::query::record_index(&sim.state, v.path[0], id);
    let reach = acceleration::kept_room(&sim.state.network, v).map_or(0.0, |h| v.spec.min_gap + h);
    find_leaders(&sim.state, v, Along::Path(own), reach, f)
}

/// The nearest vehicle `id` reacts to, stopping the search there.
pub(crate) fn leader(sim: &Simulation, id: VehicleId) -> Option<(VehicleId, f64)> {
    search(sim, id, |found| match found {
        Found::Ahead(Ahead::Vehicle(leader), gap, _) => ControlFlow::Break((leader, gap)),
        _ => ControlFlow::Continue(()),
    })
    .break_value()
}

#[test]
fn the_follower_is_the_nearest_vehicle_behind() {
    let mut sim = two_lanes();
    let far = sim.insert(car(), crate::simulation::test::driver(), at(0, 20.0, LIMIT));
    let near = sim.insert(car(), crate::simulation::test::driver(), at(0, 40.0, LIMIT));
    let _ahead = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, 0.0));
    let (found, gap) = follower(&sim, 0, 60.0, VehicleId::null()).unwrap();
    assert_eq!(found, near);
    // From its front, 3.6 m ahead of its rear axle at 40, to 60.
    assert!((gap - 16.4).abs() < 1e-9, "gap {gap}");
    assert_ne!(found, far);
}

#[test]
fn the_follower_search_passes_over_ghosts_and_the_searcher() {
    let mut sim = two_lanes();
    let behind = sim.insert(car(), crate::simulation::test::driver(), at(0, 20.0, LIMIT));
    let searcher = sim.insert(car(), crate::simulation::test::driver(), at(0, 45.0, 0.0));
    // Indicating from lane 1 into lane 0, between the two.
    let indicating = sim.insert(car(), crate::simulation::test::driver(), at(1, 35.0, 0.0));
    sim.indicate(indicating, Side::Left);
    let found = follower(&sim, 0, 60.0, searcher).map(|(id, _)| id);
    assert_eq!(found, Some(behind));
}

#[test]
fn the_follower_search_counts_a_vehicle_moving_out_of_the_lane() {
    // Changing out of lane 0 into lane 1, it is still partly in the way.
    let mut sim = two_lanes();
    let leaving = sim.insert(car(), crate::simulation::test::driver(), at(0, 30.0, LIMIT));
    sim.change_lane(leaving, Side::Right).unwrap();
    let record = sim.state.occupants[0].iter().find(|o| o.id == leaving).unwrap();
    assert_eq!(record.lateral, Lateral::Diverging(Side::Right));
    let found = follower(&sim, 0, 60.0, VehicleId::null()).map(|(id, _)| id);
    assert_eq!(found, Some(leaving));
}

#[test]
fn the_follower_search_continues_onto_the_lanes_leading_in() {
    // Two sections of two lanes: lane 0 leads into lane 2.
    let mut sim = new_sim(road(&[(0.0, 100.0), (100.0, 200.0)], 2));
    let behind = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, LIMIT));
    let (found, gap) = follower(&sim, 2, 10.0, VehicleId::null()).unwrap();
    assert_eq!(found, behind);
    // Its front is at 83.6 on lane 0, 16.4 m before lane 2 starts.
    assert!((gap - 26.4).abs() < 1e-9, "gap {gap}");
}

#[test]
fn the_follower_search_stops_at_the_followers_horizon() {
    // Only as far back as the follower's own leader search would reach.
    let mut sim = two_lanes();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 50.0, LIMIT));
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
    let lane = Lane::new(
        &[DVec2::ZERO, DVec2::new(1000.0, 0.0)],
        WIDTH,
        LIMIT,
        alone(),
    )
    .unwrap();
    let mut network = SimNetwork::new(vec![lane]);
    network.connect(LaneId(0), LaneId(0));
    let mut sim = new_sim(network);
    let behind = sim.insert(
        car(),
        crate::simulation::test::driver(),
        at(0, 970.0, LIMIT),
    );
    let (found, gap) = follower(&sim, 0, 50.0, VehicleId::null()).unwrap();
    assert_eq!(found, behind);
    assert!((gap - 76.4).abs() < 1e-9, "gap {gap}");
}

#[test]
fn on_lanes_leading_in_only_vehicles_headed_this_way_count() {
    // Lane 0 splits into lane 1, straight on, and lane 2, bearing left.
    let lane = |x0: f64, x1: f64, y1: f64| {
        Lane::new(
            &[DVec2::new(x0, 0.0), DVec2::new(x1, y1)],
            WIDTH,
            LIMIT,
            alone(),
        )
        .unwrap()
    };
    let mut network = SimNetwork::new(vec![
        lane(0.0, 100.0, 0.0),
        lane(100.0, 200.0, 0.0),
        lane(100.0, 200.0, 10.0),
    ]);
    network.connect(LaneId(0), LaneId(2));
    network.connect(LaneId(0), LaneId(1));
    let mut sim = new_sim(network);
    // It turns onto lane 2, so it isn't lane 1's follower.
    let turning = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, LIMIT));
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
        let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 100.0, 10.0));
        let s = 100.0 + tenths as f64 / 10.0;
        let other = sim.insert(car(), crate::simulation::test::driver(), at(0, s, 10.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 100.0, 10.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 100.0, 10.0));
    sim.indicate(id, Side::Left);
    let ghost = sim.state.occupants[0].iter().find(|o| o.id == id).unwrap();
    assert_eq!(ghost.lateral, Lateral::Ghost);
    let own = sim.state.occupants[1].iter().find(|o| o.id == id).unwrap();
    assert_eq!(own.lateral, Lateral::InLane);
}

// Deciding.

/// Two lanes side by side, 500 m long, limited to 25 m/s, that run on side by
/// side for 200 m as lanes 2 and 5. Lane 2, on the left, is in a junction,
/// crossed 10 m along by lane 3, and runs on as lane 4; the network makes
/// lane 5 its neighbor, as at the ramps' lane drop.
pub(crate) fn two_lanes_into_a_junction() -> Simulation {
    let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], WIDTH, 25.0, alone()).unwrap();
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
    new_sim(network)
}

/// A taper: lane 0, from 0 to 100 m, and lane 1 starting beside it and
/// easing into it from 20 m on, both leading into lane 2, from 100 to 300 m.
/// Lanes 3 and 4 lead into 0 and 1 from 100 m back. Lane 5 comes in from the
/// side and leads into lane 2 as well, without starting with them.
pub(crate) fn a_taper() -> Simulation {
    let p = DVec2::new;
    let lane = |points: &[DVec2]| Lane::new(points, WIDTH, 25.0, alone()).unwrap();
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
    new_sim(network)
}

// Merges.

/// The order of the only merge, by vehicle.
pub(crate) fn merge_order(sim: &Simulation) -> Vec<VehicleId> {
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
        Lane::new(
            &[DVec2::new(x0, 0.0), DVec2::new(x1, y1)],
            WIDTH,
            LIMIT,
            alone(),
        )
        .unwrap()
    };
    let mut network = SimNetwork::new(vec![
        lane(0.0, 100.0, 0.0),
        lane(100.0, 200.0, 0.0),
        lane(100.0, 200.0, 20.0),
    ]);
    network.connect(LaneId(0), LaneId(1));
    network.connect(LaneId(0), LaneId(2));
    let sim = new_sim(network);
    assert!(!sim.derived.overlaps[1].is_empty(), "the branches overlap");
    assert_eq!(sim.derived.merges.len(), 0);
}

#[test]
fn a_vehicle_joins_a_merges_order_on_the_way_and_leaves_it_once_in_the_merge() {
    let mut sim = a_taper();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(3, 50.0, 10.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(3, 10.0, 0.0));
    assert_eq!(
        sim.state.vehicles[id].path,
        [LaneId(3), LaneId(0), LaneId(2)]
    );
    assert_eq!(merge_order(&sim), [id]);
}

#[test]
fn the_order_is_by_distance_to_the_merge_point_and_one_pulling_ahead_goes_first() {
    let mut sim = a_taper();
    let a = sim.insert(car(), crate::simulation::test::driver(), at(3, 50.0, 10.0));
    let b = sim.insert(car(), crate::simulation::test::driver(), at(4, 49.0, 20.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(4, 92.0, 10.0));
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
    let ahead = sim.insert(car(), crate::simulation::test::driver(), at(3, 85.0, 5.0));
    let behind = sim.insert(car(), crate::simulation::test::driver(), at(3, 60.0, 20.0));
    sim.step_variable(0.1);
    assert_eq!(merge_order(&sim), [ahead, behind]);
}

#[test]
fn side_by_side_before_a_merge_one_drops_back_gently_and_they_merge_apart() {
    let mut sim = a_taper();
    let a = sim.insert(car(), crate::simulation::test::driver(), at(3, 10.0, 15.0));
    let b = sim.insert(car(), crate::simulation::test::driver(), at(4, 10.0, 15.0));
    let (mut least, mut gap) = (f64::INFINITY, f64::INFINITY);
    for _ in 0..(10.0 / STEP) as u32 {
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
        .map(|p| sim.insert(car(), crate::simulation::test::driver(), p))
        .collect();
    let waiting = |sim: &Simulation, id| {
        let lane = sim.state.vehicles.get(id).map(|v: &Vehicle| v.path[0]);
        matches!(lane, Some(LaneId(3) | LaneId(4)))
    };
    let mut entered = Vec::new();
    for _ in 0..(60.0 / STEP) as u32 {
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
    let mut sim = new_sim(road(&[(0.0, 100.0), (100.0, 200.0)], 2));
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, 10.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(3, 90.0, 10.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(4, 95.0, 15.0));
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
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 30.0, 0.0));
    let mine = records(&sim, id);
    assert!(mine.contains(&(0, false, Lateral::CONVERGING)), "{mine:?}");
}

#[test]
fn a_follower_in_a_taper_stays_behind_a_vehicle_merging_in_beside_it() {
    let mut sim = a_taper();
    // Not yet overlapping lane 0 at 30 m, so a follower there could drive by.
    let merging = sim.insert(car(), crate::simulation::test::driver(), at(1, 30.0, 0.0));
    sim.hold(merging);
    let follower = sim.insert(car(), crate::simulation::test::driver(), at(3, 60.0, 15.0));
    for _ in 0..600 {
        sim.step_variable(0.05);
    }
    let v = &sim.state.vehicles[follower];
    assert_eq!(v.path[0], LaneId(0));
    assert!(v.speed < 0.1, "{}", v.speed);
    let theirs = sim.state.occupants[0].iter().find(|o| o.id == merging).unwrap();
    assert!(rear(&sim, follower) + v.spec.length < theirs.s());
}

#[test]
fn wanting_to_change_lanes_it_keeps_room_behind_a_stopped_leader() {
    // At rest 4 m behind a stopped car: settled, it creeps up to its
    // minimum gap; wanting to change, it holds back, room to pull out.
    let accel = |indicating: bool| {
        let mut sim = two_lanes();
        let blocker = sim.insert(car(), crate::simulation::test::driver(), at(1, 300.0, 0.0));
        sim.hold(blocker);
        let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 291.4, 0.0));
        if indicating {
            sim.indicate(id, Side::Left);
        }
        sim.update_accels(&mut NoopSink);
        sim.state.vehicles[id].accel
    };
    assert!(accel(false) > 0.0, "{}", accel(false));
    assert!(accel(true) < 0.0, "{}", accel(true));
}

pub(crate) const SAFE_BRAKING_FOR_TESTS: f64 = acceleration::SAFE_BRAKING;

/// Steps `id` along at its speed until its rear axle crosses into `lane`.
pub(crate) fn advance_onto(sim: &mut Simulation, id: VehicleId, lane: u32) {
    for _ in 0..600 {
        if sim.state.vehicles[id].path[0] == LaneId(lane) {
            return;
        }
        sim.update_motion(1.0 / 60.0, &mut NoopSink);
        sim.update_occupancy(0.0);
    }
    panic!("never reached lane {lane}");
}

#[test]
fn at_a_fork_it_joins_every_branch_and_leaves_those_it_doesnt_take() {
    // Lane 0 forks at 100 m into lane 1, straight on, and lane 2, bearing left.
    let lane = |x0: f64, x1: f64, y1: f64| {
        Lane::new(
            &[DVec2::new(x0, 0.0), DVec2::new(x1, y1)],
            WIDTH,
            LIMIT,
            alone(),
        )
        .unwrap()
    };
    let mut network = SimNetwork::new(vec![
        lane(0.0, 100.0, 0.0),
        lane(100.0, 200.0, 0.0),
        lane(100.0, 200.0, 20.0),
    ]);
    network.connect(LaneId(0), LaneId(1));
    network.connect(LaneId(0), LaneId(2));
    let mut sim = new_sim(network);
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 90.0, 10.0));
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
        sim.update_motion(1.0 / 60.0, &mut NoopSink);
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
pub(crate) fn records(sim: &Simulation, id: VehicleId) -> Vec<(u32, bool, Lateral)> {
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
pub(crate) fn two_sections() -> SimNetwork {
    road(&[(0.0, 100.0), (100.0, 200.0)], 2)
}

#[test]
fn an_indicating_car_takes_its_ghost_into_the_next_section() {
    let mut sim = new_sim(two_sections());
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 97.0, 5.0));
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
    assert_eq!(sim.state.vehicles[id].intent.indicating(), Some(Side::Left));
}

#[test]
fn an_intent_ends_where_the_next_section_has_no_neighbour_there() {
    // Lane 1, on the right, ends at 100 m: lane 0 runs on as lane 2, alone.
    let straight = |x0: f64, x1: f64, y: f64| {
        Lane::new(
            &[DVec2::new(x0, y), DVec2::new(x1, y)],
            WIDTH,
            LIMIT,
            alone(),
        )
        .unwrap()
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
    let mut sim = new_sim(network);
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 97.0, 5.0));
    sim.indicate(id, Side::Right);
    advance_onto(&mut sim, id, 2);
    assert!(sim.state.vehicles[id].intent.indicating().is_none());
    assert!(
        !records(&sim, id).iter().any(|r| r.1),
        "a ghost is left: {:?}",
        records(&sim, id)
    );
}

/// Records each vehicle's leaders, from the last longitudinal update.
#[derive(Default)]
pub(crate) struct Leaders(Vec<(VehicleId, VehicleId)>);

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
    let mut sim = new_sim(two_sections());
    let mut sink = Leaders::default();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 96.0, 5.0));
    sim.change_lane(id, Side::Left).unwrap();
    // Held on the old side's next lane, within reach of the change.
    let held = sim.insert(car(), crate::simulation::test::driver(), at(3, 10.0, 0.0));
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
    sim.update_accels(&mut sink);
    assert!(sink.0.contains(&(id, held)), "leaders {:?}", sink.0);
}

#[test]
fn at_a_lane_drop_the_old_side_clears_only_when_the_last_old_lane_record_goes() {
    // Lane 1, on the right, ends at 100 m; lane 0 runs on as lane 2.
    let straight = |x0: f64, x1: f64, y: f64| {
        Lane::new(
            &[DVec2::new(x0, y), DVec2::new(x1, y)],
            WIDTH,
            LIMIT,
            alone(),
        )
        .unwrap()
    };
    let lanes = vec![
        straight(0.0, 100.0, 0.0),
        straight(0.0, 100.0, -WIDTH),
        straight(100.0, 200.0, 0.0),
    ];
    let mut network = SimNetwork::new(lanes);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    network.connect(LaneId(0), LaneId(2));
    let mut sim = new_sim(network);
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 97.0, 5.0));
    sim.change_lane(id, Side::Left).unwrap();
    // Through the boundary and on: it straddles while it has any record on a
    // lane it has left, and only then.
    for _ in 0..120 {
        sim.update_motion(1.0 / 60.0, &mut NoopSink);
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
pub(crate) fn behind_a_stopped_car(gap: f64) -> (Simulation, VehicleId) {
    let mut sim = two_lanes();
    let blocker = sim.insert(
        car(),
        crate::simulation::test::driver(),
        at(1, 300.0 + gap, 0.0),
    );
    sim.hold(blocker);
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 300.0, 5.0));
    (sim, id)
}

/// Two lanes side by side, as [`two_lanes`], lane 0 on the left, with the
/// lanes set as `lanes` says.
pub(crate) fn two_lanes_with(lanes: impl Fn(Lane) -> Lane) -> Simulation {
    let lane = |y: f64| {
        lanes(
            Lane::new(
                &[DVec2::new(0.0, y), DVec2::new(500.0, y)],
                WIDTH,
                LIMIT,
                alone(),
            )
            .unwrap(),
        )
    };
    let mut network = SimNetwork::new(vec![lane(0.0), lane(-WIDTH)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    new_sim(network)
}

/// Every leader `id` reacts to now, nearest first.
pub(crate) fn leaders(sim: &Simulation, id: VehicleId) -> Vec<VehicleId> {
    let vehicle = |ahead: Ahead| match ahead {
        Ahead::Vehicle(leader) => Some(leader),
        _ => None,
    };
    ahead(sim, id).into_iter().filter_map(vehicle).collect()
}

/// Everything `id`'s leader search finds now, nearest first.
pub(crate) fn ahead(sim: &Simulation, id: VehicleId) -> Vec<Ahead> {
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
    let c = sim.insert(car(), crate::simulation::test::driver(), at(1, 60.0, 3.75));
    sim.change_lane(c, Side::Left).unwrap();
    let lane_1 = sim.state.network.lane(LaneId(1));
    let pos = lane_1.geometry().lane_to_world(lane_1.geometry().lane_pos(60.0, 2.55));
    let lane_0 = sim.state.network.lane(LaneId(0));
    let on_0 = lane_0.geometry().world_to_lane_near(pos, lane_0.geometry().lane_pos(60.0, 0.0));
    let v = &mut sim.state.vehicles[c];
    (v.pos, v.dir, v.lane_pos) = (pos, DVec2::X, on_0);
    sim.update_occupancy(0.0);
    let record = sim.state.occupants[1].iter().find(|o| o.id == c).unwrap();
    assert_eq!(record.lateral, Lateral::Diverging(Side::Left));
    let b = sim.insert(car(), crate::simulation::test::driver(), at(1, 45.0, 10.0));
    let wide = VehicleSpec { width: 2.6, ..car() };
    let a = sim.insert(wide, driver(), at(1, 30.0, 10.0));
    assert_eq!(leaders(&sim, b), [], "B passes C");
    // A reacts to B, and to C past it, rather than finding C only once B is by.
    assert_eq!(leaders(&sim, a), [b, c]);
}

#[test]
fn a_leader_turning_off_doesnt_hide_the_queue_beyond() {
    // Lane 0 splits: lane 2 bears away, lane 1 runs straight on. B turns onto
    // lane 2; A goes straight on, where a car is held stopped.
    let lane = |points: &[DVec2]| Lane::new(points, WIDTH, LIMIT, alone()).unwrap();
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
    let mut sim = new_sim(network);
    let queued = sim.insert(car(), crate::simulation::test::driver(), at(1, 5.0, 0.0));
    sim.hold(queued);
    let b = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, 10.0));
    let path = &mut sim.state.vehicles[b].path;
    path.clear();
    path.extend([LaneId(0), LaneId(2)]);
    let a = sim.insert(car(), crate::simulation::test::driver(), at(0, 50.0, 10.0));
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
    let a = sim.insert(car(), crate::simulation::test::driver(), at(0, 20.0, 10.0));
    let b = sim.insert(car(), crate::simulation::test::driver(), at(0, 40.0, 10.0));
    let _d = sim.insert(car(), crate::simulation::test::driver(), at(0, 60.0, 10.0));
    let stopped = sim.insert(car(), crate::simulation::test::driver(), at(0, 120.0, 0.0));
    sim.hold(stopped);
    assert_eq!(leaders(&sim, a), [b]);
}

#[test]
fn a_leader_going_the_same_way_shields_the_lanes_beyond() {
    // B carries on from lane 0 onto lane 1, as A will: what is on lane 1
    // reaches A through B.
    let mut sim = new_sim(road(&[(0.0, 100.0), (100.0, 200.0)], 1));
    let a = sim.insert(car(), crate::simulation::test::driver(), at(0, 50.0, 10.0));
    let b = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, 10.0));
    let stopped = sim.insert(car(), crate::simulation::test::driver(), at(1, 20.0, 0.0));
    sim.hold(stopped);
    assert_eq!(sim.state.vehicles[b].path[1], LaneId(1));
    assert_eq!(leaders(&sim, a), [b]);
}

#[test]
fn a_ghost_beyond_the_leader_is_still_found_and_shields_nothing() {
    // C, in lane 1, indicates into lane 0 beyond B. B shields the in-lane
    // vehicles beyond it, but not a ghost, which only partly blocks the lane.
    let mut sim = two_lanes();
    let a = sim.insert(car(), crate::simulation::test::driver(), at(0, 20.0, 10.0));
    let b = sim.insert(car(), crate::simulation::test::driver(), at(0, 40.0, 10.0));
    let c = sim.insert(car(), crate::simulation::test::driver(), at(1, 60.0, 10.0));
    sim.indicate(c, Side::Left);
    assert_eq!(ahead(&sim, a), [Ahead::Vehicle(b), Ahead::Ghost(c)]);
}

#[test]
fn a_ghost_doesnt_shield_the_lane_beyond_it() {
    // C's ghost is A's nearest record, but only partly blocks the lane, so B
    // beyond it is still a leader.
    let mut sim = two_lanes();
    let a = sim.insert(car(), crate::simulation::test::driver(), at(0, 20.0, 10.0));
    let c = sim.insert(car(), crate::simulation::test::driver(), at(1, 40.0, 10.0));
    sim.indicate(c, Side::Left);
    let b = sim.insert(car(), crate::simulation::test::driver(), at(0, 60.0, 10.0));
    assert_eq!(ahead(&sim, a), [Ahead::Ghost(c), Ahead::Vehicle(b)]);
}

// The end of the known path.

/// Records every acceleration term, from the last longitudinal update.
#[derive(Default)]
pub(crate) struct Terms(Vec<(VehicleId, AccelSource, f64)>);

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
pub(crate) fn path_end_term(sections: usize, known: usize) -> Option<(f64, f64)> {
    let sections: Vec<_> =
        (0..sections).map(|k| (20.0 * k as f64, 20.0 * (k + 1) as f64)).collect();
    let mut sim = new_sim(road(&sections, 1));
    let mut sink = Terms::default();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 5.0, 10.0));
    // As a routing model would leave it, not knowing where it goes next.
    sim.state.vehicles[id].path.truncate(known);
    sim.update_accels(&mut sink);
    sink.0.iter().find_map(|&(_, source, accel)| match source {
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
pub(crate) fn short_sections(length: f64, count: usize) -> SimNetwork {
    let sections: Vec<_> =
        (0..count).map(|k| (length * k as f64, length * (k + 1) as f64)).collect();
    road(&sections, 1)
}

/// Whether a vehicle's path reaches as far as its leader search looks.
pub(crate) fn covers_horizon(sim: &Simulation, id: VehicleId) -> bool {
    let v = &sim.state.vehicles[id];
    crate::motion::path::reach_beyond_front(&sim.state.network, v)
        >= acceleration::horizon(&v.spec, v.speed)
}

#[test]
fn a_path_reaches_past_the_horizon_however_short_the_lanes() {
    // 2 m lanes: a car held stopped 30 lanes on, its rear 55.4 m ahead, is in
    // view, and there is no end of the path short of the horizon.
    let mut sim = new_sim(short_sections(2.0, 200));
    let mut sink = Terms::default();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 1.0, LIMIT));
    let stopped = sim.insert(car(), crate::simulation::test::driver(), at(30, 1.0, 0.0));
    sim.hold(stopped);
    assert!(covers_horizon(&sim, id));
    assert!(
        sim.state.vehicles[id].path.len() > 40,
        "{}",
        sim.state.vehicles[id].path.len()
    );
    sim.update_accels(&mut sink);
    let terms: Vec<_> = sink.0.iter().filter(|t| t.0 == id).map(|t| t.1).collect();
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
        sim.update_motion(1.0 / 60.0, &mut NoopSink);
        sim.update_occupancy(0.0);
        assert!(covers_horizon(&sim, id));
    }
}

#[test]
fn a_vehicle_crosses_several_short_lanes_in_one_step() {
    // 0.1 m lanes, at 14 m/s: about 0.23 m a step.
    let mut sim = new_sim(short_sections(0.1, 2000));
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 0.05, LIMIT));
    sim.update_motion(1.0 / 60.0, &mut NoopSink);
    let v = &sim.state.vehicles[id];
    assert_eq!(v.path[0], LaneId(2), "rear axle at {:?}", v.lane_pos);
    assert!((0.0..0.1).contains(&v.lane_pos.s), "{:?}", v.lane_pos);
    assert!(covers_horizon(&sim, id));
}

#[test]
fn the_follower_search_goes_back_as_far_as_paths_reach() {
    // 5 m lanes: the follower's front is 41.4 m back, eight lanes before.
    let mut sim = new_sim(short_sections(5.0, 40));
    let behind = sim.insert(car(), crate::simulation::test::driver(), at(0, 1.0, LIMIT));
    let (found, gap) = follower(&sim, 9, 1.0, VehicleId::null()).unwrap();
    assert_eq!(found, behind);
    assert!((gap - 41.4).abs() < 1e-9, "gap {gap}");
}

#[test]
fn a_car_at_rest_short_of_a_stop_line_is_released() {
    // A connector from the south, starting with a stop line, crosses one from
    // the west.
    let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], WIDTH, LIMIT, alone()).unwrap();
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
    let mut sim = new_sim(network);
    let id = sim.insert(car(), crate::simulation::test::driver(), at(0, 80.0, 0.0));
    let vehicle = &sim.state.vehicles[id];
    assert!(barrier::released(&sim.state, vehicle, line, 5.0));
}

#[test]
fn a_car_it_doesnt_hold_up_isnt_its_old_follower() {
    // Far enough back that the free road binds it, not this car.
    let mut sim = two_lanes();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 300.0, 5.0));
    sim.insert(car(), crate::simulation::test::driver(), at(1, 240.0, 5.0));
    sim.update_accels(&mut NoopSink);
    assert!(sim.state.vehicles[id].held_up.is_none());
}

#[test]
fn a_car_its_speed_cap_holds_back_isnt_held_up_by_the_car_ahead() {
    // Close behind, but held to a stop: its cap binds, so leaving frees it of
    // nothing.
    let mut sim = two_lanes();
    let id = sim.insert(car(), crate::simulation::test::driver(), at(1, 300.0, 5.0));
    let behind = sim.insert(car(), crate::simulation::test::driver(), at(1, 290.0, 5.0));
    sim.update_accels(&mut NoopSink);
    assert_eq!(sim.state.vehicles[id].held_up.map(|f| f.id), Some(behind));
    sim.hold(behind);
    sim.update_accels(&mut NoopSink);
    assert!(sim.state.vehicles[id].held_up.is_none());
}
