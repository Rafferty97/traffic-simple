//! LMRS's tests: its decisions as pure functions, and what drivers decide in
//! situations set up in a simulation, through the core's test helpers.

use glam::DVec2;

use super::*;
use crate::debug::{DebugEvent, Field, NoopSink};
use crate::models::lane_change;
use crate::network::{Lane, LaneId, SimNetwork};
use crate::query::LaneSpeeds;
use crate::simulation::Simulation;
use crate::simulation::test::*;

/// The time between updates: 10 Hz.
const DT: f64 = 0.1;

/// A measured gap with no follower there.
fn prospect(own_after: f64, room: bool) -> Gap {
    Gap { own_after, new_follower: None, room }
}

fn spec() -> VehicleSpec {
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

#[test]
fn voluntary_desire_gives_way_to_a_strong_route_desire_against_it() {
    let toward = Incentives { route: 0.0, speed: 0.4, keep: 0.0, courtesy: 0.0 };
    assert_eq!(toward.desire(), 0.4);
    // A weak route desire the other way leaves the speed desire whole.
    let weak = Incentives { route: -0.3, ..toward };
    assert!((weak.desire() - 0.1).abs() < 1e-12);
    // A route desire strong enough to cooperate overrides it.
    let strong = Incentives { route: -0.8, ..toward };
    assert_eq!(strong.desire(), -0.8);
    // In between, it counts in part.
    let some = Incentives { route: -0.7, ..toward };
    assert!(
        some.desire() < -0.3 && some.desire() > -0.7,
        "{}",
        some.desire()
    );
}

#[test]
fn route_desire_rises_to_one_at_the_lane_end() {
    assert_eq!(route_desire(ROUTE_DISTANCE, 1.0, 1), 0.0);
    assert!((route_desire(0.5 * ROUTE_DISTANCE, 1.0, 1) - 0.5).abs() < 1e-12);
    assert_eq!(route_desire(0.0, 10.0, 1), 1.0);
    // Fast, time runs out before distance does.
    assert!(route_desire(ROUTE_DISTANCE, 30.0, 1) > 0.0);
    // With two changes to make, it starts twice as far out.
    assert!((route_desire(ROUTE_DISTANCE, 1.0, 2) - 0.5).abs() < 1e-12);
}

#[test]
fn desire_is_smoothed_and_a_check_waits_for_the_threshold() {
    let mut state = State::default();
    let wants = Incentives { speed: 0.9, ..Incentives::default() };
    let dt = 1.0 / 60.0;
    // A fleeting desire: one step doesn't get there.
    aim(&mut state, [Some(wants), None]);
    let ask = step(&mut state, LaneChangeIntent::None, &spec(), 0.0, dt);
    assert_eq!(ask, Ask::Nothing);
    assert!(state.desire(Side::Left) > 0.0 && state.desire(Side::Left) < D_FREE);
    // Lasting, it does, and asks for the gap.
    let mut ask = Ask::Nothing;
    for _ in 0..60 {
        ask = step(&mut state, LaneChangeIntent::None, &spec(), 0.0, dt);
        if ask != Ask::Nothing {
            break;
        }
    }
    assert_eq!(ask, Ask::Check(Side::Left));
    // Not permitted that way: no check, however strong.
    let mut state = State { desire: [0.9, 0.0], ..State::default() };
    assert_eq!(
        {
            aim(&mut state, [None, None]);
            step(&mut state, LaneChangeIntent::None, &spec(), 0.0, dt)
        },
        Ask::Nothing
    );
}

#[test]
fn a_refused_gap_cools_off_before_the_next_check() {
    let mut state = State { desire: [0.5, 0.0], ..State::default() };
    let refused = prospect(-5.0, true);
    let (decision, eval) = resolve(
        &mut state,
        LaneChangeIntent::None,
        &spec(),
        Side::Left,
        Incentives::default(),
        refused,
        DT,
    );
    assert_eq!(
        (decision, eval.outcome),
        (Decision::Nothing, Outcome::Unsafe)
    );
    let wait = state.cooldown();
    assert_eq!(wait, cooldown(&spec(), DT));
    let wants = Some(Incentives { speed: 0.5, ..Incentives::default() });
    aim(&mut state, [wants, None]);
    for _ in 0..wait {
        let ask = step(&mut state, LaneChangeIntent::None, &spec(), 0.0, DT);
        assert_eq!(ask, Ask::Nothing);
    }
    let ask = step(&mut state, LaneChangeIntent::None, &spec(), 0.0, DT);
    assert_eq!(ask, Ask::Check(Side::Left));
    // Politer drivers wait longer, more assertive ones less.
    let polite = VehicleSpec { politeness: 0.5, ..spec() };
    let assertive = VehicleSpec { assertiveness: 1.0, ..spec() };
    assert!(cooldown(&polite, DT) > cooldown(&spec(), DT));
    assert!(cooldown(&assertive, DT) < cooldown(&spec(), DT));
}

#[test]
fn enough_desire_indicates_without_a_gap_and_more_changes_without_room() {
    let unsafe_gap = prospect(-5.0, true);
    let mut state = State { desire: [D_SYNC, 0.0], ..State::default() };
    let (decision, _) = resolve(
        &mut state,
        LaneChangeIntent::None,
        &spec(),
        Side::Left,
        Incentives::default(),
        unsafe_gap,
        DT,
    );
    assert_eq!(decision, Decision::Indicate(Side::Left));
    let no_room = prospect(0.0, false);
    let intent = LaneChangeIntent::Indicating { side: Side::Left, since: 0.0, check: None };
    let mut state = State { desire: [0.5, 0.0], ..State::default() };
    let (decision, _) = resolve(
        &mut state,
        intent,
        &spec(),
        Side::Left,
        Incentives::default(),
        no_room,
        DT,
    );
    assert_eq!(decision, Decision::Nothing);
    let mut state = State { desire: [D_COOP, 0.0], ..State::default() };
    let (decision, _) = resolve(
        &mut state,
        intent,
        &spec(),
        Side::Left,
        Incentives::default(),
        no_room,
        DT,
    );
    assert_eq!(decision, Decision::Change(Side::Left));
}

#[test]
fn desire_shortens_the_headway_and_raises_the_braking_accepted() {
    assert_eq!(headway(0.0), 1.0);
    let relaxed = crate::models::acceleration::RELAXED_HEADWAY;
    assert!((headway(1.0) - relaxed).abs() < 1e-12);
    assert!(accepted_braking(&spec(), 0.5) > accepted_braking(&spec(), 0.4));
    let assertive = VehicleSpec { assertiveness: 1.0, ..spec() };
    assert!(accepted_braking(&assertive, 0.5) > accepted_braking(&spec(), 0.5));
    assert_eq!(accepted_braking(&assertive, 1.0), SAFE_BRAKING);
}

#[test]
fn followers_make_room_only_for_a_strong_desire() {
    let state = State::default();
    let mut ids = slotmap::SlotMap::<VehicleId, ()>::with_key();
    let owner = ids.insert(());
    assert_eq!(make_room(&state, &spec(), Some((owner, 0.5, -0.5))), None);
    assert_eq!(
        make_room(&state, &spec(), Some((owner, 0.9, -0.5))),
        Some(owner)
    );
    // Beyond its courtesy, it drives on.
    assert_eq!(make_room(&state, &spec(), Some((owner, 0.9, -2.0))), None);
}

// What drivers decide in a simulation.

/// What `id` decides in one step of its lane-change model now, with a gap
/// check if it asks for one, its lane-change state afterwards, and how it
/// weighed up each side it may change to. Partway through a change, or
/// straddling, it decides nothing and weighs up nothing.
fn decision(
    sim: &Simulation,
    id: VehicleId,
) -> (Decision, lane_change::State, Vec<LaneChangeEval>) {
    let vehicle = &sim.state().vehicles[id];
    if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
        return (
            Decision::Nothing,
            sim.state().lane_changing.state(id),
            Vec::new(),
        );
    }
    let mut speeds = Vec::new();
    LaneSpeeds::refresh(&mut speeds, sim.state());
    let incentives = incentives(sim.state(), &speeds, id, vehicle);
    let mut lc = sim.state().lane_changing.state(id);
    let dt = sim.state().macro_step_dt();
    lane_change::aim(&mut lc, incentives);
    let ask = lane_change::step(&mut lc, vehicle.intent, &vehicle.spec, sim.time(), dt);
    let mut evals = Vec::new();
    let mut decision = Decision::Nothing;
    for side in [Side::Left, Side::Right] {
        let Some(incentive) = incentives[lane_change::index(side)] else {
            continue;
        };
        let eval = match ask {
            lane_change::Ask::Check(asked) if asked == side => {
                // Measured now, as the core would for the next update.
                let headway = lane_change::headway(lc.desire(side));
                let through = sim.through();
                let prospect =
                    crate::query::measure(sim.state(), through, id, vehicle, side, Some(headway))
                        .and_then(|p| p.gap)
                        .expect("asked only where permitted");
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
                let outcome = match vehicle.intent.indicating() {
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
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING, 0.0]);
    sim.state_mut().set_time(2.0);
    id
}

// Following.

#[test]
fn it_changes_when_the_gap_is_clear_and_it_has_indicated_long_enough() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Change(Side::Left), "{evals:?}");
    // Nobody behind in the target lane.
    assert_eq!(evals[0].new_follower, None);
}

/// A car on lane 1 at `s` and `speed`, indicating left for 2 s with
/// advantage built up for it, behind a car held stopped 20 m ahead.
fn indicating_behind_a_stopped_car_at(sim: &mut Simulation, s: f64, speed: f64) -> VehicleId {
    let blocker = sim.insert(car(), at(1, s + 20.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, s, speed));
    sim.indicate(id, Side::Left);
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING, 0.0]);
    sim.state_mut().set_time(2.0);
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
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING, 0.0]);
    sim.state_mut().set_time(2.0);
    let (decision, evals) = decide_now(&sim, id);
    assert_eq!(decision, Decision::Nothing, "{evals:?}");
    assert!(evals[0].safe && !evals[0].room, "{evals:?}");
    // With room to get halfway, it goes.
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), at(1, 300.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(1, 280.0, 1.0));
    sim.indicate(id, Side::Left);
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING, 0.0]);
    sim.state_mut().set_time(2.0);
    assert_eq!(decide_now(&sim, id).0, Decision::Change(Side::Left));
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
    sim.state_mut().set_time(0.5 * lane_change::MIN_INDICATING);
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
    sim.update_accels(&mut NoopSink);
    assert_eq!(sim.state().vehicles[id].held_up.map(|f| f.id), Some(behind));
    // Courtesy: it wants to let it by.
    let (_, evals) = decide_now(&sim, id);
    let courtesy = evals[0].incentives.courtesy;
    assert!(courtesy > 0.0, "courtesy {courtesy}");
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
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING; 2]);
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
    sim.state_mut().set_time(2.0);
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
    let step = sim.state().macro_step_dt() / lane_change::DESIRE_TIME;
    assert!(
        (left.desire - wanted.min(1.0) * step).abs() < 1e-9,
        "{left:?}"
    );
    // Once its desire has built up, it checks the gap and indicates.
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING, 0.0]);
    assert_eq!(decide_now(&sim, id).0, Decision::Indicate(Side::Left));
}

#[test]
fn it_wont_indicate_into_an_unsafe_gap_short_of_synchronising() {
    // A fast car close behind in lane 0: changing would cut it off.
    let (mut sim, id) = behind_a_stopped_car(20.0);
    sim.insert(car(), at(0, 290.0, 14.0));
    sim.state_mut().lane_changing.state_mut(id).set_desire([WANTING, 0.0]);
    let (action, state, evals) = decision(&sim, id);
    assert_eq!(action, Decision::Nothing);
    assert_eq!(evals[0].outcome, Outcome::Unsafe);
    // It waits a few steps before checking again.
    let dt = sim.state().macro_step_dt();
    assert_eq!(state.cooldown(), lane_change::cooldown(&car(), dt));
    // Wanting it enough to synchronise, it indicates all the same, asking
    // for room.
    sim.state_mut()
        .lane_changing
        .state_mut(id)
        .set_desire([lane_change::D_SYNC + 0.05, 0.0]);
    let (action, evals) = decide_now(&sim, id);
    assert_eq!(evals[0].outcome, Outcome::Request);
    assert_eq!(action, Decision::Indicate(Side::Left));
}

#[test]
fn a_change_starts_its_desire_afresh() {
    let mut sim = two_lanes();
    let id = indicating_behind_a_stopped_car(&mut sim);
    sim.update_lane_changes(sim.state().macro_step_dt(), &mut NoopSink);
    assert_eq!(
        sim.state().vehicles[id].path[0],
        LaneId(0),
        "it should have changed"
    );
    let lc = sim.state().lane_changing.state(id);
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
        sim.state_mut().lane_changing.state_mut(id).set_desire([desire, 0.0]);
        // As a lane-change update measures it, before the longitudinal one.
        sim.measure_prospects();
        sim.update_accels(&mut NoopSink);
        sim.state().vehicles[id].accel
    };
    assert!(accel(WANTING) > 0.0, "{}", accel(WANTING));
    let synchronising = accel(lane_change::D_SYNC + 0.1);
    assert!(synchronising < 0.0, "{synchronising}");
    assert!(synchronising >= -car().comfy_decel, "{synchronising}");
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
    let mut sim = new_sim(network);
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
    sim.state_mut()
        .lane_changing
        .state_mut(indicating)
        .set_desire([lane_change::D_COOP, 0.0]);
    let spec = VehicleSpec { courtesy, ..car() };
    let follower = sim.insert(spec, at(0, 300.0 - behind, 10.0));
    (sim, indicating, follower)
}

#[test]
fn a_courteous_follower_makes_room_for_an_indicating_car() {
    let (mut sim, indicating, follower) = behind_an_indicator(15.0, 2.0);
    sim.update_accels(&mut NoopSink);
    assert_eq!(
        sim.state().lane_changing.state(follower).yielding(),
        Some(indicating)
    );
    assert!(
        sim.state().vehicles[follower].accel < 0.0,
        "it should ease off"
    );
}

#[test]
fn followers_make_room_only_for_a_driver_wanting_it_enough_to_cooperate() {
    let (mut sim, indicating, follower) = behind_an_indicator(15.0, 2.0);
    sim.state_mut().lane_changing.state_mut(indicating).set_desire([WANTING, 0.0]);
    sim.update_accels(&mut NoopSink);
    assert_eq!(sim.state().lane_changing.state(follower).yielding(), None);
}

#[test]
fn a_follower_drives_on_if_making_room_would_take_more_than_its_courtesy() {
    // Making room would take braking harder than 0.2 m/s².
    let (mut sim, _, follower) = behind_an_indicator(15.0, 0.2);
    sim.update_accels(&mut NoopSink);
    assert_eq!(sim.state().lane_changing.state(follower).yielding(), None);
    // One with no courtesy never does, however easy.
    let (mut sim, _, follower) = behind_an_indicator(80.0, 0.0);
    sim.update_accels(&mut NoopSink);
    assert_eq!(sim.state().lane_changing.state(follower).yielding(), None);
}

#[test]
fn a_follower_keeps_making_room_once_it_has_chosen_to() {
    // Having chosen to, it carries on even where it wouldn't start: here its
    // courtesy alone wouldn't stretch to it.
    let (mut sim, indicating, follower) = behind_an_indicator(15.0, 0.2);
    sim.state_mut().lane_changing.state_mut(follower).set_yielding(Some(indicating));
    sim.update_accels(&mut NoopSink);
    assert_eq!(
        sim.state().lane_changing.state(follower).yielding(),
        Some(indicating)
    );
    // Once the indicator is gone, there is nothing to make room for.
    sim.cancel_intent(indicating);
    sim.update_accels(&mut NoopSink);
    assert_eq!(sim.state().lane_changing.state(follower).yielding(), None);
}

// Several leaders.

/// A driver's evaluations as the debug view keeps them, from the events of
/// `steps` debug steps: each decision's reset clears the vehicle's earlier
/// ones.
fn evals_after(
    sim: &mut Simulation,
    id: VehicleId,
    steps: usize,
) -> Vec<Vec<(&'static str, Field)>> {
    let mut kept = Vec::new();
    let mut events = Vec::new();
    for _ in 0..steps {
        events.clear();
        sim.step_debug(&mut events);
        for event in events.drain(..) {
            match event {
                DebugEvent::RecomputeDecision { vehicle } if vehicle == id => kept.clear(),
                DebugEvent::ModelEval { vehicle, fields, .. } if vehicle == id => kept.push(fields),
                _ => {}
            }
        }
    }
    kept
}

#[test]
fn a_driver_with_nothing_left_to_weigh_up_keeps_no_stale_evaluations() {
    // Changes allowed only from left to right.
    let lane =
        |y: f64| Lane::new(&[DVec2::new(0.0, y), DVec2::new(500.0, y)], WIDTH, LIMIT).unwrap();
    let mut network = SimNetwork::new(vec![lane(0.0), lane(-WIDTH)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, false);
    let mut sim = new_sim(network);
    // Held up on the left, it wants the free lane on its right.
    let id = sim.insert(car(), at(0, 100.0, 10.0));
    sim.insert(car(), at(0, 125.0, 0.0));
    let before = evals_after(&mut sim, id, 30);
    let side = |e: &Vec<(&'static str, Field)>| e.iter().find(|f| f.0 == "side").map(|f| f.1);
    assert!(
        before.iter().any(|e| side(e) == Some(Field::Text("right"))),
        "{before:?}"
    );
    // Once over, on the right, it may change nowhere, so has nothing to weigh up.
    let after = evals_after(&mut sim, id, 30 * 20);
    assert_eq!(
        sim.state().vehicles[id].path[0],
        LaneId(1),
        "it should have changed"
    );
    assert!(sim.state().vehicles[id].old_side.is_none());
    assert!(after.is_empty(), "stale evaluations: {after:?}");
}
