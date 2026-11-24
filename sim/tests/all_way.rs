//! All-way stops: each vehicle comes to rest at its line, and they go in the
//! order they came to rest, those whose paths don't cross together, each as
//! soon as it can go without stopping for one that went before it.

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierKind, Driver, Lane, LaneId, Placement, Route, SimNetwork, Simulation, VehicleId,
};

mod common;
use common::{car, overlap};

const LIMIT: f64 = 14.0;

fn p(x: f64, y: f64) -> DVec2 {
    DVec2::new(x, y)
}

/// The ways in, by their approach lanes: from the west, east and south (and
/// from the north, lane 3).
const WEST: u32 = 0;
const EAST: u32 = 1;
const SOUTH: u32 = 2;

/// Four ways in to an all-way stop, each 90 m to a line 10 m from the centre,
/// straight across a 20 m connector, and out. Opposite ways run side by side,
/// 3.5 m apart, so they never conflict; each crosses both of the others.
fn all_way_stop() -> SimNetwork {
    let mut net = four_ways();
    // Each takes turns with every other: only those it crosses matter.
    for c in (4..8).map(LaneId) {
        let others: Vec<LaneId> = (4..8).map(LaneId).filter(|&o| o != c).collect();
        let ties = Some(net.add_yields(&others));
        net.add_barrier(c, Barrier { ties, ..Barrier::new(0.0, BarrierKind::Stop) });
    }
    net
}

/// The four ways in, with no barriers.
fn four_ways() -> SimNetwork {
    let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], 3.5, LIMIT, common::alone()).unwrap();
    let h = 1.75;
    // (approach start, line, far side, exit end) for west, east, south, north.
    let ways = [
        (p(-100.0, -h), p(-10.0, -h), p(10.0, -h), p(100.0, -h)),
        (p(100.0, h), p(10.0, h), p(-10.0, h), p(-100.0, h)),
        (p(h, -100.0), p(h, -10.0), p(h, 10.0), p(h, 100.0)),
        (p(-h, 100.0), p(-h, 10.0), p(-h, -10.0), p(-h, -100.0)),
    ];
    let mut lanes: Vec<Lane> = ways.iter().map(|w| lane(w.0, w.1)).collect();
    lanes.extend(ways.iter().map(|w| lane(w.1, w.2)));
    lanes.extend(ways.iter().map(|w| lane(w.2, w.3)));
    let mut net = SimNetwork::new(lanes);
    for i in 0..4 {
        net.connect(LaneId(i), LaneId(4 + i));
        net.connect(LaneId(4 + i), LaneId(8 + i));
    }
    let connectors: Vec<LaneId> = (4..8).map(LaneId).collect();
    net.add_junction(&connectors);
    net
}

/// A car at rest where it stops for its line, coming from `way`.
fn arrive(sim: &mut Simulation, way: u32) -> VehicleId {
    let spec = car();
    let s = 90.0 - (spec.length - spec.rear_axle) - common::driver().line_gap;
    let at = Placement {
        lane: LaneId(way),
        s,
        speed: 0.0,
        route: Route::random(way.into()),
    };
    sim.insert(
        spec,
        Driver { seed: u64::from(way), ..common::driver() },
        at,
    )
}

/// How far a car's front is past its line, along its way in.
fn past_line(sim: &Simulation, id: VehicleId) -> Option<f64> {
    let v = sim.get(id)?;
    let lane = sim.network().lane(v.lane());
    let before = if v.lane().0 < 4 {
        lane.geometry().length()
    } else {
        0.0
    };
    let front = v.lane_pos().s - v.spec().rear_axle + v.spec().length;
    Some(front - before)
}

/// Whether a car's rear is still short of the far side of the junction.
fn in_junction(sim: &Simulation, id: VehicleId) -> bool {
    sim.get(id).is_some_and(|v| v.lane().0 < 8)
}

/// Runs until `seconds`, calling `each` after every step, and asserting that no
/// two bodies ever overlap.
fn run(sim: &mut Simulation, seconds: f64, mut each: impl FnMut(&Simulation)) {
    while sim.time() < seconds {
        sim.step();
        let bodies: Vec<_> = sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
        for (i, &(a, x)) in bodies.iter().enumerate() {
            for &(b, y) in &bodies[i + 1..] {
                assert!(
                    !overlap(x, y),
                    "{a:?} and {b:?} overlap at {:.2} s",
                    sim.time()
                );
            }
        }
        each(sim);
    }
}

/// When each car's front first crosses its line, if it has.
struct Departures(Vec<(VehicleId, Option<f64>)>);

impl Departures {
    fn new(ids: &[VehicleId]) -> Self {
        Self(ids.iter().map(|&id| (id, None)).collect())
    }

    fn note(&mut self, sim: &Simulation) {
        for (id, when) in &mut self.0 {
            if when.is_none() && past_line(sim, *id).is_none_or(|d| d > 0.0) {
                *when = Some(sim.time());
            }
        }
    }

    fn of(&self, id: VehicleId) -> f64 {
        let when = self.0.iter().find(|(i, _)| *i == id).and_then(|(_, w)| *w);
        when.unwrap_or_else(|| panic!("{id:?} never crossed its line"))
    }
}

#[test]
fn two_cars_going_opposite_ways_go_together() {
    let mut sim = common::sim(all_way_stop());
    let a = arrive(&mut sim, WEST);
    let b = arrive(&mut sim, EAST);
    let mut departures = Departures::new(&[a, b]);
    run(&mut sim, 20.0, |sim| departures.note(sim));
    let (ta, tb) = (departures.of(a), departures.of(b));
    assert!(ta < 3.0 && tb < 3.0, "they waited: {ta:.2} s and {tb:.2} s");
    assert!(
        (ta - tb).abs() < 0.5,
        "one waited for the other: {ta:.2} s and {tb:.2} s"
    );
}

#[test]
fn the_second_of_two_crossing_waits_then_goes_behind_the_first_without_stopping() {
    let mut sim = common::sim(all_way_stop());
    let first = arrive(&mut sim, WEST);
    while sim.time() < 0.5 {
        sim.step();
    }
    let second = arrive(&mut sim, SOUTH);
    let mut departures = Departures::new(&[first, second]);
    // Once the second has moved off its line, how much it slows.
    let mut moving_off = false;
    let mut top = 0.0f64;
    let mut slowed = 0.0f64;
    let mut first_in_junction_as_second_went = None;
    // Where their ways cross, along the first's connector.
    let crossing = sim.network().conflict_between(LaneId(4), LaneId(6)).unwrap().mine.clone();
    let mut first_rear_as_second_went = None;
    run(&mut sim, 30.0, |sim| {
        departures.note(sim);
        let Some(v) = sim.get(second) else { return };
        if !moving_off && v.speed() > 0.5 {
            moving_off = true;
            first_in_junction_as_second_went = Some(in_junction(sim, first));
            first_rear_as_second_went = sim.get(first).map(|f| match f.lane() {
                LaneId(4) => f.lane_pos().s - f.spec().rear_axle,
                LaneId(0) => f64::NEG_INFINITY,
                _ => f64::INFINITY,
            });
        }
        if moving_off && in_junction(sim, second) {
            top = top.max(v.speed());
            slowed = slowed.max(top - v.speed());
        }
        // Until the first has crossed the line, the second stays behind its own.
        if past_line(sim, first).is_some_and(|d| d < 0.0) {
            assert!(
                past_line(sim, second).unwrap() < 0.0,
                "the second went first"
            );
        }
    });
    let (t1, t2) = (departures.of(first), departures.of(second));
    assert!(
        t1 < t2,
        "the second, at {t2:.2} s, went before the first, at {t1:.2} s"
    );
    assert_eq!(
        first_in_junction_as_second_went,
        Some(true),
        "the second waited for the first to leave the junction altogether"
    );
    // It went behind the first: moving off before the first had cleared their
    // crossing, timed to reach it just after.
    let rear = first_rear_as_second_went.expect("the first left before the second went");
    assert!(
        rear < crossing.end,
        "the second waited until the first had cleared the crossing"
    );
    assert!(
        slowed < 0.2,
        "the second slowed by {slowed:.2} m/s for the first once it went"
    );
}

#[test]
fn a_car_waits_its_turn_behind_an_earlier_one_it_would_hold_up() {
    let mut sim = common::sim(all_way_stop());
    // A, then B, crossing A's way, then C, opposite A, so not crossing it, but
    // crossing B's: all at rest at their lines before B can go.
    let a = arrive(&mut sim, WEST);
    while sim.time() < 0.3 {
        sim.step();
    }
    let b = arrive(&mut sim, SOUTH);
    while sim.time() < 0.6 {
        sim.step();
    }
    let c = arrive(&mut sim, EAST);
    let mut departures = Departures::new(&[a, b, c]);
    run(&mut sim, 40.0, |sim| departures.note(sim));
    let (ta, tb, tc) = (departures.of(a), departures.of(b), departures.of(c));
    assert!(ta < tb, "B, at {tb:.2} s, went before A, at {ta:.2} s");
    // C could have gone with A, but B came first, and C would hold B up.
    assert!(
        tb < tc,
        "C, at {tc:.2} s, went before B, at {tb:.2} s, which came first"
    );
}
