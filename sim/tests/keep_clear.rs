//! Keep-clear zones: a vehicle doesn't enter one unless there is room for it
//! beyond the far end, so a queue backing past it leaves it empty.

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierKind, Lane, LaneId, Placement, Route, SimNetwork, Simulation, VehicleId,
};

mod common;
use common::car;

const LIMIT: f64 = 14.0;
/// Where the zone starts along the lane, and how long it is, in meters.
const START: f64 = 100.0;
const LENGTH: f64 = 20.0;

/// One straight lane, 300 m long, with a keep-clear zone 100 to 120 m along.
fn road() -> SimNetwork {
    let lane = Lane::new(
        &[DVec2::ZERO, DVec2::new(300.0, 0.0)],
        3.5,
        LIMIT,
        common::alone(),
    )
    .unwrap();
    let mut net = SimNetwork::new(vec![lane]);
    let kind = BarrierKind::KeepClear { length: LENGTH };
    net.add_barrier(LaneId(0), Barrier::new(START, kind));
    net
}

fn at(s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(0), s, speed, route: Route::random(0) }
}

fn rear(sim: &Simulation, id: VehicleId) -> f64 {
    let v = sim.get(id).unwrap();
    v.lane_pos().s - v.spec().rear_axle
}

fn front(sim: &Simulation, id: VehicleId) -> f64 {
    rear(sim, id) + sim.get(id).unwrap().spec().length
}

/// A car held at rest with its rear `beyond` past the zone's far end, and cars
/// coming up behind it from `from`, every 2 s at the limit.
fn queue(beyond: f64, cars: usize, seconds: f64) -> (Simulation, Vec<VehicleId>) {
    let mut sim = common::sim(road());
    let spec = car();
    let held = sim.insert(
        spec,
        common::driver(),
        at(START + LENGTH + beyond + spec.rear_axle, 0.0),
    );
    sim.hold(held);
    let mut ids = vec![held];
    while sim.time() < seconds {
        if ids.len() <= cars && sim.time() >= 2.0 * (ids.len() - 1) as f64 {
            ids.push(sim.insert(spec, common::driver(), at(0.0, LIMIT)));
        }
        sim.step();
    }
    (sim, ids)
}

#[test]
fn a_queue_backing_past_a_keep_clear_zone_leaves_it_empty() {
    // Room for nobody beyond it: the first car behind stops short of it.
    let (sim, ids) = queue(1.0, 3, 60.0);
    for &id in &ids[1..] {
        let (r, f) = (rear(&sim, id), front(&sim, id));
        assert!(
            f < START || r > START + LENGTH,
            "{id:?} stopped in the zone, from {r:.2} to {f:.2}"
        );
        assert!(sim.get(id).unwrap().speed() < 0.01);
    }
    assert!(
        front(&sim, ids[1]) < START,
        "the first car went into the zone"
    );
}

#[test]
fn with_room_for_one_beyond_one_goes_through_and_the_next_waits() {
    let spec = car();
    // Room for one car and a little more.
    let (sim, ids) = queue(spec.length + spec.min_gap + 1.0, 3, 60.0);
    assert!(
        rear(&sim, ids[1]) > START + LENGTH,
        "the first car behind didn't go through: its rear at {:.2}",
        rear(&sim, ids[1])
    );
    for &id in &ids[2..] {
        assert!(front(&sim, id) < START, "{id:?} went into the zone");
    }
}

#[test]
fn a_car_stopped_short_of_the_zone_goes_once_the_queue_beyond_moves_on() {
    let mut sim = common::sim(road());
    let spec = car();
    let held = sim.insert(
        spec,
        common::driver(),
        at(START + LENGTH + 1.0 + spec.rear_axle, 0.0),
    );
    sim.hold(held);
    let waiting = sim.insert(spec, common::driver(), at(40.0, LIMIT));
    while sim.time() < 20.0 {
        sim.step();
    }
    assert!(front(&sim, waiting) < START);
    sim.release(held);
    while sim.time() < 40.0 {
        sim.step();
    }
    assert!(
        rear(&sim, waiting) > START + LENGTH,
        "it never went through the zone"
    );
}

#[test]
fn a_keep_clear_zone_doesnt_slow_traffic_flowing_freely_through_it() {
    let mut sim = common::sim(road());
    let mut slowest = f64::INFINITY;
    let mut inserted = 0;
    while sim.time() < 40.0 {
        // A car every 2 s at the limit: flowing freely, each well over its
        // time headway behind the one before.
        if sim.time() >= 2.0 * inserted as f64 && inserted < 15 {
            sim.insert(car(), common::driver(), at(0.0, LIMIT));
            inserted += 1;
        }
        sim.step();
        for (_, v) in sim.vehicles() {
            if v.lane_pos().s > 10.0 && v.lane_pos().s < 280.0 {
                slowest = slowest.min(v.speed());
            }
        }
    }
    assert!(slowest > LIMIT - 0.3, "a car slowed to {slowest:.2} m/s");
}
