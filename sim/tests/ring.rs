//! Vehicles on a ring road: a single lane that is its own successor.

use traffic_sim::{LaneId, NoopSink, Placement, Simulation};

mod common;
use common::{car, fill_ring, ring};

const DT: f64 = 1.0 / 60.0;
const SPEED_LIMIT: f64 = 14.0;

fn run(sim: &mut Simulation, seconds: f64, mut each: impl FnMut(&Simulation)) {
    for _ in 0..(seconds / DT) as usize {
        sim.step(DT);
        each(sim);
    }
}

/// Every vehicle is in its own lane's list, and every list is sorted.
fn check_occupancy(sim: &Simulation) {
    let list = sim.occupants(LaneId(0));
    assert!(
        list.windows(2).all(|w| w[0].s <= w[1].s),
        "occupants out of order"
    );
    for (id, _) in sim.vehicles() {
        assert!(
            list.iter().any(|o| o.id == id),
            "{id:?} missing from its lane"
        );
    }
}

#[test]
fn lone_car_laps_forever_without_a_leader() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    let id = sim.insert(car(), Placement { lane: LaneId(0), s: 0.0, speed: 0.0 });
    let mut max_t = 0.0f64;
    run(&mut sim, 120.0, |sim| {
        let v = sim.get(id).expect("car left the ring");
        assert!(v.leader().is_none(), "found itself as a leader");
        max_t = max_t.max(v.lane_pos().t.abs());
        check_occupancy(sim);
    });
    // At the speed limit for most of two minutes: about seven laps.
    assert!((sim.get(id).unwrap().speed() - SPEED_LIMIT).abs() < 0.1);
    assert!(max_t < 0.1, "drifted {max_t} m off the centreline");
}

#[test]
fn crowded_ring_never_collides() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    fill_ring(&mut sim, 22);
    run(&mut sim, 300.0, |sim| {
        assert_eq!(sim.vehicles().count(), 22);
        for (_, v) in sim.vehicles() {
            let leader = v.leader();
            // No leader only before the first longitudinal update.
            if let Some(leader) = leader {
                assert!(leader.gap > 0.5, "gap closed to {} m", leader.gap);
            }
        }
        check_occupancy(sim);
    });
}

#[test]
fn leaders_form_a_single_cycle() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    fill_ring(&mut sim, 22);
    run(&mut sim, 30.0, |_| {});
    // Following leaders from any car visits every car once and comes back.
    let start = sim.vehicles().next().unwrap().0;
    let mut id = start;
    for _ in 0..22 {
        id = sim.get(id).unwrap().leader().expect("every car has a leader").id;
    }
    assert_eq!(id, start);
}

#[test]
fn held_car_stops_and_its_follower_queues() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    fill_ring(&mut sim, 10);
    run(&mut sim, 30.0, |_| {});

    let held = sim.vehicles().next().unwrap().0;
    sim.hold(held);
    run(&mut sim, 20.0, |_| {});
    assert_eq!(sim.get(held).unwrap().speed(), 0.0);
    let follower = sim.vehicles().find(|(_, v)| v.leader().unwrap().id == held).unwrap().1;
    assert!(follower.speed() < 0.1);
    assert!(follower.leader().unwrap().gap > 1.0);

    sim.release(held);
    run(&mut sim, 5.0, |_| {});
    assert!(sim.get(held).unwrap().speed() > 1.0);
}

#[test]
fn picking_prefers_the_nearest_body() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    fill_ring(&mut sim, 4);
    let (id, v) = sim.vehicles().next().unwrap();
    let centre = v.centre();
    assert_eq!(sim.vehicle_at(centre, 0.0), Some(id));
    // Just off the side of the car: only found with enough tolerance.
    let side = centre + 1.5 * glam::DVec2::from_angle(v.heading()).perp();
    assert_eq!(sim.vehicle_at(side, 0.1), None);
    assert_eq!(sim.vehicle_at(side, 1.0), Some(id));
}
