//! Vehicles on a ring road: a single lane that is its own successor.

use traffic_sim::{LaneId, Placement, Simulation};

mod common;
use common::{Terms, car, fill_lane, fill_ring, ring, ring_road};

type Sim = Simulation<Terms>;

const DT: f64 = 1.0 / 60.0;
const SPEED_LIMIT: f64 = 14.0;

fn run(sim: &mut Sim, seconds: f64, mut each: impl FnMut(&Sim)) {
    for _ in 0..(seconds / DT) as usize {
        sim.step(DT);
        each(sim);
    }
}

/// Every vehicle is in its own lane's list, and every list is sorted.
fn check_occupancy(sim: &Sim) {
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
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), Terms::default());
    let id = sim.insert(car(), Placement { lane: LaneId(0), s: 0.0, speed: 0.0 });
    let mut max_t = 0.0f64;
    run(&mut sim, 120.0, |sim| {
        let v = sim.get(id).expect("car left the ring");
        assert!(sim.sink().leader(id).is_none(), "found itself as a leader");
        max_t = max_t.max(v.lane_pos().t.abs());
        check_occupancy(sim);
    });
    // At the speed limit for most of two minutes: about seven laps.
    assert!((sim.get(id).unwrap().speed() - SPEED_LIMIT).abs() < 0.1);
    assert!(max_t < 0.1, "drifted {max_t} m off the centerline");
}

#[test]
fn crowded_ring_never_collides() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), Terms::default());
    fill_ring(&mut sim, 22);
    run(&mut sim, 300.0, |sim| {
        assert_eq!(sim.vehicles().count(), 22);
        for (id, _) in sim.vehicles() {
            // No leader only before the first longitudinal update.
            if let Some((_, gap)) = sim.sink().leader(id) {
                assert!(gap > 0.5, "gap closed to {gap} m");
            }
        }
        check_occupancy(sim);
    });
}

#[test]
fn leaders_form_a_single_cycle() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), Terms::default());
    fill_ring(&mut sim, 22);
    run(&mut sim, 30.0, |_| {});
    // Following leaders from any car visits every car once and comes back.
    let start = sim.vehicles().next().unwrap().0;
    let mut id = start;
    for _ in 0..22 {
        id = sim.sink().leader(id).expect("every car has a leader").0;
    }
    assert_eq!(id, start);
}

#[test]
fn held_car_stops_and_its_follower_queues() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), Terms::default());
    fill_ring(&mut sim, 10);
    run(&mut sim, 30.0, |_| {});

    let held = sim.vehicles().next().unwrap().0;
    sim.hold(held);
    run(&mut sim, 20.0, |_| {});
    assert_eq!(sim.get(held).unwrap().speed(), 0.0);
    let follower = sim
        .vehicles()
        .map(|(id, _)| id)
        .find(|&id| sim.sink().leader(id).unwrap().0 == held);
    let follower = follower.unwrap();
    assert!(sim.get(follower).unwrap().speed() < 0.1);
    assert!(sim.sink().leader(follower).unwrap().1 > 1.0);

    sim.release(held);
    run(&mut sim, 5.0, |_| {});
    assert!(sim.get(held).unwrap().speed() > 1.0);
}

#[test]
fn picking_prefers_the_nearest_body() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), Terms::default());
    fill_ring(&mut sim, 4);
    let (id, v) = sim.vehicles().next().unwrap();
    let center = v.center();
    assert_eq!(sim.vehicle_at(center, 0.0), Some(id));
    // Just off the side of the car: only found with enough tolerance.
    let side = center + 1.5 * glam::DVec2::from_angle(v.heading()).perp();
    assert_eq!(sim.vehicle_at(side, 0.1), None);
    assert_eq!(sim.vehicle_at(side, 1.0), Some(id));
}

#[test]
fn recorded_terms_reach_each_acceleration() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), Terms::default());
    fill_ring(&mut sim, 22);
    let held = sim.vehicles().next().unwrap().0;
    sim.hold(held);
    run(&mut sim, 30.0, |sim| {
        for (id, v) in sim.vehicles() {
            let terms = sim.sink().0.iter().filter(|t| t.0 == id).map(|t| t.2);
            let min = terms.fold(f64::INFINITY, f64::min);
            // Before the first longitudinal update there are no terms yet.
            if min.is_finite() {
                assert_eq!(min, v.accel(), "terms don't reach {id:?}'s acceleration");
            }
        }
    });
}

#[test]
fn three_lane_ring_keeps_to_its_lanes() {
    let mut sim = Simulation::new(ring_road(230.0, 3, SPEED_LIMIT), Terms::default());
    for lane in 0..3 {
        fill_lane(&mut sim, LaneId(lane), 22);
    }
    let start: Vec<_> = sim.vehicles().map(|(id, v)| (id, v.lane())).collect();
    run(&mut sim, 120.0, |sim| {
        for &(id, lane) in &start {
            let v = sim.get(id).unwrap();
            assert_eq!(v.lane(), lane, "{id:?} changed lane");
            assert!(
                v.lane_pos().t.abs() < 0.1,
                "{id:?} drifted {} m",
                v.lane_pos().t
            );
        }
        for lane in 0..3 {
            let list = sim.occupants(LaneId(lane));
            assert!(
                list.windows(2).all(|w| w[0].s <= w[1].s),
                "lane {lane} out of order"
            );
            // Only this lane's own cars, though a car crossing the seam briefly has two records.
            for o in list {
                assert_eq!(
                    sim.get(o.id).unwrap().lane(),
                    LaneId(lane),
                    "foreign record on lane {lane}"
                );
            }
        }
        for &(id, lane) in &start {
            let list = sim.occupants(lane);
            assert!(
                list.iter().any(|o| o.id == id),
                "{id:?} missing from its lane"
            );
        }
    });
}
