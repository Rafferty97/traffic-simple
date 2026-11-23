//! Vehicles on a ring road: a single lane that is its own successor.

use traffic_sim::{LaneId, Placement, Route, Simulation};

mod common;
use common::{Terms, car, fill_lane, fill_ring, ring, ring_road};

type Sim = Simulation<Terms>;

use common::DT;
const SPEED_LIMIT: f64 = 14.0;

fn run(sim: &mut Sim, seconds: f64, mut each: impl FnMut(&Sim)) {
    for _ in 0..(seconds / DT) as usize {
        sim.step_variable(DT);
        each(sim);
    }
}

/// Every vehicle is in its own lane's list, and every list is sorted.
fn check_occupancy(sim: &Sim) {
    let list = sim.occupants(LaneId(0));
    assert!(
        list.windows(2).all(|w| w[0].s() <= w[1].s()),
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
    let mut sim = common::sim(ring(230.0, SPEED_LIMIT), Terms::default());
    let id = sim.insert(
        car(),
        Placement { lane: LaneId(0), s: 0.0, speed: 0.0, route: Route::default() },
    );
    let mut max_t = 0.0f64;
    run(&mut sim, 120.0, |sim| {
        let v = sim.get(id).expect("car left the ring");
        assert!(sim.sink().leader(id).is_none(), "found itself as a leader");
        max_t = max_t.max(v.lane_pos().t.abs());
        check_occupancy(sim);
    });
    // At the speed limit for most of two minutes: about seven laps.
    assert!((sim.get(id).unwrap().speed() - SPEED_LIMIT).abs() < 0.1);
    // Its rear axle trails inside the bend, as a car's rear wheels do: on a
    // circle of radius r, at √(r² − L²) for wheelbase L.
    let r = 230.0 / std::f64::consts::TAU;
    let inside = r - (r * r - car().wheelbase().powi(2)).sqrt();
    // Its front axle rounds each corner of the lane's polyline, inside it by up
    // to the polyline's sagitta, a centimetre.
    assert!(
        (max_t - inside).abs() < 0.02,
        "drifted {max_t} m off the centerline, not {inside}"
    );
}

#[test]
fn crowded_ring_never_collides() {
    let mut sim = common::sim(ring(230.0, SPEED_LIMIT), Terms::default());
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
    let mut sim = common::sim(ring(230.0, SPEED_LIMIT), Terms::default());
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
    let mut sim = common::sim(ring(230.0, SPEED_LIMIT), Terms::default());
    fill_ring(&mut sim, 10);
    run(&mut sim, 30.0, |_| {});

    let held = sim.vehicles().next().unwrap().0;
    sim.hold(held);
    run(&mut sim, 20.0, |_| {});
    assert_eq!(sim.get(held).unwrap().speed(), 0.0);
    let follower = sim
        .vehicles()
        .map(|(id, _)| id)
        .find(|&id| sim.sink().leader(id).is_some_and(|(leader, _)| leader == held));
    let follower = follower.unwrap();
    assert!(sim.get(follower).unwrap().speed() < 0.1);
    assert!(sim.sink().leader(follower).unwrap().1 > 1.0);

    sim.release(held);
    run(&mut sim, 5.0, |_| {});
    assert!(sim.get(held).unwrap().speed() > 1.0);
}

#[test]
fn picking_prefers_the_nearest_body() {
    let mut sim = common::sim(ring(230.0, SPEED_LIMIT), Terms::default());
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
    let mut sim = common::sim(ring(230.0, SPEED_LIMIT), Terms::default());
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
fn three_lane_ring_occupancy_stays_consistent() {
    let mut sim = common::sim(ring_road(230.0, 3, SPEED_LIMIT), Terms::default());
    // Densest inside and sparsest outside, so drivers have reason to move out.
    // Different counts also keep the lanes' cars from starting side by side.
    for (lane, count) in [(0, 22), (1, 16), (2, 8)] {
        fill_lane(&mut sim, LaneId(lane), count);
    }
    let mut changes = 0;
    let mut lanes: Vec<_> = sim.vehicles().map(|(id, v)| (id, v.lane())).collect();
    let mut frame = 0;
    run(&mut sim, 120.0, |sim| {
        frame += 1;
        for (id, lane) in &mut lanes {
            let now = sim.get(*id).unwrap().lane();
            if now != *lane {
                changes += 1;
                *lane = now;
            }
        }
        for lane_id in (0..3).map(LaneId) {
            let list = sim.occupants(lane_id);
            assert!(
                list.windows(2).all(|w| w[0].s() <= w[1].s()),
                "{lane_id:?} out of order"
            );
            for o in list {
                let v = sim.get(o.id).unwrap();
                let beside = |side| sim.network().lane(v.lane()).adj(side).map(|n| n.lane);
                if o.is_ghost() {
                    // A ghost belongs to a vehicle indicating into this lane.
                    let side = v.intent().expect("ghost without an intent").side;
                    assert_eq!(beside(side), Some(lane_id), "stray ghost on {lane_id:?}");
                } else {
                    // A record belongs to a vehicle following this lane, or
                    // straddling out of it after a lane change.
                    let straddling = v.old_side().and_then(beside) == Some(lane_id);
                    assert!(
                        v.lane() == lane_id || straddling,
                        "foreign record on {lane_id:?}"
                    );
                }
            }
        }
        for (id, v) in sim.vehicles() {
            let own = sim.occupants(v.lane());
            assert!(
                own.iter().any(|o| o.is_record_of(id)),
                "{id:?} missing from its lane"
            );
        }
        // Bodies never overlap. Checked every tenth frame, as it's quadratic.
        if frame % 10 == 0 {
            let all: Vec<_> = sim.vehicles().collect();
            for (i, &(a, va)) in all.iter().enumerate() {
                for &(b, vb) in &all[i + 1..] {
                    if va.center().distance(vb.center()) < 6.0 {
                        let touching = vb.corners().iter().any(|&c| va.distance_to(c) == 0.0)
                            || va.corners().iter().any(|&c| vb.distance_to(c) == 0.0);
                        assert!(!touching, "{a:?} and {b:?} collided");
                    }
                }
            }
        }
    });
    // How many changed lanes depends on how the traffic evolves, so it isn't
    // asserted: the checks above hold whether or not anyone does.
    eprintln!("{changes} lane changes");
}
