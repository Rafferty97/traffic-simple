//! Lane changes on a three-lane ring, commanded directly with `change_lane`.

use traffic_sim::{
    Lane, LaneChangeError, LaneId, NoopSink, Placement, Side, SimNetwork, Simulation, VehicleId,
};

mod common;
use common::{car, ring_road};

const DT: f64 = 1.0 / 60.0;
const SPEED_LIMIT: f64 = 14.0;

fn run(sim: &mut Simulation, seconds: f64, mut each: impl FnMut(&Simulation)) {
    for _ in 0..(seconds / DT) as usize {
        sim.step(DT);
        each(sim);
    }
}

/// Lanes holding a record for `id`.
fn lanes_of(sim: &Simulation, id: VehicleId) -> Vec<u32> {
    sim.network()
        .lanes()
        .filter(|&(lane, _)| sim.occupants(lane).iter().any(|o| o.id == id))
        .map(|(lane, _)| lane.0)
        .collect()
}

/// A lone car on the middle lane of a three-lane ring, up to speed.
fn cruising() -> (Simulation, VehicleId) {
    let mut sim = Simulation::new(ring_road(230.0, 3, SPEED_LIMIT), NoopSink);
    let id = sim.insert(
        car(),
        Placement { lane: LaneId(1), s: 0.0, speed: SPEED_LIMIT },
    );
    run(&mut sim, 5.0, |_| {});
    (sim, id)
}

#[test]
fn changes_lane_smoothly_and_leaves_the_old_lane() {
    let (mut sim, id) = cruising();
    sim.change_lane(id, Side::Right).unwrap();

    let v = sim.get(id).unwrap();
    assert_eq!(v.lane(), LaneId(2));
    assert_eq!(v.old_side(), Some(Side::Left));
    // Jumped to the new lane without moving: about one lane width to its left.
    assert!(
        (v.lane_pos().t - 3.5).abs() < 0.1,
        "offset {}",
        v.lane_pos().t
    );
    assert_eq!(lanes_of(&sim, id), [1, 2]);

    // Front-wheel steering angle, from the path curvature and the wheelbase.
    let steer = |v: &traffic_sim::Vehicle| (v.curvature() * v.spec().wheelbase()).atan();
    let (mut last_steer, mut left_old_lane) = (steer(v), None);
    let mut max_steer_rate = 0.0f64;
    let mut time = 0.0;
    run(&mut sim, 10.0, |sim| {
        time += DT;
        let v = sim.get(id).unwrap();
        max_steer_rate = max_steer_rate.max((steer(v) - last_steer).abs() / DT);
        last_steer = steer(v);
        if left_old_lane.is_none() && lanes_of(sim, id) == [2] {
            left_old_lane = Some(time);
            assert_eq!(
                v.old_side(),
                None,
                "old side kept after leaving the old lane"
            );
        }
    });

    let v = sim.get(id).unwrap();
    assert!(v.lane_change().is_none(), "lane change never finished");
    assert!(
        v.lane_pos().t.abs() < 0.05,
        "ended {} m off the centerline",
        v.lane_pos().t
    );
    // The rear leaves the old lane partway through the four-second change.
    let left = left_old_lane.expect("never left the old lane");
    assert!(left > 0.5 && left < 4.0, "left the old lane after {left} s");
    // Smooth from the first frame: the cubic starts where pure pursuit was aiming.
    assert!(
        max_steer_rate < 0.15,
        "steering rate reached {max_steer_rate} rad/s"
    );
}

#[test]
fn refuses_impossible_changes() {
    let (mut sim, id) = cruising();
    sim.change_lane(id, Side::Left).unwrap();
    assert_eq!(
        sim.change_lane(id, Side::Left),
        Err(LaneChangeError::InProgress)
    );
    run(&mut sim, 10.0, |_| {});
    // Now on the innermost lane.
    assert_eq!(
        sim.change_lane(id, Side::Left),
        Err(LaneChangeError::NoNeighbor)
    );
    sim.remove(id);
    assert_eq!(
        sim.change_lane(id, Side::Right),
        Err(LaneChangeError::NoVehicle)
    );
}

#[test]
fn respects_one_way_permissions() {
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(500.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    // Lane 0 is on the left of lane 1; changing right is allowed, left isn't.
    let mut network = SimNetwork::new(vec![lane(3.5), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, false);
    let mut sim = Simulation::new(network, NoopSink);
    let a = sim.insert(car(), Placement { lane: LaneId(0), s: 10.0, speed: 10.0 });
    let b = sim.insert(car(), Placement { lane: LaneId(1), s: 30.0, speed: 10.0 });
    assert_eq!(
        sim.change_lane(b, Side::Left),
        Err(LaneChangeError::NotPermitted)
    );
    assert_eq!(sim.change_lane(a, Side::Right), Ok(()));
}

#[test]
fn a_change_across_the_seam_keeps_followers_on_the_old_side_informed() {
    // Start the change just before the seam, so the car advances while straddling.
    let mut sim = Simulation::new(ring_road(230.0, 3, SPEED_LIMIT), NoopSink);
    let length = sim.network().lane(LaneId(1)).length();
    let id = sim.insert(
        car(),
        Placement { lane: LaneId(1), s: length - 3.0, speed: SPEED_LIMIT },
    );
    sim.change_lane(id, Side::Right).unwrap();
    let mut on_old_side_after_seam = false;
    run(&mut sim, 1.0, |sim| {
        let v = sim.get(id).unwrap();
        if v.lane_pos().s < 10.0 && lanes_of(sim, id).contains(&1) {
            on_old_side_after_seam = true;
        }
    });
    assert!(
        on_old_side_after_seam,
        "dropped off the old side at the seam while straddling"
    );
    run(&mut sim, 10.0, |_| {});
    assert_eq!(lanes_of(&sim, id), [2]);
    assert_eq!(sim.get(id).unwrap().old_side(), None);
}

/// The smallest distance from any corner of either body to the other body.
fn clearance(a: &traffic_sim::Vehicle, b: &traffic_sim::Vehicle) -> f64 {
    let a_to_b = a.corners().map(|c| b.distance_to(c));
    let b_to_a = b.corners().map(|c| a.distance_to(c));
    a_to_b.into_iter().chain(b_to_a).fold(f64::INFINITY, f64::min)
}

#[test]
fn changing_out_of_a_queue_keeps_clear_of_the_old_leader() {
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(300.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    // Lane 0 is on the left of lane 1.
    let mut network = SimNetwork::new(vec![lane(3.5), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    let mut sim = Simulation::new(network, NoopSink);

    // A held car with another queued up behind it.
    let leader = sim.insert(car(), Placement { lane: LaneId(1), s: 60.0, speed: 0.0 });
    sim.hold(leader);
    let id = sim.insert(car(), Placement { lane: LaneId(1), s: 20.0, speed: 0.0 });
    run(&mut sim, 30.0, |_| {});
    assert!(sim.get(id).unwrap().speed() < 0.01, "never queued");

    // Too close to complete the change: it pulls forward but stops short.
    sim.change_lane(id, Side::Left).unwrap();
    let mut closest = f64::INFINITY;
    run(&mut sim, 20.0, |sim| {
        closest = closest.min(clearance(sim.get(id).unwrap(), sim.get(leader).unwrap()));
    });
    assert!(closest > 0.5, "came within {closest} m of the old leader");
    let v = sim.get(id).unwrap();
    assert_eq!(
        v.old_side(),
        Some(Side::Right),
        "should still be straddling"
    );
    assert_eq!(lanes_of(&sim, id), [0, 1]);

    // Once the old leader moves off, the change completes.
    sim.release(leader);
    run(&mut sim, 20.0, |_| {});
    let v = sim.get(id).unwrap();
    assert!(v.lane_change().is_none(), "lane change never finished");
    assert_eq!(lanes_of(&sim, id), [0]);
}
