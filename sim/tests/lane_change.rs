//! Lane changes on a three-lane ring, commanded directly with `change_lane`.

use traffic_sim::{
    Lane, LaneChangeError, LaneId, Lateral, NoopSink, Placement, SAFE_BRAKING, Side, SimNetwork,
    Simulation, VehicleId,
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
    let length = sim.network().lane(LaneId(1)).geometry().length();
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

/// For a car changing left out of `lane`, the `t` on `lane` of its right rear
/// corner, the one still furthest into it, while it still has a record there.
fn trailing_corner(sim: &Simulation, id: VehicleId, lane: LaneId) -> Option<f64> {
    let record = sim.occupants(lane).iter().find(|o| o.id == id && !o.ghost)?;
    assert_eq!(
        record.lateral,
        Lateral::Diverging(Side::Left),
        "it should be diverging left"
    );
    let l = sim.network().lane(lane);
    let [_, rear_right, ..] = sim.get(id)?.corners();
    Some(
        l.geometry()
            .world_to_lane_near(rear_right, l.geometry().lane_pos(record.s, 0.0))
            .t,
    )
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

    // A held car with another stopped 2 m behind it: the leader's rear is 1 m
    // behind its rear axle, and the follower's front 3.6 m ahead of its own.
    let leader = sim.insert(car(), Placement { lane: LaneId(1), s: 60.0, speed: 0.0 });
    sim.hold(leader);
    let id = sim.insert(car(), Placement { lane: LaneId(1), s: 53.4, speed: 0.0 });

    // Too close to complete the change: it pulls forward but stops short. Changed
    // at once, before the car's own decisions get a say: an empty lane beside it
    // would otherwise tempt it over earlier.
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

#[test]
fn followers_pass_a_car_mostly_out_of_their_lane() {
    // Wide lanes, so a straddling car's rear can leave room to pass while it is
    // still a member of the old lane.
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(400.0, y)];
        Lane::new(&points, 5.0, SPEED_LIMIT).unwrap()
    };
    // Lane 0 is on the left of lane 1.
    let mut network = SimNetwork::new(vec![lane(5.0), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    let mut sim = Simulation::new(network, NoopSink);
    // A slow driver, so it stops soon after being held, still straddling.
    let slow = traffic_sim::VehicleSpec { speed_factor: 0.15, ..car() };
    let id = sim.insert(slow, Placement { lane: LaneId(1), s: 60.0, speed: 2.0 });
    let follower = sim.insert(car(), Placement { lane: LaneId(1), s: 10.0, speed: 6.0 });
    sim.change_lane(id, Side::Left).unwrap();

    // Hold it once its rear is well over, but before it has left the old lane.
    let rear_on_old = |sim: &Simulation| trailing_corner(sim, id, LaneId(1));
    let mut passed_while_straddling = false;
    let mut closest = f64::INFINITY;
    for _ in 0..(30.0 / DT) as usize {
        sim.step(DT);
        if !sim.is_held(id) && rear_on_old(&sim).is_some_and(|trailing| trailing >= 1.8) {
            sim.hold(id);
        }
        let (v, f) = (sim.get(id).unwrap(), sim.get(follower).unwrap());
        closest = closest.min(clearance(v, f));
        if f.lane_pos().s > v.lane_pos().s + 5.0 && rear_on_old(&sim).is_some() {
            passed_while_straddling = true;
        }
    }
    assert!(sim.is_held(id), "never got far enough over to hold");
    assert!(closest > 0.3, "came within {closest} m");
    assert!(
        passed_while_straddling,
        "the follower never passed the straddling car"
    );
}

#[test]
fn followers_pass_where_they_will_be_not_where_they_are() {
    // Three wide lanes, 0 on the left of 1 on the left of 2.
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(600.0, y)];
        Lane::new(&points, 5.0, SPEED_LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![lane(10.0), lane(5.0), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    // Only into lane 1 from lane 2, so the fast car can't escape back out before
    // it reaches the held car: this is about how it brakes for it.
    network.set_adjacent(LaneId(1), LaneId(2), false, true);
    let mut sim = Simulation::new(network, NoopSink);

    // A slow car changes left out of lane 1 and is held while its trailing (right
    // rear) corner is 1.2 m left of lane 1's centerline: too little room for a
    // car centered in lane 1 to pass, but plenty for one still in lane 2.
    let slow = traffic_sim::VehicleSpec { speed_factor: 0.15, ..car() };
    let ahead = sim.insert(slow, Placement { lane: LaneId(1), s: 300.0, speed: 2.0 });
    sim.change_lane(ahead, Side::Left).unwrap();
    let trailing = |sim: &Simulation| trailing_corner(sim, ahead, LaneId(1));
    // Held at 0.6 m, it drifts to about 1.1 m while braking to a stop.
    while !trailing(&sim).is_some_and(|t| t >= 0.6) {
        sim.step(DT);
        assert!(sim.time() < 30.0, "never got far enough over");
    }
    sim.hold(ahead);
    run(&mut sim, 5.0, |_| {});
    let settled = trailing(&sim).expect("it should still be on lane 1");
    assert!(
        settled + 0.5 > 0.95,
        "a centered car could pass: {settled} m"
    );

    // A fast car changes from lane 2 into lane 1 well behind it. It is still
    // mostly in lane 2 while far back, but will be centered by the time it
    // arrives, so it must treat the held car as a leader from the start.
    let s = sim.get(ahead).unwrap().lane_pos().s - 70.0;
    let fast = sim.insert(car(), Placement { lane: LaneId(2), s, speed: SPEED_LIMIT });
    sim.change_lane(fast, Side::Left).unwrap();
    let (mut closest, mut slowest, mut drew_level) = (f64::INFINITY, f64::INFINITY, false);
    let mut hardest = 0.0f64;
    run(&mut sim, 20.0, |sim| {
        let (a, f) = (sim.get(ahead).unwrap(), sim.get(fast).unwrap());
        closest = closest.min(clearance(a, f));
        if f.lane() == LaneId(1) {
            slowest = slowest.min(f.speed());
            hardest = hardest.min(f.accel());
            // Its front past the held car's rear.
            let front = f.lane_pos().s + f.spec().length - f.spec().rear_axle;
            drew_level |= front > a.lane_pos().s - a.spec().rear_axle;
        }
    });
    assert!(!drew_level, "it drew level with the held car in lane 1");
    assert!(slowest < 0.5, "it should have stopped behind the held car");
    // Seen late, from where it is rather than where it will be, it would brake
    // at about 9 m/s².
    assert!(hardest > -SAFE_BRAKING, "braked at {hardest} m/s²");
    assert!(closest > 0.3, "came within {closest} m");
}

/// A car held in lane 1, and a car behind it that changes into lane 0, leaving
/// `gap` meters between its front and the held car's rear and moving at `speed`.
/// Returns how far the changer got past the held car's rear, its closest
/// approach, and whether it was still straddling at the end.
fn change_away_from_a_held_car(gap: f64, speed: f64) -> (f64, f64, bool) {
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(400.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    // Lane 0 is on the left of lane 1.
    let mut network = SimNetwork::new(vec![lane(3.5), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    let mut sim = Simulation::new(network, NoopSink);
    let held = sim.insert(car(), Placement { lane: LaneId(1), s: 100.0, speed: 0.0 });
    sim.hold(held);
    // Rear axles: the changer's front is `gap` behind the held car's rear.
    let spec = car();
    let s = 100.0 - spec.rear_axle - gap - (spec.length - spec.rear_axle);
    let changer = sim.insert(car(), Placement { lane: LaneId(1), s, speed });
    sim.change_lane(changer, Side::Left).unwrap();
    let mut closest = f64::INFINITY;
    run(&mut sim, 20.0, |sim| {
        closest = closest.min(clearance(sim.get(held).unwrap(), sim.get(changer).unwrap()));
    });
    let (h, c) = (sim.get(held).unwrap(), sim.get(changer).unwrap());
    let past = (c.lane_pos().s - c.spec().rear_axle) - (h.lane_pos().s - h.spec().rear_axle);
    (past, closest, c.old_side().is_some())
}

#[test]
fn a_straddling_car_drives_past_its_old_leader_once_it_will_be_clear() {
    // Its course takes it clear of the held car's corner before it gets there.
    // Before, a car settled in the old lane always blocked it, so it stopped
    // behind, straddling, for good.
    let (past, closest, _) = change_away_from_a_held_car(12.0, 3.0);
    assert!(past > 10.0, "it should have driven past, got {past:.1} m");
    assert!(closest > 0.3, "came within {closest} m");
}

#[test]
fn a_straddling_car_waits_behind_its_old_leader_when_it_wont_be_clear() {
    // Stopped close behind, its course can't take it clear in time.
    let (past, closest, straddling) = change_away_from_a_held_car(2.0, 0.0);
    assert!(
        past < 0.0,
        "it should have waited behind, got {past:.1} m past"
    );
    assert!(straddling, "it should still be straddling");
    assert!(closest > 0.3, "came within {closest} m");
}

#[test]
fn a_car_changing_in_behind_a_slower_one_follows_it() {
    // Two straight lanes, 0 on the left of 1.
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(600.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![lane(3.5), lane(0.0)]);
    // Changes are only permitted rightward, so the slower car stays in lane 1
    // rather than moving over to let the faster one by.
    network.set_adjacent(LaneId(0), LaneId(1), true, false);
    let mut sim = Simulation::new(network, NoopSink);
    // A slower car in lane 1, and a faster one in lane 0 that changes in about a
    // metre behind it. Still a whole lane to the side as it starts, it would
    // stay alongside the slower car while moving across, so it must follow it.
    let slow = traffic_sim::VehicleSpec { speed_factor: 0.8, ..car() };
    let ahead = sim.insert(slow, Placement { lane: LaneId(1), s: 100.0, speed: 11.2 });
    let spec = car();
    let s = 100.0 - spec.length - 1.0;
    let fast = sim.insert(spec, Placement { lane: LaneId(0), s, speed: 14.0 });
    sim.change_lane(fast, Side::Right).unwrap();
    let (mut closest, mut drew_level) = (f64::INFINITY, false);
    run(&mut sim, 15.0, |sim| {
        let (a, f) = (sim.get(ahead).unwrap(), sim.get(fast).unwrap());
        closest = closest.min(clearance(a, f));
        let front = f.lane_pos().s + f.spec().length - f.spec().rear_axle;
        drew_level |= f.lane() == LaneId(1) && front > a.lane_pos().s - a.spec().rear_axle;
    });
    assert!(closest > 0.3, "came within {closest} m");
    assert!(
        !drew_level,
        "it drew level with the car it changed in behind"
    );
}

/// Three straight lanes, 0 on the left of 1 on the left of 2, with a slow car on
/// lane 1 that changes left and is held partway out, its right rear corner about
/// 1.1 m left of lane 1's centerline. Returns the simulation and that car.
fn held_partway_out_of_lane_1() -> (Simulation, VehicleId) {
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(600.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![lane(7.0), lane(3.5), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    network.set_adjacent(LaneId(1), LaneId(2), true, true);
    let mut sim = Simulation::new(network, NoopSink);
    let slow = traffic_sim::VehicleSpec { speed_factor: 0.15, ..car() };
    let ahead = sim.insert(slow, Placement { lane: LaneId(1), s: 300.0, speed: 2.0 });
    sim.change_lane(ahead, Side::Left).unwrap();
    while !trailing_corner(&sim, ahead, LaneId(1)).is_some_and(|t| t >= 0.6) {
        sim.step(DT);
    }
    sim.hold(ahead);
    run(&mut sim, 5.0, |_| {});
    (sim, ahead)
}

/// Inserts a car on lane 1, 12 m behind `ahead`, moving at 3 m/s, changes it to
/// `side`, and runs. Returns how far its rear got past `ahead`'s and its
/// closest approach.
fn change_behind(mut sim: Simulation, ahead: VehicleId, side: Side) -> (f64, f64) {
    let spec = car();
    let rear = sim.get(ahead).unwrap().lane_pos().s - spec.rear_axle;
    let s = rear - 12.0 - (spec.length - spec.rear_axle);
    let follower = sim.insert(car(), Placement { lane: LaneId(1), s, speed: 3.0 });
    sim.change_lane(follower, side).unwrap();
    let mut closest = f64::INFINITY;
    run(&mut sim, 20.0, |sim| {
        closest = closest.min(clearance(
            sim.get(ahead).unwrap(),
            sim.get(follower).unwrap(),
        ));
    });
    let (a, f) = (sim.get(ahead).unwrap(), sim.get(follower).unwrap());
    let past = (f.lane_pos().s - f.spec().rear_axle) - (a.lane_pos().s - a.spec().rear_axle);
    (past, closest)
}

#[test]
fn changes_in_opposite_directions_pass_each_other() {
    // The follower changes right, away from the car leaving lane 1 to the left,
    // so it may drive past once its course takes it clear.
    let (sim, ahead) = held_partway_out_of_lane_1();
    let (past, closest) = change_behind(sim, ahead, Side::Right);
    assert!(past > 10.0, "it should have driven past, got {past:.1} m");
    assert!(closest > 0.3, "came within {closest} m");
}

#[test]
fn changes_in_the_same_direction_never_pass() {
    // The follower changes left too, following the held car out of lane 1: it
    // waits behind it.
    let (sim, ahead) = held_partway_out_of_lane_1();
    let (past, closest) = change_behind(sim, ahead, Side::Left);
    assert!(
        past < 0.0,
        "it should have waited behind, got {past:.1} m past"
    );
    assert!(closest > 0.3, "came within {closest} m");
}
