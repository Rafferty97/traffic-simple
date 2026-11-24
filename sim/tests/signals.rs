//! Signals and the barriers they hold: queues short of a red line, crossing
//! only on green or amber, the amber dilemma, and room beyond the junction.

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierKind, Lane, LaneId, Light, Movement, MovementId, Phase, Placement, Route,
    SignalPlan, SimNetwork, Simulation, VehicleId,
};

mod common;
use common::car;

use common::DT;
const LIMIT: f64 = 14.0;

fn p(x: f64, y: f64) -> DVec2 {
    DVec2::new(x, y)
}

/// The crossroads of `junction.rs`, signalised: lane 0 comes from the west and
/// lane 1 from the south, each to its own line at the junction, 10 m from the
/// origin; connectors 2 and 3 cross it to lanes 4 and 5. Movement 0, west to
/// east, is green first, for 20 s, then movement 1, south to north; each has 4 s
/// of amber, enough for a car that commits as it turns amber, at its
/// comfortable stopping distance, to reach its line, and 2 s of clearance.
fn crossroads() -> (SimNetwork, [MovementId; 2]) {
    let movement = |conflicts| Movement { conflicts, amber: 4.0, clearance: 2.0 };
    crossroads_with(SignalPlan {
        movements: vec![movement(0b10), movement(0b01)],
        phases: vec![
            Phase { green: 0b01, duration: 20.0 },
            Phase { green: 0b10, duration: 20.0 },
        ],
    })
}

/// As [`crossroads`], with `plan` for its signals.
fn crossroads_with(plan: SignalPlan) -> (SimNetwork, [MovementId; 2]) {
    let lane = |points: &[DVec2]| Lane::new(points, 3.5, LIMIT, common::alone()).unwrap();
    let mut net = SimNetwork::new(vec![
        lane(&[p(-100.0, 0.0), p(-10.0, 0.0)]),
        lane(&[p(0.0, -100.0), p(0.0, -10.0)]),
        lane(&[p(-10.0, 0.0), p(10.0, 0.0)]),
        lane(&[p(0.0, -10.0), p(0.0, 10.0)]),
        lane(&[p(10.0, 0.0), p(100.0, 0.0)]),
        lane(&[p(0.0, 10.0), p(0.0, 100.0)]),
    ]);
    for (from, to) in [(0, 2), (1, 3), (2, 4), (3, 5)] {
        net.connect(LaneId(from), LaneId(to));
    }
    net.add_junction(&[LaneId(2), LaneId(3)]);
    let first = net.add_signal(plan);
    let movements = [first, MovementId(first.0 + 1)];
    for (lane, movement) in [(2, movements[0]), (3, movements[1])] {
        let kind = BarrierKind::Signal { movement, on_red: None };
        net.add_barrier(LaneId(lane), Barrier::new(0.0, kind));
    }
    (net, movements)
}

fn at(lane: u32, s: f64, speed: f64) -> Placement {
    Placement {
        lane: LaneId(lane),
        s,
        speed,
        route: Route::random(0),
        seed: 0,
    }
}

/// How far a vehicle's front is along its heading: past the line at the
/// junction once its front is beyond `line` that way.
fn front(sim: &Simulation, id: VehicleId) -> DVec2 {
    let v = sim.get(id).unwrap();
    let spec = v.spec();
    v.pos() + (spec.length - spec.rear_axle) * DVec2::from_angle(v.heading())
}

fn run_until(sim: &mut Simulation, time: f64) {
    while sim.time() < time {
        sim.step();
    }
}

#[test]
fn a_red_light_holds_its_queue_a_line_gap_short() {
    let (net, _) = crossroads();
    let mut sim = common::sim(net);
    // Coming from the south, red for the first 25 s.
    let ids: Vec<_> =
        (0..4).map(|i| sim.insert(car(), at(1, 70.0 - 12.0 * i as f64, 10.0))).collect();
    run_until(&mut sim, 20.0);
    let head = front(&sim, ids[0]).y;
    let line_gap = car().line_gap;
    assert!(
        (head - (-10.0 - line_gap)).abs() < 0.3,
        "the head's front is at y = {head}, its line 10 m south of the origin"
    );
    for &id in &ids {
        assert!(front(&sim, id).y < -10.0, "{id:?} is over the line on red");
        assert!(sim.get(id).unwrap().speed() < 0.1, "{id:?} isn't stopped");
    }
}

#[test]
fn vehicles_cross_only_on_green_or_amber() {
    let (net, movements) = crossroads();
    let mut sim = common::sim(net);
    let mut crossed = Vec::new();
    let mut fronts = std::collections::HashMap::new();
    for step in 0..(120.0 / DT) as u64 {
        if step % (2.5 / DT).round() as u64 == 0 {
            for lane in [0, 1] {
                let _ = sim.try_insert(car(), at(lane, 5.0, LIMIT));
            }
        }
        sim.step();
        // When each vehicle's front crosses its line, its light then.
        let lights = [sim.light(movements[0]).light, sim.light(movements[1]).light];
        let vehicles: Vec<_> = sim.vehicles().map(|(id, _)| id).collect();
        for id in vehicles {
            let f = front(&sim, id);
            let (along, light) = match sim.get(id).unwrap().heading().cos() > 0.5 {
                true => (f.x, lights[0]),
                false => (f.y, lights[1]),
            };
            let before = fronts.insert(id, along);
            if before.is_some_and(|b: f64| b < -10.0) && along >= -10.0 {
                crossed.push((id, light));
            }
        }
    }
    assert!(crossed.len() > 10, "few crossed: {crossed:?}");
    let on_red: Vec<_> = crossed.iter().filter(|(_, light)| *light == Light::Red).collect();
    assert!(on_red.is_empty(), "crossed on red: {on_red:?}");
}

#[test]
fn drivers_of_every_speed_cross_only_on_green_or_amber_with_ites_amber() {
    use traffic_sim::VehicleSpec;
    // As the network crate times amber: a second plus the time to stop from the
    // limit at 3 m/s².
    let (mut net, movements) = crossroads();
    let mut plan = net.signals().next().unwrap().0.clone();
    for m in &mut plan.movements {
        m.amber = 1.0 + LIMIT / 6.0;
    }
    net = {
        let (mut fresh, _) = crossroads_with(plan);
        std::mem::swap(&mut fresh, &mut net);
        net
    };
    let mut sim = common::sim(net);
    let mut fronts = std::collections::HashMap::new();
    let mut on_red = Vec::new();
    for step in 0..(300.0 / DT) as u64 {
        let every = (5.0 / 3.0 / DT).round() as u64;
        if step % every == 0 {
            for lane in [0, 1] {
                let factor = [0.9, 1.0, 1.1, 1.2, 1.3][(step / every) as usize % 5];
                let spec = VehicleSpec { speed_factor: factor, ..VehicleSpec::passenger_car() };
                let _ = sim.try_insert(spec, at(lane, 5.0, LIMIT * factor));
            }
        }
        sim.step();
        let lights = [sim.light(movements[0]).light, sim.light(movements[1]).light];
        let vehicles: Vec<_> = sim.vehicles().map(|(id, _)| id).collect();
        for id in vehicles {
            let f = front(&sim, id);
            let (along, light) = match sim.get(id).unwrap().heading().cos() > 0.5 {
                true => (f.x, lights[0]),
                false => (f.y, lights[1]),
            };
            let before = fronts.insert(id, along);
            if before.is_some_and(|b: f64| b < -10.0) && along >= -10.0 && light == Light::Red {
                on_red.push((id, sim.time(), sim.get(id).unwrap().speed()));
            }
        }
    }
    assert!(on_red.is_empty(), "crossed on red: {on_red:?}");
}

#[test]
fn on_amber_it_goes_if_it_committed_on_green_and_stops_if_it_can() {
    let (net, movements) = crossroads();
    let mut sim = common::sim(net);
    // Movement 0, from the west, turns amber 22 s in.
    run_until(&mut sim, 20.5);
    assert_eq!(sim.light(movements[0]).light, Light::Green);
    // Near: about 16 m short of its line as it turns amber, too close to stop
    // comfortably at 14 m/s, so committed while it was green. Far: about 60 m
    // short, room to stop comfortably.
    let near = sim.insert(car(), at(0, 49.0, LIMIT));
    let far = sim.insert(car(), at(0, 5.0, LIMIT));
    run_until(&mut sim, 22.2);
    assert_eq!(sim.light(movements[0]).light, Light::Amber);
    run_until(&mut sim, 25.0);
    let gone = sim.get(near).is_none_or(|_| front(&sim, near).x > -10.0);
    assert!(gone, "the near one didn't go through");
    run_until(&mut sim, 45.0);
    let stopped = front(&sim, far).x;
    assert!(stopped < -10.0, "the far one went through on amber or red");
    assert!(
        sim.get(far).unwrap().speed() < 0.1,
        "the far one isn't stopped"
    );
}

#[test]
fn on_green_the_whole_queue_goes_each_seeing_the_line_once_its_leader_commits() {
    let (net, movements) = crossroads();
    let mut sim = common::sim(net);
    // From the south: red until 26 s, green from 28 s to 48 s.
    let ids: Vec<_> =
        (0..4).map(|i| sim.insert(car(), at(1, 70.0 - 12.0 * i as f64, 10.0))).collect();
    run_until(&mut sim, 27.0);
    assert_ne!(sim.light(movements[1]).light, Light::Green);
    assert!(
        ids.iter().all(|&id| front(&sim, id).y < -10.0),
        "one went on red"
    );
    run_until(&mut sim, 45.0);
    assert_eq!(sim.light(movements[1]).light, Light::Green);
    let through = |id: VehicleId| sim.get(id).is_none_or(|_| front(&sim, id).y > -10.0);
    assert!(
        ids.iter().all(|&id| through(id)),
        "not all of the queue went on green"
    );
}

#[test]
fn a_green_light_with_no_room_beyond_holds_it_until_there_is() {
    let (net, movements) = crossroads();
    let mut sim = common::sim(net);
    // Held just past the junction, where the next car from the west would go.
    let blocker = sim.insert(car(), at(4, 3.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), at(0, 30.0, 10.0));
    run_until(&mut sim, 15.0);
    assert_eq!(sim.light(movements[0]).light, Light::Green);
    assert!(front(&sim, id).x < -10.0, "it entered with no room beyond");
    assert!(sim.get(id).unwrap().speed() < 0.1, "it isn't waiting");
    sim.release(blocker);
    run_until(&mut sim, 21.0);
    assert!(
        front(&sim, id).x > -10.0,
        "it didn't go once there was room"
    );
}

/// The crossroads without signals: connector 3, from the south, gives way to
/// connector 2, from the west, at a line where they start to conflict.
fn give_way() -> SimNetwork {
    let lane = |points: &[DVec2]| Lane::new(points, 3.5, LIMIT, common::alone()).unwrap();
    let mut net = SimNetwork::new(vec![
        lane(&[p(-100.0, 0.0), p(-10.0, 0.0)]),
        lane(&[p(0.0, -100.0), p(0.0, -10.0)]),
        lane(&[p(-10.0, 0.0), p(10.0, 0.0)]),
        lane(&[p(0.0, -10.0), p(0.0, 10.0)]),
        lane(&[p(10.0, 0.0), p(100.0, 0.0)]),
        lane(&[p(0.0, 10.0), p(0.0, 100.0)]),
    ]);
    for (from, to) in [(0, 2), (1, 3), (2, 4), (3, 5)] {
        net.connect(LaneId(from), LaneId(to));
    }
    net.add_junction(&[LaneId(2), LaneId(3)]);
    let start = net.conflict_between(LaneId(3), LaneId(2)).unwrap().mine.start;
    let yields = Some(net.add_yields(&[LaneId(2)]));
    net.add_barrier(
        LaneId(3),
        Barrier { yields, ..Barrier::new(start, BarrierKind::GiveWay) },
    );
    net
}

#[test]
fn a_give_way_line_waits_for_traffic_with_priority_then_goes() {
    let mut sim = common::sim(give_way());
    // The line, along the connector from the south, which starts 10 m south.
    let line = sim.network().lane(LaneId(3)).barriers()[0].s - 10.0;
    // At rest short of its line, with a car with priority coming 4 s away.
    let yielding = sim.insert(car(), at(3, 0.0, 0.0));
    let priority = sim.insert(car(), at(0, 40.0, LIMIT));
    let mut waited_for_it = true;
    let mut slowest = f64::INFINITY;
    for _ in 0..(20.0 / DT) as usize {
        sim.step();
        if let Some(v) = sim.get(priority) {
            slowest = slowest.min(v.speed());
        }
        // Until the car with priority is across, the yielding one hasn't gone.
        let across = sim.get(priority).is_none_or(|_| front(&sim, priority).x > 3.0);
        if !across && front(&sim, yielding).y > line {
            waited_for_it = false;
        }
    }
    assert!(waited_for_it, "it went ahead of the car with priority");
    // Waiting short of the conflict, it holds back nobody with priority.
    assert!(
        slowest > LIMIT - 0.5,
        "the car with priority slowed to {slowest:.2} m/s"
    );
    let gone = sim.get(yielding).is_none_or(|_| front(&sim, yielding).y > 10.0);
    assert!(gone, "it never went once the way was clear");
}

#[test]
fn a_give_way_line_goes_through_a_wide_gap() {
    let mut sim = common::sim(give_way());
    let yielding = sim.insert(car(), at(3, 0.0, 0.0));
    // Far off: about 85 m away at 14 m/s, 6 s.
    let _far = sim.insert(car(), at(0, 5.0, LIMIT));
    run_until(&mut sim, 4.0);
    assert!(front(&sim, yielding).y > 5.0, "it waited for a car far off");
}

#[test]
fn a_give_way_line_waits_for_traffic_with_priority_at_rest_at_a_green_light() {
    let mut net = give_way();
    // The road with priority has a signal that is always green.
    let first = net.add_signal(SignalPlan {
        movements: vec![Movement { conflicts: 0, amber: 3.0, clearance: 0.0 }],
        phases: vec![Phase { green: 0b1, duration: 1e9 }],
    });
    let kind = BarrierKind::Signal { movement: first, on_red: None };
    net.add_barrier(LaneId(2), Barrier::new(0.0, kind));
    let mut sim = common::sim(net);
    sim.step();
    assert_eq!(sim.light(first).light, Light::Green);
    let line = sim.network().lane(LaneId(3)).barriers()[0].s - 10.0;
    // Both at rest at their lines: the one at the green light is about to go,
    // so the one at the give-way line waits for it.
    let yielding = sim.insert(car(), at(3, 0.0, 0.0));
    let priority = sim.insert(car(), at(0, 84.0, 0.0));
    let mut waited_for_it = true;
    for _ in 0..(20.0 / DT) as usize {
        sim.step();
        let across = sim.get(priority).is_none_or(|_| front(&sim, priority).x > 3.0);
        if !across && front(&sim, yielding).y > line {
            waited_for_it = false;
        }
    }
    assert!(
        waited_for_it,
        "it went ahead of the car about to move off at its green light"
    );
}

/// The crossroads with every movement's amber `amber` seconds long.
fn crossroads_with_amber(amber: f64) -> (SimNetwork, [MovementId; 2]) {
    let (net, _) = crossroads();
    let mut plan = net.signals().next().unwrap().0.clone();
    for m in &mut plan.movements {
        m.amber = amber;
    }
    crossroads_with(plan)
}

#[test]
fn a_committed_car_stopped_short_of_its_line_is_released_on_red_and_waits() {
    let (net, movements) = crossroads();
    let mut sim = common::sim(net);
    // At 14 m/s, 40 m short of its line at 16 s: committed, as it can't stop
    // comfortably, then held to a stop well short of the line.
    run_until(&mut sim, 16.0);
    let id = sim.insert(car(), at(0, 90.0 - 40.0 - 3.6, LIMIT));
    run_until(&mut sim, 16.2);
    assert_eq!(
        sim.get(id).unwrap().committed(),
        1,
        "it should have committed on green"
    );
    sim.hold(id);
    // Red from 26 s; the hold lifted while red.
    run_until(&mut sim, 28.0);
    assert_eq!(sim.light(movements[0]).light, Light::Red);
    sim.release(id);
    run_until(&mut sim, 50.0);
    assert_ne!(sim.light(movements[0]).light, Light::Green);
    assert_eq!(
        sim.get(id).unwrap().committed(),
        0,
        "it wasn't released on red"
    );
    assert!(front(&sim, id).x < -10.0, "it went on red");
    // And it goes on the next green.
    run_until(&mut sim, 62.0);
    assert!(
        sim.get(id).is_none_or(|_| front(&sim, id).x > -10.0),
        "it didn't go on green"
    );
}

#[test]
fn on_red_a_committed_car_goes_on_if_it_cant_stop_safely_and_stops_if_it_can() {
    // A short amber, so committed cars are still short of the line on red.
    let (net, movements) = crossroads_with_amber(1.0);
    let mut sim = common::sim(net);
    // Amber at 22 s, red at 23 s. At 14 m/s, `near` is 20 m short at 22 s and so
    // 6 m short on red, too near to stop at 4 m/s²; `far` is 47 m short, so
    // 33 m short on red, with room. Both committed on green, unable to stop
    // comfortably, and far enough apart that `far` isn't slowed by `near`.
    run_until(&mut sim, 21.0);
    let near = sim.insert(car(), at(0, 70.0 - 14.0 - 3.6, LIMIT));
    let far = sim.insert(car(), at(0, 43.0 - 14.0 - 3.6, LIMIT));
    run_until(&mut sim, 21.9);
    assert_eq!(sim.light(movements[0]).light, Light::Green);
    assert_eq!(sim.get(near).unwrap().committed(), 1);
    assert_eq!(sim.get(far).unwrap().committed(), 1);
    run_until(&mut sim, 23.1);
    assert_eq!(sim.light(movements[0]).light, Light::Red);
    run_until(&mut sim, 40.0);
    assert!(
        sim.get(near).is_none_or(|_| front(&sim, near).x > -10.0),
        "near stopped"
    );
    assert!(front(&sim, far).x < -10.0, "far went on red");
    assert!(sim.get(far).unwrap().speed() < 0.1, "far isn't stopped");
}

#[test]
fn drivers_stop_short_of_a_line_by_a_gap_drawn_for_it() {
    use traffic_sim::VehicleSpec;
    let (net, _) = crossroads();
    let mut sim = common::sim(net);
    // From the south, red for the first 26 s: each car the head of its own run.
    let mut gaps = Vec::new();
    for i in 0..6 {
        // A few meters short of its line, slow, so it settles there in time.
        // Each its own driver.
        let place = Placement { route: Route::random(i), seed: i, ..at(1, 80.0, 2.0) };
        let id = sim.insert(VehicleSpec::passenger_car(), place);
        let drawn = sim.line_gap(id).unwrap();
        let spec = VehicleSpec::passenger_car();
        assert!(
            (drawn - spec.line_gap).abs() <= spec.line_gap_spread,
            "drew {drawn}"
        );
        run_until(&mut sim, 2.0 + 4.0 * (i + 1) as f64);
        let head = front(&sim, id).y;
        assert!(
            (head - (-10.0 - drawn)).abs() < 0.3,
            "stopped {:.2} short, drew {drawn:.2}",
            -10.0 - head
        );
        gaps.push(drawn);
        sim.remove(id);
    }
    // They vary.
    let spread = gaps.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
        - gaps.iter().cloned().fold(f64::INFINITY, f64::min);
    assert!(spread > 0.2, "gaps hardly vary: {gaps:?}");
}

#[test]
fn a_give_way_line_never_lets_a_car_go_behind_one_at_rest_in_its_way() {
    let mut sim = common::sim(give_way());
    let crossing = sim.network().conflict_between(LaneId(2), LaneId(3)).unwrap().mine.clone();
    // A car with priority, stopped in the crossing, and held there: it never
    // clears, so there's no going behind it.
    let stopped = sim.insert(car(), at(2, crossing.start + 2.0, 0.0));
    sim.hold(stopped);
    let line = sim.network().lane(LaneId(3)).barriers()[0].s - 10.0;
    let yielding = sim.insert(car(), at(3, 0.0, 0.0));
    run_until(&mut sim, 20.0);
    assert!(
        front(&sim, yielding).y < line,
        "it went into the car stopped in its way"
    );
    // Once that one goes, so does it.
    sim.release(stopped);
    run_until(&mut sim, 40.0);
    assert!(
        sim.get(yielding).is_none_or(|_| front(&sim, yielding).y > line),
        "it never went"
    );
}

#[test]
fn a_car_stopping_at_a_red_light_comes_to_rest_close_to_its_line_and_stays_there() {
    // A light at 400 m that stays red: its movement is never green.
    let lane = Lane::new(
        &[DVec2::ZERO, DVec2::new(800.0, 0.0)],
        3.5,
        LIMIT,
        common::alone(),
    );
    let mut network = SimNetwork::new(vec![lane.unwrap()]);
    let movement = Movement { conflicts: 0b10, amber: 3.0, clearance: 2.0 };
    let plan = SignalPlan {
        movements: vec![movement, Movement { conflicts: 0b01, ..movement }],
        phases: vec![Phase { green: 0b10, duration: 1.0e6 }],
    };
    let movement = network.add_signal(plan);
    let on_red = None;
    network.add_barrier(
        LaneId(0),
        Barrier::new(400.0, BarrierKind::Signal { movement, on_red }),
    );
    // The shortest gap to lines drawn, where coming to rest short of it would
    // soonest make IDM pull it on: from slow, and from the limit.
    for speed in [2.0, LIMIT] {
        let mut sim = common::sim(network.clone());
        let spec = traffic_sim::VehicleSpec { line_gap: 0.5, line_gap_spread: 0.0, ..car() };
        let s = (400.0 - 4.0 * speed * speed - 30.0).max(0.0);
        let at = Placement { lane: LaneId(0), s, speed, route: Route::default(), seed: 0 };
        let id = sim.insert(spec, at);
        let short = |sim: &Simulation| {
            let v = sim.get(id).unwrap();
            400.0 - (v.lane_pos().s - spec.rear_axle + spec.length) - 0.5
        };
        let mut rested = None;
        for _ in 0..common::steps(60.0) {
            sim.step();
            let v = sim.get(id).unwrap();
            match rested {
                None if v.speed() == 0.0 => rested = Some(v.lane_pos().s),
                // Once at rest, it doesn't creep on.
                Some(s) => assert_eq!(v.lane_pos().s, s, "crept on from {speed} m/s"),
                None => {}
            }
        }
        assert!(rested.is_some(), "never came to rest from {speed} m/s");
        let short = short(&sim);
        assert!(
            (0.0..0.05).contains(&short),
            "{short} m short of its line gap from {speed} m/s"
        );
    }
}
