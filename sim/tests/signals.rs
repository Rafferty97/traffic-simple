//! Signals and the barriers they hold: queues short of a red line, crossing
//! only on green or amber, the amber dilemma, and room beyond the junction.

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierKind, Lane, LaneId, Light, Movement, MovementId, NoopSink, Phase, Placement,
    Route, SignalPlan, SimNetwork, Simulation, VehicleId,
};

mod common;
use common::car;

const DT: f64 = 1.0 / 60.0;
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
    let lane = |points: &[DVec2]| Lane::new(points, 3.5, LIMIT).unwrap();
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
        let kind = BarrierKind::Signal { movement };
        net.add_barrier(LaneId(lane), Barrier { s: 0.0, kind });
    }
    (net, movements)
}

fn at(lane: u32, s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(lane), s, speed, route: Route::random(0) }
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
    let mut sim = Simulation::new(net, NoopSink);
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
    let mut sim = Simulation::new(net, NoopSink);
    let mut crossed = Vec::new();
    let mut fronts = std::collections::HashMap::new();
    for step in 0..(120.0 / DT) as u64 {
        if step % 150 == 0 {
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
    let mut sim = Simulation::new(net, NoopSink);
    let mut fronts = std::collections::HashMap::new();
    let mut on_red = Vec::new();
    for step in 0..(300.0 / DT) as u64 {
        if step % 100 == 0 {
            for lane in [0, 1] {
                let factor = [0.9, 1.0, 1.1, 1.2, 1.3][(step / 100) as usize % 5];
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
    let mut sim = Simulation::new(net, NoopSink);
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
    let mut sim = Simulation::new(net, NoopSink);
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
    let mut sim = Simulation::new(net, NoopSink);
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
    let lane = |points: &[DVec2]| Lane::new(points, 3.5, LIMIT).unwrap();
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
    net.set_priority(LaneId(2), 3);
    net.set_priority(LaneId(3), 2);
    let start = net.conflict_between(LaneId(3), LaneId(2)).unwrap().mine.start;
    net.add_barrier(LaneId(3), Barrier { s: start, kind: BarrierKind::GiveWay });
    net
}

#[test]
fn a_give_way_line_waits_for_traffic_with_priority_then_goes() {
    let mut sim = Simulation::new(give_way(), NoopSink);
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
    let mut sim = Simulation::new(give_way(), NoopSink);
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
    let kind = BarrierKind::Signal { movement: first };
    net.add_barrier(LaneId(2), Barrier { s: 0.0, kind });
    let mut sim = Simulation::new(net, NoopSink);
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
