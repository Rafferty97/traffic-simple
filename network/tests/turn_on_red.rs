//! Turning on red: where the network allows it, a near-side turn at signals is
//! a stop line on red, giving way to every lane it crosses or merges with.

use traffic_network::{Compiled, RoadId, Turn, compile, scenarios};
use traffic_sim::{
    BarrierKind, LaneId, Light, MovementId, Placement, Route, Simulation, VehicleId, VehicleSpec,
};

mod common;

const LIMIT: f64 = 50.0 / 3.6;

/// The connector from `road` going `turn`, and its movement.
fn connector(compiled: &Compiled, road: usize, turn: Turn) -> (LaneId, MovementId) {
    let net = &compiled.sim;
    let c = compiled.connectors[0]
        .iter()
        .zip(&compiled.turns[0])
        .map(|(&c, &t)| (c, t))
        .find(|&(c, t)| {
            let from = net.lane(c).predecessors()[0];
            t == turn && compiled.road_lanes[road].contains(&from)
        })
        .unwrap()
        .0;
    let BarrierKind::Signal { movement, .. } = net.lane(c).barriers()[0].kind else {
        panic!("{c:?} has no signal");
    };
    (c, movement)
}

/// A car on the lane into `connector`, at `s`, at rest if that is where it
/// stops for its line, otherwise at the limit, on a route through it.
fn insert_going(sim: &mut Simulation, connector: LaneId, s: Option<f64>) -> VehicleId {
    let spec = VehicleSpec::passenger_car();
    let lane = sim.network().lane(connector).predecessors()[0];
    let length = sim.network().lane(lane).geometry().length();
    let (s, speed) = match s {
        Some(s) => (s, LIMIT),
        None => (length - (spec.length - spec.rear_axle) - spec.line_gap, 0.0),
    };
    (0..64)
        .find_map(|seed| {
            let id = sim.insert(
                spec,
                Placement { lane, s, speed, route: Route::random(seed), seed },
            );
            if sim.get(id).unwrap().path().contains(&connector) {
                return Some(id);
            }
            sim.remove(id);
            None
        })
        .expect("a route through the connector")
}

#[test]
fn near_side_turns_may_turn_on_red_only_where_the_network_allows_it() {
    for allowed in [false, true] {
        let net = match allowed {
            true => scenarios::signalised_crossroads_turn_on_red(2, LIMIT),
            false => scenarios::signalised_crossroads(2, LIMIT),
        };
        let compiled = compile(&net).unwrap();
        for (&c, &turn) in compiled.connectors[0].iter().zip(&compiled.turns[0]) {
            let BarrierKind::Signal { on_red, .. } = compiled.sim.lane(c).barriers()[0].kind else {
                panic!("{c:?} has no signal");
            };
            // Driving on the left, the near side is the left.
            let may = allowed && turn == Turn::Left;
            assert_eq!(on_red.is_some(), may, "{c:?}, turning {turn:?}");
        }
    }
}

#[test]
fn an_intersection_may_allow_a_movement_to_turn_on_red() {
    let mut net = scenarios::signalised_crossroads(1, LIMIT);
    net.intersections[0].turn_on_red.push((RoadId(1), Turn::Right));
    let compiled = compile(&net).unwrap();
    for (c, (&id, &turn)) in compiled.connectors[0].iter().zip(&compiled.turns[0]).enumerate() {
        let from = net.intersections[0].connections[c].0.road;
        let BarrierKind::Signal { on_red, .. } = compiled.sim.lane(id).barriers()[0].kind else {
            panic!("{id:?} has no signal");
        };
        assert_eq!(on_red.is_some(), from == RoadId(1) && turn == Turn::Right);
    }
}

/// From the north (road 1), red for the first 30 s: a car at its line turning
/// near-side goes on red, once at rest, and one going straight on waits.
#[test]
fn on_red_a_near_side_turner_stops_then_goes_and_straight_traffic_waits() {
    let compiled = compile(&scenarios::signalised_crossroads_turn_on_red(1, LIMIT)).unwrap();
    let (left, movement) = connector(&compiled, 1, Turn::Left);
    let (straight, _) = connector(&compiled, 3, Turn::Straight);
    let mut sim = common::sim(compiled.sim);
    sim.step();
    assert_eq!(sim.light(movement).light, Light::Red);
    // Coming up to its line at the limit, so it must stop there first.
    let turner = insert_going(&mut sim, left, Some(200.0));
    let waiting = insert_going(&mut sim, straight, None);
    let mut stopped = false;
    let mut turned = None;
    while sim.time() < 28.0 {
        sim.step();
        if let Some(v) = sim.get(turner) {
            stopped |= v.speed() == 0.0 && v.path()[1] == left;
            if turned.is_none() && v.path()[0] == left {
                assert!(stopped, "it turned on red without stopping");
                turned = Some(sim.time());
            }
        }
        let w = sim.get(waiting).unwrap();
        assert_ne!(w.path()[0], straight, "the straight car went on red");
    }
    assert!(turned.is_some(), "it never turned on red");
}

/// A near-side turner at rest at its red line gives way to the traffic on its
/// green that it would merge with, going straight on from its right.
#[test]
fn on_red_a_near_side_turner_gives_way_to_the_traffic_it_merges_with() {
    let compiled = compile(&scenarios::signalised_crossroads_turn_on_red(1, LIMIT)).unwrap();
    // From the north turning left, east; from the west (road 2) straight on,
    // east, on its green.
    let (left, _) = connector(&compiled, 1, Turn::Left);
    let (through, green) = connector(&compiled, 2, Turn::Straight);
    let mut sim = common::sim(compiled.sim);
    while sim.light(green).light != Light::Green {
        sim.step();
    }
    let turner = insert_going(&mut sim, left, None);
    // About 4 s from the junction.
    let lane = sim.network().lane(sim.network().lane(through).predecessors()[0]);
    let s = lane.geometry().length() - 4.0 * LIMIT;
    let priority = insert_going(&mut sim, through, Some(s));
    let mut order = Vec::new();
    while sim.time() < 20.0 {
        sim.step();
        for id in [turner, priority] {
            let entered = sim.get(id).is_some_and(|v| [left, through].contains(&v.path()[0]));
            if entered && !order.contains(&id) {
                order.push(id);
            }
        }
    }
    assert_eq!(order, [priority, turner], "the turner went first");
}

#[test]
fn a_crossroads_turning_on_red_flooded_with_traffic_has_no_collisions() {
    for lanes in 1..=2 {
        common::flood(
            &scenarios::signalised_crossroads_turn_on_red(lanes, LIMIT),
            150.0,
        );
    }
}

/// Busy, traffic keeps flowing, and near-side turners do turn on red, through
/// gaps in the traffic on its green.
#[test]
fn a_busy_crossroads_turning_on_red_keeps_flowing_and_turners_go_on_red() {
    let compiled = compile(&scenarios::signalised_crossroads_turn_on_red(2, LIMIT)).unwrap();
    // Each near-side connector, with its movement.
    let near: Vec<(LaneId, MovementId)> = compiled.connectors[0]
        .iter()
        .zip(&compiled.turns[0])
        .filter(|&(_, &t)| t == Turn::Left)
        .map(|(&c, _)| match compiled.sim.lane(c).barriers()[0].kind {
            BarrierKind::Signal { movement, .. } => (c, movement),
            kind => panic!("{c:?} starts with a {kind:?}"),
        })
        .collect();
    let mut sim = common::sim(compiled.sim);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut seen: Vec<VehicleId> = Vec::new();
    let (mut on_red, mut spawned, mut left_before) = (0, 0, 0);
    for step in 1..=common::steps(300.0) {
        if step % common::steps(0.75) == 0 {
            let lane = entries[(step / common::steps(0.75) * 7) as usize % entries.len()];
            let at = Placement {
                lane,
                s: 5.0,
                speed: LIMIT,
                route: Route::random(step),
                seed: step,
            };
            if sim.try_insert(VehicleSpec::passenger_car(), at).is_ok() {
                spawned += 1;
            }
        }
        sim.step();
        for &(c, movement) in &near {
            for o in sim.occupants(c) {
                if !seen.contains(&o.id) && !o.is_ghost() {
                    seen.push(o.id);
                    on_red += (sim.light(movement).light == Light::Red) as usize;
                }
            }
        }
        if step % common::steps(60.0) == 0 && step > common::steps(60.0) {
            let left = spawned - sim.vehicles().count();
            assert!(
                left > left_before + 5,
                "standstill in the minute to {:.0} s",
                sim.time()
            );
            left_before = left;
        }
    }
    assert!(on_red > 5, "only {on_red} of {} turned on red", seen.len());
}
