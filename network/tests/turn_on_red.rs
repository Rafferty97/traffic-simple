//! Turning on red: where the network allows it, a near-side turn at signals is
//! a stop line on red, giving way to every lane it crosses or merges with.

use traffic_network::{Compiled, RoadId, Turn, compile, scenarios};
use traffic_sim::{
    BarrierKind, LaneId, Light, MovementId, NoopSink, Placement, Route, Simulation, VehicleId,
    VehicleSpec,
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
                Placement { lane, s, speed, route: Route::random(seed) },
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
    let mut sim = Simulation::new(compiled.sim, NoopSink);
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
    let mut sim = Simulation::new(compiled.sim, NoopSink);
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
