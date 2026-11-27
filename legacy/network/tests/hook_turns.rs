//! Hook turns, as in Melbourne: each waits in the intersection in front of the
//! road on its near side until that road's traffic goes straight on, then
//! turns ahead of it.

use std::collections::{HashMap, HashSet};

use traffic_network_legacy::{Turn, compile, scenarios};
use traffic_sim::{
    Barrier, BarrierKind, Driver, LaneId, Light, MovementId, Placement, Route, Simulation,
    VehicleId, VehicleSpec,
};

mod common;

const SPEED: f64 = 40.0 / 3.6;

/// Each hook turn's connector, and how far along it it waits; the lane out;
/// the near-side road's connector going straight on into it; and the movement
/// whose light it waits for.
struct Hook {
    connector: LaneId,
    wait: f64,
    exit: LaneId,
    near: LaneId,
    movement: MovementId,
}

fn hooks(compiled: &traffic_network_legacy::Compiled) -> Vec<Hook> {
    let (firsts, turns, sim) = (&compiled.connectors[0], &compiled.turns[0], &compiled.sim);
    let next = |lane: LaneId| sim.lane(lane).successors()[0];
    firsts
        .iter()
        .zip(turns)
        .filter(|&(_, &turn)| turn == Turn::Right)
        .map(|(&connector, _)| {
            let exit = next(connector);
            let mut straight = firsts.iter().zip(turns).filter(|&(_, &t)| t == Turn::Straight);
            let near = straight.find(|&(&f, _)| next(f) == exit).map(|(&f, _)| f).unwrap();
            let waits = sim.lane(connector).barriers().iter().find(|b| b.s > 0.0);
            let Some(&Barrier { s: wait, kind: BarrierKind::Signal { movement, .. }, .. }) = waits
            else {
                panic!("a hook turn waits for a light");
            };
            Hook { connector, wait, exit, near, movement }
        })
        .collect()
}

#[test]
fn a_hook_turn_waits_inside_for_the_light_of_the_near_side_road_going_straight_on() {
    let compiled = compile(&scenarios::hook_turn_crossroads(SPEED)).unwrap();
    let hooks = hooks(&compiled);
    assert_eq!(hooks.len(), 4);
    let sim = &compiled.sim;
    for hook in &hooks {
        // At its stop line, and again where it waits along its connector.
        assert!(sim.lane(hook.exit).junction().is_none());
        assert_eq!(sim.lane(hook.connector).barriers().len(), 2);
        // Waiting for the light the near-side road's straight traffic goes on.
        let near = sim.lane(hook.near).barriers()[0];
        assert_eq!(
            near.kind,
            BarrierKind::Signal { movement: hook.movement, on_red: None }
        );
        // It crosses no oncoming traffic, so gives way to none on its way in.
        assert!(
            sim.lane(hook.connector)
                .barriers()
                .iter()
                .all(|b| matches!(b.kind, BarrierKind::Signal { .. }))
        );
    }
}

#[test]
fn a_hook_turn_crossroads_flooded_with_traffic_has_no_collisions() {
    common::flood(&scenarios::hook_turn_crossroads(SPEED), 300.0);
}

/// Cars at `flow` an hour into each entry lane, choosing their way at random,
/// for `seconds`: how many hook turns there were, how many times the near-side
/// road's light went green with a hook turner waiting at its line, and how
/// many of those times one of that road's cars reached the lane out before it.
/// A turner queued behind the one at the line may fairly come after them.
/// How far back from its waiting line a hook turner's rear axle is at most,
/// waiting at it, in meters: a car, less its rear overhang, and the most it
/// stops short of the line.
const AT_LINE: f64 = 4.6 - 1.0 + 1.5 + 0.5;

fn busy(flow: f64, seconds: f64) -> (usize, usize, usize) {
    let compiled = compile(&scenarios::hook_turn_crossroads(SPEED)).unwrap();
    let hooks = hooks(&compiled);
    let mut sim = Simulation::with_config(compiled.sim.clone(), common::CONFIG);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .filter(|&id| compiled.sim.lane(id).allows().contains(traffic_sim::VehicleClass::Car))
        .collect();
    let mut state = 1u64;
    let mut uniform = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut credit = 0.0;
    let mut lights: Vec<Light> = hooks.iter().map(|h| sim.light(h.movement).light).collect();
    let mut waiting: Vec<HashSet<VehicleId>> = vec![HashSet::new(); hooks.len()];
    let mut lanes: HashMap<VehicleId, LaneId> = HashMap::new();
    let (mut turned, mut greens, mut beaten) = (0, 0, 0);
    for _ in 0..common::steps(seconds) {
        credit += flow / 3600.0 * entries.len() as f64 * common::DT;
        if credit >= 1.0 {
            let lane = entries[(uniform() * entries.len() as f64) as usize];
            let seed = (uniform() * 1e9) as u64;
            let at = Placement { lane, s: 5.0, speed: SPEED, route: Route::random(seed) };
            let driver = Driver { seed, ..Driver::default() };
            if sim.try_insert(VehicleSpec::passenger_car(), driver, at).is_ok() {
                credit -= 1.0;
            }
        }
        sim.step();
        for (h, hook) in hooks.iter().enumerate() {
            let light = sim.light(hook.movement).light;
            if light == Light::Green && lights[h] != Light::Green {
                // At its line: its rear axle within a car and the most it
                // stops short of its line.
                let at_line = |(_, v): &(VehicleId, &traffic_sim::Vehicle)| {
                    let to_line = hook.wait - v.lane_pos().s;
                    let near = (0.0..AT_LINE).contains(&to_line);
                    v.lane() == hook.connector && v.speed() < 0.5 && near
                };
                waiting[h] = sim.vehicles().filter(at_line).map(|(id, _)| id).collect();
                greens += usize::from(!waiting[h].is_empty());
            }
            lights[h] = light;
        }
        let now: HashMap<VehicleId, LaneId> =
            sim.vehicles().map(|(id, v)| (id, v.lane())).collect();
        for (&id, &lane) in &now {
            let before = lanes.get(&id).copied();
            if before == Some(lane) {
                continue;
            }
            for (h, hook) in hooks.iter().enumerate() {
                turned += usize::from(lane == hook.exit && before == Some(hook.connector));
                if lane == hook.exit {
                    waiting[h].remove(&id);
                    if before == Some(hook.near) && !waiting[h].is_empty() {
                        beaten += 1;
                        waiting[h].clear();
                    }
                }
            }
        }
        lanes = now;
    }
    (turned, greens, beaten)
}

#[test]
fn hook_turners_waiting_go_before_the_near_side_roads_traffic() {
    // Nothing in the barriers or the junction treats hook turns specially: they
    // go first because they are nearer, committing to the junction first, and
    // the near-side road's cars give way to vehicles already inside.
    let (turned, greens, beaten) = busy(120.0, 900.0);
    assert!(
        turned > 25 && greens > 10,
        "{turned} hook turns, {greens} greens with turners waiting"
    );
    assert_eq!(
        beaten, 0,
        "{beaten} of {greens} greens, a near-side car went first"
    );
}
