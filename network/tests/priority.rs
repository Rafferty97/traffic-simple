//! Intersections controlled by priority: minor roads give way, or stop, at
//! their lines to the traffic with priority, and far-side turns from the major
//! road wait inside for their oncoming traffic.

use std::collections::HashMap;

use traffic_network::{Compiled, RoadNetwork, Sign, Turn, compile, scenarios};
use traffic_sim::{
    AccelSource, BarrierKind, DebugEvent, Driver, LaneId, Placement, Route, SAFE_BRAKING,
    Simulation, VehicleId, VehicleSpec,
};

mod common;

const LIMIT: f64 = 50.0 / 3.6;

/// Every priority scenario, named.
fn scenarios() -> Vec<(String, RoadNetwork)> {
    let mut nets = Vec::new();
    // Lanes each way on the major road and the minor one.
    for sign in [Sign::GiveWay, Sign::Stop] {
        for lanes in [[1, 1], [2, 1], [2, 2]] {
            let [major, minor] = lanes;
            nets.push((
                format!("crossroads, {major} by {minor} lanes, {sign:?}"),
                scenarios::priority_crossroads(lanes, LIMIT, sign),
            ));
            nets.push((
                format!("T, {major} by {minor} lanes, {sign:?}"),
                scenarios::priority_t_junction(lanes, LIMIT, sign),
            ));
        }
    }
    nets
}

/// Whether connection `c` comes from a road with priority: the first road's,
/// numbered 0 and 2 at a crossroads and 0 and 1 at a T.
fn major(compiled: &Compiled, net: &RoadNetwork, c: usize) -> bool {
    let road = net.intersections[0].connections[c].0.road;
    let roads = match compiled.road_lanes.len() {
        4 => [0, 2],
        _ => [0, 1],
    };
    roads.contains(&road.0)
}

#[test]
fn minor_roads_have_their_sign_at_their_lines_and_major_far_side_turns_wait_inside() {
    for (name, net) in scenarios() {
        let compiled = compile(&net).unwrap();
        let sim = &compiled.sim;
        let sign = match name.ends_with("Stop") {
            true => BarrierKind::Stop,
            false => BarrierKind::GiveWay,
        };
        for (c, (&connector, &turn)) in
            compiled.connectors[0].iter().zip(&compiled.turns[0]).enumerate()
        {
            let barriers = sim.lane(connector).barriers();
            let next = sim.lane(sim.lane(connector).successors()[0]);
            assert!(next.junction().is_none(), "{name}: {connector:?} is split");
            match (major(&compiled, &net, c), turn) {
                // A far-side turn from the major road: nothing at its line, and a
                // give-way line inside, along its connector, giving way to its
                // oncoming traffic.
                (true, Turn::Right) => {
                    assert_eq!(barriers.len(), 1, "{name}: {connector:?} has {barriers:?}");
                    assert_eq!(barriers[0].kind, BarrierKind::GiveWay);
                    assert!(
                        barriers[0].s > 0.0,
                        "{name}: {connector:?} waits at its line"
                    );
                    assert!(!sim.yields(&barriers[0]).is_empty());
                }
                (true, _) => {
                    assert!(
                        barriers.is_empty(),
                        "{name}: {connector:?} has {barriers:?}"
                    );
                }
                // From the minor road: its sign at its line, giving way, and one
                // gap from there.
                (false, _) => {
                    assert_eq!(barriers.len(), 1, "{name}: {connector:?} has {barriers:?}");
                    assert_eq!(barriers[0].kind, sign, "{name}");
                    assert!(!sim.yields(&barriers[0]).is_empty(), "{name}");
                }
            }
        }
    }
}

#[test]
fn priority_intersections_flooded_with_traffic_have_no_collisions() {
    for (_, net) in scenarios() {
        common::flood(&net, 120.0);
    }
}

/// Every way through is taken, and vehicles keep leaving every minute, with a
/// car every 5 s at the entries in turn: light enough that the major road has
/// gaps for a minor-road car to cross both its carriageways, as the Highway
/// Capacity Manual's critical gaps of about 7 s need.
#[test]
fn priority_intersections_keep_traffic_flowing_every_way() {
    for (name, net) in scenarios() {
        let compiled = compile(&net).unwrap();
        let connectors = compiled.connectors[0].clone();
        let mut sim = common::sim(compiled.sim);
        let entries = entries(&sim);
        let mut used = vec![false; connectors.len()];
        let (mut spawned, mut left_before) = (0, 0);
        for step in 1..=common::steps(300.0) {
            if step % common::steps(5.0) == 0 {
                let lane = entries[(step / common::steps(5.0) * 7) as usize % entries.len()];
                let at = Placement { lane, s: 5.0, speed: LIMIT, route: Route::random(step) };
                if sim
                    .try_insert(
                        VehicleSpec::passenger_car(),
                        Driver { seed: step, ..Driver::default() },
                        at,
                    )
                    .is_ok()
                {
                    spawned += 1;
                }
            }
            sim.step();
            for (c, &lane) in connectors.iter().enumerate() {
                used[c] |= !sim.occupants(lane).is_empty();
            }
            if step % common::steps(60.0) == 0 && step > common::steps(60.0) {
                let left = spawned - sim.vehicles().count();
                assert!(
                    left > left_before + 3,
                    "{name}: standstill in the minute to {:.0} s",
                    sim.time()
                );
                left_before = left;
            }
        }
        let unused: Vec<_> = connectors.iter().zip(&used).filter(|(_, u)| !**u).collect();
        assert!(unused.is_empty(), "{name}: never used: {unused:?}");
    }
}

fn entries(sim: &Simulation) -> Vec<LaneId> {
    sim.network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect()
}

/// Each vehicle's junction terms from the last longitudinal update: the
/// vehicle, the one it keeps out of the way of, and the term.
#[derive(Default)]
struct JunctionTerms(Vec<(VehicleId, VehicleId, f64)>);

impl common::Collect for JunctionTerms {
    fn collect(&mut self, event: DebugEvent) {
        match event {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::AccelTerm { vehicle, source: AccelSource::Junction { id, .. }, accel } => {
                self.0.push((vehicle, id, accel))
            }
            _ => {}
        }
    }
}

/// Traffic on the major road, straight on or turning near-side, never brakes
/// harder than safely for a vehicle from the minor road. It may stop for one:
/// a minor-road driver grows impatient waiting, and cuts in, reckoning on the
/// major road's traffic slowing for it, which can take more than comfortable
/// braking (see `docs/models/gap-acceptance.md`, Impatience).
#[test]
fn major_road_traffic_never_brakes_hard_for_minor_road_traffic() {
    for (name, net) in scenarios() {
        let compiled = compile(&net).unwrap();
        // Each connector's first lane, and whether it is from the major road
        // and not turning across.
        let mut through: HashMap<LaneId, bool> = HashMap::new();
        let mut minor: Vec<LaneId> = Vec::new();
        for (c, (&connector, &turn)) in
            compiled.connectors[0].iter().zip(&compiled.turns[0]).enumerate()
        {
            match major(&compiled, &net, c) {
                true => {
                    through.insert(connector, turn != Turn::Right);
                }
                false => minor.push(connector),
            }
        }
        let mut sim = common::watched(compiled.sim, JunctionTerms::default());
        let entries = entries(&sim);
        let on = |sim: &common::Watched<JunctionTerms>,
                  id: VehicleId,
                  lanes: &dyn Fn(LaneId) -> bool| {
            sim.get(id).is_some_and(|v| v.path().iter().any(|&l| lanes(l)))
        };
        let mut hard = Vec::new();
        for step in 0..common::steps(300.0) {
            if step % common::steps(0.5) == 0 {
                let lane = entries[(step / common::steps(0.5) * 7) as usize % entries.len()];
                let at = Placement { lane, s: 5.0, speed: LIMIT, route: Route::random(step) };
                let _ = sim.try_insert(
                    VehicleSpec::passenger_car(),
                    Driver { seed: step, ..Driver::default() },
                    at,
                );
            }
            sim.step();
            if step % 3 != 0 {
                continue;
            }
            for &(id, other, accel) in &sim.sink().0 {
                let mine = on(&sim, id, &|l| through.get(&l) == Some(&true));
                let theirs = on(&sim, other, &|l| minor.contains(&l));
                let Some(v) = sim.get(id) else { continue };
                let binding = v.accel() == accel;
                let braking = accel < -SAFE_BRAKING;
                if mine && theirs && binding && braking {
                    hard.push((id, other, sim.time(), accel));
                }
            }
        }
        assert!(
            hard.is_empty(),
            "{name}: {} hard: {:?}",
            hard.len(),
            &hard[..hard.len().min(5)]
        );
    }
}

/// Every vehicle from a road with a stop sign comes to rest at its line
/// before its front crosses it, with a car every 5 s at the entries in turn, so
/// that the major road has gaps: at rest no further back than its gap to lines
/// and its spread, and half a metre more.
#[test]
fn at_a_stop_sign_every_car_comes_to_rest_at_the_line_before_it_goes() {
    for (name, net) in scenarios().into_iter().filter(|(n, _)| n.ends_with("Stop")) {
        let compiled = compile(&net).unwrap();
        let mut sim = common::sim(compiled.sim);
        let entries = entries(&sim);
        // Each vehicle's front's distance to a stop line that is its next
        // barrier, as of the step before, and whether it has come to rest at it.
        let mut before: HashMap<VehicleId, (f64, bool)> = HashMap::new();
        let mut stopped = 0;
        for step in 0..common::steps(300.0) {
            if step % common::steps(5.0) == 0 {
                let lane = entries[(step / common::steps(5.0) * 7) as usize % entries.len()];
                let at = Placement { lane, s: 5.0, speed: LIMIT, route: Route::random(step) };
                let _ = sim.try_insert(
                    VehicleSpec::passenger_car(),
                    Driver { seed: step, ..Driver::default() },
                    at,
                );
            }
            sim.step();
            let network = sim.network();
            for (id, v) in sim.vehicles() {
                let spec = v.spec();
                // The next barrier ahead of its rear axle, and how far its front
                // is short of it along its path.
                let mut start = -(v.lane_pos().s - spec.rear_axle + spec.length);
                let mut next = None;
                for (j, &l) in v.path().iter().enumerate() {
                    let from = if j == 0 {
                        v.lane_pos().s
                    } else {
                        f64::NEG_INFINITY
                    };
                    let lane = network.lane(l);
                    if let Some(b) = lane.barriers().iter().find(|b| b.s > from) {
                        next = Some((b.kind, start + b.s));
                        break;
                    }
                    start += lane.geometry().length();
                }
                let Some((BarrierKind::Stop, gap)) = next else {
                    before.remove(&id);
                    continue;
                };
                let driver = sim.driver(id).unwrap();
                let reach = driver.line_gap + driver.line_gap_spread + 0.5;
                let (was, rested) = before.get(&id).copied().unwrap_or((f64::INFINITY, false));
                let rested = rested || v.speed() == 0.0 && gap < reach;
                if was > 0.0 && gap <= 0.0 {
                    assert!(
                        rested,
                        "{name}: {id:?} went without stopping at {:.1} s",
                        sim.time()
                    );
                    stopped += 1;
                }
                before.insert(id, (gap, rested));
            }
        }
        assert!(stopped > 10, "{name}: only {stopped} went from a stop line");
    }
}
