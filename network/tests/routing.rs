//! Vehicles routed to destinations: each takes the lane changes and turns its
//! route needs, and leaves the network where it was sent.

use traffic_network::{Sign, compile, scenarios};
use traffic_sim::{DestinationId, Driver, LaneId, Placement, Route, Simulation, VehicleSpec};

mod common;

const SPEED: f64 = 50.0 / 3.6;

/// Each exit of the network, the lanes leading nowhere grouped by road,
/// registered as a destination.
fn exits(sim: &mut Simulation) -> Vec<DestinationId> {
    let mut roads: Vec<(traffic_sim::RoadId, Vec<LaneId>)> = Vec::new();
    for (id, lane) in sim.network().lanes() {
        if lane.successors().is_empty() {
            match roads.iter_mut().find(|(road, _)| *road == lane.road()) {
                Some((_, lanes)) => lanes.push(id),
                None => roads.push((lane.road(), vec![id])),
            }
        }
    }
    let roads: Vec<Vec<LaneId>> = roads.into_iter().map(|(_, lanes)| lanes).collect();
    roads.iter().map(|lanes| sim.add_destination(lanes)).collect()
}

/// Sends a car alone from every entry lane to every exit it has a way to, one
/// after another, and checks each leaves by its exit. Returns how many it sent.
fn every_car_leaves_by_its_exit(net: &traffic_network::RoadNetwork) -> usize {
    let mut sim = common::sim(compile(net).unwrap().sim);
    let exits = exits(&mut sim);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut sent = 0;
    for &entry in &entries {
        for &exit in &exits {
            if sim.time_to(exit, entry).is_none() {
                continue;
            }
            let at = Placement { lane: entry, s: 5.0, speed: SPEED, route: Route::to(exit) };
            let id = sim.insert(
                VehicleSpec::passenger_car(),
                Driver { seed: 1, ..Driver::default() },
                at,
            );
            let mut last = entry;
            // Long enough to wait at every red light on the way.
            for _ in 0..(600.0 / common::DT) as usize {
                match sim.get(id) {
                    Some(v) => last = v.lane(),
                    None => break,
                }
                sim.step();
            }
            assert!(
                sim.get(id).is_none(),
                "{entry:?} to {exit:?}: still going on {last:?}"
            );
            assert!(
                sim.destination(exit).contains(&last),
                "{entry:?} to {exit:?} ({:?}) left by {last:?}",
                sim.destination(exit)
            );
            sent += 1;
        }
    }
    sent
}

#[test]
fn a_car_sent_to_each_exit_of_a_crossroads_leaves_by_it() {
    for lanes in 1..=3 {
        // From every lane in, to each of the three roads out but its own.
        let sent = every_car_leaves_by_its_exit(&scenarios::signalised_crossroads(lanes, SPEED));
        assert_eq!(sent, 4 * lanes * 3, "{lanes} lanes");
        every_car_leaves_by_its_exit(&scenarios::priority_crossroads(
            [lanes, lanes],
            SPEED,
            Sign::GiveWay,
        ));
    }
}

#[test]
fn a_car_sent_to_each_exit_of_a_grid_leaves_by_it() {
    let sent = every_car_leaves_by_its_exit(&scenarios::grid(3, 3, 2, 100.0, SPEED));
    assert!(sent > 0);
}

#[test]
fn a_car_sent_off_the_freeway_takes_the_off_ramp() {
    let sent = every_car_leaves_by_its_exit(&scenarios::ramps(100.0 / 3.6, 60.0 / 3.6));
    assert!(sent >= 2, "{sent}");
}

/// Sends cars into `net` for a quarter of an hour, one every 1.5 s into a
/// random entry lane, each to a random exit it has a way to, and returns how
/// many left by their exit, and how many by another.
fn busy(net: &traffic_network::RoadNetwork) -> (usize, usize) {
    let mut sim = common::sim(compile(net).unwrap().sim);
    let exits = exits(&mut sim);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    // A small, seeded generator: a linear congruential one.
    let mut state = 1u64;
    let mut draw = |n: usize| {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 33) as usize % n
    };
    let mut sent = std::collections::HashMap::new();
    let (mut right, mut wrong) = (0, 0);
    let mut lanes = Vec::new();
    for step in 0..(900.0 / common::DT) as usize {
        if step % 45 == 0 {
            let entry = entries[draw(entries.len())];
            let ways: Vec<_> = exits.iter().filter(|&&e| sim.time_to(e, entry).is_some()).collect();
            let exit = *ways[draw(ways.len())];
            let route = Route::to(exit);
            let at = Placement { lane: entry, s: 5.0, speed: SPEED, route };
            if let Ok(id) = sim.try_insert(
                VehicleSpec::passenger_car(),
                Driver { seed: step as u64, ..Driver::default() },
                at,
            ) {
                sent.insert(id, exit);
            }
        }
        lanes.clear();
        lanes.extend(sim.vehicles().map(|(id, v)| (id, v.lane())));
        sim.step();
        for &(id, lane) in &lanes {
            if sim.get(id).is_none() {
                match sim.destination(sent[&id]).contains(&lane) {
                    true => right += 1,
                    false => wrong += 1,
                }
            }
        }
    }
    (right, wrong)
}

#[test]
fn in_busy_traffic_nearly_every_car_leaves_by_its_exit() {
    for net in [
        scenarios::grid(4, 4, 2, 100.0, SPEED),
        scenarios::signalised_crossroads(3, SPEED),
    ] {
        // Those that miss a lane change in traffic take the best way from where
        // they are, which may lead elsewhere.
        let (right, wrong) = busy(&net);
        assert!(
            right > 400 && wrong * 100 <= right,
            "{right} right, {wrong} wrong"
        );
    }
}
