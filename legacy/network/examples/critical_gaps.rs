//! The shortest lag a car at rest at a give-way line goes ahead of, by the way
//! it turns, on the priority crossroads with one lane each way at 50 km/h: one
//! car on the major road coming at the limit, its lag the time it would take to
//! reach the junction's centre. Compare the Highway Capacity Manual's critical
//! gaps, about 6.5 s straight across a two-lane road, 7 s turning across it
//! and 6 s turning into it.
//!
//! `cargo run --release -p traffic-network --example critical_gaps`

use traffic_network::{Sign, Turn, compile, scenarios};
use traffic_sim::{Driver, LaneId, Placement, Route, Simulation, VehicleId, VehicleSpec};

/// The length of a step, in seconds: 30 Hz.
const DT: f64 = 1.0 / 30.0;

/// A simulation of `network` stepping at 30 Hz, with the slower updates at 10 Hz.
fn sim(network: traffic_sim::SimNetwork) -> Simulation {
    let config = traffic_sim::Config { dt: DT, macro_step: 3 };
    Simulation::with_config(network, config)
}

fn main() {
    let limit = 50.0 / 3.6;
    let net = scenarios::priority_crossroads([1, 1], limit, Sign::GiveWay);
    let compiled = compile(&net).unwrap();
    let connections = &net.intersections[0].connections;
    // From the south (road 3), across or into the road from the east and west.
    let (south, east, west) = (
        compiled.road_lanes[3][0],
        compiled.road_lanes[0][0],
        compiled.road_lanes[2][0],
    );
    let spec = VehicleSpec::passenger_car();
    for turn in [Turn::Straight, Turn::Right, Turn::Left] {
        let c = (0..connections.len())
            .find(|&c| connections[c].0.road.0 == 3 && compiled.turns[0][c] == turn)
            .unwrap();
        let connector = compiled.connectors[0][c];
        // Turning near-side, it merges with traffic from the far side.
        let major_lane = if turn == Turn::Left { east } else { west };
        let mut accepted = None;
        for tenths in (20..140).rev() {
            let lag = f64::from(tenths) / 10.0;
            let mut sim = sim(compiled.sim.clone());
            let length = sim.network().lane(south).geometry().length();
            let s = length - (spec.length - spec.rear_axle) - Driver::default().line_gap;
            let minor = insert_going(&mut sim, south, s, connector);
            let major_length = sim.network().lane(major_lane).geometry().length();
            // Its front `lag` from the junction's centre, about 8 m past its line,
            // going straight on.
            let front = (major_length + 8.0 - limit * lag).max(0.0);
            let straight = (0..connections.len())
                .find(|&c| {
                    compiled.connectors[0][c] != connector
                        && compiled.road_lanes[connections[c].0.road.0][connections[c].0.lane]
                            == major_lane
                        && compiled.turns[0][c] == Turn::Straight
                })
                .map(|c| compiled.connectors[0][c])
                .unwrap();
            let major = insert_going(
                &mut sim,
                major_lane,
                front - spec.length + spec.rear_axle,
                straight,
            );
            let (first, hardest) = race(&mut sim, minor, major, connector);
            if !first {
                break;
            }
            accepted = Some((lag, hardest));
        }
        match accepted {
            Some((lag, hardest)) => println!(
                "{turn:?}: goes ahead of lags down to {lag:.1} s, the major car then braking at most {:.2} m/s²",
                -hardest
            ),
            None => println!("{turn:?}: never goes ahead"),
        }
    }
}

/// A car on `lane` at `s`, at its desired speed if `s` isn't where a car
/// stops for its line, on a route through `connector`.
fn insert_going(sim: &mut Simulation, lane: LaneId, s: f64, connector: LaneId) -> VehicleId {
    let spec = VehicleSpec::passenger_car();
    let length = sim.network().lane(lane).geometry().length();
    let at_line = s >= length - spec.length;
    let speed = if at_line {
        0.0
    } else {
        sim.network().lane(lane).speed_limit()
    };
    (0..64)
        .find_map(|seed| {
            let id = sim.insert(
                spec,
                Driver { seed, ..Driver::default() },
                Placement { lane, s, speed, route: Route::random(seed) },
            );
            match sim.get(id).unwrap().path().contains(&connector) {
                true => Some(id),
                false => {
                    sim.remove(id);
                    None
                }
            }
        })
        .expect("a route through the connector")
}

/// Runs for 20 s: whether the minor car reaches its connector before the
/// major car is in the junction, and the major car's hardest acceleration.
fn race(
    sim: &mut Simulation,
    minor: VehicleId,
    major: VehicleId,
    connector: LaneId,
) -> (bool, f64) {
    let mut first = None;
    let mut hardest = 0.0f64;
    for _ in 0..(20.0 / DT) as u32 {
        sim.step();
        if let Some(v) = sim.get(major) {
            hardest = hardest.min(v.accel());
            if first.is_none() && sim.network().lane(v.path()[0]).junction().is_some() {
                first = Some(false);
            }
        }
        if first.is_none() && sim.get(minor).is_some_and(|v| v.path()[0] == connector) {
            first = Some(true);
        }
    }
    (first == Some(true), hardest)
}
