//! How much traffic busy intersections and grids get through, measured rather
//! than checked: each scenario runs for 10 minutes over [`SEEDS`] seeds, a car
//! tried at a random entry every second (every half second on the grids),
//! drivers varied as in the stress test. It prints how many cars got through, and how
//! many vehicle-seconds were spent at rest on a junction's lanes: waiting at a
//! line there, as a turner waits for a gap, or stuck, in the junction short of
//! nothing it waits at. None has a reference to pass or fail against: run it
//! before and after a change to compare,
//! `cargo test --release --test throughput -- --ignored --nocapture`. A change
//! that alters one decision alters every run from there on, and a few long
//! episodes, gridlocks above all, make up much of the time stuck, so over
//! [`SEEDS`] seeds it can move by tens of percent from noise alone: widen the
//! seeds before reading much into a change of that size.

use traffic_network_legacy::{RoadNetwork, Sign, compile, scenarios};
use traffic_sim::{Driver, LaneId, Placement, Route, SimNetwork, Vehicle, VehicleSpec};

mod common;

const SPEED: f64 = 50.0 / 3.6;

/// How many seeds each scenario runs over.
const SEEDS: u64 = 10;

/// How near, in meters, a vehicle's front is to the next barrier it hasn't
/// committed to when it waits at it: its gap to lines, their spread, and how
/// far back it may come to rest, with room to spare.
const AT_LINE: f64 = 3.0;

/// Whether `vehicle` is waiting at a line: its front within [`AT_LINE`] of the
/// next barrier on its path it hasn't committed to.
fn at_a_line(net: &SimNetwork, vehicle: &Vehicle) -> bool {
    let axle = vehicle.lane_pos().s;
    let front = axle - vehicle.spec().rear_axle + vehicle.spec().length;
    let (mut start, mut rank) = (0.0, 0);
    for (j, &lane) in vehicle.path().iter().enumerate() {
        for barrier in net.lane(lane).barriers() {
            if j == 0 && barrier.s <= axle {
                continue;
            }
            if rank == usize::from(vehicle.committed()) {
                return start + barrier.s - front < AT_LINE;
            }
            rank += 1;
        }
        start += net.lane(lane).geometry().length();
        if start - front > AT_LINE {
            return false;
        }
    }
    false
}

/// How a run went: how many cars got through, and how many vehicle-seconds
/// were spent at rest on a junction's lanes, waiting at a line there and stuck.
#[derive(Default)]
struct Totals {
    through: usize,
    waiting: f64,
    stuck: f64,
}

/// Runs `net` for `minutes`, a car tried every `every` seconds at a random
/// entry, its driver and route from `seed`.
fn run(net: &RoadNetwork, seed: u64, every: f64, minutes: f64) -> Totals {
    let compiled = compile(net).unwrap();
    let mut sim = common::sim(compiled.sim);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    // SplitMix64, so every run is the same.
    let mut state = seed;
    let mut uniform = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut inserted = Vec::new();
    let mut totals = Totals::default();
    let every = common::steps(every);
    for step in 0..common::steps(minutes * 60.0) {
        if step % every == 0 {
            let lane = entries[(uniform() * entries.len() as f64) as usize];
            let driver = Driver {
                speed_factor: 0.85 + 0.4 * uniform(),
                assertiveness: uniform().powi(3),
                courtesy: 2.0 * uniform(),
                politeness: uniform(),
                line_gap: 0.5 + uniform(),
                line_gap_spread: 0.3,
                ..Driver::default()
            };
            let speed = sim.network().lane(lane).speed_limit() * driver.speed_factor;
            let at = Placement { lane, s: 5.0, speed, route: Route::random(step ^ seed) };
            let driver = Driver { seed: step ^ seed, ..driver };
            if let Ok(id) = sim.try_insert(VehicleSpec::passenger_car(), driver, at) {
                inserted.push(id);
            }
        }
        sim.step();
        let net = sim.network();
        for (_, v) in sim.vehicles() {
            if net.lane(v.lane()).junction().is_some() && v.speed() < 0.05 {
                match at_a_line(net, v) {
                    true => totals.waiting += common::DT,
                    false => totals.stuck += common::DT,
                }
            }
        }
    }
    totals.through = inserted.iter().filter(|&&id| sim.get(id).is_none()).count();
    totals
}

#[test]
#[ignore = "a measurement, not a check: slow, and prints its results"]
fn busy_intersections_and_grids_get_traffic_through() {
    let nets = [
        (
            "signals, 1 lane",
            scenarios::signalised_crossroads(1, SPEED),
        ),
        (
            "signals, 2 lanes",
            scenarios::signalised_crossroads(2, SPEED),
        ),
        (
            "signals, 3 lanes",
            scenarios::signalised_crossroads(3, SPEED),
        ),
        (
            "turn on red, 2 lanes",
            scenarios::signalised_crossroads_turn_on_red(2, SPEED),
        ),
        (
            "keep clear, 2 lanes",
            scenarios::signalised_crossroads_keep_clear(2, SPEED),
        ),
        (
            "give way, 1 lane",
            scenarios::priority_crossroads([1, 1], SPEED, Sign::GiveWay),
        ),
        (
            "give way, 2 lanes",
            scenarios::priority_crossroads([2, 2], SPEED, Sign::GiveWay),
        ),
        (
            "T, stop",
            scenarios::priority_t_junction([1, 1], SPEED, Sign::Stop),
        ),
        (
            "all-way, 1 lane",
            scenarios::all_way_stop_crossroads(1, SPEED),
        ),
        (
            "all-way, 2 lanes",
            scenarios::all_way_stop_crossroads(2, SPEED),
        ),
        ("grid, 1 lane", scenarios::grid(4, 4, 1, 100.0, SPEED)),
        ("grid, 2 lanes", scenarios::grid(4, 4, 2, 100.0, SPEED)),
    ];
    for (name, net) in &nets {
        let every = if name.starts_with("grid") { 0.5 } else { 1.0 };
        let mut sum = Totals::default();
        for seed in 1..=SEEDS {
            let run = run(net, seed, every, 10.0);
            sum.through += run.through;
            sum.waiting += run.waiting;
            sum.stuck += run.stuck;
        }
        let Totals { through, waiting, stuck } = sum;
        println!(
            "{name}: {through} through; at rest in a junction, {waiting:.0} vehicle-seconds \
             waiting at a line and {stuck:.0} stuck"
        );
    }
}
