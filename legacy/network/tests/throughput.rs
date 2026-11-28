//! How much traffic busy intersections and grids get through, measured rather
//! than checked: each scenario runs for 10 minutes over 10 seeds, a car tried
//! at a random entry every second (every half second on the grids), drivers
//! varied as in the stress test. It prints how many cars got through, and how
//! many vehicle-seconds were spent at rest on a junction's lanes, waiting
//! inside the junction or stuck in it. Neither has a reference to pass or fail
//! against: run it before and after a change to compare,
//! `cargo test --release --test throughput -- --ignored --nocapture`.

use traffic_network_legacy::{RoadNetwork, Sign, compile, scenarios};
use traffic_sim::{Driver, LaneId, Placement, Route, VehicleSpec};

mod common;

const SPEED: f64 = 50.0 / 3.6;

/// Runs `net` for `minutes`, a car tried every `every` seconds at a random
/// entry, its driver and route from `seed`: how many cars got through, and how
/// many vehicle-seconds were spent at rest on a junction's lanes.
fn run(net: &RoadNetwork, seed: u64, every: f64, minutes: f64) -> (usize, f64) {
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
    let mut at_rest = 0.0;
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
        let resting = sim
            .vehicles()
            .filter(|(_, v)| net.lane(v.lane()).junction().is_some() && v.speed() < 0.05)
            .count();
        at_rest += resting as f64 * common::DT;
    }
    let through = inserted.iter().filter(|&&id| sim.get(id).is_none()).count();
    (through, at_rest)
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
        let (mut through, mut at_rest) = (0, 0.0);
        for seed in 1..=10 {
            let (t, r) = run(net, seed, every, 10.0);
            through += t;
            at_rest += r;
        }
        println!("{name}: {through} through, {at_rest:.0} vehicle-seconds at rest in a junction");
    }
}
