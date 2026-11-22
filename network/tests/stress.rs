//! Busy signalised crossroads for a quarter of an hour each, square and
//! oblique, one to three lanes each way, drivers varied, over many seeds: none
//! may lock up, and the junction's queue invariants must hold throughout. Slow,
//! so ignored by default: `cargo test --release --test stress -- --ignored`.

use traffic_network::{RoadNetwork, compile, scenarios};
use traffic_sim::{LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};

mod common;

/// How long the vehicles in a junction may all stand still before it counts as
/// locked up, in seconds: longer than any red light.
const STILL: f64 = 40.0;

/// Runs `net` for 15 minutes, a vehicle tried every `every` steps at a random
/// entry, its driver and route from `seed`: how many left, or what went wrong.
fn run(net: &RoadNetwork, seed: u64, every: u64) -> Result<usize, String> {
    let compiled = compile(net).unwrap();
    let mut sim = Simulation::new(compiled.sim, NoopSink);
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
    let (mut spawned, mut still) = (0, 0);
    for step in 0..(15 * 60 * 60) as u64 {
        if step % every == 0 {
            let lane = entries[(uniform() * entries.len() as f64) as usize];
            let car = VehicleSpec {
                speed_factor: 0.85 + 0.4 * uniform(),
                assertiveness: uniform().powi(3),
                courtesy: 2.0 * uniform(),
                politeness: uniform(),
                line_gap: 0.5 + uniform(),
                line_gap_spread: 0.3,
                ..VehicleSpec::passenger_car()
            };
            let speed = sim.network().lane(lane).speed_limit() * car.speed_factor;
            let at = Placement { lane, s: 5.0, speed, route: Route::random(step ^ seed) };
            if sim.try_insert(car, at).is_ok() {
                spawned += 1;
            }
        }
        sim.step();
        if step % 6 != 0 {
            continue;
        }
        if let Some(what) = common::queue_violation(&sim) {
            return Err(format!("{what} at {:.1} s", sim.time()));
        }
        let net = sim.network();
        let inside = sim.vehicles().filter(|(_, v)| net.lane(v.lane()).junction().is_some());
        let (mut any, mut moving) = (false, false);
        for (_, v) in inside {
            any = true;
            moving |= v.speed() > 0.05;
        }
        still = if any && !moving { still + 6 } else { 0 };
        if still as f64 / 60.0 > STILL {
            return Err(format!("locked up at {:.0} s", sim.time()));
        }
    }
    Ok(spawned - sim.vehicles().count())
}

#[test]
#[ignore = "slow: a few minutes in release"]
fn busy_signalised_crossroads_never_lock_up() {
    let speed = 50.0 / 3.6;
    let nets = [
        ("1 lane", scenarios::signalised_crossroads(1, speed)),
        ("2 lanes", scenarios::signalised_crossroads(2, speed)),
        ("3 lanes", scenarios::signalised_crossroads(3, speed)),
        (
            "60°",
            scenarios::signalised_oblique_crossroads(2, speed, 60f64.to_radians()),
        ),
        (
            "75°",
            scenarios::signalised_oblique_crossroads(2, speed, 75f64.to_radians()),
        ),
    ];
    let mut failures = Vec::new();
    for seed in 1..=20u64 {
        for (name, net) in &nets {
            for every in [20, 40] {
                if let Err(what) = run(net, seed, every) {
                    failures.push(format!("{name}, seed {seed}, every {every}: {what}"));
                }
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}
