//! Busy intersections for a quarter of an hour each, drivers varied, over many
//! seeds: signalised crossroads, square and oblique, one to three lanes each
//! way, turning on red or with a keep-clear zone; priority crossroads and
//! T-junctions; all-way stops; and grids of signalised and priority crossroads
//! joined by short roads. None may lock up, and the junction's queue invariants
//! must hold throughout. A grid may gridlock, as a queue backs up from one
//! crossroads through the next and round to the first, which the design allows:
//! that is counted and printed, not failed. Slow, so ignored by default:
//! `cargo test --release --test stress -- --ignored --nocapture`.

use traffic_network::{RoadNetwork, Sign, compile, scenarios};
use traffic_sim::{LaneId, Placement, Route, Simulation, VehicleSpec};

mod common;

/// How long the vehicles in a junction may all stand still before it counts as
/// locked up, in seconds: longer than any red light.
const STILL: f64 = 40.0;

/// Runs `net` for 15 minutes, a vehicle tried every `every` seconds at a random
/// entry, its driver and route from `seed`: how many times it gridlocked, or
/// what went wrong.
///
/// A junction whose vehicles all stand still for [`STILL`] is locked up,
/// unless meanwhile one of them has had nowhere to go: the lane it leaves the
/// junction by has had a vehicle stopped too near its start to leave room for
/// it. That is gridlock, a queue backed up from beyond, and counts as one; the
/// junction is then watched afresh.
fn run(net: &RoadNetwork, seed: u64, every: f64) -> Result<usize, String> {
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
    let mut gridlocks = 0;
    // How long each junction's vehicles have all stood still, in steps.
    // and whether a queue has backed up into it from beyond meanwhile.
    let mut still = vec![0u64; sim.network().junctions().count()];
    let mut backed = vec![false; still.len()];
    let (every, check) = (common::steps(every), common::steps(0.1));
    for step in 0..common::steps(15.0 * 60.0) {
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
            let at = Placement {
                lane,
                s: 5.0,
                speed,
                route: Route::random(step ^ seed),
                seed: step ^ seed,
            };
            let _ = sim.try_insert(car, at);
        }
        sim.step();
        if step % check != 0 {
            continue;
        }
        if let Some(what) = common::queue_violation(&sim) {
            return Err(format!("{what} at {:.1} s", sim.time()));
        }
        // Each junction's vehicles, whether any is moving.
        let net = sim.network();
        let mut any = vec![false; still.len()];
        let mut moving = vec![false; still.len()];
        for (_, v) in sim.vehicles() {
            if let Some(j) = net.lane(v.lane()).junction() {
                any[j.0 as usize] = true;
                moving[j.0 as usize] |= v.speed() > 0.05;
            }
        }
        for j in 0..still.len() {
            if !any[j] || moving[j] {
                (still[j], backed[j]) = (0, false);
                continue;
            }
            still[j] += check;
            backed[j] = backed[j] || backed_up(&sim, j);
            if common::steps(STILL) < still[j] {
                if !backed[j] {
                    return Err(format!("junction {j} locked up at {:.0} s", sim.time()));
                }
                gridlocks += 1;
                (still[j], backed[j]) = (0, false);
            }
        }
    }
    Ok(gridlocks)
}

/// Whether a vehicle stopped in junction `j` has no room on the lane it leaves
/// it by: a vehicle there is stopped short of a car's length and minimum gap
/// from its start. Or one in its queue has stopped on its way out, its rear
/// axle past the junction's lanes, so it waits on traffic beyond.
fn backed_up(sim: &Simulation, j: usize) -> bool {
    let net = sim.network();
    let junction = traffic_sim::JunctionId(j as u32);
    // On its way out, its path's lanes in the junction behind it, not ahead.
    let stuck_leaving = sim.queue(junction).iter().any(|e| {
        sim.get(e.id).is_some_and(|v| {
            let ahead = v.path().iter().any(|&l| net.lane(l).junction() == Some(junction));
            v.speed() <= 0.05 && net.lane(v.lane()).junction().is_none() && !ahead
        })
    });
    if stuck_leaving {
        return true;
    }
    let inside = |l: LaneId| net.lane(l).junction().is_some_and(|k| k.0 as usize == j);
    sim.vehicles()
        .filter(|(_, v)| inside(v.lane()) && v.speed() <= 0.05)
        .any(|(_, v)| {
            let Some(&exit) = v.path().iter().find(|&&l| net.lane(l).junction().is_none()) else {
                return false;
            };
            let room = v.spec().length + v.spec().min_gap;
            sim.occupants(exit)
                .iter()
                .filter(|o| !o.is_ghost())
                .any(|o| o.s() < room && sim.get(o.id).is_some_and(|w| w.speed() <= 0.05))
        })
}

/// Runs each of `nets` over 20 seeds, a vehicle tried every third and every
/// two thirds of a second, asserting that none went wrong.
fn stress(nets: &[(&str, RoadNetwork)]) {
    stress_every(nets, &[1.0 / 3.0, 2.0 / 3.0]);
}

/// As [`stress`], a vehicle tried every so many seconds as `every` says, and
/// printing how many runs of each network gridlocked.
fn stress_every(nets: &[(&str, RoadNetwork)], every: &[f64]) {
    let mut failures = Vec::new();
    for (name, net) in nets {
        for &every in every {
            let mut gridlocked = 0;
            for seed in 1..=20u64 {
                match run(net, seed, every) {
                    Ok(gridlocks) => gridlocked += usize::from(gridlocks > 0),
                    Err(what) => {
                        failures.push(format!("{name}, seed {seed}, every {every:.2} s: {what}"))
                    }
                }
            }
            if gridlocked > 0 {
                println!("{name}, every {every:.2} s: {gridlocked} of 20 runs gridlocked");
            }
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

const SPEED: f64 = 50.0 / 3.6;

/// Two runs that once locked up, quick enough to run with the other tests: on
/// the 75° crossroads, a turner taking its gap was queued for the rest of its
/// turn without the junction checking it closed no cycle, and on the 60°, a car
/// committed to the junction held itself back to change into the lane beside,
/// which it no longer could, behind one the junction held.
#[test]
fn the_oblique_crossroads_runs_that_once_locked_up_dont() {
    let runs = [(75.0, 4, 2.0 / 3.0), (60.0, 13, 1.0 / 3.0)];
    for (angle, seed, every) in runs {
        let net = scenarios::signalised_oblique_crossroads(2, SPEED, f64::to_radians(angle));
        if let Err(what) = run(&net, seed, every) {
            panic!("{angle}°, seed {seed}: {what}");
        }
    }
}

#[test]
#[ignore = "slow: a few minutes in release"]
fn busy_signalised_crossroads_never_lock_up() {
    stress(&[
        ("1 lane", scenarios::signalised_crossroads(1, SPEED)),
        ("2 lanes", scenarios::signalised_crossroads(2, SPEED)),
        ("3 lanes", scenarios::signalised_crossroads(3, SPEED)),
        (
            "60°",
            scenarios::signalised_oblique_crossroads(2, SPEED, 60f64.to_radians()),
        ),
        (
            "75°",
            scenarios::signalised_oblique_crossroads(2, SPEED, 75f64.to_radians()),
        ),
    ]);
}

#[test]
#[ignore = "slow: a few minutes in release"]
fn busy_signalised_crossroads_turning_on_red_or_keeping_clear_never_lock_up() {
    stress(&[
        (
            "turn on red, 1 lane",
            scenarios::signalised_crossroads_turn_on_red(1, SPEED),
        ),
        (
            "turn on red, 2 lanes",
            scenarios::signalised_crossroads_turn_on_red(2, SPEED),
        ),
        (
            "keep clear",
            scenarios::signalised_crossroads_keep_clear(2, SPEED),
        ),
    ]);
}

#[test]
#[ignore = "slow: a few minutes in release"]
fn busy_priority_intersections_never_lock_up() {
    stress(&[
        (
            "give way, 1 lane",
            scenarios::priority_crossroads([1, 1], SPEED, Sign::GiveWay),
        ),
        (
            "give way, 2 lanes",
            scenarios::priority_crossroads([2, 2], SPEED, Sign::GiveWay),
        ),
        (
            "stop, 1 lane",
            scenarios::priority_crossroads([1, 1], SPEED, Sign::Stop),
        ),
        (
            "T, give way",
            scenarios::priority_t_junction([1, 1], SPEED, Sign::GiveWay),
        ),
        (
            "T, stop",
            scenarios::priority_t_junction([1, 1], SPEED, Sign::Stop),
        ),
        (
            "give way, 2 by 1 lanes",
            scenarios::priority_crossroads([2, 1], SPEED, Sign::GiveWay),
        ),
        (
            "T, give way, 2 by 1 lanes",
            scenarios::priority_t_junction([2, 1], SPEED, Sign::GiveWay),
        ),
    ]);
}

#[test]
#[ignore = "slow: a few minutes in release"]
fn busy_all_way_stops_never_lock_up() {
    stress(&[
        (
            "all-way, 1 lane",
            scenarios::all_way_stop_crossroads(1, SPEED),
        ),
        (
            "all-way, 2 lanes",
            scenarios::all_way_stop_crossroads(2, SPEED),
        ),
    ]);
}

#[test]
#[ignore = "slow: a few minutes in release"]
fn busy_grids_never_lock_up() {
    stress_every(
        &[
            ("grid, 1 lane", scenarios::grid(4, 4, 1, 100.0, SPEED)),
            ("grid, 2 lanes", scenarios::grid(4, 4, 2, 100.0, SPEED)),
        ],
        &[1.0 / 3.0, 2.0 / 3.0, 4.0 / 3.0],
    );
}
