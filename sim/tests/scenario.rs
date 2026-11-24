//! The UI's original demo scenario, headless: an empty three-lane ring,
//! vehicles spawned into the outer lane and despawned from the inner lane, so
//! congestion on the outside drives lane changes leftward. The UI has since moved
//! on to spawning and despawning anywhere to keep a target count; this keeps the
//! old scenario, at fixed rates, as a stress test for lane changes.

use std::f64::consts::TAU;

use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use traffic_sim::{LaneId, Placement, Route, Side, VehicleId, VehicleSpec};

mod common;
use common::{Terms, ring_road};

use common::DT;
/// The UI's ring: the inner lane's circumference, in meters.
const RING_CIRCUMFERENCE: f64 = 1000.0;
const SPEED_LIMIT: f64 = 50.0 / 3.6;
const SPAWN_LANE: LaneId = LaneId(2);
const DESPAWN_LANE: LaneId = LaneId(0);

/// The UI's demo car.
fn demo_car(speed_factor: f64) -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        speed_factor,
        max_accel: 1.0,
        comfy_decel: 1.5,
        time_headway: 1.0,
        min_gap: 1.5,
        rear_axle: 1.0,
        front_axle: 3.7,
        courtesy: 1.0,
        politeness: 0.2,
        assertiveness: 0.0,
        accel_noise: 0.0,
        line_gap: 1.0,
        line_gap_spread: 0.0,
    }
}

/// A speed factor as the UI draws them: normal with mean 1 and deviation 0.1,
/// by Box-Muller, clamped to [0.8, 1.2].
fn speed_factor(rng: &mut Xoshiro256PlusPlus) -> f64 {
    let u = 1.0 - rng.random::<f64>();
    let v = rng.random::<f64>();
    let normal = (-2.0 * u.ln()).sqrt() * (TAU * v).cos();
    (1.0 + 0.1 * normal).clamp(0.8, 1.2)
}

#[derive(Debug, Default)]
struct Stats {
    spawned: u32,
    despawned: u32,
    changes_left: u32,
    changes_right: u32,
    intents: u32,
    most_on_road: usize,
}

fn run_scenario(spawn_rate: f64, despawn_rate: f64, minutes: f64) -> Stats {
    let mut sim = common::watched(
        ring_road(RING_CIRCUMFERENCE, 3, SPEED_LIMIT),
        Terms::default(),
    );
    let spawn_s = 0.25 * sim.network().lane(SPAWN_LANE).geometry().length();
    let despawn_s = 0.75 * sim.network().lane(DESPAWN_LANE).geometry().length();
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(0);
    let (mut spawn_credit, mut despawn_credit) = (0.0f64, 0.0f64);
    let mut stats = Stats::default();
    let mut last: std::collections::HashMap<VehicleId, (f64, LaneId, bool)> = Default::default();

    for frame in 0..(minutes * 60.0 / DT) as usize {
        sim.step_variable(DT);

        spawn_credit = (spawn_credit + spawn_rate * DT).min(1.0);
        if spawn_credit >= 1.0 {
            let car = demo_car(speed_factor(&mut rng));
            let at = Placement {
                lane: SPAWN_LANE,
                s: spawn_s,
                speed: car.desired_speed(SPEED_LIMIT),
                route: Route::default(),
            };
            if sim.try_insert(car, at).is_ok() {
                spawn_credit -= 1.0;
                stats.spawned += 1;
            }
        }

        despawn_credit = (despawn_credit + despawn_rate * DT).min(3.0);
        let crossed: Vec<_> = sim
            .vehicles()
            .filter(|(id, v)| {
                let prev = last.get(id).map_or(f64::INFINITY, |p| p.0);
                v.lane() == DESPAWN_LANE && prev < despawn_s && v.lane_pos().s >= despawn_s
            })
            .map(|(id, _)| id)
            .collect();
        for id in crossed {
            if despawn_credit >= 1.0 {
                sim.remove(id);
                despawn_credit -= 1.0;
                stats.despawned += 1;
            }
        }

        // Count lane changes and new intents.
        for (id, v) in sim.vehicles() {
            if let Some(&(_, lane, indicating)) = last.get(&id) {
                if lane != v.lane() {
                    match v.old_side() {
                        Some(Side::Right) => stats.changes_left += 1,
                        _ => stats.changes_right += 1,
                    }
                }
                if !indicating && v.intent().indicating().is_some() {
                    stats.intents += 1;
                }
            }
        }
        last.clear();
        last.extend(sim.vehicles().map(|(id, v)| {
            (
                id,
                (v.lane_pos().s, v.lane(), v.intent().indicating().is_some()),
            )
        }));
        stats.most_on_road = stats.most_on_road.max(sim.vehicles().count());

        // No two bodies overlap. Checked every tenth frame, as it's quadratic.
        if frame % 10 == 0 {
            let all: Vec<_> = sim.vehicles().collect();
            for (i, &(a, va)) in all.iter().enumerate() {
                for &(b, vb) in &all[i + 1..] {
                    if va.center().distance(vb.center()) < 6.0 {
                        let touching = vb.corners().iter().any(|&c| va.distance_to(c) == 0.0)
                            || va.corners().iter().any(|&c| vb.distance_to(c) == 0.0);
                        assert!(!touching, "{a:?} and {b:?} collided at {:.1} s", sim.time());
                    }
                }
            }
        }
    }
    stats
}

/// Ten minutes of it never collides. That is all it asserts: what the traffic
/// does, how many spawn, despawn and change lanes, depends on how it happens to
/// evolve, so it is printed rather than asserted.
#[test]
fn congestion_on_the_ring_never_collides() {
    let stats = run_scenario(0.5, 0.3, 10.0);
    eprintln!("{stats:?}");
}

#[test]
#[ignore = "slow; run with --ignored to stress dense traffic"]
fn dense_traffic_never_collides() {
    for (spawn, despawn) in [(1.0, 0.3), (2.0, 0.2), (2.0, 1.0)] {
        let stats = run_scenario(spawn, despawn, 10.0);
        eprintln!("spawn {spawn}/s, despawn {despawn}/s: {stats:?}");
    }
}
