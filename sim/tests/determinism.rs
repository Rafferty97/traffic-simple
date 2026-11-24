//! Replaying the same inputs reproduces the same simulation, bit for bit.
//!
//! The inputs are the network, the sequence of `step` lengths and every call
//! that changes the simulation (insertions, removals, holds, lane changes).
//! The run below exercises all of them, with irregular frame lengths so that
//! substepping and the per-rate accumulators are covered too.

use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use traffic_sim::{Driver, LaneId, Lateral, Placement, Route, Side, VehicleId};

mod common;
use common::{car, ring_road};

const SPEED_LIMIT: f64 = 50.0 / 3.6;

/// Runs a busy three-lane ring for `seconds` and returns every vehicle's full
/// state and every lane's occupancy after each frame, as raw bits.
fn trace(seed: u64, seconds: f64) -> Vec<u64> {
    let network = ring_road(600.0, 3, SPEED_LIMIT);
    let lanes: Vec<LaneId> = network.lanes().map(|(id, _)| id).collect();
    let mut sim = common::sim(network);
    // Seeded, so the run's inputs are themselves reproducible.
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(seed);
    let mut out = Vec::new();

    while sim.time() < seconds {
        // Irregular frames, some long enough to be split into substeps.
        let dt = 0.005 + 0.06 * rng.random::<f64>();
        sim.step_variable(dt);

        let roll = rng.random::<f64>();
        let picked = sim.vehicles().nth((roll * 1000.0) as usize % 50).map(|(id, _)| id);
        if roll < 0.04 {
            let lane = LaneId(2);
            let driver = Driver {
                speed_factor: 0.8 + 0.4 * rng.random::<f64>(),
                ..common::driver()
            };
            let s = rng.random::<f64>() * sim.network().lane(lane).geometry().length();
            let speed = driver.desired_speed(&car(), SPEED_LIMIT);
            let _ = sim.try_insert(
                car(),
                driver,
                Placement { lane, s, speed, route: Route::default() },
            );
        } else if let Some(id) = picked {
            match (roll * 1000.0) as usize % 7 {
                0 if roll < 0.06 => {
                    sim.remove(id);
                }
                1 if roll < 0.08 => sim.hold(id),
                2 if roll < 0.10 => sim.release(id),
                3 if roll < 0.11 => {
                    let side = if roll < 0.105 {
                        Side::Left
                    } else {
                        Side::Right
                    };
                    let _ = sim.change_lane(id, side);
                }
                _ => {}
            }
        }

        out.push(sim.time().to_bits());
        for (id, v) in sim.vehicles() {
            push_id(&mut out, id);
            out.extend(
                [
                    v.pos().x,
                    v.pos().y,
                    v.heading(),
                    v.speed(),
                    v.accel(),
                    v.curvature(),
                ]
                .map(f64::to_bits),
            );
            out.extend(v.path().iter().map(|l| l.0 as u64));
            out.push(v.intent().indicating().is_some() as u64);
        }
        for &lane in &lanes {
            for o in sim.occupants(lane) {
                push_id(&mut out, o.id);
                out.push(o.s().to_bits());
                out.push(match o.lateral {
                    Lateral::InLane => 0,
                    Lateral::Diverging(Side::Left) => 1,
                    Lateral::Diverging(Side::Right) => 2,
                    Lateral::Converging { turn, side } => turn.to_bits() ^ (3 + side as u64),
                    Lateral::Ghost => 4,
                });
            }
        }
    }
    out
}

/// Vehicle ids are opaque, so they're recorded by their debug form.
fn push_id(out: &mut Vec<u64>, id: VehicleId) {
    out.extend(format!("{id:?}").bytes().map(u64::from));
}

#[test]
fn replay_is_exact() {
    let first = trace(0x2545_F491_4F6C_DD1D, 120.0);
    let second = trace(0x2545_F491_4F6C_DD1D, 120.0);
    assert!(first.len() > 100_000, "the run should be busy");
    let diverged = first.iter().zip(&second).position(|(a, b)| a != b);
    assert_eq!(diverged, None, "replay diverged at word {diverged:?}");
    assert_eq!(first.len(), second.len());
}

#[test]
fn different_inputs_differ() {
    // Guards against a trace that ignores the state it should be recording.
    assert_ne!(trace(1, 20.0), trace(2, 20.0));
}
