//! Freeze frames: a snapshot of a simulation's whole state, through JSON and back.
//! Needs the `serde` feature.

use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use traffic_sim::{Lateral, Placement, Route, Side, SimState, Simulation, VehicleId, VehicleSpec};

mod common;
use common::{car, frame, ring_road};

const SPEED_LIMIT: f64 = 50.0 / 3.6;

/// Every vehicle's state and every lane's records, as raw bits.
fn state(sim: &Simulation) -> Vec<u64> {
    let mut out = vec![sim.time().to_bits()];
    for (id, v) in sim.vehicles() {
        out.extend(format!("{id:?}").bytes().map(u64::from));
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
        out.push(v.lane_change().is_some() as u64);
    }
    for (lane, _) in sim.network().lanes() {
        for o in sim.occupants(lane) {
            out.extend(format!("{:?}", o.id).bytes().map(u64::from));
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
    out
}

/// A busy three-lane ring after a minute of seeded insertions and removals, so
/// the vehicle store has freed slots, lane changes are under way and some
/// vehicles are indicating.
fn busy() -> Simulation {
    let mut sim = common::sim(ring_road(600.0, 3, SPEED_LIMIT));
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(3);
    while sim.time() < 60.0 {
        sim.step_variable(0.005 + 0.03 * rng.random::<f64>());
        let roll = rng.random::<f64>();
        if roll < 0.05 {
            let spec = VehicleSpec { speed_factor: 0.8 + 0.4 * rng.random::<f64>(), ..car() };
            let at = Placement {
                lane: traffic_sim::LaneId(2),
                s: 600.0 * roll * 20.0,
                speed: 12.0,
                route: Route::default(),
            };
            let _ = sim.try_insert(spec, at);
        } else if roll < 0.06 {
            let id = sim.vehicles().nth(rng.random_range(0..8)).map(|(id, _)| id);
            if let Some(id) = id {
                sim.remove(id);
            }
        }
    }
    sim
}

/// Steps `sim` for 30 s with seeded frame lengths, holds, releases, lane changes
/// and removals, but no insertions, returning its state after every frame.
fn continue_for_30_s(sim: &mut Simulation) -> Vec<u64> {
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(9);
    let mut out = Vec::new();
    let end = sim.time() + 30.0;
    while sim.time() < end {
        sim.step_variable(0.005 + 0.03 * rng.random::<f64>());
        let roll = rng.random::<f64>();
        let pick = rng.random_range(0..20);
        let id: Option<VehicleId> = sim.vehicles().nth(pick).map(|(id, _)| id);
        if let Some(id) = id {
            match roll {
                r if r < 0.01 => sim.hold(id),
                r if r < 0.02 => sim.release(id),
                r if r < 0.025 => drop(sim.change_lane(id, Side::Left)),
                r if r < 0.03 => drop(sim.remove(id)),
                _ => {}
            }
        }
        out.extend(state(sim));
    }
    out
}

#[test]
fn a_restored_snapshot_continues_exactly_like_the_original() {
    let mut original = busy();
    let json = serde_json::to_string(original.state()).unwrap();
    let snapshot: SimState = serde_json::from_str(&json).unwrap();
    let mut restored = Simulation::restore(snapshot);
    assert!(original.vehicles().count() > 20, "the run should be busy");
    assert!(state(&original) == state(&restored), "restored differently");
    let (a, b) = (
        continue_for_30_s(&mut original),
        continue_for_30_s(&mut restored),
    );
    let diverged = a.iter().zip(&b).position(|(x, y)| x != y);
    assert_eq!(diverged, None, "diverged at word {diverged:?}");
    assert_eq!(a.len(), b.len());
}

#[test]
fn a_freeze_frame_loads_with_its_selection() {
    // As the UI writes it.
    let sim = busy();
    let selected = sim.vehicles().nth(3).map(|(id, _)| id);
    let json = serde_json::json!({ "selected": selected, "snapshot": sim.state() });
    let (loaded, loaded_selected) = frame::load(&serde_json::to_string_pretty(&json).unwrap());
    assert_eq!(loaded_selected, selected);
    assert!(state(&loaded) == state(&sim), "loaded differently");
}
