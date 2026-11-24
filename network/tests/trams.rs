//! Trams on the tracks down the middle of the hook-turn crossroads, going
//! straight through on their road's lights, among its cars.

use traffic_network::{compile, scenarios};
use traffic_sim::{Driver, LaneId, Placement, Route, Simulation, VehicleClass, VehicleSpec};

mod common;

const SPEED: f64 = 40.0 / 3.6;

#[test]
fn trams_cross_the_hook_turn_crossroads_among_cars_without_touching_them() {
    let compiled = compile(&scenarios::hook_turn_crossroads(SPEED)).unwrap();
    let mut sim = Simulation::with_config(compiled.sim.clone(), common::CONFIG);
    let (trams, cars): (Vec<LaneId>, Vec<LaneId>) = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .partition(|&id| !compiled.sim.lane(id).allows().contains(VehicleClass::Car));
    assert_eq!(trams.len(), 4, "a track in from each road");
    let mut state = 3u64;
    let mut uniform = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let (mut tram_ids, mut credit) = (Vec::new(), 0.0);
    for step in 0..common::steps(900.0) {
        // A tram on each track every minute, cars at 150 an hour a lane.
        if step % common::steps(60.0) == 0 {
            for &lane in &trams {
                let at = Placement { lane, s: 5.0, speed: SPEED, route: Route::random(step) };
                let driver = Driver { accel_noise: 0.0, seed: step, ..Driver::default() };
                tram_ids.extend(sim.try_insert(VehicleSpec::tram(), driver, at).ok());
            }
        }
        credit += 150.0 / 3600.0 * cars.len() as f64 * common::DT;
        if credit >= 1.0 {
            let lane = cars[(uniform() * cars.len() as f64) as usize];
            let seed = (uniform() * 1e9) as u64;
            let at = Placement { lane, s: 5.0, speed: SPEED, route: Route::random(seed) };
            let driver = Driver { seed, ..Driver::default() };
            if sim.try_insert(VehicleSpec::passenger_car(), driver, at).is_ok() {
                credit -= 1.0;
            }
        }
        sim.step();
        // Trams stay on lanes only trams may use, and never touch another
        // vehicle; cars never take a track.
        if step % common::steps(0.1) == 0 {
            let tracks =
                |lane: LaneId| !sim.network().lane(lane).allows().contains(VehicleClass::Car);
            for (_, v) in sim.vehicles() {
                assert_eq!(tracks(v.lane()), v.spec().class == VehicleClass::Tram);
            }
            for &tram in &tram_ids {
                let Some(t) = sim.get(tram) else { continue };
                for (other, v) in sim.vehicles() {
                    if other != tram && common::overlap(t.corners(), v.corners()) {
                        panic!(
                            "at {:.1} s, tram {} touches {}",
                            sim.time(),
                            tram.index(),
                            other.index()
                        );
                    }
                }
            }
        }
    }
    let through = tram_ids.iter().filter(|&&id| sim.get(id).is_none()).count();
    assert!(
        through >= 40,
        "{through} of {} trams through",
        tram_ids.len()
    );
}
