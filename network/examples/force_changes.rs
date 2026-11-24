//! Experiment: forces lane changes on random vehicles in busy traffic, in a
//! debug build, to catch broken occupancy invariants.
use traffic_network::{compile, scenarios};
use traffic_sim::{Driver, LaneId, Placement, Route, Side, Simulation, VehicleSpec};

fn main() {
    let mut state = 7u64;
    let mut uniform = || {
        state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        ((z ^ (z >> 31)) >> 11) as f64 / (1u64 << 53) as f64
    };
    let net = match std::env::args().nth(1).as_deref() {
        Some("ring") => scenarios::ring(1000.0, 3, 14.0),
        _ => scenarios::ramps(100.0 / 3.6, 80.0 / 3.6),
    };
    let compiled = compile(&net).unwrap();
    let mut sim = Simulation::new(compiled.sim);
    let entries: Vec<LaneId> = sim
        .network()
        .lanes()
        .filter(|(_, l)| l.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let lanes = sim.network().lanes().count();
    for step in 0..(600 * 60) {
        if step % 10 == 0 {
            let car = VehicleSpec {
                length: 4.6,
                width: 1.9,
                max_speed: 40.0,
                max_accel: 1.0,
                comfy_decel: 1.5,
                time_headway: 1.0,
                min_gap: 1.5,
                rear_axle: 1.0,
                front_axle: 3.7,
            };
            let driver = Driver {
                speed_factor: 1.0,
                courtesy: 1.0,
                politeness: 0.2,
                assertiveness: 0.3,
                accel_noise: 0.0,
                line_gap: 1.0,
                line_gap_spread: 0.0,
                seed: step,
            };
            let at = if entries.is_empty() {
                let lane = LaneId((uniform() * lanes as f64) as u32);
                Placement {
                    lane,
                    s: uniform() * 900.0,
                    speed: 10.0,
                    route: Route::random(step),
                }
            } else {
                let lane = entries[(uniform() * entries.len() as f64) as usize];
                Placement { lane, s: 5.0, speed: 25.0, route: Route::random(step) }
            };
            if !entries.is_empty() || sim.vehicles().count() < 200 {
                let _ = sim.try_insert(car, driver, at);
            }
        }
        if step % 30 == 0 {
            let ids: Vec<_> = sim.vehicles().map(|(id, _)| id).collect();
            if !ids.is_empty() {
                let id = ids[(uniform() * ids.len() as f64) as usize];
                let side = if uniform() < 0.5 {
                    Side::Left
                } else {
                    Side::Right
                };
                let lane = sim.get(id).unwrap().lane();
                if sim.change_lane(id, side).is_ok() {
                    eprintln!("t {:.2} forced {id:?} {side:?} from {lane:?}", sim.time());
                }
            }
        }
        let before = sim.time();
        sim.step_variable(1.0 / 60.0);
        for (id, v) in sim.vehicles() {
            let lane = v.lane();
            // On a loop, a second record lingers at the far end: not its own.
            let half = 0.5 * sim.network().lane(lane).geometry().length();
            let rear = v.lane_pos().s - v.spec().rear_axle;
            let n = sim
                .occupants(lane)
                .iter()
                .filter(|o| o.is_record_of(id) && (o.s() - rear).abs() <= half)
                .count();
            if n > 1 {
                let records: Vec<_> = (0..lanes as u32)
                    .flat_map(|l| {
                        sim.occupants(LaneId(l))
                            .iter()
                            .filter(move |o| o.id == id)
                            .map(move |o| (l, o.s(), o.lateral))
                    })
                    .collect();
                eprintln!(
                    "at {before:.2}: {id:?} follows {lane:?} s {:.1} path {:?} change {:?} old {:?} records {records:?}",
                    v.lane_pos().s,
                    v.path(),
                    v.lane_change().map(|c| c.s0),
                    v.old_side()
                );
                std::process::exit(1);
            }
        }
    }
    eprintln!("no panic");
}
