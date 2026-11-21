//! Records every call that changes the simulation, so a session can be saved and
//! replayed exactly: the simulation is deterministic, so the same calls in the
//! same order reproduce the same state, bit for bit. `sim/tests/common/replay.rs`
//! reads the format back.
//!
//! The format is one call per line. Vehicles are named by their index in the
//! order they were inserted, counting only insertions that succeeded, and floats
//! are written so they parse back exactly. Lines starting with `#` are comments.
//!
//! The first line is the network, as JSON, as a snapshot holds it, so a replay
//! doesn't depend on whatever built it. Each lane's geometry is a short string
//! of millimeters (see `LaneGeometry::to_mm_string`).
//!
//! ```text
//! # traffic-sim replay v3
//! network {"lanes":[{"geometry":"0,0,1750 100000,0", ...}], ...}
//! step 0.016666666666666666
//! try_insert ok length=4.6 width=1.9 ... lane=2 s=255.4 speed=14.18 seed=3
//! cap 3 0.0
//! cap 4 5.555555555555555
//! change_lane 7 left refused
//! ```

use std::collections::HashMap;
use std::fmt::Write;
use std::ops::Deref;

use traffic_sim::{
    DebugSink, InsertError, LaneChangeError, Placement, Route, Side, Simulation, Vehicle,
    VehicleId, VehicleSpec,
};

/// A simulation that logs every call that changes it.
///
/// It derefs to the simulation for reading, but never hands out `&mut`, so a
/// change that isn't logged doesn't compile.
pub struct Recorded<D: DebugSink> {
    sim: Simulation<D>,
    log: String,
    /// Each inserted vehicle's index in insertion order, which is how the log names it.
    index: HashMap<VehicleId, usize>,
}

impl<D: DebugSink> Deref for Recorded<D> {
    type Target = Simulation<D>;

    fn deref(&self) -> &Simulation<D> {
        &self.sim
    }
}

impl<D: DebugSink> Recorded<D> {
    /// Starts recording a new simulation, from its network.
    pub fn new(sim: Simulation<D>) -> Self {
        let network = serde_json::to_string(sim.network()).expect("a network serializes");
        let log = format!("# traffic-sim replay v3\nnetwork {network}\n");
        Self { sim, log, index: HashMap::new() }
    }

    /// Advances by a frame's `dt`, as [`Simulation::step_variable`], logged as
    /// `step`, and returns how many fixed steps it took.
    pub fn step_variable(&mut self, dt: f64) -> u32 {
        writeln!(self.log, "step {dt:?}").unwrap();
        self.sim.step_variable(dt)
    }

    pub fn try_insert(
        &mut self,
        spec: VehicleSpec,
        at: Placement,
    ) -> Result<VehicleId, InsertError> {
        let result = self.sim.try_insert(spec, at);
        let outcome = if result.is_ok() { "ok" } else { "refused" };
        // Destructured, so a new field fails to compile until it is logged.
        let VehicleSpec {
            length,
            width,
            max_speed,
            speed_factor,
            max_accel,
            comfy_decel,
            time_headway,
            min_gap,
            rear_axle,
            front_axle,
            courtesy,
            politeness,
            assertiveness,
            accel_noise,
            line_gap,
            line_gap_spread,
        } = spec;
        let Placement { lane, s, speed, route } = at;
        let Route::Random { state: seed } = route;
        writeln!(
            self.log,
            "try_insert {outcome} length={length:?} width={width:?} max_speed={max_speed:?} \
             speed_factor={speed_factor:?} max_accel={max_accel:?} comfy_decel={comfy_decel:?} \
             time_headway={time_headway:?} min_gap={min_gap:?} rear_axle={rear_axle:?} \
             front_axle={front_axle:?} courtesy={courtesy:?} politeness={politeness:?} \
             assertiveness={assertiveness:?} accel_noise={accel_noise:?} \
             line_gap={line_gap:?} line_gap_spread={line_gap_spread:?} lane={} s={s:?} speed={speed:?} seed={seed}",
            lane.0
        )
        .unwrap();
        if let Ok(id) = result {
            let next = self.index.len();
            self.index.insert(id, next);
        }
        result
    }

    pub fn remove(&mut self, id: VehicleId) -> Option<Vehicle> {
        writeln!(self.log, "remove {}", self.index[&id]).unwrap();
        self.sim.remove(id)
    }

    pub fn cap_speed(&mut self, id: VehicleId, max_speed: f64) {
        writeln!(self.log, "cap {} {max_speed:?}", self.index[&id]).unwrap();
        self.sim.cap_speed(id, max_speed);
    }

    pub fn release(&mut self, id: VehicleId) {
        writeln!(self.log, "release {}", self.index[&id]).unwrap();
        self.sim.release(id);
    }

    pub fn change_lane(&mut self, id: VehicleId, side: Side) -> Result<(), LaneChangeError> {
        let result = self.sim.change_lane(id, side);
        let side = match side {
            Side::Left => "left",
            Side::Right => "right",
        };
        let outcome = if result.is_ok() { "ok" } else { "refused" };
        writeln!(self.log, "change_lane {} {side} {outcome}", self.index[&id]).unwrap();
        result
    }

    /// The log so far, with comments saying when it was captured and which
    /// vehicle was selected.
    pub fn capture(&self, selected: Option<VehicleId>) -> String {
        let mut text = self.log.clone();
        writeln!(text, "# captured at time={:?}", self.sim.time()).unwrap();
        if let Some(index) = selected.and_then(|id| self.index.get(&id)) {
            writeln!(text, "# selected={index}").unwrap();
        }
        text
    }
}

#[cfg(test)]
mod test {
    use rand::rngs::Xoshiro256PlusPlus;
    use rand::{RngExt, SeedableRng};
    use traffic_sim::{LaneId, NoopSink};

    use super::*;
    use crate::common::{car, ring_road};
    use crate::replay_reader::replay;

    /// Every vehicle's state and every lane's records, as raw bits.
    fn state<D: DebugSink>(sim: &Simulation<D>) -> Vec<u64> {
        let mut out = vec![sim.time().to_bits()];
        for (_, v) in sim.vehicles() {
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
            out.push(v.intent().is_some() as u64);
        }
        for (lane, _) in sim.network().lanes() {
            for o in sim.occupants(lane) {
                out.extend([o.s().to_bits(), o.is_ghost() as u64]);
            }
        }
        out
    }

    #[test]
    fn a_capture_replays_exactly() {
        let speed_limit = 50.0 / 3.6;
        let network = ring_road(600.0, 3, speed_limit);
        let mut sim = Recorded::new(Simulation::new(network, NoopSink));
        let mut rng = Xoshiro256PlusPlus::seed_from_u64(7);
        let mut ids = Vec::new();
        while sim.time() < 90.0 {
            sim.step_variable(0.005 + 0.03 * rng.random::<f64>());
            let roll = rng.random::<f64>();
            if roll < 0.05 {
                let spec = VehicleSpec { speed_factor: 0.8 + 0.4 * rng.random::<f64>(), ..car() };
                let s = 600.0 * rng.random::<f64>();
                let at = Placement {
                    lane: LaneId(2),
                    s,
                    speed: spec.desired_speed(speed_limit),
                    route: Route::random(rng.random()),
                };
                ids.extend(sim.try_insert(spec, at).ok());
            } else if roll < 0.07 && !ids.is_empty() {
                let id = ids[rng.random_range(0..ids.len())];
                if sim.get(id).is_some() {
                    match rng.random_range(0..4) {
                        0 => sim.cap_speed(id, rng.random_range(0..2) as f64 * 5.0),
                        1 => sim.release(id),
                        2 => drop(sim.change_lane(id, Side::Left)),
                        _ => drop(sim.remove(id)),
                    }
                }
            }
        }
        let selected = sim.vehicles().next().map(|(id, _)| id);
        let replayed = replay(&sim.capture(selected), NoopSink);
        assert!(sim.vehicles().count() > 10, "the run should be busy");
        assert!(state(&sim) == state(&replayed.sim), "replay diverged");
        assert_eq!(replayed.selected, selected);
    }
}
