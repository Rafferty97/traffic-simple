//! The driver model: each driver's own traits, given at insertion, and the
//! state they vary with: the speed it wants relative to the limit, how its
//! acceleration wanders (dawdling), how far short of lines it stops, how it
//! weighs others when changing lanes, and its own random generator. Replaced
//! by editing this module.
//!
//! The vehicle, its dimensions and the longitudinal model's parameters for its
//! class, is apart, in [`VehicleSpec`]. The lane-change model copies the
//! traits it reads into its own state when a vehicle is inserted.

use slotmap::SecondaryMap;

use crate::vehicle::{VehicleId, VehicleSpec};

/// A driver's traits, given when its vehicle is inserted: how it varies from
/// other drivers. Its distribution is the caller's concern.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Driver {
    /// How fast the driver likes to go relative to the speed limit: 1 is exactly
    /// the limit. SUMO's `speedFactor`.
    pub speed_factor: f64,
    /// How much its acceleration wanders on the open road, in m/s², the
    /// standard deviation of a slowly varying noise added to its free-road
    /// term: drivers don't hold a speed exactly, as in Treiber and Kesting's
    /// IDM with acceleration noise (2017). 0 for none. Following stays exact.
    pub accel_noise: f64,
    /// How far short of a line the driver stops, nose to line, in meters, on
    /// average: each time it heads for a barrier, it draws how far, uniformly
    /// within [`Driver::line_gap_spread`] of this, so a row of vehicles at a
    /// line doesn't stand in perfect step.
    pub line_gap: f64,
    /// How far either way of [`Driver::line_gap`] its draws go, in meters.
    pub line_gap_spread: f64,
    /// The hardest the driver will brake to let another vehicle in ahead, in
    /// m/s²: 0 never yields to one indicating into its lane.
    pub courtesy: f64,
    /// How much the driver weighs other drivers' gains and losses against its own
    /// when changing lanes: MOBIL's politeness.
    pub politeness: f64,
    /// How readily the driver changes lanes, from 0 to 1: it intends a change
    /// for a smaller gain, and accepts harder braking, for itself and the
    /// vehicle it cuts in front of, before it grows frustrated. As SUMO's
    /// `lcAssertive` and `lcSpeedGain`. A few assertive drivers pull out into a
    /// faster lane and slow it, so a fast lane doesn't form.
    pub assertiveness: f64,
    /// Seeds the driver's own generator, for its dawdling and its gaps to
    /// lines, mixed with where it starts, so drivers placed alike still differ.
    pub seed: u64,
}

impl Default for Driver {
    /// A driver of textbook habits: the limit exactly, MOBIL's politeness of
    /// 0.2 as Treiber's simulator uses it, a little dawdling, and stopping
    /// about a metre short of lines.
    fn default() -> Self {
        Self {
            speed_factor: 1.0,
            accel_noise: 0.2,
            line_gap: 1.0,
            line_gap_spread: 0.5,
            courtesy: 1.0,
            politeness: 0.2,
            assertiveness: 0.0,
            seed: 0,
        }
    }
}

impl Driver {
    /// The speed it wants on a lane with the given limit, in `vehicle`, no
    /// faster than the vehicle goes.
    pub fn desired_speed(&self, vehicle: &VehicleSpec, speed_limit: f64) -> f64 {
        (speed_limit * self.speed_factor).min(vehicle.max_speed)
    }
}

/// A driver's traits and the state they vary with, kept beside its vehicle.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
struct State {
    traits: Driver,
    /// Its acceleration noise now, added to its free-road term, in m/s².
    noise: f64,
    /// The state of its own generator, for that noise and its gaps to lines.
    rng: u64,
    /// How far short of the barrier it is heading for it will stop, in meters,
    /// drawn afresh for each (see [`State::draw_line_gap`]).
    line_gap: f64,
}

impl State {
    /// Draws how far short of the next barrier it heads for the driver will
    /// stop, from its own generator, uniformly within its spread of its mean.
    fn draw_line_gap(&mut self) {
        let draw = crate::models::routing::splitmix64(&mut self.rng);
        let unit = (draw >> 11) as f64 / (1u64 << 53) as f64;
        let Driver { line_gap, line_gap_spread, .. } = self.traits;
        self.line_gap = (line_gap + line_gap_spread * (2.0 * unit - 1.0)).max(0.0);
    }
}

/// The driver model's state for every vehicle, which it owns.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub(crate) struct Model {
    drivers: SecondaryMap<VehicleId, State>,
}

impl Model {
    /// Takes on a vehicle just inserted, with its driver's traits, its
    /// generator starting at `seed`, and its gap to lines drawn.
    pub fn insert(&mut self, id: VehicleId, traits: Driver, seed: u64) {
        let mut state = State { traits, noise: 0.0, rng: seed, line_gap: traits.line_gap };
        state.draw_line_gap();
        self.drivers.insert(id, state);
    }

    /// Lets go of a vehicle removed.
    pub fn remove(&mut self, id: VehicleId) {
        self.drivers.remove(id);
    }

    /// The vehicle's driver's traits.
    pub fn traits(&self, id: VehicleId) -> &Driver {
        &self.drivers[id].traits
    }

    /// The vehicle's driver's traits, if there is such a vehicle.
    pub fn get(&self, id: VehicleId) -> Option<&Driver> {
        self.drivers.get(id).map(|state| &state.traits)
    }

    /// The speed the vehicle's driver wants on a lane with the given limit.
    pub fn desired_speed(&self, id: VehicleId, vehicle: &VehicleSpec, speed_limit: f64) -> f64 {
        self.drivers[id].traits.desired_speed(vehicle, speed_limit)
    }

    /// The vehicle's acceleration noise now, added to its free-road term.
    pub fn noise(&self, id: VehicleId) -> f64 {
        self.drivers[id].noise
    }

    /// How far short of the barrier it is heading for the vehicle will stop.
    pub fn line_gap(&self, id: VehicleId) -> f64 {
        self.drivers[id].line_gap
    }

    /// Heading for another barrier, the vehicle's driver draws how far short
    /// of it it will stop.
    pub fn new_barrier(&mut self, id: VehicleId) {
        self.drivers[id].draw_line_gap();
    }

    /// Moves the vehicle's acceleration noise on by `step`.
    pub fn advance_noise(&mut self, id: VehicleId, step: NoiseStep) {
        let state = &mut self.drivers[id];
        state.noise = noise(state.noise, state.traits.accel_noise, step, &mut state.rng);
    }
}

/// The time constant of a driver's acceleration noise, in seconds: how long it
/// takes to wander away from what it was (see [`Driver::accel_noise`]).
pub const NOISE_TIME: f64 = 10.0;

/// How an acceleration noise steps `dt` seconds on: how much of it is left,
/// and how much of its spread each step's kick carries, so it wanders with
/// that spread. Worked out once per update, not per driver.
#[derive(Clone, Copy, Debug)]
pub struct NoiseStep {
    decay: f64,
    kick: f64,
}

impl NoiseStep {
    pub fn new(dt: f64) -> Self {
        let decay = (-dt / NOISE_TIME).exp();
        Self { decay, kick: (1.0 - decay * decay).sqrt() }
    }
}

/// A driver's acceleration noise one `step` on, an Ornstein-Uhlenbeck process:
/// it decays toward zero with [`NOISE_TIME`] and is kicked by a draw with mean
/// 0 and deviation 1, so it wanders with standard deviation `spread`. The draw
/// is the sum of four uniform ones, scaled: near enough normal, and cheaper.
/// `rng` is the driver's own SplitMix64 state, so the noise is the same every
/// run.
pub fn noise(previous: f64, spread: f64, step: NoiseStep, rng: &mut u64) -> f64 {
    if spread <= 0.0 {
        return 0.0;
    }
    let mut sum = 0.0;
    for _ in 0..4 {
        sum += (crate::models::routing::splitmix64(rng) >> 11) as f64 / (1u64 << 53) as f64;
    }
    // Four uniforms have mean 2 and variance 1/3.
    let draw = (sum - 2.0) * 3f64.sqrt();
    previous * step.decay + spread * step.kick * draw
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn acceleration_noise_wanders_with_its_spread() {
        let (mut rng, mut xi) = (7u64, 0.0);
        let (mut sum, mut squares, n) = (0.0, 0.0, 200_000);
        let step = NoiseStep::new(0.05);
        for _ in 0..n {
            xi = noise(xi, 0.2, step, &mut rng);
            sum += xi;
            squares += xi * xi;
        }
        let mean = sum / n as f64;
        let sd = (squares / n as f64 - mean * mean).sqrt();
        assert!(mean.abs() < 0.03, "{mean}");
        assert!((sd - 0.2).abs() < 0.03, "{sd}");
        assert_eq!(noise(0.5, 0.0, step, &mut rng), 0.0);
    }

    #[test]
    fn line_gaps_are_drawn_within_their_spread() {
        let traits = Driver { line_gap: 1.0, line_gap_spread: 0.5, ..Driver::default() };
        let mut model = Model::default();
        let mut keys = slotmap::SlotMap::<VehicleId, ()>::with_key();
        let id = keys.insert(());
        model.insert(id, traits, 3);
        let gaps: Vec<f64> = (0..200)
            .map(|_| {
                model.new_barrier(id);
                model.line_gap(id)
            })
            .collect();
        assert!(gaps.iter().all(|g| (0.5..=1.5).contains(g)));
        assert!(gaps.iter().any(|&g| g < 0.7) && gaps.iter().any(|&g| g > 1.3));
    }
}
