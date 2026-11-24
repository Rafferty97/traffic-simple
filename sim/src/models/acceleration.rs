//! The longitudinal model: IDM+ (Schakel, van Arem and Netten, 2010).
//!
//! IDM+ splits the Intelligent Driver Model's acceleration into a free-road term
//! and an interaction term and takes the minimum, rather than summing them as
//! IDM does. Each term is a separate function here, so the simulation can take
//! the minimum over the free-road term and one interaction term per leader.

use slotmap::SecondaryMap;

use crate::motion::lateral::halfway_from_rest;
use crate::network::SimNetwork;
use crate::vehicle::{Vehicle, VehicleId, VehicleSpec};

/// The acceleration exponent δ.
const DELTA: i32 = 4;

/// The hardest a vehicle can brake, in m/s². Caps both terms, which are
/// unbounded below when a vehicle is far over its desired speed or almost
/// touching its leader.
pub const MAX_BRAKING: f64 = 9.0;

/// The hardest braking, in m/s², that a driver will accept being forced into by
/// another vehicle's maneuver, such as a lane change in front of it or a vehicle
/// inserted ahead of it. MOBIL's `b_safe`; 4 m/s² is Treiber's default.
pub const SAFE_BRAKING: f64 = 4.0;

/// A driver's own state under the longitudinal model, kept beside its vehicle.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
struct Driver {
    /// Its acceleration noise now, added to its free-road term, in m/s² (see
    /// [`VehicleSpec::accel_noise`]).
    pub noise: f64,
    /// The state of its own generator, for that noise and its gaps to lines.
    pub rng: u64,
    /// How far short of the barrier it is heading for it will stop, in meters,
    /// drawn afresh for each (see [`draw_line_gap`]).
    pub line_gap: f64,
    /// Whether it is finishing a stop, braking at least firmly enough to come
    /// to rest promptly, or staying at rest, until it can move off properly
    /// (see [`stopping`]).
    pub stopping: bool,
}

impl Driver {
    /// A driver whose generator starts at `seed`, its gap to lines drawn.
    fn new(spec: &VehicleSpec, seed: u64) -> Self {
        let mut driver = Self {
            noise: 0.0,
            rng: seed,
            line_gap: spec.line_gap,
            stopping: false,
        };
        draw_line_gap(&mut driver, spec);
        driver
    }
}

/// The longitudinal model's state for every vehicle, which it owns: each
/// driver's [`Driver`], kept beside its vehicle.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub(crate) struct Model {
    drivers: SecondaryMap<VehicleId, Driver>,
}

impl Model {
    /// Takes on a vehicle just inserted, its generator starting at `seed`.
    pub fn insert(&mut self, id: VehicleId, spec: &VehicleSpec, seed: u64) {
        self.drivers.insert(id, Driver::new(spec, seed));
    }

    /// Lets go of a vehicle removed.
    pub fn remove(&mut self, id: VehicleId) {
        self.drivers.remove(id);
    }

    /// The vehicle's acceleration noise now, added to its free-road term.
    pub fn noise(&self, id: VehicleId) -> f64 {
        self.drivers[id].noise
    }

    /// How far short of the barrier it is heading for the vehicle will stop.
    pub fn line_gap(&self, id: VehicleId) -> f64 {
        self.drivers[id].line_gap
    }

    /// Whether the vehicle is finishing a stop (see [`stopping`]).
    pub fn stopping(&self, id: VehicleId) -> bool {
        self.drivers[id].stopping
    }

    /// Heading for another barrier, the vehicle draws how far short of it it
    /// will stop (see [`draw_line_gap`]).
    pub fn new_barrier(&mut self, id: VehicleId, spec: &VehicleSpec) {
        draw_line_gap(&mut self.drivers[id], spec);
    }

    /// Moves the vehicle's acceleration noise on by `step`.
    pub fn advance_noise(&mut self, id: VehicleId, spec: &VehicleSpec, step: NoiseStep) {
        let driver = &mut self.drivers[id];
        driver.noise = noise(driver.noise, spec.accel_noise, step, &mut driver.rng);
    }

    /// Whether the vehicle, at `speed`, with every other term giving `accel`,
    /// is finishing a stop now, and the term that finishes it if so (see
    /// [`stopping`]).
    pub fn finish_stop(&mut self, id: VehicleId, speed: f64, accel: f64) -> Option<f64> {
        let driver = &mut self.drivers[id];
        driver.stopping = stopping(driver.stopping, speed, accel);
        driver.stopping.then_some(-STOPPING_DECEL)
    }
}

/// The room a vehicle that wants to change lanes keeps behind its leaders, so
/// it isn't boxed in: as far as its rear axle needs to get halfway through a
/// change from rest. Its leader search along its own lanes looks at least that
/// far, past its minimum gap. `None` if it doesn't want to change.
pub(crate) fn kept_room(network: &SimNetwork, vehicle: &Vehicle) -> Option<f64> {
    vehicle.intent.wants_change().then(|| halfway_from_rest(network, vehicle))
}

/// Draws how far short of the next barrier it heads for a driver will stop,
/// from its own generator, uniformly within its spec's spread of its mean.
fn draw_line_gap(driver: &mut Driver, spec: &VehicleSpec) {
    let draw = crate::models::routing::splitmix64(&mut driver.rng);
    let unit = (draw >> 11) as f64 / (1u64 << 53) as f64;
    driver.line_gap = (spec.line_gap + spec.line_gap_spread * (2.0 * unit - 1.0)).max(0.0);
}

/// The term holding a vehicle `gap` short of a line: a stationary leader there,
/// with its gap to lines, `line_gap`, as the minimum gap.
pub(crate) fn hold(spec: &VehicleSpec, line_gap: f64, speed: f64, gap: f64) -> f64 {
    let at_line = VehicleSpec { min_gap: line_gap, ..*spec };
    interaction(&at_line, speed, gap, 0.0)
}

/// Free-road term: `a · (1 − (v / v0)^δ)` up to the desired speed. Above it,
/// as Treiber's Improved IDM (IIDM) has it, `−b · (1 − (v0 / v)^(a·δ / b))`:
/// it slows toward the desired speed at no more than its comfortable
/// deceleration, where IDM's term would brake as hard as it could well above
/// it, as when the speed limit or the traffic beside it drops.
pub fn free_road(spec: &VehicleSpec, speed: f64, desired_speed: f64) -> f64 {
    if speed <= desired_speed {
        return spec.max_accel * (1.0 - (speed / desired_speed).powi(DELTA));
    }
    let exponent = spec.max_accel * f64::from(DELTA) / spec.comfy_decel;
    -spec.comfy_decel * (1.0 - (desired_speed / speed).powf(exponent))
}

/// The term for a lower speed limit `gap` ahead of the vehicle's front, where
/// it would go no faster than `limit`: braking to be down to it there,
/// `(limit² − v²) / 2·gap`, once that is harsher than its comfortable
/// deceleration, and never harsher than twice that, so it slows ahead of the
/// limit rather than once on it, and brakes at about its comfortable
/// deceleration as it does. Nothing before then.
pub fn limit_ahead(spec: &VehicleSpec, speed: f64, gap: f64, limit: f64) -> Option<f64> {
    let needed = (speed * speed - limit * limit) / (2.0 * gap.max(0.1));
    (needed > spec.comfy_decel).then(|| -needed.min(2.0 * spec.comfy_decel))
}

/// How far ahead of its front a lower speed limit can matter to a vehicle: any
/// further, slowing to a stop there takes no more than comfortable braking.
pub fn limit_reach(spec: &VehicleSpec, speed: f64) -> f64 {
    speed * speed / (2.0 * spec.comfy_decel)
}

/// The time constant of a driver's acceleration noise, in seconds: how long it
/// takes to wander away from what it was (see [`VehicleSpec::accel_noise`]).
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

/// The desired gap behind a leader going at `leader_speed`:
/// `s* = s0 + max(0, v·T + v·Δv / (2·√(a·b)))`.
fn desired_gap(spec: &VehicleSpec, speed: f64, leader_speed: f64) -> f64 {
    let approach = speed - leader_speed;
    let dynamic = speed * spec.time_headway
        + speed * approach / (2.0 * (spec.max_accel * spec.comfy_decel).sqrt());
    spec.min_gap + dynamic.max(0.0)
}

/// Interaction term for one leader: `a · (1 − (s* / s)²)`, with `s*` the
/// [`desired_gap`].
///
/// `gap` is bumper to bumper, and `leader_speed` is the leader's speed.
pub fn interaction(spec: &VehicleSpec, speed: f64, gap: f64, leader_speed: f64) -> f64 {
    // A gap of zero or less means the vehicles touch: brake as hard as possible.
    if gap <= 0.0 {
        return -MAX_BRAKING;
    }
    let desired_gap = desired_gap(spec, speed, leader_speed);
    let accel = spec.max_accel * (1.0 - (desired_gap / gap).powi(2));
    accel.max(-MAX_BRAKING)
}

/// The share of its time headway a vehicle keeps to a leader that has just
/// appeared in front of it, or behind which it has just appeared: by changing
/// lanes or merging.
/// LMRS's `T_min / T_max`, 0.56 s over 1.2 s.
pub const RELAXED_HEADWAY: f64 = 0.56 / 1.2;

/// LMRS's relaxation time `τ`, in seconds: the time headway relaxes back from
/// [`RELAXED_HEADWAY`] toward the whole of it exponentially, with this time
/// constant.
pub const RELAXATION_TIME: f64 = 25.0;

/// The age of a record long settled, whose headway has relaxed fully: an
/// inserted vehicle's, and that of a record from before records had one.
pub const SETTLED: f64 = 1.0e9;

/// The share of its time headway a vehicle keeps to a leader, from how long
/// the newer of the two has been in the lane: relaxation, as in Schakel, Knoop
/// and van Arem's LMRS (2012), so a vehicle that cuts in, or is cut in on,
/// accepts a shorter gap at first and drops back gradually.
pub fn relaxation(age: f64) -> f64 {
    1.0 - (1.0 - RELAXED_HEADWAY) * (-age.max(0.0) / RELAXATION_TIME).exp()
}

/// As [`interaction`], keeping only `headway`, a share, of its time headway.
pub fn relaxed(spec: &VehicleSpec, speed: f64, gap: f64, leader_speed: f64, headway: f64) -> f64 {
    let spec = VehicleSpec { time_headway: spec.time_headway * headway, ..*spec };
    interaction(&spec, speed, gap, leader_speed)
}

/// The least distance, in meters, a vehicle looks ahead for leaders: at low
/// speed, the desired gap to a stopped leader shrinks to the minimum gap, where
/// a queue comes to rest, and a leader there would flicker in and out of view.
const MIN_HORIZON: f64 = 5.0;

/// How far ahead of its front bumper a vehicle looks for leaders, in meters:
/// `s*`, the desired gap approaching a stopped leader, where that leader's term
/// is zero, or [`MIN_HORIZON`] if that is further.
///
/// A stopped leader at gap `s` gives `a · (1 − (s* / s)²)`, so one coming into
/// view at `s*` changes nothing unless the free road allows acceleration, and
/// then it stops accelerating: the point at which a driver lifts off and starts
/// braking gently. The desired gap behind a leader at the vehicle's own speed,
/// `s0 + v·T`, is shorter than `s*` by `v² / (2·√(a·b))`, so a leader followed at
/// steady state is in view, and the floor keeps it clearly so at low speed. The
/// horizon grows with speed, as `s*` does.
pub fn horizon(spec: &VehicleSpec, speed: f64) -> f64 {
    desired_gap(spec, speed, 0.0).max(MIN_HORIZON)
}

/// How quickly a vehicle eases up to a speed imposed on it from below, in
/// seconds: the time constant of its approach.
const CAP_EASE: f64 = 0.5;

/// The soonest a vehicle at `speed` can cover `distance`, in seconds,
/// accelerating at `accel` up to `max_speed` and holding it: `(√(v² + 2ad) − v) / a`,
/// or, once at its top speed, `t₁ + (d − d₁) / v_max`, with `t₁` and `d₁` the
/// time and distance to reach it. Zero for no distance; one already above its
/// top speed is held to its speed. Braking, with `accel` negative, or holding
/// its speed, it never covers more than it does before it stops: infinite.
pub fn reach_time(speed: f64, accel: f64, max_speed: f64, distance: f64) -> f64 {
    if distance <= 0.0 {
        return 0.0;
    }
    if accel <= 0.0 {
        let left = speed * speed + 2.0 * accel * distance;
        return match accel == 0.0 {
            true if speed > 0.0 => distance / speed,
            false if left > 0.0 => (left.sqrt() - speed) / accel,
            _ => f64::INFINITY,
        };
    }
    let max_speed = max_speed.max(speed);
    let to_top = (max_speed - speed) / accel;
    let over = 0.5 * (speed + max_speed) * to_top;
    if distance <= over {
        ((speed * speed + 2.0 * accel * distance).sqrt() - speed) / accel
    } else {
        to_top + (distance - over) / max_speed
    }
}

/// Below this speed, in m/s (3 km/h), a vehicle that is slowing starts
/// finishing its stop (see `stopping`).
pub const STOPPING_SPEED: f64 = 3.0 / 3.6;

/// The least a vehicle finishing its stop brakes, in m/s².
pub const STOPPING_DECEL: f64 = 0.05;

/// The acceleration a vehicle finishing its stop, or at rest, must be able to
/// manage to move off again, in m/s².
pub const MOVING_OFF: f64 = 0.3;

/// Whether a vehicle is finishing a stop, given whether it was (`stopping`),
/// its speed, and its acceleration from every other term. It starts once it
/// is slowing below [`STOPPING_SPEED`], and goes on until it can accelerate at
/// [`MOVING_OFF`]; meanwhile it brakes at least at [`STOPPING_DECEL`], or harder
/// if it must, one more term. So it comes to rest promptly, as drivers do,
/// rather than easing toward rest ever more slowly, as IDM approaching a
/// stationary leader does, never quite reaching it; it doesn't creep on after
/// a leader that edges forward, but waits until it can move off properly, as
/// a driver at a light turning green takes a moment to; and, starting and
/// ending at different accelerations, it doesn't flick between the two.
pub fn stopping(stopping: bool, speed: f64, accel: f64) -> bool {
    match stopping {
        true => accel <= MOVING_OFF,
        false => speed < STOPPING_SPEED && accel < 0.0,
    }
}

/// The acceleration a speed cap allows: braking at [`SAFE_BRAKING`] while above
/// it, so a held vehicle stops promptly, easing up to it from below, and with a
/// cap of 0 braking until it stops. Like any term, it only binds when it is the
/// lowest.
pub fn capped(speed: f64, max_speed: f64) -> f64 {
    if max_speed <= 0.0 {
        -SAFE_BRAKING
    } else {
        ((max_speed - speed) / CAP_EASE).max(-SAFE_BRAKING)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn reach_time_accelerates_then_holds_its_top_speed() {
        // As the old prototype's tests have it.
        assert!((reach_time(0.0, 2.0, 50.0, 25.0) - 5.0).abs() < 1e-9);
        assert!((reach_time(0.0, 2.0, 10.0, 25.0) - 5.0).abs() < 1e-9);
        assert!((reach_time(0.0, 2.0, 9.0, 25.0) - 5.027777777777).abs() < 1e-9);
        assert!((reach_time(5.0, 2.0, 15.0, 50.0) - 5.0).abs() < 1e-9);
        assert!((reach_time(5.0, 2.0, 14.0, 50.0) - 5.01785714285).abs() < 1e-9);
        assert_eq!(reach_time(5.0, 2.0, 14.0, -1.0), 0.0);
    }

    fn spec() -> VehicleSpec {
        VehicleSpec {
            length: 4.6,
            width: 1.9,
            max_speed: 30.0,
            speed_factor: 1.0,
            max_accel: 1.5,
            comfy_decel: 2.0,
            time_headway: 1.2,
            min_gap: 2.0,
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

    #[test]
    fn full_accel_from_rest() {
        assert_eq!(free_road(&spec(), 0.0, 14.0), 1.5);
    }

    #[test]
    fn no_accel_at_desired_speed() {
        assert_eq!(free_road(&spec(), 14.0, 14.0), 0.0);
    }

    #[test]
    fn no_accel_at_equilibrium_gap() {
        // Same speed as the leader, at exactly s0 + v·T.
        let (v, gap) = (10.0, 2.0 + 10.0 * 1.2);
        assert!(interaction(&spec(), v, gap, v).abs() < 1e-12);
    }

    #[test]
    fn brakes_when_closing_in() {
        let (v, gap) = (10.0, 14.0);
        assert!(interaction(&spec(), v, gap, 5.0) < interaction(&spec(), v, gap, v));
        assert!(interaction(&spec(), v, gap, 5.0) < 0.0);
    }

    #[test]
    fn approaches_max_accel_far_from_leader() {
        assert!(interaction(&spec(), 10.0, 1e6, 10.0) > 1.5 - 1e-6);
    }

    #[test]
    fn a_stopped_leader_at_the_horizon_doesnt_brake() {
        let spec = spec();
        for speed in [0.0, 1.0, 2.0, 5.0, 14.0, 30.0] {
            let horizon = horizon(&spec, speed);
            let at = interaction(&spec, speed, horizon, 0.0);
            assert!(at > -1e-9, "at {speed} m/s: {at}");
            // A leader followed at the desired gap, at the same speed, is in view.
            let following = desired_gap(&spec, speed, speed);
            assert!(following < horizon - 0.5, "at {speed} m/s");
        }
        // At speed, it is where a stopped leader's term is zero.
        let at = interaction(&spec, 30.0, horizon(&spec, 30.0), 0.0);
        assert!(at.abs() < 1e-9);
        assert_eq!(horizon(&spec, 0.0), MIN_HORIZON);
        assert!(horizon(&spec, 30.0) > horizon(&spec, 14.0));
    }

    #[test]
    fn the_headway_relaxes_from_lmrs_minimum_to_whole() {
        assert!((relaxation(0.0) - RELAXED_HEADWAY).abs() < 1e-12);
        assert!(relaxation(RELAXATION_TIME / 2.0) > RELAXED_HEADWAY);
        // Most of the way back after one time constant, all of it once settled.
        let one = 1.0 - (1.0 - RELAXED_HEADWAY) / std::f64::consts::E;
        assert!((relaxation(RELAXATION_TIME) - one).abs() < 1e-12);
        assert_eq!(relaxation(SETTLED), 1.0);
        // A relaxed follower brakes less behind the same leader.
        let spec = spec();
        assert!(relaxed(&spec, 20.0, 15.0, 20.0, 0.5) > interaction(&spec, 20.0, 15.0, 20.0));
        assert_eq!(
            relaxed(&spec, 20.0, 15.0, 20.0, 1.0),
            interaction(&spec, 20.0, 15.0, 20.0)
        );
    }

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
    fn braking_is_capped() {
        assert_eq!(interaction(&spec(), 30.0, 0.5, 0.0), -MAX_BRAKING);
        assert_eq!(interaction(&spec(), 30.0, -1.0, 0.0), -MAX_BRAKING);
        // Above its desired speed, it slows no harder than comfortably.
        let over = free_road(&spec(), 30.0, 10.0);
        assert!(over < 0.0 && over >= -spec().comfy_decel, "{over}");
        assert!(free_road(&spec(), 10.5, 10.0) > over);
    }

    #[test]
    fn a_speed_cap_brakes_above_it_and_eases_up_to_it() {
        assert_eq!(capped(14.0, 5.0), -SAFE_BRAKING, "well above: safe braking");
        assert!(capped(5.2, 5.0) < 0.0 && capped(5.2, 5.0) > -SAFE_BRAKING);
        assert!(
            capped(4.0, 5.0) > 0.0,
            "below it, it doesn't hold the vehicle back"
        );
        assert_eq!(capped(0.0, 0.0), -SAFE_BRAKING, "a cap of 0 stops it");
    }
}
