//! The longitudinal model: IDM+ (Schakel, van Arem and Netten, 2010).
//!
//! IDM+ splits the Intelligent Driver Model's acceleration into a free-road term
//! and an interaction term and takes the minimum, rather than summing them as
//! IDM does. Each term is a separate function here, so the simulation can take
//! the minimum over the free-road term and one interaction term per leader.

use crate::vehicle::VehicleSpec;

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

/// Free-road term: `a · (1 − (v / v0)^δ)`.
pub fn free_road(spec: &VehicleSpec, speed: f64, desired_speed: f64) -> f64 {
    let accel = spec.max_accel * (1.0 - (speed / desired_speed).powi(DELTA));
    accel.max(-MAX_BRAKING)
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

/// The most a leader beyond a vehicle's [`horizon`] can lower its acceleration,
/// in m/s²: so a leader coming into view never changes it by more.
pub const NEGLIGIBLE: f64 = 0.1;

/// How far ahead of its front bumper a vehicle needs to look, in meters: beyond
/// this gap, no leader, even a stopped one, brings its acceleration more than
/// [`NEGLIGIBLE`] below the free-road term.
///
/// A stopped leader at gap `s` gives `a · (1 − (s* / s)²)`, with `s*` the desired
/// gap approaching it. That is at least the free-road term `f` less `ε` when
/// `s ≥ s* · √(a / (a − f + ε))`. So the horizon grows with speed, as the desired
/// gap does, and shrinks when the free road already holds the vehicle back.
pub fn horizon(spec: &VehicleSpec, speed: f64, desired_speed: f64) -> f64 {
    let free = free_road(spec, speed, desired_speed);
    let stopped = desired_gap(spec, speed, 0.0);
    stopped * (spec.max_accel / (spec.max_accel - free + NEGLIGIBLE)).sqrt()
}

/// How quickly a vehicle eases up to a speed imposed on it from below, in
/// seconds: the time constant of its approach.
const CAP_EASE: f64 = 0.5;

/// The acceleration a speed cap allows: braking at the vehicle's comfortable
/// deceleration while above it, easing up to it from below, and with a cap of 0
/// braking until it stops. Like any term, it only binds when it is the lowest.
pub fn capped(spec: &VehicleSpec, speed: f64, max_speed: f64) -> f64 {
    if max_speed <= 0.0 {
        -spec.comfy_decel
    } else {
        ((max_speed - speed) / CAP_EASE).max(-spec.comfy_decel)
    }
}

#[cfg(test)]
mod test {
    use super::*;

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
    fn a_stopped_leader_at_the_horizon_is_negligible() {
        let spec = spec();
        for (speed, desired) in [
            (0.0, 14.0),
            (5.0, 14.0),
            (14.0, 14.0),
            (30.0, 30.0),
            (20.0, 14.0),
        ] {
            let horizon = horizon(&spec, speed, desired);
            let free = free_road(&spec, speed, desired);
            let at = interaction(&spec, speed, horizon, 0.0);
            assert!(
                (at - (free - NEGLIGIBLE)).abs() < 1e-9,
                "at {speed} m/s: {at} vs free {free}"
            );
            // Any further, it matters less, and so does a leader that is moving.
            assert!(interaction(&spec, speed, 1.01 * horizon, 0.0) > free - NEGLIGIBLE);
            assert!(interaction(&spec, speed, horizon, 1.0) > free - NEGLIGIBLE - 1e-9);
        }
        // Faster, it looks further: at motorway speed, about 290 m, past the 200 m
        // that used to be fixed.
        assert!(horizon(&spec, 30.0, 30.0) > 280.0);
        assert!(horizon(&spec, 30.0, 30.0) > horizon(&spec, 14.0, 14.0));
    }

    #[test]
    fn braking_is_capped() {
        assert_eq!(interaction(&spec(), 30.0, 0.5, 0.0), -MAX_BRAKING);
        assert_eq!(interaction(&spec(), 30.0, -1.0, 0.0), -MAX_BRAKING);
        assert_eq!(free_road(&spec(), 30.0, 10.0), -MAX_BRAKING);
    }

    #[test]
    fn a_speed_cap_brakes_above_it_and_eases_up_to_it() {
        let spec = spec();
        assert_eq!(
            capped(&spec, 14.0, 5.0),
            -spec.comfy_decel,
            "well above: comfortable braking"
        );
        assert!(capped(&spec, 5.2, 5.0) < 0.0 && capped(&spec, 5.2, 5.0) > -spec.comfy_decel);
        assert!(
            capped(&spec, 4.0, 5.0) > 0.0,
            "below it, it doesn't hold the vehicle back"
        );
        assert_eq!(
            capped(&spec, 0.0, 0.0),
            -spec.comfy_decel,
            "a cap of 0 stops it"
        );
    }
}
