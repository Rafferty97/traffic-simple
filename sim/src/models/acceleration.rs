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

/// The acceleration a speed cap allows: braking at the safe deceleration while above it,
/// easing up to it from below, and with a cap of 0 braking until it stops.
/// Like any term, it only binds when it is the lowest.
pub fn capped(spec: &VehicleSpec, speed: f64, max_speed: f64) -> f64 {
    if max_speed <= 0.0 {
        -SAFE_BRAKING
    } else {
        ((max_speed - speed) / CAP_EASE).max(-SAFE_BRAKING)
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
