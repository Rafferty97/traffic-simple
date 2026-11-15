//! MOBIL (Kesting, Treiber and Helbing, 2007): whether a lane change is worth it
//! and safe, from accelerations under the longitudinal model.
//!
//! Symmetric for now: no bias toward either side.

use crate::models::acceleration::SAFE_BRAKING;

/// What a vehicle decided about changing to one side, at a decision tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Not indicating: not enough advantage has built up yet.
    NotWorthIt,
    /// Not indicating: enough advantage has built up, but it, or the follower it
    /// would cut in front of, would have to brake too hard.
    Unsafe,
    /// Not indicating to this side: the other has more built up, or it is
    /// already indicating the other way.
    OtherSide,
    /// Started indicating.
    Indicate,
    /// Started indicating without a safe gap, frustrated, asking the follower in
    /// the target lane to make room.
    Request,
    /// Still indicating, waiting for a gap: until the change is safe and it has
    /// indicated long enough.
    Wait,
    /// Changing lanes now.
    Change,
    /// Stopped indicating: its advantage has faded, or the side is no longer permitted.
    Cancel,
}

/// The least net gain, in m/s², worth changing lanes for.
pub const THRESHOLD: f64 = 0.1;
/// The least time a driver indicates before changing lanes, in seconds.
pub const MIN_INDICATING: f64 = 0.5;

/// The net gain of a lane change: the driver's own change in acceleration plus
/// its `politeness` times the changes for its new and old followers.
pub fn incentive(politeness: f64, own_gain: f64, new_follower: f64, old_follower: f64) -> f64 {
    own_gain + politeness * (new_follower + old_follower)
}

/// How long a driver must have been blocked, in seconds, before it accepts
/// braking as hard as [`SAFE_BRAKING`], and indicates even without a gap.
pub const FULLY_FRUSTRATED: f64 = 10.0;
/// How fast frustration fades while a driver isn't blocked: it halves every
/// this many seconds.
pub const FRUSTRATION_HALF_LIFE: f64 = 5.0;
/// The least speed, in m/s, the follower in the target lane must have for a
/// frustrated driver to indicate without a gap: in a stopped queue, there is
/// nobody to make room.
pub const MOVING: f64 = 2.0;

/// Frustration after `dt` seconds, in seconds blocked: a driver that wants to
/// change lanes but can't safely grows frustrated, and otherwise its frustration
/// fades. As the critical gap shrinks with waiting in Ahmed's (1999) model and
/// SUMO's impatience grows while a driver is held up.
pub fn frustrate(frustration: f64, blocked: bool, dt: f64) -> f64 {
    match blocked {
        true => (frustration + dt).min(FULLY_FRUSTRATED),
        false => frustration * 0.5f64.powf(dt / FRUSTRATION_HALF_LIFE),
    }
}

/// The hardest braking, positive, a driver accepts from a lane change, for
/// itself and for the follower it cuts in front of: its comfortable deceleration
/// when calm, rising to [`SAFE_BRAKING`] when fully frustrated.
pub fn accepted_braking(comfy_decel: f64, frustration: f64) -> f64 {
    let f = (frustration / FULLY_FRUSTRATED).clamp(0.0, 1.0);
    comfy_decel + (SAFE_BRAKING - comfy_decel).max(0.0) * f
}

/// Whether a driver is frustrated enough to indicate without a gap, asking the
/// follower in the target lane to make room.
pub fn forcing(frustration: f64) -> bool {
    frustration >= FULLY_FRUSTRATED
}

/// How much advantage must build up before a driver intends a change, in m/s:
/// the net gain above [`THRESHOLD`], summed over time. A clear gain of 1 m/s²
/// gets there in about a third of a second; a marginal one has to last.
pub const INTENT: f64 = 0.3;
/// The most advantage that builds up, so that a long wait for a gap doesn't leave
/// a store that takes long to fade.
pub const MAX_ADVANTAGE: f64 = 2.0 * INTENT;
/// How fast built-up advantage fades while changing isn't worth it: it halves
/// every this many seconds.
pub const HALF_LIFE: f64 = 1.0;

/// Built-up advantage for a change, after `dt` seconds with this net gain: the
/// gain above [`THRESHOLD`] adds up while there is some, and otherwise what has
/// built up fades. As SUMO's lane-change model builds up a speed gain before
/// changing, so a fleeting or marginal advantage doesn't make a driver change.
pub fn accumulate(advantage: f64, incentive: f64, dt: f64) -> f64 {
    match incentive > THRESHOLD {
        true => (advantage + (incentive - THRESHOLD) * dt).min(MAX_ADVANTAGE),
        false => advantage * 0.5f64.powf(dt / HALF_LIFE),
    }
}

/// Whether enough advantage has built up for a driver to intend a change.
pub fn intends(advantage: f64) -> bool {
    advantage >= INTENT
}

/// Whether a driver already indicating should keep going: a lower bar than
/// starting, so advantage hovering around the bar doesn't make the indicator
/// flicker.
pub fn still_intends(advantage: f64) -> bool {
    advantage >= 0.5 * INTENT
}

/// Whether an acceleration after a change is gentle enough to accept, braking no
/// harder than `accepted`.
pub fn safe(accel: f64, accepted: f64) -> bool {
    accel >= -accepted
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn politeness_weighs_followers() {
        assert_eq!(incentive(0.2, 0.5, -1.0, 0.0), 0.5 - 0.2);
        assert_eq!(incentive(0.0, 0.5, -1.0, 0.0), 0.5, "an impolite driver");
    }

    #[test]
    fn frustration_builds_while_blocked_and_fades_after() {
        let blocked = (0..25).fold(0.0, |f, _| frustrate(f, true, 0.2));
        assert!((blocked - 5.0).abs() < 1e-9);
        assert_eq!(
            frustrate(FULLY_FRUSTRATED, true, 0.2),
            FULLY_FRUSTRATED,
            "capped"
        );
        assert!((frustrate(4.0, false, FRUSTRATION_HALF_LIFE) - 2.0).abs() < 1e-12);
        assert!(forcing(FULLY_FRUSTRATED) && !forcing(0.99 * FULLY_FRUSTRATED));
    }

    #[test]
    fn frustration_raises_the_braking_a_change_accepts() {
        assert_eq!(accepted_braking(1.5, 0.0), 1.5, "calm: comfortable braking");
        assert_eq!(accepted_braking(1.5, FULLY_FRUSTRATED), SAFE_BRAKING);
        assert_eq!(
            accepted_braking(1.5, 2.0 * FULLY_FRUSTRATED),
            SAFE_BRAKING,
            "capped"
        );
        let half = accepted_braking(1.5, 0.5 * FULLY_FRUSTRATED);
        assert!((half - (1.5 + SAFE_BRAKING) / 2.0).abs() < 1e-12);
    }

    #[test]
    fn advantage_builds_up_above_the_threshold_and_fades_below_it() {
        let dt = 0.2;
        // A clear gain intends a change within a few decisions...
        let clear = (0..2).fold(0.0, |a, _| accumulate(a, 1.0, dt));
        assert!(intends(clear), "{clear}");
        // ...a marginal one only after several seconds of it.
        let marginal = |ticks| (0..ticks).fold(0.0, |a, _| accumulate(a, 0.15, dt));
        assert!(!intends(marginal(25)), "after 5 s");
        assert!(intends(marginal(35)), "after 7 s");
        // Below the threshold, nothing builds up.
        assert_eq!(accumulate(0.0, THRESHOLD, dt), 0.0);
        // It fades by half every half-life, and is capped.
        assert!((accumulate(0.4, 0.0, HALF_LIFE) - 0.2).abs() < 1e-12);
        assert_eq!(accumulate(MAX_ADVANTAGE, 5.0, dt), MAX_ADVANTAGE);
        // Indicating, it keeps going down to half the bar.
        assert!(still_intends(0.5 * INTENT) && !still_intends(0.49 * INTENT));
    }

    #[test]
    fn safety_is_bounded_braking() {
        assert!(safe(-2.0, 2.0));
        assert!(!safe(-2.01, 2.0));
    }
}
