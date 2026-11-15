//! MOBIL (Kesting, Treiber and Helbing, 2007): whether a lane change is worth it
//! and safe, from accelerations under the longitudinal model.
//!
//! Symmetric for now: no bias toward either side.

use crate::models::acceleration::SAFE_BRAKING;

/// What a vehicle decided about changing to one side, at a decision tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Not indicating: changing wouldn't gain enough.
    NotWorthIt,
    /// Not indicating: it would have to brake too hard in the target lane.
    Unsafe,
    /// Not indicating: the other side is better.
    OtherSide,
    /// Started indicating.
    Indicate,
    /// Still indicating, waiting for a gap: until the change is safe, worth it to
    /// everyone under politeness, and it has indicated long enough.
    Wait,
    /// Changing lanes now.
    Change,
    /// Stopped indicating: no longer worth it to the driver, or no longer permitted.
    Cancel,
}

/// How much a driver weighs the followers' gains and losses against its own.
pub const POLITENESS: f64 = 0.2;
/// The least net gain, in m/s², worth changing lanes for.
pub const THRESHOLD: f64 = 0.1;
/// The least time a driver indicates before changing lanes, in seconds.
pub const MIN_INDICATING: f64 = 1.0;

/// The net gain of a lane change: the driver's own change in acceleration plus
/// politeness times the changes for its new and old followers.
pub fn incentive(own_gain: f64, new_follower: f64, old_follower: f64) -> f64 {
    own_gain + POLITENESS * (new_follower + old_follower)
}

/// Whether a change with this net gain is worth making, or worth starting to
/// indicate for.
pub fn worth_it(incentive: f64) -> bool {
    incentive > THRESHOLD
}

/// Whether a driver already indicating should keep going: a lower bar than
/// starting, so a gain hovering around the threshold doesn't make the indicator
/// flicker.
pub fn still_worth_it(own_gain: f64) -> bool {
    own_gain > 0.0
}

/// Whether an acceleration after a change is gentle enough to accept.
pub fn safe(accel: f64) -> bool {
    accel >= -SAFE_BRAKING
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn politeness_weighs_followers() {
        assert_eq!(incentive(0.5, -1.0, 0.0), 0.5 - POLITENESS);
        assert!(worth_it(incentive(0.5, -1.0, 0.0)));
        assert!(!worth_it(incentive(0.2, -1.0, 0.0)));
    }

    #[test]
    fn safety_is_bounded_braking() {
        assert!(safe(-SAFE_BRAKING));
        assert!(!safe(-SAFE_BRAKING - 0.01));
    }
}
