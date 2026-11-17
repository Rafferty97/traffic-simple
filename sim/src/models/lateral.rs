//! Pure pursuit path tracking: steer the rear axle along an arc through a point
//! a speed-dependent distance ahead on the target path. During a lane change, the
//! target path is offset from the lane's centerline along a cubic.

use glam::DVec2;

/// Shortest lookahead distance in meters, used at low speed.
const MIN_LOOKAHEAD: f64 = 2.0;
/// Lookahead distance added per m/s of speed, in seconds.
const LOOKAHEAD_TIME: f64 = 0.3;
/// `tan` of the largest front-wheel steering angle, 0.6 rad (about 34°). A
/// vehicle's sharpest curvature is this divided by its wheelbase.
const TAN_MAX_STEER: f64 = 0.684_136_808_341_692_4;

/// How long a lane change takes, in seconds. Peak lateral acceleration is about
/// `6 · offset / T²`: about 1.3 m/s² across a 3.5 m lane.
const LANE_CHANGE_TIME: f64 = 4.0;
/// Shortest lane change in meters, used at low speed.
const MIN_LANE_CHANGE_LENGTH: f64 = 15.0;

/// A lane change in progress. The target path's offset from the new lane's
/// centerline follows a cubic in `u = s − s0`, from the old path's offset and slope
/// at the start to zero offset and zero slope at `u = length`. It starts at the
/// point pure pursuit was aiming at, so the target doesn't jump.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LaneChange {
    /// Where the change started, as `s` along the vehicle's current lane.
    pub s0: f64,
    /// Distance over which the offset returns to zero, in meters.
    pub length: f64,
    /// Lateral offset at the start, in meters, positive to the left.
    pub offset: f64,
    /// Rate of change of the offset at the start, dt/ds.
    pub slope: f64,
}

impl LaneChange {
    pub fn new(s0: f64, offset: f64, slope: f64, speed: f64) -> Self {
        Self { s0, length: change_length(speed), offset, slope }
    }

    /// The target offset at `s`. With `x = (s − s0) / length`, clamped to [0, 1]:
    /// `offset · (2x³ − 3x² + 1) + length · slope · (x³ − 2x² + x)`.
    pub fn offset_at(&self, s: f64) -> f64 {
        let x = ((s - self.s0) / self.length).clamp(0.0, 1.0);
        let (x2, x3) = (x * x, x * x * x);
        self.offset * (2.0 * x3 - 3.0 * x2 + 1.0) + self.length * self.slope * (x3 - 2.0 * x2 + x)
    }

    /// Where the change is complete, as `s` along the vehicle's current lane.
    pub fn end(&self) -> f64 {
        self.s0 + self.length
    }
}

/// How far a lane change started at `speed` runs, in meters.
pub fn change_length(speed: f64) -> f64 {
    (speed * LANE_CHANGE_TIME).max(MIN_LANE_CHANGE_LENGTH)
}

/// How far ahead along the path to aim, in meters.
pub fn lookahead_distance(speed: f64) -> f64 {
    MIN_LOOKAHEAD + LOOKAHEAD_TIME * speed
}

/// The curvature that puts the rear axle on an arc through `target`, in 1/m,
/// positive to the left, limited by the vehicle's steering lock.
///
/// Pure pursuit's `2·sin(α) / d`, where `α` is the angle from the heading to the
/// target. `sin(α)` is the cross product of the heading with the unit vector to
/// the target, so this needs no trigonometry beyond the heading's direction.
pub fn curvature_toward(pos: DVec2, heading: f64, wheelbase: f64, target: DVec2) -> f64 {
    let d = target - pos;
    let max = TAN_MAX_STEER / wheelbase;
    (2.0 * DVec2::from_angle(heading).perp_dot(d) / d.length_squared()).clamp(-max, max)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn lane_change_offset_meets_its_ends() {
        let change = LaneChange::new(10.0, 3.5, 0.05, 14.0);
        assert_eq!(change.length, 56.0);
        assert!((change.offset_at(10.0) - 3.5).abs() < 1e-12);
        assert!(change.offset_at(change.end()).abs() < 1e-12);
        assert_eq!(change.offset_at(change.end() + 5.0), 0.0);
        // Slopes by finite differences: the given slope at the start, flat at the end.
        let h = 1e-6;
        let start_slope = (change.offset_at(10.0 + h) - change.offset_at(10.0)) / h;
        let end_slope = (change.offset_at(change.end()) - change.offset_at(change.end() - h)) / h;
        assert!((start_slope - 0.05).abs() < 1e-5);
        assert!(end_slope.abs() < 1e-5);
    }

    #[test]
    fn slow_lane_changes_have_a_minimum_length() {
        assert_eq!(
            LaneChange::new(0.0, 3.5, 0.0, 0.0).length,
            MIN_LANE_CHANGE_LENGTH
        );
    }

    #[test]
    fn straight_ahead_needs_no_steering() {
        let curvature = curvature_toward(DVec2::ZERO, 0.0, 2.7, DVec2::new(10.0, 0.0));
        assert_eq!(curvature, 0.0);
    }

    #[test]
    fn steers_toward_the_side_of_the_target() {
        assert!(curvature_toward(DVec2::ZERO, 0.0, 2.7, DVec2::new(10.0, 1.0)) > 0.0);
        assert!(curvature_toward(DVec2::ZERO, 0.0, 2.7, DVec2::new(10.0, -1.0)) < 0.0);
    }

    #[test]
    fn arc_passes_through_target() {
        // Target on a circle of radius 20 m tangent to the heading at the origin.
        let r = 20.0;
        let target = DVec2::new(r * 0.5f64.sin(), r * (1.0 - 0.5f64.cos()));
        let curvature = curvature_toward(DVec2::ZERO, 0.0, 2.7, target);
        assert!((curvature - 1.0 / r).abs() < 1e-12);
    }

    #[test]
    fn steering_lock_limits_curvature() {
        // Directly to the left: far beyond the steering lock.
        let curvature = curvature_toward(DVec2::ZERO, 0.0, 2.7, DVec2::new(0.0, 1.0));
        assert!((curvature - 0.6f64.tan() / 2.7).abs() < 1e-12);
    }
}
