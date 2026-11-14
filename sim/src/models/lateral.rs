//! Pure pursuit path tracking: steer the rear axle along an arc through a point
//! a speed-dependent distance ahead on the target path. During a lane change, the
//! target path is offset from the lane's centreline along a cubic.

use glam::DVec2;

use crate::math::angle_diff;

/// Shortest lookahead distance in metres, used at low speed.
const MIN_LOOKAHEAD: f64 = 2.0;
/// Lookahead distance added per m/s of speed, in seconds.
const LOOKAHEAD_TIME: f64 = 0.3;
/// Largest front-wheel steering angle in radians (about 34°).
const MAX_STEER: f64 = 0.6;

/// How long a lane change takes, in seconds. Peak lateral acceleration is about
/// `6 · offset / T²`: about 1.3 m/s² across a 3.5 m lane.
const LANE_CHANGE_TIME: f64 = 4.0;
/// Shortest lane change in metres, used at low speed.
const MIN_LANE_CHANGE_LENGTH: f64 = 15.0;

/// A lane change in progress. The target path's offset from the new lane's
/// centreline follows a cubic in `u = s − s0`, from the vehicle's offset and slope
/// at the start to zero offset and zero slope at `u = length`.
#[derive(Clone, Copy, Debug)]
pub struct LaneChange {
    /// Where the change started, as `s` along the vehicle's current lane.
    pub s0: f64,
    /// Distance over which the offset returns to zero, in metres.
    pub length: f64,
    /// Lateral offset at the start, in metres, positive to the left.
    pub offset: f64,
    /// Rate of change of the offset at the start, dt/ds.
    pub slope: f64,
}

impl LaneChange {
    pub fn new(s0: f64, offset: f64, slope: f64, speed: f64) -> Self {
        let length = (speed * LANE_CHANGE_TIME).max(MIN_LANE_CHANGE_LENGTH);
        Self { s0, length, offset, slope }
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

/// How far ahead along the path to aim, in metres.
pub fn lookahead_distance(speed: f64) -> f64 {
    MIN_LOOKAHEAD + LOOKAHEAD_TIME * speed
}

/// The steering angle that puts the rear axle on an arc through `target`.
pub fn steer_towards(pos: DVec2, heading: f64, wheelbase: f64, target: DVec2) -> f64 {
    let d = target - pos;
    let alpha = angle_diff(d.to_angle(), heading);
    let curvature = 2.0 * alpha.sin() / d.length();
    (wheelbase * curvature).atan().clamp(-MAX_STEER, MAX_STEER)
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
        let steer = steer_towards(DVec2::ZERO, 0.0, 2.7, DVec2::new(10.0, 0.0));
        assert_eq!(steer, 0.0);
    }

    #[test]
    fn steers_towards_the_side_of_the_target() {
        assert!(steer_towards(DVec2::ZERO, 0.0, 2.7, DVec2::new(10.0, 1.0)) > 0.0);
        assert!(steer_towards(DVec2::ZERO, 0.0, 2.7, DVec2::new(10.0, -1.0)) < 0.0);
    }

    #[test]
    fn arc_passes_through_target() {
        // Target on a circle of radius 20 m tangent to the heading at the origin.
        let r = 20.0;
        let target = DVec2::new(r * 0.5f64.sin(), r * (1.0 - 0.5f64.cos()));
        let steer = steer_towards(DVec2::ZERO, 0.0, 2.7, target);
        assert!((steer.tan() / 2.7 - 1.0 / r).abs() < 1e-12);
    }
}
