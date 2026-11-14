//! Pure pursuit path tracking: steer the rear axle along an arc through a point
//! a speed-dependent distance ahead on the target path.

use glam::DVec2;

use crate::math::angle_diff;

/// Shortest lookahead distance in metres, used at low speed.
const MIN_LOOKAHEAD: f64 = 2.0;
/// Lookahead distance added per m/s of speed, in seconds.
const LOOKAHEAD_TIME: f64 = 0.3;
/// Largest front-wheel steering angle in radians (about 34°).
const MAX_STEER: f64 = 0.6;

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
