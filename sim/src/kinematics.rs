//! The kinematic bicycle model, referenced at the rear axle.

use glam::DVec2;

use crate::math::{sinc, wrap_angle};

/// Moves the rear axle `dist` metres along the arc set by the steering angle.
///
/// Exact for a constant steering angle: the rear axle travels on a circle of
/// curvature `tan(steer) / wheelbase`, and the chord of that arc is
/// `dist · sinc(Δθ / 2)` in the direction of the mean heading.
pub fn advance(pos: DVec2, heading: f64, steer: f64, wheelbase: f64, dist: f64) -> (DVec2, f64) {
    let dtheta = dist * steer.tan() / wheelbase;
    let chord = dist * sinc(0.5 * dtheta);
    let pos = pos + chord * DVec2::from_angle(heading + 0.5 * dtheta);
    (pos, wrap_angle(heading + dtheta))
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn straight_without_steering() {
        let (pos, heading) = advance(DVec2::ZERO, 0.0, 0.0, 2.7, 1.0);
        assert!(pos.distance(DVec2::X) < 1e-12);
        assert_eq!(heading, 0.0);
    }

    #[test]
    fn stays_on_the_turning_circle() {
        let (wheelbase, steer) = (2.7, 0.3);
        let r = wheelbase / f64::tan(steer);
        let centre = DVec2::new(0.0, r);
        let (mut pos, mut heading) = (DVec2::ZERO, 0.0);
        for _ in 0..100 {
            (pos, heading) = advance(pos, heading, steer, wheelbase, 0.5);
        }
        assert!((pos.distance(centre) - r).abs() < 1e-9);
        assert!((heading - wrap_angle(50.0 / r)).abs() < 1e-9);
    }
}
