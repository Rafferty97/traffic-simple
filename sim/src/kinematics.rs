//! The kinematic bicycle model, referenced at the rear axle.

use glam::DVec2;

use crate::math::{sinc, wrap_angle};

/// Moves the rear axle `dist` meters along an arc of the given curvature, in 1/m.
///
/// Exact for a constant curvature: the heading turns by `Δθ = dist · curvature`,
/// and the chord of the arc is `dist · sinc(Δθ / 2)` in the direction of the
/// mean heading. A front-wheel steering angle `δ` gives curvature `tan(δ) / L`
/// for wheelbase `L`.
pub fn advance(pos: DVec2, heading: f64, curvature: f64, dist: f64) -> (DVec2, f64) {
    let dtheta = dist * curvature;
    let chord = dist * sinc(0.5 * dtheta);
    let pos = pos + chord * DVec2::from_angle(heading + 0.5 * dtheta);
    (pos, wrap_angle(heading + dtheta))
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn straight_without_steering() {
        let (pos, heading) = advance(DVec2::ZERO, 0.0, 0.0, 1.0);
        assert!(pos.distance(DVec2::X) < 1e-12);
        assert_eq!(heading, 0.0);
    }

    #[test]
    fn stays_on_the_turning_circle() {
        let r = 2.7 / f64::tan(0.3);
        let center = DVec2::new(0.0, r);
        let (mut pos, mut heading) = (DVec2::ZERO, 0.0);
        for _ in 0..100 {
            (pos, heading) = advance(pos, heading, 1.0 / r, 0.5);
        }
        assert!((pos.distance(center) - r).abs() < 1e-9);
        assert!((heading - wrap_angle(50.0 / r)).abs() < 1e-9);
    }
}
