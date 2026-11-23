//! The kinematic bicycle model, driven by the front axle's course.

use glam::DVec2;

/// Pulls the rear axle, at `rear` heading along `dir`, toward the front axle,
/// now at `front`, to `wheelbase` behind it: where the rear axle is now, and the
/// body's heading as a unit vector. A rear axle can't slide sideways, so it
/// moves along the body, and the path it traces is the tractrix of the front
/// axle's: the kinematic bicycle model driven by the front axle's path rather
/// than by a steering angle. It moves along the mean of its heading before and,
/// all but, after, so the step is second order in how far the front axle moved,
/// with a single square root.
pub fn trail(rear: DVec2, dir: DVec2, front: DVec2, wheelbase: f64) -> (DVec2, DVec2) {
    let v = front - rear;
    // Twice the mean heading, near enough: `v` is about a wheelbase long.
    let along = dir + v / wheelbase;
    // How far along `along` the rear axle goes to be `wheelbase` from the
    // front: the smaller root of |v − h·along|² = wheelbase².
    let (a, b) = (along.length_squared(), v.dot(along));
    let c = v.length_squared() - wheelbase * wheelbase;
    let h = c / (b + (b * b - a * c).max(0.0).sqrt());
    let rear = rear + h * along;
    (rear, (front - rear) / wheelbase)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn straight_on_the_rear_axle_follows() {
        let (rear, dir) = trail(DVec2::ZERO, DVec2::X, DVec2::new(3.7, 0.0), 2.7);
        assert!(rear.distance(DVec2::new(1.0, 0.0)) < 1e-12);
        assert!(dir.distance(DVec2::X) < 1e-12);
    }

    #[test]
    fn round_a_circle_the_rear_axle_runs_inside_the_front() {
        // The front axle on a circle of radius r: the rear settles at √(r² − L²).
        let (r, wheelbase) = (10.0, 2.7);
        let on = |a: f64| r * DVec2::new(a.sin(), 1.0 - a.cos());
        let (mut rear, mut dir) = (DVec2::new(-wheelbase, 0.0), DVec2::X);
        for i in 1..=400 {
            (rear, dir) = trail(rear, dir, on(i as f64 * 0.03), wheelbase);
        }
        let inside = (r * r - wheelbase * wheelbase).sqrt();
        assert!((rear.distance(DVec2::new(0.0, r)) - inside).abs() < 1e-3);
        assert!((dir.length() - 1.0).abs() < 1e-12);
    }
}
