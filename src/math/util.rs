use super::{Point2d, Vector2d};
use cgmath::prelude::*;

/// Projects a point onto a local coordinate system.
///
/// # Parameters
/// * `point` - The point to project
/// * `origin` - The origin of the coordinate system
/// * `x_axis` - The basis vector pointing in the positive x-axis.
/// * `y_axis` - The basis vector pointing in the positive y-axis.
pub fn project_local(
    point: Point2d,
    origin: Point2d,
    x_axis: Vector2d,
    y_axis: Vector2d,
) -> Point2d {
    let point = point - origin;
    Point2d::new(point.dot(x_axis), point.dot(y_axis))
}

/// Rotates a vector 90 degrees clockwise.
pub fn rot90(vec: Vector2d) -> Vector2d {
    Vector2d::new(-vec.y, vec.x)
}

/// Normalises a vector and computes the derivative of the normalised vector.
///
/// # Parameters
/// * `v` - The vector to normalize
/// * `dv` - The derivative of `v`
///
/// # Returns
/// A tuple containing the the normalization of `v`, and its derivative.
#[inline(always)]
pub fn normalize_with_derivative(v: Vector2d, dv: Vector2d) -> (Vector2d, Vector2d) {
    let mag = v.magnitude();
    let s = v / mag;
    let ds = (dv - s * s.dot(dv)) / mag;
    (s, ds)
}
