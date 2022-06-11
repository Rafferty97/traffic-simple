use crate::math::{Point2d, Vector2d};
use cgmath::InnerSpace;

pub fn calc_direction(pos: Point2d, dir: Vector2d, new_pos: Point2d, radius: f64) -> Vector2d {
    let b = pos - radius * dir;
    let v = pos - b;
    let h = (v.magnitude2() - radius.powi(2)) / (2.0 * (radius + v.dot(dir)));
    let bp = b + h * dir;
    (new_pos - bp).normalize()
}
