use cgmath::{Point2, Vector2, InnerSpace};
pub use cubic::CubicFn;
use crate::util::Interval;

mod cubic;

/// A 2D point
pub type Point2d = Point2<f64>;

/// A 2D vector
pub type Vector2d = Vector2<f64>;

pub struct LookupTable {
    offset: f64,
    step: f64,
    values: Vec<f64>
}

impl LookupTable {
    /// Creates a lookup table from a sample function.
    pub fn from_samples(range: Interval<f64>, step: f64, f: impl FnMut(f64) -> f64) -> Self {
        let offset = range.min;
        let num_samples = (range.length() / step).ceil() as usize;
        let xs = (0..num_samples).map(|i| offset + ((i as f64) + 0.5) * step);
        let values = xs.map(f).collect();
        Self { offset, step, values }
    }

    /// Samples the lookup table.
    pub fn sample(&self, x: f64) -> f64 {
        let idx = (x - self.offset) / self.step;
        let idx = usize::min(idx as usize, self.values.len() - 1);
        self.values[idx]
    }
}

/// Projects a point onto a local coordinate system.
/// 
/// # Parameters
/// * `point` - The point to project
/// * `origin` - The origin of the coordinate system
/// * `x_axis` - The basis vector pointing in the positive x-axis.
/// * `y_axis` - The basis vector pointing in the positive y-axis.
pub fn project_local(point: Point2d, origin: Point2d, x_axis: Vector2d, y_axis: Vector2d) -> Point2d {
    let point = point - origin;
    Point2d::new(point.dot(x_axis), point.dot(y_axis))
}