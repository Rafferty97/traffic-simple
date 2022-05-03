use super::{Point2d, Vector2d};
use crate::util::Interval;
pub use algorithms::*;
pub use bezier::*;
pub use offset::*;
pub use reverse::*;
pub use subcurve::*;

mod algorithms;
mod bezier;
mod offset;
mod reverse;
mod subcurve;

/// A parametric curve in 2D space.
pub trait ParametricCurve2d {
    /// Samples the parametric curve.
    fn sample(&self, t: f64) -> Point2d;

    /// Returns the minimum and maximum t-values that define the bounds of the curve.
    fn bounds(&self) -> Interval<f64>;

    /// Samples the derivative of the parametric curve.
    ///
    /// The default implementation approximates the derivative by sampling
    /// two very nearby points along the curve.
    fn sample_dt(&self, t: f64) -> Vector2d {
        let delta = self.bounds().length() * 0.0001;
        let p1 = self.sample(t);
        let p2 = self.sample(t + delta);
        (p2 - p1) / delta
    }

    /// Samples the second derivative of the parametric curve.
    ///
    /// The default implementation approximates the derivative by sampling
    /// two very nearby points along the curve.
    fn sample_dt2(&self, t: f64) -> Vector2d {
        let delta = self.bounds().length() * 0.0001;
        let p1 = self.sample_dt(t);
        let p2 = self.sample_dt(t + delta);
        (p2 - p1) / delta
    }
}

impl<T: ParametricCurve2d + ?Sized> ParametricCurve2d for &T {
    fn sample(&self, t: f64) -> Point2d {
        (&**self).sample(t)
    }

    fn bounds(&self) -> Interval<f64> {
        (&**self).bounds()
    }

    fn sample_dt(&self, t: f64) -> Vector2d {
        (&**self).sample_dt(t)
    }

    fn sample_dt2(&self, t: f64) -> Vector2d {
        (&**self).sample_dt2(t)
    }
}
