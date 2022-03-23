//! Mathematical structs and functions.

use cgmath::{Point2, Vector2};
pub use util::*;
pub use lut::LookupTable;
pub use cubic::CubicFn;
pub use curve::ParametricCurve2d;
pub use bezier::{QuadraticBezier2d, CubicBezier2d};

mod util;
mod lut;
mod cubic;
mod curve;
mod bezier;

/// A 2D point
pub type Point2d = Point2<f64>;

/// A 2D vector
pub type Vector2d = Vector2<f64>;