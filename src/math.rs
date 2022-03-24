//! Mathematical structs and functions.

use cgmath::{Point2, Vector2};
pub use util::*;
pub use lut::*;
pub use cubic::*;
pub use curve::*;

mod util;
mod lut;
mod cubic;
mod curve;

/// A 2D point
pub type Point2d = Point2<f64>;

/// A 2D vector
pub type Vector2d = Vector2<f64>;