//! Mathematical structs and functions.

use cgmath::{Point2, Vector2};
pub use cubic::*;
pub use curve::*;
pub use lut::*;
pub use util::*;

mod cubic;
mod curve;
mod lut;
mod util;

/// A 2D point
pub type Point2d = Point2<f64>;

/// A 2D vector
pub type Vector2d = Vector2<f64>;
