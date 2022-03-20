use crate::math::{Point2d, Vector2d};

pub struct LinkCurve {
    // todo
}

impl LinkCurve {
    /// The length of the curve in m.
    pub fn length(&self) -> f64 {
        unimplemented!()
    }

    /// Samples the curve and returns the position and tangent vector.
    /// 
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    /// * `offset` - The lateral offset from the centre line
    /// * `slope` - The derivative of `offset` with respect to `pos`.
    pub fn sample(&self, pos: f64, offset: f64, slope: f64) -> (Point2d, Vector2d) {
        unimplemented!();
    }

    /// Samples the curve and returns the position and tangent vector.
    /// 
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    pub fn sample_centre(&self, pos: f64) -> (Point2d, Vector2d) {
        self.sample(pos, 0.0, 0.0)
    }
}