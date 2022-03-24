use cgmath::InnerSpace;

use crate::math::rot90;

use super::ParametricCurve2d;

pub struct OffsetCurve<C, F>
    where C: ParametricCurve2d, F: Fn(f64) -> f64
{
    inner: C,
    offset: F
}

impl<C, F> OffsetCurve<C, F>
    where C: ParametricCurve2d, F: Fn(f64) -> f64
{
    pub fn new(curve: C, offset: F) -> Self {
        Self { inner: curve, offset }
    }
}

impl<C, F> ParametricCurve2d for OffsetCurve<C, F>
    where C: ParametricCurve2d, F: Fn(f64) -> f64
{
    fn sample(&self, t: f64) -> crate::math::Point2d {
        let c = self.inner.sample(t);
        let p = rot90(self.inner.sample_dt(t).normalize());
        let o = (self.offset)(t);
        c + p * o
    }

    fn bounds(&self) -> crate::util::Interval<f64> {
        self.inner.bounds()
    }
}