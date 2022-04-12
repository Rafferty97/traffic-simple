use super::ParametricCurve2d;
use crate::Interval;

pub struct Subcurve<C>
where
    C: ParametricCurve2d,
{
    inner: C,
    bounds: Interval<f64>,
}

impl<C> Subcurve<C>
where
    C: ParametricCurve2d,
{
    pub fn new(curve: C, bounds: Interval<f64>) -> Self {
        Self {
            inner: curve,
            bounds,
        }
    }
}

impl<C> ParametricCurve2d for Subcurve<C>
where
    C: ParametricCurve2d,
{
    #[inline(always)]
    fn sample(&self, t: f64) -> crate::math::Point2d {
        self.inner.sample(t)
    }

    #[inline(always)]
    fn bounds(&self) -> Interval<f64> {
        self.bounds
    }

    #[inline(always)]
    fn sample_dt(&self, t: f64) -> crate::math::Vector2d {
        self.inner.sample_dt(t)
    }

    #[inline(always)]
    fn sample_dt2(&self, t: f64) -> crate::math::Vector2d {
        self.inner.sample_dt2(t)
    }
}
