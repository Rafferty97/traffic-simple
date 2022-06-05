use super::ParametricCurve2d;
use crate::Interval;

pub struct ReversedCurve<C>
where
    C: ParametricCurve2d,
{
    inner: C,
}

impl<C> ReversedCurve<C>
where
    C: ParametricCurve2d,
{
    pub fn new(curve: C) -> Self {
        Self { inner: curve }
    }
}

impl<C> ParametricCurve2d for ReversedCurve<C>
where
    C: ParametricCurve2d,
{
    fn sample(&self, t: f64) -> crate::math::Point2d {
        self.inner.sample(self.map_t(t))
    }

    fn sample_dt(&self, t: f64) -> crate::math::Vector2d {
        -self.inner.sample_dt(self.map_t(t))
    }

    fn sample_dt2(&self, t: f64) -> crate::math::Vector2d {
        self.inner.sample_dt2(self.map_t(t))
    }

    fn bounds(&self) -> crate::util::Interval<f64> {
        self.inner.bounds()
    }
}

impl<C> ReversedCurve<C>
where
    C: ParametricCurve2d,
{
    fn map_t(&self, t: f64) -> f64 {
        let Interval { min, max } = self.inner.bounds();
        max - (t - min)
    }
}
