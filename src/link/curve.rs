use crate::math::{
    equidistant_points_along_curve, normalize_with_derivative, project_point_onto_curve, rot90,
    ParametricCurve2d, Point2d, QuadraticBezier2d, Vector2d,
};
use cgmath::prelude::*;
use serde::{Deserialize, Serialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct LinkCurve {
    scale: f64,
    length: f64,
    segments: Vec<QuadraticBezier2d>,
}

/// The result of sampling a [LinkCurve].
pub struct LinkSample {
    /// The vehicle position
    pub pos: Point2d,
    /// The vehicle direction unit vector
    pub dir: Vector2d,
    /// The tangent unit vector of the link.
    pub tan: Vector2d,
}

impl LinkCurve {
    /// Creates a new [LinkCurve] from the given parametric curve,
    /// with the default step size.
    pub fn new(curve: &impl ParametricCurve2d) -> Self {
        const LINK_SEGMENT_LEN: f64 = 0.5;
        Self::with_step(curve, LINK_SEGMENT_LEN)
    }

    /// Creates a new [LinkCurve] from the given parametric curve,
    /// with the given step size.
    pub fn with_step(curve: &impl ParametricCurve2d, step: f64) -> Self {
        let (mut points, length) = equidistant_points_along_curve(curve, step);

        // Ensure number of points are odd so they can be evenly divided among segments
        if points.len() % 2 == 0 {
            let p1 = points[points.len() - 2];
            let p2 = points[points.len() - 1];
            let p3 = Point2d::from_vec(Vector2d::lerp(p1.to_vec(), p2.to_vec(), 2.0));
            points.push(p3);
        }

        let segments = points
            .windows(3)
            .step_by(2)
            .map(|points| {
                let [p1, p2, p3]: [_; 3] = points.try_into().unwrap();
                let mid = Vector2d::lerp(p1.to_vec(), p3.to_vec(), 0.5);
                let control = Point2d::from_vec(Vector2d::lerp(p2.to_vec(), mid, -1.0));
                QuadraticBezier2d::new(&[p1, control, p3])
            })
            .collect::<Vec<_>>();

        Self {
            scale: 0.5 / step,
            length,
            segments,
        }
    }

    /// The length of the curve in m.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Samples the curve and returns the position and tangent unit vector.
    ///
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    /// * `offset` - The lateral offset from the centre line
    /// * `slope` - The derivative of `offset` with respect to `pos`.
    pub fn sample(&self, pos: f64, offset: f64, slope: f64) -> LinkSample {
        // Calculate centre point and its derivatives
        let (segment, t) = self.sample_internal(pos);
        let c = segment.sample(t);
        let c_dp = segment.sample_dt(t);
        let c_dp2 = segment.sample_dt2(t);

        // Calculate tangent unit vector and its derivative
        let (t, t_dp) = normalize_with_derivative(c_dp, c_dp2);

        // Rotate to get perpendicular unit vector and its derivative
        let [p, p_dp] = [t, t_dp].map(rot90);

        // Calculate offset point and its derivative
        let o = c + p * offset;
        let o_dp = c_dp + p_dp * offset + p * slope;

        LinkSample {
            pos: o,
            dir: o_dp.normalize(),
            tan: t,
        }
    }

    /// The inverse of the `sample` function.
    pub fn inverse_sample(&self, point: Point2d, tangent: Vector2d) -> Option<(f64, f64, f64)> {
        // Find the `pos` value
        let pos = project_point_onto_curve(self, point, 0.001, None)?;

        // Calculate centre point and its derivatives
        let (segment, t) = self.sample_internal(pos);
        let c = segment.sample(t);
        let c_dp = segment.sample_dt(t);
        let c_dp2 = segment.sample_dt2(t);

        // Calculate tangent unit vector and its derivative
        let (t, t_dp) = normalize_with_derivative(c_dp, c_dp2);

        // Rotate to get perpendicular unit vector and its derivative
        let [p, p_dp] = [t, t_dp].map(rot90);

        // Calculate offset point and its derivative
        let offset = (point - c).dot(p);
        let slope = -(c_dp + p_dp * offset).perp_dot(tangent) / p.perp_dot(tangent);

        Some((pos, offset, slope))
    }

    /// Samples the curve and returns the position and tangent unit vector.
    ///
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    pub fn sample_centre(&self, pos: f64) -> LinkSample {
        let (segment, t) = self.sample_internal(pos);
        let c = segment.sample(t);
        let c_dp = segment.sample_dt(t);
        let tan = c_dp.normalize();
        LinkSample {
            pos: c,
            dir: tan,
            tan,
        }
    }

    /// Samples the curve and returns the position, as well as the
    /// first derivative and second derivatives with respect to `pos`.
    ///
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    fn sample_internal(&self, pos: f64) -> (&QuadraticBezier2d, f64) {
        let pos = pos * self.scale;

        let idx = usize::min(pos as u32 as _, self.segments.len() - 1);
        let segment = unsafe {
            // SAFETY: The way `idx` is calculated above ensures its within bounds
            self.segments.get_unchecked(idx)
        };

        let t = pos - (idx as f64);

        (segment, t)
    }
}

impl ParametricCurve2d for LinkCurve {
    fn sample(&self, t: f64) -> Point2d {
        let (segment, t) = self.sample_internal(t);
        segment.sample(t)
    }

    fn bounds(&self) -> crate::util::Interval<f64> {
        crate::util::Interval::new(0.0, self.length())
    }

    fn sample_dt(&self, t: f64) -> Vector2d {
        let (segment, t) = self.sample_internal(t);
        segment.sample_dt(t)
    }

    fn sample_dt2(&self, t: f64) -> Vector2d {
        let (segment, t) = self.sample_internal(t);
        segment.sample_dt2(t)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn curve_is_arclength_parameterised() {
        let curve = QuadraticBezier2d::new(&[
            Point2d::new(10.0, 10.0),
            Point2d::new(60.0, 40.0),
            Point2d::new(100.0, 45.0),
        ]);
        let curve = LinkCurve::new(&curve);

        let ts = (0..100)
            .map(|i| i as f64 * 0.01 * curve.length())
            .collect::<Vec<_>>();
        for ts in ts.windows(2) {
            let p1 = curve.sample_centre(ts[0]).pos;
            let p2 = curve.sample_centre(ts[1]).pos;
            assert_approx_eq::assert_approx_eq!((p2 - p1).magnitude(), ts[1] - ts[0], 0.01);
        }
    }
}
