use cgmath::prelude::*;
use crate::math::{Point2d, Vector2d, rot90, normalize_with_derivative, QuadraticBezier2d, ParametricCurve2d, equidistant_points_along_curve};

#[derive(Clone)]
pub struct LinkCurve {
    scale: f64,
    length: f64,
    segments: Vec<QuadraticBezier2d>
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
            scale: 2.0 / step,
            length,
            segments
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
    pub fn sample(&self, pos: f64, offset: f64, slope: f64) -> (Point2d, Vector2d) {
        // Calculate centre point and its derivatives
        let (c, c_dp, c_dp2) = self.sample_internal(pos);

        // Calculate tangent unit vector and its derivative
        let (t, t_dp) = normalize_with_derivative(c_dp, c_dp2);

        // Rotate to get perpendicular unit vector and its derivative
        let [p, p_dp] = [t, t_dp].map(rot90);
        
        // Calculate offset point and its derivative
        let o = c + p * offset;
        let o_dp = c_dp + p_dp * offset + p * slope;

        (o, o_dp.normalize())
    }

    /// Samples the curve and returns the position and tangent unit vector.
    /// 
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    pub fn sample_centre(&self, pos: f64) -> (Point2d, Vector2d) {
        let (c, c_dp, _) = self.sample_internal(pos);
        (c, c_dp.normalize())
    }

    /// Samples the curve and returns the position, as well as the
    /// first derivative and second derivatives with respect to `pos`.
    /// 
    /// # Parameters
    /// * `pos` - The longitudinal position along the curve
    fn sample_internal(&self, pos: f64) -> (Point2d, Vector2d, Vector2d) {
        let pos = pos * self.scale;

        let idx = usize::min(pos as u32 as _, self.segments.len() - 1);
        let segment = unsafe {
            // SAFETY: The way `idx` is calculated above ensures its within bounds
            self.segments.get_unchecked(idx)
        };

        let t = pos - (idx as f64);
        (
            segment.sample(t),
            segment.sample_dt(t),
            segment.sample_dt2(t)
        )
    }
}