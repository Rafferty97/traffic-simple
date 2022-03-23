use cgmath::prelude::*;
use crate::util::Interval;
use super::{Point2d, Vector2d};

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

pub fn equidistant_points_along_curve(curve: &impl ParametricCurve2d, dist: f64) -> (Vec<Point2d>, f64) {
    let end_ts = curve.bounds();
    let end_ps = [curve.sample(end_ts.min), curve.sample(end_ts.max)];

    let mut ts = end_ts;
    let mut ps = end_ps;
    let mut dists = Interval::new(0.0, (ps[1] - ps[0]).magnitude());
    
    let mut points = vec![end_ps[0]];
    let mut last_p = end_ps[0];
    
    while dists.max > dist {
        loop {
            let new_t = ts.lerp(dists.inv_lerp(dist));
            let new_p = curve.sample(new_t);
            let new_dist = (new_p - last_p).magnitude();
            let f = new_dist / dist;

            if f < 0.99 {
                ts.min = new_t;
                ps[0] = new_p;
                dists.min = new_dist;
            }
            else if f > 1.01 {
                ts.max = new_t;
                ps[1] = new_p;
                dists.max = new_dist;
            }
            else {
                // Append the point
                points.push(new_p);
                last_p = new_p;

                // Setup for the next iteration
                ts = Interval::new(new_t, end_ts.max);
                ps = [new_p, end_ps[1]];
                dists = Interval::new(0.0, (ps[1] - ps[0]).magnitude());

                break;
            }
        }
    }

    let last_point = *points.last().unwrap();
    let end_vec = ps[1] - last_point;
    let end_magnitude = end_vec.magnitude();
    let mut length = (points.len() - 1) as f64 * dist;
    if end_magnitude > 0.001 * dist {
        length += end_magnitude;
        points.push(last_point + end_vec.normalize_to(dist));
    }

    (points, length)
}