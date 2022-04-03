use cgmath::prelude::*;
use crate::{math::Point2d, util::Interval};
use super::ParametricCurve2d;

/// Projects a point onto a parametric curve.
pub fn project_point_onto_curve(
    curve: &impl ParametricCurve2d,
    point: Point2d,
    max_error: f64,
    t0: Option<f64>
) -> Option<f64> {
    let bounds = curve.bounds();
    
    // Get initial guess for `t`
    let mut t = t0.unwrap_or_else(|| {
        (0..=8).map(|i| bounds.lerp(i as f64 / 8.0))
        .map(|t| (t, (point - curve.sample(t)).magnitude()))
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap().0
    });
    let (mut p, mut p_dt) = (curve.sample(t), curve.sample_dt(t));
    
    // Refine `t` with Newton's method
    for _ in 0..64 {
        let error = p_dt.dot(point - p) / p_dt.magnitude();
        t += error;
        if error.abs() < max_error || !bounds.contains(t) {
            return Some(t);
        }
        (p, p_dt) = (curve.sample(t), curve.sample_dt(t));
    }
    
    None
}

/// Finds a set of evenly spaced points along the given parametric curve.
pub fn equidistant_points_along_curve(curve: &impl ParametricCurve2d, dist: f64) -> (Vec<Point2d>, f64) {
    let end_ts = curve.bounds();
    let end_ps = [curve.sample(end_ts.min), curve.sample(end_ts.max)];

    let mut ts = end_ts;
    let mut ps = end_ps;
    let mut dists = Interval::new(0.0, (ps[1] - ps[0]).magnitude());
    
    let mut points = vec![end_ps[0]];
    let mut last_p = end_ps[0];
    
    while dists.max > dist {
        for _ in 0..100 {
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

/// Approximates a curve by subdividing it until all segments are no longer than `dist` units in length.
pub fn subdivided_points_along_curve(curve: &impl ParametricCurve2d, dist: f64) -> Vec<Point2d> {
    let dist2 = dist.powi(2);

    let Interval { min, max } = curve.bounds();
    let (mut last_t, mut last_p) = (min, curve.sample(min));
    let (mut next_t, mut next_p) = (max, curve.sample(max));
    let mut out = vec![last_p];
    let mut stack = vec![];

    loop {
        if (next_p - last_p).magnitude2() > dist2 {
            let mid_t = 0.5 * (next_t + last_t);
            let mid_p = curve.sample(mid_t);
            stack.push((next_t, next_p));
            (next_t, next_p) = (mid_t, mid_p);
        } else {
            out.push(next_p);
            (last_t, last_p) = (next_t, next_p);
            if let Some(v) = stack.pop() {
                (next_t, next_p) = v;
            } else {
                break;
            }
        }
    }

    out
}

#[cfg(test)]
mod test {
    use assert_approx_eq::assert_approx_eq;
    use crate::math::LineSegment2d;
    use super::*;

    #[test]
    pub fn equidistant_points_along_curve_is_stable() {
        for i in 0..100 {
            let len = 0.1 * i as f64;
            let points = equidistant_points_along_curve(
                &LineSegment2d::from_ends(Point2d::new(10.0, 10.0), Point2d::new(10.0 + len, 10.0)),
                0.5
            );
            assert_approx_eq!(points.1, len);
            for point in points.0.into_iter() {
                assert!(!point.x.is_nan() && !point.y.is_nan());
            }
        }
    }
}