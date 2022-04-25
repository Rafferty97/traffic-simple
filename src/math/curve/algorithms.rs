use super::ParametricCurve2d;
use crate::{math::Point2d, util::Interval};
use cgmath::prelude::*;

/// Projects a point onto a parametric curve.
pub fn project_point_onto_curve(
    curve: &impl ParametricCurve2d,
    point: Point2d,
    max_error: f64,
    t0: Option<f64>,
) -> Option<f64> {
    let bounds = curve.bounds();

    // Get initial guess for `t`
    let mut t = t0.unwrap_or_else(|| {
        (0..=8)
            .map(|i| bounds.lerp(i as f64 / 8.0))
            .map(|t| (t, (point - curve.sample(t)).magnitude()))
            .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap()
            .0
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

/// Finds intersection between two curves, if one exists.
pub fn intersect_curves(
    curve1: &impl ParametricCurve2d,
    curve2: &impl ParametricCurve2d,
) -> Option<(f64, f64)> {
    let mut pos1 = curve1.bounds().min;
    let mut pos2 = None;
    while pos1 < curve1.bounds().max {
        let point1 = curve1.sample(pos1);
        pos2 = project_point_onto_curve(curve2, point1, 0.1, pos2);
        if let Some(pos2) = pos2 {
            let point2 = curve2.sample(pos2);
            let distance = point1.distance(point2);
            if distance < 0.01 {
                return Some((pos1, pos2));
            } else {
                pos1 += distance;
            }
        } else {
            pos1 += 1.0;
        }
    }
    None
}

/// Finds a set of evenly spaced points along the given parametric curve.
pub fn equidistant_points_along_curve(
    curve: &impl ParametricCurve2d,
    dist: f64,
) -> (Vec<Point2d>, f64) {
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
            } else if f > 1.01 {
                ts.max = new_t;
                ps[1] = new_p;
                dists.max = new_dist;
            } else {
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

/// Approximates a curve by subdividing it until all segments are no longer than `max_length` units in length.
pub fn subdivided_points_along_curve(
    curve: &impl ParametricCurve2d,
    max_length: f64,
) -> Vec<Point2d> {
    SubdividedSamples::new(curve, max_length)
        .map(|(_, p)| p)
        .collect()
}

/// Approximates a curve by subdividing it until all segments are no longer than `max_length` units in length.
pub fn subdivided_samples_along_curve(
    curve: &impl ParametricCurve2d,
    max_length: f64,
) -> impl Iterator<Item = (f64, Point2d)> + '_ {
    SubdividedSamples::new(curve, max_length)
}

pub struct SubdividedSamples<'a, C> {
    curve: &'a C,
    stack: Vec<(f64, Point2d)>,
    length2: f64,
}

impl<'a, C: ParametricCurve2d> SubdividedSamples<'a, C> {
    fn new(curve: &'a C, max_length: f64) -> Self {
        let Interval { min, max } = curve.bounds();
        let mid = 0.5 * (min + max);
        Self {
            curve,
            stack: vec![
                (max, curve.sample(max)),
                (mid, curve.sample(mid)),
                (min, curve.sample(min)),
            ],
            length2: max_length.powi(2),
        }
    }
}

impl<'a, C: ParametricCurve2d> Iterator for SubdividedSamples<'a, C> {
    type Item = (f64, Point2d);

    fn next(&mut self) -> Option<Self::Item> {
        let (t1, p1) = self.stack.pop()?;
        if let Some((mut t2, mut p2)) = self.stack.last().copied() {
            while (p2 - p1).magnitude2() > self.length2 {
                let mid_t = 0.5 * (t1 + t2);
                (t2, p2) = (mid_t, self.curve.sample(mid_t));
                self.stack.push((t2, p2));
            }
        }
        Some((t1, p1))
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::math::LineSegment2d;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    pub fn equidistant_points_along_curve_is_stable() {
        for i in 0..100 {
            let len = 0.1 * i as f64;
            let points = equidistant_points_along_curve(
                &LineSegment2d::from_ends(Point2d::new(10.0, 10.0), Point2d::new(10.0 + len, 10.0)),
                0.5,
            );
            assert_approx_eq!(points.1, len);
            for point in points.0.into_iter() {
                assert!(!point.x.is_nan() && !point.y.is_nan());
            }
        }
    }
}
