use cgmath::prelude::*;
use crate::util::Interval;
use super::{Point2d, Vector2d};
use super::curve::ParametricCurve2d;

/// A line segment
#[derive(Copy, Clone)]
pub struct LineSegment2d {
    start: Point2d,
    tangent: Vector2d,
    length: f64
}

impl LineSegment2d {
    /// Creates a 2D line segment given two endpoints.
    pub fn from_ends(start: Point2d, end: Point2d) -> Self {
        let diff = end.to_vec() - start.to_vec();
        let length = diff.magnitude();
        let mut tangent = diff / length;
        if tangent.x.is_nan() {
            tangent = Vector2d::zero();
        }
        Self { start, tangent, length }
    }

    /// The start point of the line segment.
    pub const fn start(&self) -> Point2d {
        self.start
    }

    /// The end point of the line segment.
    pub fn end(&self) -> Point2d {
        self.start + self.length * self.tangent
    }
}

impl ParametricCurve2d for LineSegment2d {
    fn sample(&self, t: f64) -> Point2d {
        self.start + t * self.tangent
    }

    fn bounds(&self) -> Interval<f64> {
        Interval::new(0.0, self.length)
    }
    
    fn sample_dt(&self, _t: f64) -> Vector2d {
        self.tangent
    }
}

/// A quadratic bezier curve
#[derive(Copy, Clone)]
pub struct QuadraticBezier2d {
    points: [Point2d; 3]
}

impl QuadraticBezier2d {
    pub const fn new(points: &[Point2d; 3]) -> Self {
        Self { points: *points }
    }
}

impl ParametricCurve2d for QuadraticBezier2d {
    fn sample(&self, t: f64) -> Point2d {
        let t1 = 1.0 - t;
        Point2d::from_vec(t1 * t1 * self.points[0].to_vec()
            + 2.0 * t1 * t * self.points[1].to_vec()
            + t * t * self.points[2].to_vec())
    }

    fn bounds(&self) -> Interval<f64> {
        Interval { min: 0.0, max: 1.0 }
    }
    
    fn sample_dt(&self, t: f64) -> Vector2d {
        let t1 = 1.0 - t;
        -2.0 * t1 * self.points[0].to_vec()
            + (2.0 - 4.0 * t) * self.points[1].to_vec()
            + 2.0 * t * self.points[2].to_vec()
    }
    
    fn sample_dt2(&self, t: f64) -> Vector2d {
        let t1 = 1.0 - t;
        2.0 * self.points[0].to_vec()
            - 4.0 * self.points[1].to_vec()
            + 2.0 * self.points[2].to_vec()
    }
}

/// A cubic bezier curve
#[derive(Copy, Clone, Debug)]
pub struct CubicBezier2d {
    points: [Point2d; 4]
}

impl CubicBezier2d {
    pub const fn new(points: &[Point2d; 4]) -> Self {
        Self { points: *points }
    }

    pub fn line(start: Point2d, end: Point2d) -> Self {
        let s = start.to_vec();
        let e = end.to_vec();
        let ps = [s, s.lerp(e, 1./3.), s.lerp(e, 2./3.), e];
        Self { points: ps.map(Point2d::from_vec) }
    }

    pub fn subdivide(&self, t: f64) -> [CubicBezier2d; 2] {
        let [p00, p01, p02, p03] = self.points.map(|x| x.to_vec());
        let p10 = p00.lerp(p01, t);
        let p11 = p01.lerp(p02, t);
        let p12 = p02.lerp(p03, t);
        let p20 = p10.lerp(p11, t);
        let p21 = p11.lerp(p12, t);
        let p30 = p20.lerp(p21, t);
        let curves = [
            [p00, p10, p20, p30],
            [p30, p21, p12, p03]
        ];
        curves.map(|p| CubicBezier2d { points: p.map(Point2d::from_vec) })
    }

    pub fn reverse(&mut self) {
        self.points.reverse()
    }
}

impl ParametricCurve2d for CubicBezier2d {
    fn sample(&self, t: f64) -> Point2d {
        let t1 = 1.0 - t;
        Point2d::from_vec(t1 * t1 * t1 * self.points[0].to_vec()
            + 3.0 * t1 * t1 * t * self.points[1].to_vec()
            + 3.0 * t1 * t * t * self.points[2].to_vec()
            + t * t * t * self.points[3].to_vec())
    }

    fn bounds(&self) -> Interval<f64> {
        Interval { min: 0.0, max: 1.0 }
    }
    
    fn sample_dt(&self, t: f64) -> Vector2d {
        let t1 = 1.0 - t;
        (-3.0 * t1 * t1) * self.points[0].to_vec()
            + (9.0 * t * t - 12.0 * t + 3.0) * self.points[1].to_vec()
            + (-9.0 * t * t + 6.0 * t) * self.points[2].to_vec()
            + (3.0 * t * t) * self.points[3].to_vec()
    }
}