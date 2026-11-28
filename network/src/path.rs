//! Paths, the curves roads are laid along: a start point and segments each
//! continuing from the last, and sampling one as a polyline that lanes are
//! offset from.

use glam::DVec2;

/// A curve in the plane: a start point and segments, each from where the last
/// ended, or a slice of one.
///
/// Corners are allowed where segments meet: vehicles round every vertex of a
/// lane (see `SimNetwork::smooth_point_along`), a sharp one only tightly.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct Path {
    start: DVec2,
    segments: Vec<Segment>,
    /// The stretch of it that is the path, by distance along the whole, if it
    /// is a slice of it.
    #[cfg_attr(
        feature = "serde",
        serde(default, skip_serializing_if = "Option::is_none")
    )]
    slice: Option<[f64; 2]>,
}

/// A piece of a path, from where the last ended.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Segment {
    /// Straight to a point.
    Line { to: DVec2 },
    /// A cubic Bézier to a point, with two control points. It continues
    /// smoothly from the last segment only if `c1` lies straight ahead.
    Cubic { c1: DVec2, c2: DVec2, to: DVec2 },
}

impl Path {
    /// A path starting at `start`, with no length until a segment is added.
    pub fn new(start: DVec2) -> Self {
        Self { start, segments: Vec::new(), slice: None }
    }

    /// The path, then straight on to `to`.
    pub fn line_to(mut self, to: DVec2) -> Self {
        self.segments.push(Segment::Line { to });
        self
    }

    /// The path, then a cubic Bézier to `to` with control points `c1` and `c2`.
    pub fn cubic_to(mut self, c1: DVec2, c2: DVec2, to: DVec2) -> Self {
        self.segments.push(Segment::Cubic { c1, c2, to });
        self
    }

    /// The stretch of the path from `from` to `to`, distances along it. A
    /// slice of a slice is measured along the slice. Segments added to a slice
    /// extend the whole path, not the slice.
    pub fn slice(&self, from: f64, to: f64) -> Self {
        let start = self.slice.map_or(0.0, |[a, _]| a);
        Self { slice: Some([start + from, start + to]), ..self.clone() }
    }

    /// How long it is, in meters.
    pub fn length(&self) -> f64 {
        let [a, b] = self.range(&self.placed());
        b - a
    }

    /// Each segment placed, with where it starts along the whole path.
    fn placed(&self) -> Vec<(f64, Placed)> {
        let mut out = Vec::new();
        let (mut from, mut s0) = (self.start, 0.0);
        for segment in &self.segments {
            let placed = match *segment {
                Segment::Line { to } => Placed::Line { start: from, end: to },
                Segment::Cubic { c1, c2, to } => Placed::cubic([from, c1, c2, to]),
            };
            let length = placed.length();
            out.push((s0, placed));
            s0 += length;
            from = segment.end();
        }
        out
    }

    /// The stretch of the whole path that is this one.
    fn range(&self, placed: &[(f64, Placed)]) -> [f64; 2] {
        let total = placed.last().map_or(0.0, |(s0, p)| s0 + p.length());
        self.slice.unwrap_or([0.0, total])
    }

    /// The path sampled as a polyline straying at most `tolerance` meters from
    /// it, or why it can't be: too short, or a slice reaching beyond it. The
    /// simulation's lanes and what is drawn of them are each sampled at their
    /// own tolerance.
    pub(crate) fn sample(&self, tolerance: f64) -> Result<Polyline, PathError> {
        let placed: Vec<_> = self.placed().into_iter().filter(|(_, p)| p.length() > 0.0).collect();
        let total = placed.last().map_or(0.0, |(s0, p)| s0 + p.length());
        if total < MIN_LENGTH {
            return Err(PathError::TooShort);
        }
        let [a, b] = self.range(&placed);
        if !(0.0 <= a && a < b && b <= total + 1e-9) {
            return Err(PathError::Slice { from: a, to: b, length: total });
        }
        let b = b.min(total);
        if b - a < MIN_LENGTH {
            return Err(PathError::TooShort);
        }
        let at = |s: f64| {
            let i = placed.partition_point(|(s0, _)| *s0 <= s).saturating_sub(1);
            let (s0, p) = &placed[i];
            (p, s - s0)
        };
        let pose = |s: f64| {
            let (p, s) = at(s);
            p.pose(s)
        };
        let curvature = |s: f64| {
            let (p, s) = at(s);
            p.curvature(s)
        };

        let mut stations = vec![a, b];
        for (s0, p) in &placed {
            stations.push(*s0);
            stations.extend(p.stations(tolerance).into_iter().map(|s| s0 + s));
        }
        stations.retain(|&s| (a..=b).contains(&s));
        // The vertices next to the ends go on the end tangents, so the
        // polyline's end segments run along them: then lanes offset from it end
        // square across the road, side by side, as the simulation needs of
        // neighbours, and meet the next road's exactly. Each as far out as the
        // tolerance allows, and on a curve no further than a quarter of the way
        // to the next vertex in: the polyline comes back onto the curve between
        // them, so it turns there by more than the curve does, by up to a
        // third more with the next vertex three times as far again, and
        // without limit as the two come together.
        let inner = || stations.iter().copied().filter(|&s| s > a && s < b);
        let first_inner = inner().fold(b, f64::min);
        let last_inner = inner().fold(a, f64::max);
        let step = |k: f64, gap: f64| match k > 0.0 {
            true => end_step(k, tolerance).min(gap / 4.0),
            false => gap,
        };
        let start_step = step(curvature(a), first_inner - a);
        let end_step = step(curvature(b), b - last_inner);
        stations.extend([a + start_step, b - end_step]);
        stations.sort_by(f64::total_cmp);
        stations.dedup_by(|x, y| (*x - *y).abs() < 1e-6);

        let (first, last) = (pose(a), pose(b));
        let points: Vec<DVec2> = stations
            .iter()
            .map(|&s| {
                if s == a + start_step {
                    first.0 + start_step * first.1
                } else if s == b - end_step {
                    last.0 - end_step * last.1
                } else {
                    pose(s).0
                }
            })
            .collect();

        let n = points.len();
        let normal = |i: usize| (points[i + 1] - points[i]).normalize().perp();
        let miters = (0..n)
            .map(|i| match i {
                0 => normal(0),
                i if i == n - 1 => normal(n - 2),
                i => {
                    let (l, r) = (normal(i - 1), normal(i));
                    (l + r) / (1.0 + l.dot(r))
                }
            })
            .collect();
        Ok(Polyline { points, miters })
    }
}

impl Segment {
    /// Where it ends.
    fn end(&self) -> DVec2 {
        match *self {
            Segment::Line { to } | Segment::Cubic { to, .. } => to,
        }
    }
}

/// Why a path can't be sampled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PathError {
    /// Shorter than [`MIN_LENGTH`], it or the slice.
    TooShort,
    /// A slice that isn't within the path it slices, or runs backward.
    Slice { from: f64, to: f64, length: f64 },
}

impl std::fmt::Display for PathError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match *self {
            PathError::TooShort => write!(f, "the path is shorter than {MIN_LENGTH} m"),
            PathError::Slice { from, to, length } => {
                write!(
                    f,
                    "the slice from {from} m to {to} m isn't within the path's {length} m"
                )
            }
        }
    }
}

/// The shortest a path may be, in meters.
pub const MIN_LENGTH: f64 = 0.01;

/// The most a sampled polyline turns at one vertex, in radians: 15°, a backstop
/// only. Vehicles' front axles round each vertex, so the polyline's own turns
/// don't show; the tolerance sets the spacing.
const MAX_TURN: f64 = 15.0 * std::f64::consts::PI / 180.0;
/// Samples per Bézier for measuring its length.
const BEZIER_TABLE: usize = 256;

/// A segment placed in the world, ready to evaluate by distance along it.
enum Placed {
    Line {
        start: DVec2,
        end: DVec2,
    },
    /// Its control points, and the length at each of [`BEZIER_TABLE`] + 1 even
    /// steps in its parameter.
    Cubic {
        p: [DVec2; 4],
        lengths: Vec<f64>,
    },
}

impl Placed {
    fn cubic(p: [DVec2; 4]) -> Self {
        let mut lengths = vec![0.0];
        let mut last = p[0];
        for i in 1..=BEZIER_TABLE {
            let point = bezier(&p, i as f64 / BEZIER_TABLE as f64);
            lengths.push(lengths[i - 1] + point.distance(last));
            last = point;
        }
        Placed::Cubic { p, lengths }
    }

    fn length(&self) -> f64 {
        match self {
            Placed::Line { start, end } => start.distance(*end),
            Placed::Cubic { lengths, .. } => lengths[BEZIER_TABLE],
        }
    }

    /// The Bézier's parameter `s` meters along it.
    fn parameter(lengths: &[f64], s: f64) -> f64 {
        let i = lengths.partition_point(|&l| l <= s).clamp(1, BEZIER_TABLE);
        let (l0, l1) = (lengths[i - 1], lengths[i]);
        let u = if l1 > l0 {
            ((s - l0) / (l1 - l0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        (i as f64 - 1.0 + u) / BEZIER_TABLE as f64
    }

    /// The point `s` meters along, and the unit direction there.
    fn pose(&self, s: f64) -> (DVec2, DVec2) {
        match *self {
            Placed::Line { start, end } => {
                let dir = (end - start).normalize();
                (start + s * dir, dir)
            }
            Placed::Cubic { ref p, ref lengths } => {
                let t = Self::parameter(lengths, s);
                (bezier(p, t), bezier_tangent(p, t).normalize())
            }
        }
    }

    /// Its curvature `s` meters along, unsigned, in 1/m.
    fn curvature(&self, s: f64) -> f64 {
        match *self {
            Placed::Line { .. } => 0.0,
            Placed::Cubic { ref p, ref lengths } => {
                let t = Self::parameter(lengths, s);
                let d1 = bezier_tangent(p, t);
                // A cubic's second derivative: 3·2 times the Bézier of its
                // control points' second differences.
                let d2: Vec<DVec2> = p.windows(3).map(|w| w[2] - 2.0 * w[1] + w[0]).collect();
                let d2 = bezier(&d2, t) * 6.0;
                d1.perp_dot(d2).abs() / d1.length().powi(3).max(1e-12)
            }
        }
    }

    /// Distances along it at which to sample it, so the polyline strays at most
    /// `tolerance` and turns at most [`MAX_TURN`] per vertex. Excludes 0 and
    /// its length.
    fn stations(&self, tolerance: f64) -> Vec<f64> {
        let length = self.length();
        let n = match *self {
            Placed::Line { .. } => 1,
            Placed::Cubic { ref p, ref lengths } => {
                // Evenly spaced, as densely as its most demanding stretch of the
                // table needs.
                let mut most: f64 = 0.0;
                let mut last = bezier_tangent(p, 0.0);
                for i in 1..=BEZIER_TABLE {
                    let tangent = bezier_tangent(p, i as f64 / BEZIER_TABLE as f64);
                    let step = lengths[i] - lengths[i - 1];
                    if step > 0.0 {
                        most = most.max(density(last.angle_to(tangent).abs() / step, tolerance));
                    }
                    last = tangent;
                }
                (most * length).ceil() as usize
            }
        };
        let n = n.max(1);
        (1..n).map(|i| length * i as f64 / n as f64).collect()
    }
}

/// How many vertices per meter a curve of curvature `k` needs, so the polyline
/// strays at most `tolerance` from it, a chord's sagitta, and turns at most
/// [`MAX_TURN`] per vertex.
fn density(k: f64, tolerance: f64) -> f64 {
    let sagitta = 2.0 * (1.0 - (tolerance * k).min(1.0)).acos();
    k / MAX_TURN.min(sagitta)
}

/// How far from an end of curvature `k` the vertex next to it can go on the
/// tangent there and stray at most `tolerance` from the curve: on the
/// tangent, a point `d` along is about `d²·k / 2` off it.
fn end_step(k: f64, tolerance: f64) -> f64 {
    (2.0 * tolerance / k).sqrt()
}

/// The Bézier with control points `p`, of any degree, at `t`: by de Casteljau's
/// algorithm.
fn bezier(p: &[DVec2], t: f64) -> DVec2 {
    let mut q = p.to_vec();
    for k in (1..q.len()).rev() {
        for i in 0..k {
            q[i] = q[i].lerp(q[i + 1], t);
        }
    }
    q[0]
}

/// The derivative of the Bézier with control points `p` at `t`: its degree
/// times the Bézier of their differences, or, where a control point on an end
/// leaves none, the way along the curve.
fn bezier_tangent(p: &[DVec2], t: f64) -> DVec2 {
    let d: Vec<DVec2> = p.windows(2).map(|w| w[1] - w[0]).collect();
    let d = bezier(&d, t) * (p.len() - 1) as f64;
    if d.length_squared() > 1e-18 {
        d
    } else {
        p[p.len() - 1] - p[0]
    }
}

/// A path sampled as a polyline.
pub(crate) struct Polyline {
    pub points: Vec<DVec2>,
    /// At each vertex, the left normal scaled so that offsetting by it keeps
    /// each segment parallel to the path's: the miter.
    pub miters: Vec<DVec2>,
}

impl Polyline {
    /// The polyline offset by `offset` to the left.
    pub fn offset(&self, offset: f64) -> Vec<DVec2> {
        self.points.iter().zip(&self.miters).map(|(&p, &m)| p + offset * m).collect()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    const fn p(x: f64, y: f64) -> DVec2 {
        DVec2::new(x, y)
    }

    #[test]
    fn a_line_is_its_two_ends() {
        let line = Path::new(p(0.0, 0.0)).line_to(p(100.0, 0.0)).sample(0.01).unwrap();
        assert_eq!(line.points, [p(0.0, 0.0), p(100.0, 0.0)]);
    }

    #[test]
    fn a_slice_runs_from_and_to_where_it_says() {
        let path = Path::new(p(0.0, 0.0)).line_to(p(100.0, 0.0)).line_to(p(100.0, 100.0));
        assert!((path.length() - 200.0).abs() < 1e-9);
        let slice = path.slice(50.0, 150.0);
        assert!((slice.length() - 100.0).abs() < 1e-9);
        let line = slice.sample(0.01).unwrap();
        assert_eq!(line.points, [p(50.0, 0.0), p(100.0, 0.0), p(100.0, 50.0)]);
        // A slice of the slice is measured along it.
        let inner = slice.slice(25.0, 75.0).sample(0.01).unwrap();
        assert_eq!(inner.points, [p(75.0, 0.0), p(100.0, 0.0), p(100.0, 25.0)]);
    }

    #[test]
    fn a_slice_beyond_the_path_is_refused() {
        let path = Path::new(p(0.0, 0.0)).line_to(p(100.0, 0.0));
        assert!(matches!(
            path.slice(50.0, 150.0).sample(0.01),
            Err(PathError::Slice { .. })
        ));
        assert!(matches!(
            path.slice(60.0, 40.0).sample(0.01),
            Err(PathError::Slice { .. })
        ));
        assert!(matches!(
            Path::new(p(0.0, 0.0)).sample(0.01),
            Err(PathError::TooShort)
        ));
    }

    /// A quarter circle of radius `r`, near enough, from a cubic.
    fn quarter_circle(r: f64) -> Path {
        let k = 0.5523 * r;
        Path::new(p(0.0, 0.0)).cubic_to(p(k, 0.0), p(r, r - k), p(r, r))
    }

    #[test]
    fn a_cubic_s_curvature_is_its_curve_s() {
        let placed = quarter_circle(50.0).placed();
        let (_, cubic) = &placed[0];
        for s in [0.0, 0.5 * cubic.length(), cubic.length()] {
            let k = cubic.curvature(s);
            assert!((k * 50.0 - 1.0).abs() < 0.03, "{} at {s}", 1.0 / k);
        }
    }

    #[test]
    fn a_curve_s_polyline_turns_little_more_sharply_at_its_ends_than_it_does() {
        // Over the mean of the segments either side, as the simulation's speed
        // profiles take a lane's bends: a sharper turn slows vehicles.
        for r in [20.0, 50.0, 160.0, 500.0] {
            let points = quarter_circle(r).sample(0.01).unwrap().points;
            for w in points.windows(3) {
                let (a, b) = (w[1] - w[0], w[2] - w[1]);
                let k = a.angle_to(b).abs() / (0.5 * (a.length() + b.length()));
                assert!(k * r < 1.4, "radius {r}: a vertex turns as at {}", 1.0 / k);
            }
        }
    }

    #[test]
    fn a_cubic_is_sampled_within_tolerance_and_ends_on_its_tangents() {
        // A quarter circle, near enough, of radius 50.
        let k = 0.5523 * 50.0;
        let path = Path::new(p(0.0, 0.0)).cubic_to(p(k, 0.0), p(50.0, 50.0 - k), p(50.0, 50.0));
        let line = path.sample(0.01).unwrap();
        assert!(line.points.len() > 10);
        assert!(
            (line.points[1].y).abs() < 1e-9,
            "the second vertex is on the start tangent"
        );
        let n = line.points.len();
        assert!(
            (line.points[n - 2].x - 50.0).abs() < 1e-9,
            "and the last but one on the end's"
        );
        // Each chord's midpoint is within the tolerance of the circle.
        for w in line.points.windows(2) {
            let mid = 0.5 * (w[0] + w[1]);
            let off = (mid.distance(p(0.0, 50.0)) - 50.0).abs();
            assert!(off < 0.05, "{off}");
        }
    }
}
