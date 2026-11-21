//! A road's reference line: evaluating its pieces, and sampling it as a
//! polyline that lanes are offset from.

use glam::DVec2;

use crate::{Piece, Pose, Profile};

/// The most a sampled polyline strays from the curve it samples, in meters.
const TOLERANCE: f64 = 0.01;
/// The most a sampled polyline turns at one vertex, in radians: 3°.
const MAX_TURN: f64 = 3.0 * std::f64::consts::PI / 180.0;
/// Samples per Bézier for measuring its length.
const BEZIER_TABLE: usize = 256;

/// A piece placed in the world, ready to evaluate by distance along it.
enum Placed {
    Line {
        start: DVec2,
        dir: DVec2,
        length: f64,
    },
    Arc {
        start: DVec2,
        heading: f64,
        curvature: f64,
        length: f64,
    },
    /// Control points, and the length at each of [`BEZIER_TABLE`] + 1 even
    /// steps in its parameter.
    Bezier {
        p: [DVec2; 4],
        lengths: Vec<f64>,
    },
}

impl Placed {
    fn new(from: Pose, piece: &Piece) -> Self {
        match *piece {
            Piece::Line { length } => Placed::Line { start: from.pos, dir: from.dir(), length },
            Piece::Arc { curvature, length } if curvature.abs() < 1e-12 => {
                Placed::Line { start: from.pos, dir: from.dir(), length }
            }
            Piece::Arc { length, curvature } => {
                Placed::Arc { start: from.pos, heading: from.heading, curvature, length }
            }
            Piece::Bezier { c1, c2, end } => {
                let p = [from.pos, c1, c2, end];
                let mut lengths = vec![0.0];
                let mut last = p[0];
                for i in 1..=BEZIER_TABLE {
                    let point = bezier(&p, i as f64 / BEZIER_TABLE as f64);
                    lengths.push(lengths[i - 1] + point.distance(last));
                    last = point;
                }
                Placed::Bezier { p, lengths }
            }
        }
    }

    fn length(&self) -> f64 {
        match self {
            Placed::Line { length, .. } | Placed::Arc { length, .. } => *length,
            Placed::Bezier { lengths, .. } => lengths[BEZIER_TABLE],
        }
    }

    /// The point `s` meters along, and the heading there.
    fn pose(&self, s: f64) -> Pose {
        match *self {
            Placed::Line { start, dir, .. } => {
                Pose { pos: start + s * dir, heading: dir.to_angle() }
            }
            Placed::Arc { start, heading, curvature: k, .. } => {
                let h = heading + k * s;
                let offset = DVec2::new(h.sin() - heading.sin(), heading.cos() - h.cos()) / k;
                Pose { pos: start + offset, heading: h }
            }
            Placed::Bezier { ref p, ref lengths } => {
                let i = lengths.partition_point(|&l| l <= s).clamp(1, BEZIER_TABLE);
                let (l0, l1) = (lengths[i - 1], lengths[i]);
                let u = if l1 > l0 {
                    ((s - l0) / (l1 - l0)).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let t = (i as f64 - 1.0 + u) / BEZIER_TABLE as f64;
                Pose { pos: bezier(p, t), heading: bezier_tangent(p, t).to_angle() }
            }
        }
    }

    /// Its curvature at its start, or at its end if `at_end`, in 1/m.
    fn end_curvature(&self, at_end: bool) -> f64 {
        match *self {
            Placed::Line { .. } => 0.0,
            Placed::Arc { curvature, .. } => curvature.abs(),
            Placed::Bezier { ref p, .. } => {
                let t = if at_end { 1.0 } else { 0.0 };
                let d1 = bezier_tangent(p, t);
                let u = 1.0 - t;
                let d2 =
                    6.0 * u * (p[2] - 2.0 * p[1] + p[0]) + 6.0 * t * (p[3] - 2.0 * p[2] + p[1]);
                d1.perp_dot(d2).abs() / d1.length().powi(3)
            }
        }
    }

    /// Distances along it at which to sample it, so the polyline strays at most
    /// [`TOLERANCE`] and turns at most [`MAX_TURN`] per vertex. Excludes 0 and
    /// its length.
    fn stations(&self) -> Vec<f64> {
        let length = self.length();
        let n = match *self {
            Placed::Line { .. } => 1,
            Placed::Arc { curvature, .. } => (density(curvature.abs()) * length).ceil() as usize,
            Placed::Bezier { ref p, ref lengths } => {
                // Evenly spaced, as densely as its most demanding stretch of the
                // table needs.
                let mut most: f64 = 0.0;
                let mut last = bezier_tangent(p, 0.0);
                for i in 1..=BEZIER_TABLE {
                    let tangent = bezier_tangent(p, i as f64 / BEZIER_TABLE as f64);
                    let step = lengths[i] - lengths[i - 1];
                    if step > 0.0 {
                        most = most.max(density(last.angle_to(tangent).abs() / step));
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
/// strays at most [`TOLERANCE`] from it, a chord's sagitta, and turns at most
/// [`MAX_TURN`] per vertex.
fn density(k: f64) -> f64 {
    let sagitta = 2.0 * (1.0 - (TOLERANCE * k).min(1.0)).acos();
    k / MAX_TURN.min(sagitta)
}

/// How far from an end of curvature `k` the vertex next to it can go on the
/// tangent there and stray at most [`TOLERANCE`] from the curve: on the
/// tangent, a point `d` along is about `d²·k / 2` off it.
fn end_step(k: f64) -> f64 {
    (2.0 * TOLERANCE / k).sqrt()
}

fn bezier(p: &[DVec2; 4], t: f64) -> DVec2 {
    let u = 1.0 - t;
    u * u * u * p[0] + 3.0 * u * u * t * p[1] + 3.0 * u * t * t * p[2] + t * t * t * p[3]
}

fn bezier_tangent(p: &[DVec2; 4], t: f64) -> DVec2 {
    let u = 1.0 - t;
    let d = 3.0 * u * u * (p[1] - p[0]) + 6.0 * u * t * (p[2] - p[1]) + 3.0 * t * t * (p[3] - p[2]);
    // A control point on an end leaves no tangent there; look along the curve.
    if d.length_squared() > 1e-18 {
        d
    } else {
        p[3] - p[0]
    }
}

/// A cubic Bézier from `start`, sampled as evenly as the tolerance and the turn
/// per vertex allow, with no extra vertices at its ends: each vertex's distance
/// along it, and the vertex. For curves no lane runs beside, such as a
/// junction's connectors, which needn't end square.
pub(crate) fn sample_bezier(
    start: Pose,
    c1: DVec2,
    c2: DVec2,
    end: DVec2,
) -> (Vec<f64>, Vec<DVec2>) {
    let placed = Placed::new(start, &Piece::Bezier { c1, c2, end });
    let length = placed.length();
    let s: Vec<f64> = std::iter::once(0.0)
        .chain(placed.stations())
        .chain(std::iter::once(length))
        .collect();
    let points = s.iter().map(|&s| placed.pose(s).pos).collect();
    (s, points)
}

/// Where a reference line ends, and its heading there.
pub(crate) fn end_pose(start: Pose, pieces: &[Piece]) -> Pose {
    pieces.iter().fold(start, |pose, piece| {
        let placed = Placed::new(pose, piece);
        placed.pose(placed.length())
    })
}

/// A reference line sampled as a polyline.
pub(crate) struct Polyline {
    /// Distance along the reference line at each vertex.
    pub s: Vec<f64>,
    pub points: Vec<DVec2>,
    /// At each vertex, the left normal scaled so that offsetting by it keeps
    /// each segment parallel to the reference's: the miter.
    pub miters: Vec<DVec2>,
}

impl Polyline {
    /// Samples a road's reference line, with extra vertices where `profiles`
    /// ease, so lanes offset from it follow them.
    pub fn new(start: Pose, pieces: &[Piece], profiles: &[Profile]) -> Self {
        let mut placed = Vec::new();
        let mut pose = start;
        let mut s0 = 0.0;
        // Each piece, where it starts along the line.
        for piece in pieces {
            let p = Placed::new(pose, piece);
            pose = p.pose(p.length());
            let length = p.length();
            placed.push((s0, p));
            s0 += length;
        }
        let total = s0;
        let mut stations = vec![0.0, total];
        for (s0, p) in &placed {
            stations.push(*s0);
            stations.extend(p.stations().into_iter().map(|s| s0 + s));
        }
        for profile in profiles {
            if let Profile::Ease { from, to, start, end } = *profile {
                // Smoothstep's second derivative is at most 6·Δ/L², so linear
                // between n even vertices it strays at most 3·Δ / (4·n²).
                let n = (0.75 * (to - from).abs() / TOLERANCE).sqrt().ceil().max(1.0) as usize;
                stations.extend((0..=n).map(|i| start + (end - start) * i as f64 / n as f64));
            }
        }
        stations.retain(|&s| (0.0..=total).contains(&s));
        // The vertices next to the ends go on the end tangents, so the
        // polyline's end segments run along them: then lanes offset from it end
        // square across the road, side by side, as the simulation needs of
        // neighbors, and meet the next road's exactly. Each as far out as the
        // tolerance allows, and no further than the next vertex in.
        let inner = || stations.iter().copied().filter(|&s| s > 0.0 && s < total);
        let first_inner = inner().fold(total, f64::min);
        let last_inner = inner().fold(0.0, f64::max);
        let (first_piece, last_piece) = (&placed[0].1, &placed[placed.len() - 1].1);
        let start_step = end_step(first_piece.end_curvature(false)).min(first_inner);
        let end_step = end_step(last_piece.end_curvature(true)).min(total - last_inner);
        stations.extend([start_step, total - end_step]);
        stations.sort_by(f64::total_cmp);
        stations.dedup_by(|a, b| (*a - *b).abs() < 1e-6);

        let pose_at = |s: f64| {
            let i = placed.partition_point(|(s0, _)| *s0 <= s).saturating_sub(1);
            let (s0, p) = &placed[i];
            p.pose(s - s0)
        };
        let (first, last) = (pose_at(0.0), pose_at(total));
        let points: Vec<DVec2> = stations
            .iter()
            .map(|&s| {
                if s == start_step {
                    first.pos + start_step * first.dir()
                } else if s == total - end_step {
                    last.pos - end_step * last.dir()
                } else {
                    pose_at(s).pos
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
                    let (a, b) = (normal(i - 1), normal(i));
                    (a + b) / (1.0 + a.dot(b))
                }
            })
            .collect();
        Self { s: stations, points, miters }
    }

    /// The polyline offset by `offset` at each vertex.
    pub fn offset(&self, offset: &Profile) -> Vec<DVec2> {
        (0..self.points.len())
            .map(|i| self.points[i] + offset.at(self.s[i]) * self.miters[i])
            .collect()
    }
}
