//! A lane's geometry: its centerline as a polyline, its edges, and the mapping
//! between world positions and lane coordinates.

use std::f64::consts::TAU;
use std::ops::Range;

use arrayvec::ArrayVec;

use glam::DVec2;

use crate::network::Side;

/// The sharpest turn allowed at a single vertex: 1/16 of a revolution.
const MAX_TURN: f64 = TAU / 16.0;

/// A lane's shape: a polyline centerline, and how far its edges lie to either
/// side of it at each vertex. The centerline needn't be centered: each side's
/// half-width is its own, and varies linearly in `s` between vertices.
///
/// Positions are expressed in lane coordinates `(s, t)`: `s` is the distance in
/// meters along the centerline and `t` is the lateral offset in meters, positive
/// to the left. The mapping is continuous across vertices, so a vehicle can move
/// along the lane without `s` jumping.
///
/// Every point and half-width is snapped to a 1 mm grid as the geometry is
/// built, so that it serializes exactly as a short string: see
/// [`LaneGeometry::to_mm_string`].
#[derive(Clone, Debug)]
pub struct LaneGeometry {
    /// The segments comprising the lane.
    segs: Vec<Segment>,
    /// How far the left and right edges lie from the centerline at each vertex,
    /// in meters: one more than there are segments.
    half_widths: Vec<[f64; 2]>,
    /// Length of the lane in meters.
    length: f64,
}

#[derive(Clone, Copy, Debug)]
struct Segment {
    /// Position of the centerline at the start of the segment.
    start: DVec2,
    /// Unit direction vector of the segment.
    dir: DVec2,
    /// Cumulative length of the lane at the start of the segment.
    cum_s: f64,
    /// Adjustment factors, -tan(θ₀/2) and (t0 - t1) / len.
    adj: (f64, f64),
}

/// A position in lane coordinates.
///
/// Besides `s` and `t`, this carries a private cache of the segment the point was
/// found in, which [`LaneGeometry::world_to_lane_near`] uses to make the next query
/// cheap. The cache is only ever a hint: a stale one (even from a different lane)
/// makes a query slower, never wrong.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LanePos {
    /// Longitudinal position along the centerline in meters.
    pub s: f64,
    /// Lateral offset from the centerline in meters, positive to the left.
    pub t: f64,
    seg: u32,
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LaneError {
    #[error("a lane needs at least two distinct points")]
    TooFewPoints,
    /// `vertex` indexes the points after consecutive duplicates have been removed.
    #[error("turn at vertex {vertex} is sharper than the allowed maximum")]
    TurnTooSharp { vertex: usize },
    #[error("lane folds at segment {segment}: turn too sharp for its width")]
    Folds { segment: usize },
}

impl LaneGeometry {
    /// Builds a lane's geometry from its centerline polyline and, at each point,
    /// how far its left and right edges lie from it, in meters.
    ///
    /// Points and half-widths are snapped to the nearest millimeter, then
    /// consecutive duplicate points are removed, with their half-widths.
    pub(crate) fn new(points: &[DVec2], half_widths: &[[f64; 2]]) -> Result<Self, LaneError> {
        assert_eq!(
            points.len(),
            half_widths.len(),
            "a half-width pair per point"
        );
        let max_tan_half = (0.5 * MAX_TURN).tan();

        let mut vertices: Vec<(DVec2, [f64; 2])> = points
            .iter()
            .zip(half_widths)
            .map(|(p, w)| (DVec2::new(snap(p.x), snap(p.y)), w.map(snap)))
            .collect();
        vertices.dedup_by(|a, b| a.0 == b.0);
        if vertices.len() < 2 {
            return Err(LaneError::TooFewPoints);
        }
        let (points, half_widths): (Vec<DVec2>, Vec<[f64; 2]>) = vertices.into_iter().unzip();

        let lens: Vec<f64> = points.windows(2).map(|w| w[0].distance(w[1])).collect();
        let dirs: Vec<DVec2> = points.windows(2).map(|w| (w[1] - w[0]).normalize()).collect();

        let n = dirs.len();
        let mut tan_half = vec![0.0; n + 1];
        for i in 1..n {
            let (a, b) = (dirs[i - 1], dirs[i]);
            let th = a.perp_dot(b) / (1.0 + a.dot(b));
            // NaN means a full reversal (1 + a·b = 0).
            if th.is_nan() || th.abs() > max_tan_half {
                return Err(LaneError::TurnTooSharp { vertex: i });
            }
            tan_half[i] = th;
        }

        let mut cum_s = 0.0;
        let mut segs = Vec::with_capacity(n);
        for i in 0..n {
            let t0 = -tan_half[i];
            let t1 = tan_half[i + 1];
            let k = (t0 - t1) / lens[i];
            // The mapping stretches by 1 + t·k across the segment, which mustn't
            // reach zero anywhere between its edges, on either side.
            let [left, right] = [0, 1].map(|e| half_widths[i][e].max(half_widths[i + 1][e]));
            if 1.0 + left * k <= 0.0 || 1.0 - right * k <= 0.0 {
                return Err(LaneError::Folds { segment: i });
            }
            segs.push(Segment { start: points[i], dir: dirs[i], cum_s, adj: (t0, k) });
            cum_s += lens[i];
        }
        Ok(Self { segs, half_widths, length: cum_s })
    }

    /// Length of the centerline in meters.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// How far the edge on `side` lies from the centerline at `pos`, in meters:
    /// interpolated linearly in `s` between the vertices either side of it.
    pub fn half_width(&self, pos: LanePos, side: Side) -> f64 {
        let i = pos.seg as usize;
        let s0 = self.segs[i].cum_s;
        let s1 = self.segs.get(i + 1).map_or(self.length, |seg| seg.cum_s);
        let u = ((pos.s - s0) / (s1 - s0)).clamp(0.0, 1.0);
        let e = match side {
            Side::Left => 0,
            Side::Right => 1,
        };
        let (a, b) = (self.half_widths[i][e], self.half_widths[i + 1][e]);
        a + u * (b - a)
    }

    /// Whether `pos` lies between the lane's edges, at its `s`.
    pub fn between_edges(&self, pos: LanePos) -> bool {
        pos.t <= self.half_width(pos, Side::Left) && -pos.t <= self.half_width(pos, Side::Right)
    }

    /// Creates a new `LanePos` for the given `s` and `t`.
    pub fn lane_pos(&self, s: f64, t: f64) -> LanePos {
        LanePos { s, t, seg: self.seg_for_s(s) as u32 }
    }

    /// Converts lane coordinates to a world position.
    pub fn lane_to_world(&self, pos: LanePos) -> DVec2 {
        self.segs[pos.seg as usize].lane_to_world(pos.s, pos.t)
    }

    /// Unit direction of the centerline at `s`.
    ///
    /// The centerline is piecewise straight, so this jumps at vertices.
    pub fn tangent_at(&self, pos: LanePos) -> DVec2 {
        self.segs[pos.seg as usize].dir
    }

    /// Unit direction, in world space, of the curve through `pos` whose offset
    /// changes by `slope` (dt/ds) along the lane. With no slope it follows the
    /// line of constant `t` through `pos`, which is the centerline's direction
    /// only on the centerline. Exact: within a segment, lane coordinates map to
    /// world space by a fixed bilinear map.
    pub fn direction_at(&self, pos: LanePos, slope: f64) -> DVec2 {
        self.segs[pos.seg as usize].direction(pos.s, pos.t, slope)
    }

    /// The slope, dt/ds, at `pos` of a curve heading along `dir` in world space:
    /// the inverse of [`LaneGeometry::direction_at`].
    pub fn slope_toward(&self, pos: LanePos, dir: DVec2) -> f64 {
        self.segs[pos.seg as usize].slope(pos.s, pos.t, dir)
    }

    /// Converts a world position to lane coordinates, searching the whole lane.
    ///
    /// Use this for the first query; afterwards prefer
    /// [`LaneGeometry::world_to_lane_near`]. The result is only meaningful for
    /// points within roughly the lane's width of the centerline, since the lines
    /// of constant `s` can cross further out.
    pub fn world_to_lane(&self, pos: DVec2) -> LanePos {
        let seg = self.nearest_start(pos);
        self.world_to_lane_near(pos, LanePos { s: 0.0, t: 0.0, seg: seg as u32 })
    }

    /// Converts a world position to lane coordinates, starting the search from the
    /// segment of a previous result. Cheap when the point has moved only a little.
    pub fn world_to_lane_near(&self, pos: DVec2, hint: LanePos) -> LanePos {
        let seg = self.find_segment_with_hint(pos, hint.seg as usize);
        let (s, t) = self.segs[seg].world_to_lane(pos);
        LanePos { s, t, seg: seg as u32 }
    }

    /// How far the left and right edges lie from the centerline at each vertex,
    /// in meters, in the order of [`LaneGeometry::centerline`].
    pub fn half_widths(&self) -> &[[f64; 2]] {
        &self.half_widths
    }

    /// The geometry as a string of vertices in millimeters, separated by spaces,
    /// each `x,y,left,right`: the centerline point and the half-widths there.
    /// With three values the half-widths are equal, and with two they are the
    /// previous vertex's, so a lane of constant width gives them once:
    ///
    /// ```text
    /// 0,0,1750 90000,0 100000,2000,1750,2000
    /// ```
    ///
    /// Everything is on the millimeter grid, so it reads back exactly.
    pub fn to_mm_string(&self) -> String {
        let mm = |v: f64| (v * PER_METER).round() as i64;
        let mut out = String::new();
        let mut last = None;
        for (point, &[left, right]) in self.centerline().zip(&self.half_widths) {
            if last.is_some() {
                out.push(' ');
            }
            out += &format!("{},{}", mm(point.x), mm(point.y));
            if last != Some([left, right]) {
                out += &format!(",{}", mm(left));
                if right != left {
                    out += &format!(",{}", mm(right));
                }
            }
            last = Some([left, right]);
        }
        out
    }

    /// Reads a string from [`LaneGeometry::to_mm_string`] back into a lane's geometry.
    pub fn from_mm_string(text: &str) -> Result<Self, ParseLaneError> {
        let meters = |v: &str| v.parse::<i64>().map(|mm| mm as f64 / PER_METER);
        let mut points = Vec::new();
        let mut half_widths: Vec<[f64; 2]> = Vec::new();
        for vertex in text.split_whitespace() {
            let values = vertex
                .split(',')
                .map(meters)
                .collect::<Result<Vec<f64>, _>>()
                .map_err(|_| ParseLaneError::Malformed(vertex.into()))?;
            let widths = match values[..] {
                [_, _] => *half_widths.last().ok_or(ParseLaneError::NoWidths)?,
                [_, _, w] => [w, w],
                [_, _, left, right] => [left, right],
                _ => return Err(ParseLaneError::Malformed(vertex.into())),
            };
            points.push(DVec2::new(values[0], values[1]));
            half_widths.push(widths);
        }
        Ok(Self::new(&points, &half_widths)?)
    }

    /// The centerline vertices, from start to end.
    /// Each vertex's `s`, from the start: where the lane's geometry, piecewise
    /// linear, may change direction or width.
    pub fn vertex_s(&self) -> impl Iterator<Item = f64> + '_ {
        self.segs.iter().map(|s| s.cum_s).chain(std::iter::once(self.length))
    }

    /// The centerline's vertices, as in [`LaneGeometry::vertex_s`].
    pub fn centerline(&self) -> impl Iterator<Item = DVec2> + '_ {
        let seg = (self.segs.len() - 1) as u32;
        let end = self.lane_to_world(LanePos { seg, s: self.length, t: 0.0 });
        self.segs.iter().map(|s| s.start).chain(std::iter::once(end))
    }

    /// The closed outline of the lane: the left edge forwards, then the right edge
    /// backwards, through each vertex's own half-widths.
    pub fn outline(&self) -> impl Iterator<Item = DVec2> + '_ {
        let last = self.segs.len() - 1;
        // The vertices' `s`, and the segment that each lies on, the last one's end
        // on the last segment.
        let vertex = move |i: usize| match i {
            i if i <= last => (i, self.segs[i].cum_s),
            _ => (last, self.length),
        };
        let edge = move |i: usize, e: usize, sign: f64| {
            let (seg, s) = vertex(i);
            self.segs[seg].lane_to_world(s, sign * self.half_widths[i][e])
        };
        let left = (0..=last + 1).map(move |i| edge(i, 0, 1.0));
        let right = (0..=last + 1).rev().map(move |i| edge(i, 1, -1.0));
        left.chain(right)
    }

    /// Where this lane's outline overlaps `other`'s: the stretch of each, in `s`,
    /// from the first overlap to the last, or `None` if they don't overlap. Two
    /// lanes that overlap in several places get the union.
    ///
    /// Each segment's piece of the outline, between its ribs, is a convex
    /// quadrilateral, so every pair of pieces is clipped one against the other.
    /// Pieces that only touch, as neighboring lanes' do along their shared edge,
    /// leave a sliver no thicker than rounding and don't count: an overlap
    /// counts only if it is, on average, at least [`SAME_POINT`] thick. It compares every
    /// pair of segments, so it is for building networks, not for hot paths.
    pub(crate) fn overlap(&self, other: &LaneGeometry) -> Option<[Range<f64>; 2]> {
        let mut ranges: Option<[Range<f64>; 2]> = None;
        for i in 0..self.segs.len() {
            let mine = self.quad(i);
            let (lo, hi) = bounds(&mine);
            for j in 0..other.segs.len() {
                let theirs = other.quad(j);
                let (other_lo, other_hi) = bounds(&theirs);
                if lo.cmpgt(other_hi).any() || other_lo.cmpgt(hi).any() {
                    continue;
                }
                let clipped = clip(&mine, &theirs);
                // An empty clip has neither area nor perimeter: it doesn't count.
                if area(&clipped) <= SAME_POINT * 0.5 * perimeter(&clipped) {
                    continue;
                }
                for &p in &clipped {
                    let s = [
                        self.segs[i].world_to_lane(p).0.clamp(0.0, self.length),
                        other.segs[j].world_to_lane(p).0.clamp(0.0, other.length),
                    ];
                    let ranges = ranges.get_or_insert([s[0]..s[0], s[1]..s[1]]);
                    for (range, s) in ranges.iter_mut().zip(s) {
                        range.start = range.start.min(s);
                        range.end = range.end.max(s);
                    }
                }
            }
        }
        ranges
    }

    /// Segment `i`'s piece of the outline, between its ribs, counterclockwise from
    /// its rear left corner.
    fn quad(&self, i: usize) -> [DVec2; 4] {
        let seg = &self.segs[i];
        let end = self.segs.get(i + 1).map_or(self.length, |next| next.cum_s);
        let corner = |s: f64, vertex: usize, side: Side| {
            let [left, right] = self.half_widths[vertex];
            let t = match side {
                Side::Left => left,
                Side::Right => -right,
            };
            seg.lane_to_world(s, t)
        };
        [
            corner(seg.cum_s, i, Side::Left),
            corner(seg.cum_s, i, Side::Right),
            corner(end, i + 1, Side::Right),
            corner(end, i + 1, Side::Left),
        ]
    }

    /// The last segment whose start is at or before `s`.
    fn seg_for_s(&self, s: f64) -> usize {
        self.segs.partition_point(|g| g.cum_s <= s).saturating_sub(1)
    }

    /// The segment whose start point is closest to `pos`; a starting guess for the walk.
    fn nearest_start(&self, pos: DVec2) -> usize {
        self.segs
            .iter()
            .enumerate()
            .map(|(i, seg)| (i, (pos - seg.start).length_squared()))
            .filter(|(_, d)| !d.is_nan())
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map_or(0, |(i, _)| i)
    }

    /// Walks from `hint` to the segment containing `pos`. Any `hint` is safe.
    fn find_segment_with_hint(&self, pos: DVec2, hint: usize) -> usize {
        let last = self.segs.len() - 1;
        let mut idx = hint.min(last);
        while idx > 0 && !self.segs[idx].test_point(pos) {
            idx -= 1;
        }
        while idx < last && self.segs[idx + 1].test_point(pos) {
            idx += 1;
        }
        idx
    }
}

/// The nearest millimeter to a length in meters.
/// Grid steps per meter.
const PER_METER: f64 = 1000.0;

/// The grid every lane point and half-width is snapped to, in meters: 1 mm.
pub const GRID: f64 = 1.0 / PER_METER;

/// How far apart, in meters, two points on the [`GRID`] can be and still have
/// come from the same point, or from points apart only by rounding: under two
/// steps. Snapping moves a point at most half a step along each axis, so two
/// such points are at most a step apart along each, √2 steps, and the next
/// distance between grid points is two steps. Comparing against this needs no
/// allowance for rounding.
pub const SAME_POINT: f64 = 2.0 * GRID;

fn snap(v: f64) -> f64 {
    (v * PER_METER).round() / PER_METER
}

/// Why a string isn't a lane's geometry.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ParseLaneError {
    #[error("malformed vertex {0:?}: expected 2 to 4 whole millimeters")]
    Malformed(String),
    #[error("the first vertex has no half-widths")]
    NoWidths,
    #[error(transparent)]
    Lane(#[from] LaneError),
}

#[cfg(feature = "serde")]
impl serde::Serialize for LaneGeometry {
    fn serialize<S: serde::Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.to_mm_string())
    }
}

#[cfg(feature = "serde")]
impl<'de> serde::Deserialize<'de> for LaneGeometry {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let text = <std::borrow::Cow<'de, str>>::deserialize(deserializer)?;
        LaneGeometry::from_mm_string(&text).map_err(serde::de::Error::custom)
    }
}

/// The length of a polygon's boundary. A sliver's half of it is about its
/// length, so area over that is about its thickness.
fn perimeter(points: &[DVec2]) -> f64 {
    let n = points.len();
    (0..n).map(|i| points[i].distance(points[(i + 1) % n])).sum()
}

/// The corners of the box around a polygon.
fn bounds(points: &[DVec2]) -> (DVec2, DVec2) {
    points.iter().fold((DVec2::INFINITY, DVec2::NEG_INFINITY), |(lo, hi), &p| {
        (lo.min(p), hi.max(p))
    })
}

/// The part of `subject` inside `clip`, a convex polygon wound counterclockwise:
/// Sutherland–Hodgman clipping, one edge of `clip` at a time.
fn clip(subject: &[DVec2; 4], clip: &[DVec2; 4]) -> ArrayVec<DVec2, 8> {
    let mut out: ArrayVec<DVec2, 8> = subject.iter().copied().collect();
    for (k, &a) in clip.iter().enumerate() {
        let edge = clip[(k + 1) % clip.len()] - a;
        let inside = |p: DVec2| edge.perp_dot(p - a);
        let input = std::mem::take(&mut out);
        for (m, &p) in input.iter().enumerate() {
            let q = input[(m + 1) % input.len()];
            let (dp, dq) = (inside(p), inside(q));
            if dp >= 0.0 {
                out.push(p);
            }
            if (dp >= 0.0) != (dq >= 0.0) {
                out.push(p + (q - p) * (dp / (dp - dq)));
            }
        }
        if out.is_empty() {
            break;
        }
    }
    out
}

/// A polygon's area, by the shoelace formula; positive if wound counterclockwise.
fn area(points: &[DVec2]) -> f64 {
    let n = points.len();
    0.5 * (0..n).map(|i| points[i].perp_dot(points[(i + 1) % n])).sum::<f64>()
}

impl Segment {
    /// Returns `true` if the point lies in front of the start of the segment.
    fn test_point(&self, pos: DVec2) -> bool {
        let Self { start, dir, adj, .. } = *self;
        let x = dir.dot(pos - start);
        let y = dir.perp_dot(pos - start);
        (x + y * adj.0) >= 0.0
    }

    fn world_to_lane(&self, pos: DVec2) -> (f64, f64) {
        let Self { start, dir, cum_s, adj } = *self;
        let x = dir.dot(pos - start);
        let y = dir.perp_dot(pos - start);
        let s = cum_s + (x + y * adj.0) / (1.0 + y * adj.1);
        let t = y;
        (s, t)
    }

    fn lane_to_world(&self, s: f64, t: f64) -> DVec2 {
        let Self { start, dir, cum_s, adj } = *self;
        let y = t;
        let x = (s - cum_s) * (1.0 + y * adj.1) - y * adj.0;
        start + x * dir + y * dir.perp()
    }

    /// The derivative of [`Segment::lane_to_world`] along `s`, with `t`
    /// changing by `slope`, normalized.
    fn direction(&self, s: f64, t: f64, slope: f64) -> DVec2 {
        let Self { dir, cum_s, adj, .. } = *self;
        let dx = (1.0 + t * adj.1) + ((s - cum_s) * adj.1 - adj.0) * slope;
        (dx * dir + slope * dir.perp()).normalize()
    }

    /// The `slope` for which [`Segment::direction`] is `heading`.
    fn slope(&self, s: f64, t: f64, heading: DVec2) -> f64 {
        let Self { dir, cum_s, adj, .. } = *self;
        let (dx, dy) = (dir.dot(heading), dir.perp_dot(heading));
        // dx = ds·(1 + t·k) + dt·((s − c)·k − a₀), and dy = dt.
        let ds = (dx - dy * ((s - cum_s) * adj.1 - adj.0)) / (1.0 + t * adj.1);
        dy / ds
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use proptest::prelude::*;
    use proptest::sample::Index;

    const WIDTH: f64 = 3.5;
    const EPS: f64 = 1e-9;

    /// A lane `WIDTH` wide, centered on `points`.
    fn lane(points: &[DVec2]) -> Result<LaneGeometry, LaneError> {
        LaneGeometry::new(points, &vec![[0.5 * WIDTH; 2]; points.len()])
    }

    fn arb_lane() -> impl Strategy<Value = LaneGeometry> {
        prop::collection::vec((0.5f64..5.0, -0.5f64..0.5), 1..20).prop_filter_map(
            "lane folds",
            |steps| {
                let mut p = DVec2::new(100.0, -50.0);
                let mut heading = 0.3f64;
                let mut pts = vec![p];
                for (len, turn) in steps {
                    heading += turn;
                    p += len * DVec2::from_angle(heading);
                    pts.push(p);
                }
                lane(&pts).ok()
            },
        )
    }

    /// A lane position inside segment `i`, from unit parameters.
    fn pos_in(lane: &LaneGeometry, i: usize, u: f64, v: f64) -> LanePos {
        let s0 = lane.segs[i].cum_s;
        let s1 = lane.segs.get(i + 1).map_or(lane.length, |s| s.cum_s);
        LanePos { s: s0 + u * (s1 - s0), t: v * 0.5 * WIDTH, seg: i as u32 }
    }

    proptest! {
        #[test]
        fn direction_is_the_derivative_of_position(
            lane in arb_lane(), seg in any::<Index>(), u in 0.01f64..0.99, v in -1.0f64..1.0,
            slope in -0.2f64..0.2,
        ) {
            let i = seg.index(lane.segs.len());
            let at = pos_in(&lane, i, u, v);
            let h = 1e-6;
            let ahead = LanePos { s: at.s + h, t: at.t + slope * h, seg: at.seg };
            let diff = (lane.lane_to_world(ahead) - lane.lane_to_world(at)) / h;
            let dir = lane.direction_at(at, slope);
            prop_assert!((dir - diff.normalize()).length() < 1e-6, "{dir} vs {diff}");
            prop_assert!((lane.slope_toward(at, dir) - slope).abs() < 1e-9);
        }

        #[test]
        fn round_trip(lane in arb_lane(), seg in any::<Index>(), u in 0.0f64..1.0, v in -1.0f64..1.0) {
            let i = seg.index(lane.segs.len());
            let c = pos_in(&lane, i, u, v);
            let (s, t) = lane.segs[i].world_to_lane(lane.segs[i].lane_to_world(c.s, c.t));
            prop_assert!((s - c.s).abs() < EPS && (t - c.t).abs() < EPS);
        }

        #[test]
        fn public_lane_to_world_picks_the_right_segment(
            lane in arb_lane(), seg in any::<Index>(), u in 0.0f64..1.0, v in -1.0f64..1.0,
        ) {
            let i = seg.index(lane.segs.len());
            let c = pos_in(&lane, i, u, v);
            let expected = lane.segs[i].lane_to_world(c.s, c.t);
            prop_assert!(lane.lane_to_world(lane.lane_pos(c.s, c.t)).distance(expected) < EPS);
        }

        #[test]
        fn ribs_are_continuous(lane in arb_lane(), rib in any::<Index>(), v in -1.0f64..1.0) {
            prop_assume!(lane.segs.len() > 1);
            let i = 1 + rib.index(lane.segs.len() - 1);
            let (s, t) = (lane.segs[i].cum_s, v * 0.5 * WIDTH);
            let a = lane.segs[i - 1].lane_to_world(s, t);
            let b = lane.segs[i].lane_to_world(s, t);
            prop_assert!(a.distance(b) < EPS);
        }

        #[test]
        fn near_query_is_consistent_for_any_hint(
            lane in arb_lane(), seg in any::<Index>(), offset in -2i64..=2,
            u in 0.0f64..1.0, v in -1.0f64..1.0,
        ) {
            let i = seg.index(lane.segs.len());
            let c = pos_in(&lane, i, u, v);
            let p = lane.segs[i].lane_to_world(c.s, c.t);
            let hint = LanePos { s: 0.0, t: 0.0, seg: (i as i64 + offset).max(0) as u32 };
            let got = lane.world_to_lane_near(p, hint);
            prop_assert!((got.s - c.s).abs() < EPS && (got.t - c.t).abs() < EPS);
        }

        #[test]
        fn full_search_agrees_with_hinted_search(
            lane in arb_lane(), seg in any::<Index>(), u in 0.0f64..1.0, v in -1.0f64..1.0,
        ) {
            let i = seg.index(lane.segs.len());
            let c = pos_in(&lane, i, u, v);
            let p = lane.segs[i].lane_to_world(c.s, c.t);
            let got = lane.world_to_lane(p);
            prop_assert!((got.s - c.s).abs() < EPS && (got.t - c.t).abs() < EPS);
        }
    }

    #[test]
    fn centerline_s_is_arc_length() {
        let pts = [DVec2::ZERO, DVec2::new(8.0, 0.0), DVec2::new(16.0, 0.5)];
        let lane = lane(&pts).unwrap();
        let p = DVec2::new(12.0, 0.25); // midway along the second chord
        let c = lane.world_to_lane(p);
        let expected_s = 8.0 + DVec2::new(4.0, 0.25).length();
        assert!((c.s - expected_s).abs() < EPS);
        assert!(c.t.abs() < EPS);
    }

    #[test]
    fn centerline_iterator_matches_input() {
        let pts = [DVec2::ZERO, DVec2::new(8.0, 0.0), DVec2::new(16.0, 0.5)];
        let lane = lane(&pts).unwrap();
        let got: Vec<_> = lane.centerline().collect();
        assert_eq!(got.len(), pts.len());
        for (a, b) in got.iter().zip(pts) {
            assert!(a.distance(b) < EPS);
        }
        assert!((lane.length() - (8.0 + DVec2::new(8.0, 0.5).length())).abs() < EPS);
    }

    #[test]
    fn tangent_follows_segments() {
        // A bend of about 0.3 rad (~17°), within MAX_TURN, with its points on the
        // millimeter grid that lanes are snapped to.
        let step = DVec2::new(7.642, 2.364);
        let bend = step.normalize();
        let pts = [
            DVec2::ZERO,
            DVec2::new(8.0, 0.0),
            DVec2::new(8.0, 0.0) + step,
        ];
        let lane = lane(&pts).unwrap();
        assert!(lane.tangent_at(lane.lane_pos(4.0, 0.0)).distance(DVec2::X) < EPS);
        assert!(lane.tangent_at(lane.lane_pos(12.0, 0.0)).distance(bend) < EPS);
    }

    #[test]
    fn errors_report_where() {
        assert_eq!(
            lane(&[DVec2::ZERO, DVec2::ZERO]).unwrap_err(),
            LaneError::TooFewPoints
        );
        let hairpin = [DVec2::ZERO, DVec2::new(5.0, 0.0), DVec2::new(5.0, 5.0)];
        assert_eq!(
            lane(&hairpin).unwrap_err(),
            LaneError::TurnTooSharp { vertex: 1 }
        );
    }

    #[test]
    fn each_side_has_its_own_half_width_varying_along_the_lane() {
        // 10 m straight: 1 m to the left and 2 m to the right at the start, 3 m
        // and 4 m at the end.
        let pts = [DVec2::ZERO, DVec2::new(10.0, 0.0)];
        let lane = LaneGeometry::new(&pts, &[[1.0, 2.0], [3.0, 4.0]]).unwrap();
        let pos = lane.lane_pos(5.0, 0.0);
        assert!((lane.half_width(pos, Side::Left) - 2.0).abs() < EPS);
        assert!((lane.half_width(pos, Side::Right) - 3.0).abs() < EPS);
        assert!(lane.between_edges(lane.lane_pos(5.0, 1.9)));
        assert!(!lane.between_edges(lane.lane_pos(5.0, 2.1)));
        assert!(lane.between_edges(lane.lane_pos(5.0, -2.9)));
        assert!(!lane.between_edges(lane.lane_pos(5.0, -3.1)));
        let outline: Vec<_> = lane.outline().collect();
        let expected = [(0.0, 1.0), (10.0, 3.0), (10.0, -4.0), (0.0, -2.0)];
        assert_eq!(outline.len(), expected.len());
        for (got, (x, y)) in outline.iter().zip(expected) {
            assert!(got.distance(DVec2::new(x, y)) < EPS, "{got}");
        }
    }

    #[test]
    fn a_lane_too_wide_on_the_inside_of_a_bend_folds() {
        // A 0.3 rad bend to the left over 2 m segments: lines of constant `s` meet
        // about 13 m inside it, so a lane that wide on the left folds, however
        // narrow on the right, and one that wide on the right doesn't.
        let bend = DVec2::from_angle(0.3);
        let pts = [
            DVec2::ZERO,
            DVec2::new(2.0, 0.0),
            DVec2::new(2.0, 0.0) + 2.0 * bend,
        ];
        assert!(LaneGeometry::new(&pts, &[[0.1, 14.0]; 3]).is_ok());
        assert_eq!(
            LaneGeometry::new(&pts, &[[14.0, 0.1]; 3]).unwrap_err(),
            LaneError::Folds { segment: 0 }
        );
    }

    /// Everything about a lane's geometry, as raw bits.
    fn bits(lane: &LaneGeometry) -> Vec<u64> {
        let mut out = vec![lane.length.to_bits()];
        for seg in &lane.segs {
            let Segment { start, dir, cum_s, adj } = *seg;
            out.extend([start.x, start.y, dir.x, dir.y, cum_s, adj.0, adj.1].map(f64::to_bits));
        }
        out.extend(lane.half_widths.iter().flatten().map(|w| w.to_bits()));
        out
    }

    proptest! {
        #[test]
        fn the_mm_string_reads_back_exactly(lane in arb_lane()) {
            let read = LaneGeometry::from_mm_string(&lane.to_mm_string()).unwrap();
            prop_assert_eq!(bits(&read), bits(&lane));
        }
    }

    #[test]
    fn points_and_widths_snap_to_the_millimeter() {
        let lane = LaneGeometry::new(
            &[DVec2::new(0.0004, 0.0), DVec2::new(10.0006, 0.0)],
            &[[1.7504; 2], [1.7496, 2.0]],
        )
        .unwrap();
        assert_eq!(lane.centerline().next(), Some(DVec2::ZERO));
        assert_eq!(lane.length(), 10.001);
        assert_eq!(lane.half_widths(), [[1.75, 1.75], [1.75, 2.0]]);
    }

    #[test]
    fn the_mm_string_gives_widths_only_where_they_change() {
        let lane = LaneGeometry::new(
            &[
                DVec2::ZERO,
                DVec2::new(90.0, 0.0),
                DVec2::new(100.0, 2.0),
                DVec2::new(110.0, 2.0),
            ],
            &[[1.75; 2], [1.75; 2], [2.0, 1.75], [2.0, 1.75]],
        )
        .unwrap();
        assert_eq!(
            lane.to_mm_string(),
            "0,0,1750 90000,0 100000,2000,2000,1750 110000,2000"
        );
    }

    #[test]
    fn a_malformed_mm_string_is_refused() {
        let parse = LaneGeometry::from_mm_string;
        assert_eq!(parse("0,0 10000,0").unwrap_err(), ParseLaneError::NoWidths);
        assert!(matches!(
            parse("0,0,1750 1.5,0").unwrap_err(),
            ParseLaneError::Malformed(_)
        ));
        assert!(matches!(
            parse("0,0,1,2,3 9,0").unwrap_err(),
            ParseLaneError::Malformed(_)
        ));
        assert_eq!(
            parse("0,0,1750").unwrap_err(),
            ParseLaneError::Lane(LaneError::TooFewPoints)
        );
    }
}
