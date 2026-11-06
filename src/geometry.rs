use std::f64::consts::TAU;

use glam::DVec2;

/// The sharpest turn allowed at a single vertex: 1/16 of a revolution.
const MAX_TURN: f64 = TAU / 16.0;

/// A lane described by a polyline centreline and a constant width.
///
/// Positions are expressed in lane coordinates `(s, t)`: `s` is the distance in
/// metres along the centreline and `t` is the lateral offset in metres, positive
/// to the left. The mapping is continuous across vertices, so a vehicle can move
/// along the lane without `s` jumping.
#[derive(Clone, Debug)]
pub struct Lane {
    /// The segments comprising the lane.
    segs: Vec<Segment>,
    /// Width of the lane in metres.
    width: f64,
    /// Length of the lane in metres.
    length: f64,
}

#[derive(Clone, Copy, Debug)]
struct Segment {
    /// Position of the centreline at the start of the segment.
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
/// found in, which [`Lane::world_to_lane_near`] uses to make the next query cheap.
/// The cache is only ever a hint: a stale one (even from a different lane) makes a
/// query slower, never wrong.
#[derive(Clone, Copy, Debug)]
pub struct LanePos {
    /// Longitudinal position along the centreline in metres.
    pub s: f64,
    /// Lateral offset from the centreline in metres, positive to the left.
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

impl Lane {
    /// Builds a lane from its centreline polyline and a constant width.
    ///
    /// Consecutive duplicate points are removed before the lane is built.
    pub fn new(points: &[DVec2], width: f64) -> Result<Self, LaneError> {
        let max_tan_half = (0.5 * MAX_TURN).tan();

        let mut points = points.to_vec();
        points.dedup_by(|a, b| a.distance_squared(*b) < 1e-12);
        if points.len() < 2 {
            return Err(LaneError::TooFewPoints);
        }

        let lens: Vec<f64> = points.windows(2).map(|w| w[0].distance(w[1])).collect();
        let dirs: Vec<DVec2> = points
            .windows(2)
            .map(|w| (w[1] - w[0]).normalize())
            .collect();

        let n = dirs.len();
        let mut tan_half = vec![0.0; n + 1];
        for i in 1..n {
            let (a, b) = (dirs[i - 1], dirs[i]);
            let th = a.perp_dot(b) / (1.0 + a.dot(b));
            // Written as a negated `<=` so that NaN is rejected too.
            if !(th.abs() <= max_tan_half) {
                return Err(LaneError::TurnTooSharp { vertex: i });
            }
            tan_half[i] = th;
        }

        let hw = 0.5 * width;
        let mut cum_s = 0.0;
        let mut segs = Vec::with_capacity(n);
        for i in 0..n {
            let t0 = -tan_half[i];
            let t1 = tan_half[i + 1];
            let k = (t0 - t1) / lens[i];
            if hw * k.abs() >= 1.0 {
                return Err(LaneError::Folds { segment: i });
            }
            segs.push(Segment { start: points[i], dir: dirs[i], cum_s, adj: (t0, k) });
            cum_s += lens[i];
        }
        Ok(Self { segs, length: cum_s, width })
    }

    /// Length of the centreline in metres.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Width of the lane in metres.
    pub fn width(&self) -> f64 {
        self.width
    }

    /// Creates a new `LanePos` for the given `s` and `t`.
    pub fn lane_pos(&self, s: f64, t: f64) -> LanePos {
        LanePos { s, t, seg: self.seg_for_s(s) as u32 }
    }

    /// Converts lane coordinates to a world position.
    pub fn lane_to_world(&self, pos: LanePos) -> DVec2 {
        self.segs[pos.seg as usize].lane_to_world(pos.s, pos.t)
    }

    /// Unit direction of the centreline at `s`.
    ///
    /// The centreline is piecewise straight, so this jumps at vertices.
    pub fn tangent_at(&self, pos: LanePos) -> DVec2 {
        self.segs[pos.seg as usize].dir
    }

    /// Converts a world position to lane coordinates, searching the whole lane.
    ///
    /// Use this for the first query; afterwards prefer [`Lane::world_to_lane_near`].
    /// The result is only meaningful for points within roughly the lane's width of
    /// the centreline, since the lines of constant `s` can cross further out.
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

    /// The centreline vertices, from start to end.
    pub fn centreline(&self) -> impl Iterator<Item = DVec2> + '_ {
        let seg = (self.segs.len() - 1) as u32;
        let end = self.lane_to_world(LanePos { seg, s: self.length, t: 0.0 });
        self.segs
            .iter()
            .map(|s| s.start)
            .chain(std::iter::once(end))
    }

    /// The closed outline of the lane: the left edge forwards, then the right edge backwards.
    pub fn outline(&self) -> impl Iterator<Item = DVec2> + '_ {
        let hw = self.width / 2.0;
        let left = self.segs.iter().map(move |s| s.lane_to_world(s.cum_s, hw));
        let right = self.segs.iter().map(move |s| s.lane_to_world(s.cum_s, -hw));
        let last_seg = self.segs.last().unwrap();
        let end = [hw, -hw].map(|t| last_seg.lane_to_world(self.length, t));
        left.chain(end).chain(right.rev())
    }

    /// The last segment whose start is at or before `s`.
    fn seg_for_s(&self, s: f64) -> usize {
        self.segs
            .partition_point(|g| g.cum_s <= s)
            .saturating_sub(1)
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
}

#[cfg(test)]
mod test {
    use super::*;
    use proptest::prelude::*;
    use proptest::sample::Index;

    const WIDTH: f64 = 3.5;
    const EPS: f64 = 1e-9;

    fn arb_lane() -> impl Strategy<Value = Lane> {
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
                Lane::new(&pts, WIDTH).ok()
            },
        )
    }

    /// A lane position inside segment `i`, from unit parameters.
    fn pos_in(lane: &Lane, i: usize, u: f64, v: f64) -> LanePos {
        let s0 = lane.segs[i].cum_s;
        let s1 = lane.segs.get(i + 1).map_or(lane.length, |s| s.cum_s);
        LanePos { s: s0 + u * (s1 - s0), t: v * 0.5 * WIDTH, seg: i as u32 }
    }

    proptest! {
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
    fn centreline_s_is_arc_length() {
        let pts = [DVec2::ZERO, DVec2::new(8.0, 0.0), DVec2::new(16.0, 0.5)];
        let lane = Lane::new(&pts, WIDTH).unwrap();
        let p = DVec2::new(12.0, 0.25); // midway along the second chord
        let c = lane.world_to_lane(p);
        let expected_s = 8.0 + DVec2::new(4.0, 0.25).length();
        assert!((c.s - expected_s).abs() < EPS);
        assert!(c.t.abs() < EPS);
    }

    #[test]
    fn centreline_iterator_matches_input() {
        let pts = [DVec2::ZERO, DVec2::new(8.0, 0.0), DVec2::new(16.0, 0.5)];
        let lane = Lane::new(&pts, WIDTH).unwrap();
        let got: Vec<_> = lane.centreline().collect();
        assert_eq!(got.len(), pts.len());
        for (a, b) in got.iter().zip(pts) {
            assert!(a.distance(b) < EPS);
        }
        assert!((lane.length() - (8.0 + DVec2::new(8.0, 0.5).length())).abs() < EPS);
    }

    #[test]
    fn tangent_follows_segments() {
        // A 0.3 rad (~17°) bend, within MAX_TURN.
        let bend = DVec2::from_angle(0.3);
        let pts = [
            DVec2::ZERO,
            DVec2::new(8.0, 0.0),
            DVec2::new(8.0, 0.0) + 8.0 * bend,
        ];
        let lane = Lane::new(&pts, WIDTH).unwrap();
        assert!(lane.tangent_at(lane.lane_pos(4.0, 0.0)).distance(DVec2::X) < EPS);
        assert!(lane.tangent_at(lane.lane_pos(12.0, 0.0)).distance(bend) < EPS);
    }

    #[test]
    fn errors_report_where() {
        assert_eq!(
            Lane::new(&[DVec2::ZERO, DVec2::ZERO], WIDTH).unwrap_err(),
            LaneError::TooFewPoints
        );
        let hairpin = [DVec2::ZERO, DVec2::new(5.0, 0.0), DVec2::new(5.0, 5.0)];
        assert_eq!(
            Lane::new(&hairpin, WIDTH).unwrap_err(),
            LaneError::TurnTooSharp { vertex: 1 }
        );
    }
}
