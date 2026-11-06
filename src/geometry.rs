use std::f64::consts::TAU;

use glam::DVec2;
use ordered_float::NotNan;

/// The sharpest turn allowed at a single vertex: 1/16 of a revolution.
const MAX_TURN: f64 = TAU / 16.0;

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

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct LaneCoord {
    /// Index of the lane segment the point lies within.
    pub index: usize,
    /// The longitudinal position along the lane in metres.
    pub s: f64,
    /// Lateral offset from the centreline in metres.
    pub t: f64,
}

impl Lane {
    /// Builds a lane from its centreline polyline and a constant width.
    pub fn new(points: &[DVec2], width: f64) -> Result<Self, &'static str> {
        let max_tan_half = (0.5 * MAX_TURN).tan();

        let mut points = points.to_vec();
        points.dedup_by(|a, b| a.distance_squared(*b) < 1e-12);
        if points.len() < 2 {
            return Err("a lane needs at least two distinct points");
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
            if !(th.abs() <= max_tan_half) {
                return Err("turn sharper than MAX_TURN");
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
                return Err("lane folds: turn too sharp for its width");
            }
            segs.push(Segment { start: points[i], dir: dirs[i], cum_s, adj: (t0, k) });
            cum_s += lens[i];
        }
        Ok(Self { segs, length: cum_s, width })
    }

    pub fn world_to_lane(&self, pos: DVec2) -> LaneCoord {
        let index = self.find_segment(pos);
        let (s, t) = self.segs[index].world_to_lane(pos);
        LaneCoord { index, s, t }
    }

    pub fn lane_to_world(&self, s: f64, t: f64, hint: usize) -> DVec2 {
        self.segs[self.s_to_segment(s, hint)].lane_to_world(s, t)
    }

    pub fn lane_to_world_old(&self, coords: LaneCoord) -> DVec2 {
        self.segs[coords.index].lane_to_world(coords.s, coords.t)
    }

    pub fn find_segment(&self, pos: DVec2) -> usize {
        let hint = self
            .segs
            .iter()
            .enumerate()
            .flat_map(|(idx, seg)| {
                let len = (pos - seg.start).length_squared();
                NotNan::try_from(len).map(|len| (idx, len)).ok()
            })
            .min_by_key(|&(_, len)| len)
            .map_or(0, |(i, _)| i);
        self.find_segment_with_hint(pos, hint)
    }

    pub fn find_segment_with_hint(&self, pos: DVec2, hint: usize) -> usize {
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

    pub fn s_to_segment(&self, s: f64, hint: usize) -> usize {
        let last = self.segs.len() - 1;
        let mut idx = hint.min(last);
        while idx > 0 && !self.segs[idx].test_s(s) {
            idx -= 1;
        }
        while idx < last && self.segs[idx + 1].test_s(s) {
            idx += 1;
        }
        idx
    }

    pub fn outline(&self) -> impl Iterator<Item = DVec2> + '_ {
        let hw = self.width / 2.0;
        let left = self.segs.iter().map(move |s| s.lane_to_world(s.cum_s, hw));
        let right = self.segs.iter().map(move |s| s.lane_to_world(s.cum_s, -hw));
        let last_seg = self.segs.last().unwrap();
        let end = [hw, -hw].map(|t| last_seg.lane_to_world(self.length, t));
        left.chain(end).chain(right.rev())
    }
}

impl Segment {
    /// Returns `true` if the point lies in front of the start of the segment.
    pub fn test_point(&self, pos: DVec2) -> bool {
        let Self { start, dir, adj, .. } = *self;
        let x = dir.dot(pos - start);
        let y = dir.perp_dot(pos - start);
        (x + y * adj.0) >= 0.0
    }

    pub fn test_s(&self, s: f64) -> bool {
        s >= self.cum_s
    }

    pub fn world_to_lane(&self, pos: DVec2) -> (f64, f64) {
        let Self { start, dir, cum_s, adj, .. } = *self;
        let x = dir.dot(pos - start);
        let y = dir.perp_dot(pos - start);
        let s = cum_s + (x + y * adj.0) / (1.0 + y * adj.1);
        let t = y;
        (s, t)
    }

    pub fn lane_to_world(&self, s: f64, t: f64) -> DVec2 {
        let Self { start, dir, cum_s, adj, .. } = *self;
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

    /// A lane coordinate inside segment `i`, from unit parameters.
    fn coord_in(lane: &Lane, i: usize, u: f64, v: f64) -> LaneCoord {
        let s0 = lane.segs[i].cum_s;
        let s1 = lane.segs.get(i + 1).map_or(lane.length, |s| s.cum_s);
        LaneCoord { index: i, s: s0 + u * (s1 - s0), t: v * 0.5 * WIDTH }
    }

    proptest! {
        #[test]
        fn round_trip(lane in arb_lane(), seg in any::<Index>(), u in 0.0f64..1.0, v in -1.0f64..1.0) {
            let i = seg.index(lane.segs.len());
            let c = coord_in(&lane, i, u, v);
            let (s, t) = lane.segs[i].world_to_lane(lane.lane_to_world_old(c));
            prop_assert!((s - c.s).abs() < EPS && (t - c.t).abs() < EPS);
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
        fn walk_finds_consistent_segment(
            lane in arb_lane(), seg in any::<Index>(), offset in -2i64..=2,
            u in 0.0f64..1.0, v in -1.0f64..1.0,
        ) {
            let i = seg.index(lane.segs.len());
            let c = coord_in(&lane, i, u, v);
            let p = lane.lane_to_world_old(c);
            let hint = (i as i64 + offset).max(0) as usize;
            let j = lane.find_segment_with_hint(p, hint);
            let (s, t) = lane.segs[j].world_to_lane( p);
            prop_assert!((s - c.s).abs() < EPS && (t - c.t).abs() < EPS);
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
}
