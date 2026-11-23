//! How roads meet intersections: where their lanes are cut short so that the
//! stretches nearest an intersection belong to its connectors, and where their
//! stop lines run. Shared by compilation and render geometry.

use glam::DVec2;

use crate::reference::Polyline;
use crate::{Arm, Direction, RoadLane, RoadNetwork};

/// Each road's ends, start and end: the intersection each meets, if any, and
/// how.
pub(crate) fn road_ends(net: &RoadNetwork) -> Vec<[Option<(usize, Arm)>; 2]> {
    let mut ends = vec![[None; 2]; net.roads.len()];
    for (k, intersection) in net.intersections.iter().enumerate() {
        for &(from, to) in &intersection.connections {
            for (l, arriving) in [(from, true), (to, false)] {
                let at_end = forward(&net.roads[l.road.0].lanes[l.lane]) == arriving;
                ends[l.road.0][at_end as usize] = Some((k, intersection.arm(l.road)));
            }
        }
    }
    ends
}

fn forward(lane: &RoadLane) -> bool {
    lane.direction == Direction::Forward
}

/// Whether `lane` comes into the intersection at its road's end (`at_end`) or
/// start, rather than going out of it.
pub(crate) fn arriving(lane: &RoadLane, at_end: bool) -> bool {
    forward(lane) == at_end
}

/// Where the reference line is at a road end: `total` long, at its end or
/// its start.
fn station(total: f64, at_end: bool, back: f64) -> f64 {
    if at_end { total - back } else { back }
}

/// How far back from the road end, along its reference line, its stop line
/// crosses `offset` from it.
pub(crate) fn back(arm: &Arm, at_end: bool, offset: f64) -> f64 {
    let sign = if at_end { 1.0 } else { -1.0 };
    arm.stop_line + sign * offset * arm.skew.tan()
}

/// How far back from the road end `lane` is cut: coming in, where its stop line
/// crosses its centre, so that a vehicle stopping by the line, its nearest
/// corner short of it, has its front short of the lane's end, never in the
/// junction before it commits; going out, the arm's run out. Each lane coming
/// in ends at its own stop line, so lanes beside each other end apart where
/// the line is skewed.
pub(crate) fn cut(lane: &RoadLane, total: f64, arm: &Arm, at_end: bool) -> f64 {
    if !arriving(lane, at_end) {
        return arm.run_out;
    }
    let s = station(total, at_end, 0.0);
    back(arm, at_end, lane.offset.at(s)).max(0.0)
}

/// A road end's stop line: a point on it, its direction, and its normal,
/// pointing back toward the vehicles coming up to it.
pub(crate) struct StopLine {
    pub point: DVec2,
    pub along: DVec2,
    pub normal: DVec2,
    pub skew: f64,
}

impl StopLine {
    pub fn new(line: &Polyline, arm: &Arm, at_end: bool) -> Self {
        let total = *line.s.last().expect("a line has its ends");
        let (point, u) = line.at(station(total, at_end, arm.stop_line));
        let (sin, cos) = arm.skew.sin_cos();
        let along = u.perp() * cos - u * sin;
        // Into the intersection: along the road at its end, against it at its start.
        let into = if at_end { u } else { -u };
        let normal = along.perp() * -along.perp().dot(into).signum();
        StopLine { point, along, normal, skew: arm.skew }
    }

    /// Where it crosses `offset` from the reference line.
    pub fn at(&self, offset: f64) -> DVec2 {
        self.point + offset / self.skew.cos() * self.along
    }
}
