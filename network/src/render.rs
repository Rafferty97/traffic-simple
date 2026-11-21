//! What to draw for a road network: road and intersection surfaces, lane
//! markings, stop lines and turn arrows.

use glam::DVec2;

use crate::reference::Polyline;
use crate::{Direction, Intersection, LaneRef, RoadNetwork};

/// A road network's drawable geometry, in world meters.
#[derive(Clone, Debug, Default)]
pub struct RenderGeometry {
    /// Each road's surface, between its outermost lane edges.
    pub roads: Vec<Strip>,
    /// Each intersection's surface, counterclockwise: the mouths of the roads
    /// that meet there, joined by curbs. Not convex, but every point of it can
    /// see its centroid, so it can be drawn as a fan from there.
    pub intersections: Vec<Vec<DVec2>>,
    pub markings: Vec<Marking>,
    pub arrows: Vec<Arrow>,
}

/// A surface between two edges, vertex for vertex: drawn as quads between
/// consecutive pairs.
#[derive(Clone, Debug, Default)]
pub struct Strip {
    pub left: Vec<DVec2>,
    pub right: Vec<DVec2>,
}

/// A painted line.
#[derive(Clone, Debug)]
pub struct Marking {
    pub points: Vec<DVec2>,
    pub style: MarkingStyle,
}

/// A turn arrow painted on a lane coming into an intersection, saying which
/// ways it goes on: at least one of them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arrow {
    /// Its anchor, on the lane's centreline, where its shaft starts.
    pub pos: DVec2,
    /// The way the lane runs there, in radians counterclockwise from +x.
    pub heading: f64,
    pub left: bool,
    pub straight: bool,
    pub right: bool,
}

/// How far back from the end of a lane coming into an intersection its arrow's
/// anchor is, in meters: the arrow's tail is there, and it runs 6 m on.
const ARROW_SETBACK: f64 = 8.0;
/// How wide a stop line is, in meters: 300 mm, as Australian standards (AS
/// 1742.2) have it.
const STOP_LINE_WIDTH: f64 = 0.3;
/// The most a connection turns, either way, and still goes straight on: 30°.
const STRAIGHT_ON: f64 = std::f64::consts::PI / 6.0;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MarkingStyle {
    /// Between lanes going the same way: changing lanes is allowed.
    Dashed,
    /// A road's outer edge, or an intersection's curb.
    Edge,
    /// Between lanes going opposite ways.
    Centre,
    /// Across a lane where it comes into an intersection: a stop line, 300 mm
    /// wide, its far edge where the intersection starts.
    Stop,
}

pub(crate) fn build(net: &RoadNetwork, polylines: &[Polyline]) -> RenderGeometry {
    let mut out = RenderGeometry::default();
    for (road, line) in net.roads.iter().zip(polylines) {
        let edge = |i: usize, lane: usize, side: f64| {
            let l = &road.lanes[lane];
            let s = line.s[i];
            l.offset.at(s) + side * 0.5 * l.width.at(s)
        };
        let along = |f: &dyn Fn(usize) -> f64| -> Vec<DVec2> {
            (0..line.points.len()).map(|i| line.points[i] + f(i) * line.miters[i]).collect()
        };
        let lanes = 0..road.lanes.len();
        let left = |i: usize| lanes.clone().map(|l| edge(i, l, 1.0)).fold(f64::MIN, f64::max);
        let right = |i: usize| lanes.clone().map(|l| edge(i, l, -1.0)).fold(f64::MAX, f64::min);
        out.roads.push(Strip { left: along(&left), right: along(&right) });
        out.markings.push(Marking { points: along(&left), style: MarkingStyle::Edge });
        out.markings.push(Marking { points: along(&right), style: MarkingStyle::Edge });

        // Between each lane and the next to its right, by offset at the start.
        let s0 = line.s[0];
        let mut order: Vec<usize> = lanes.clone().collect();
        order.sort_by(|&a, &b| road.lanes[b].offset.at(s0).total_cmp(&road.lanes[a].offset.at(s0)));
        let s1 = *line.s.last().unwrap();
        for pair in order.windows(2) {
            let (upper, lower) = (pair[0], pair[1]);
            let (a, b) = (&road.lanes[upper], &road.lanes[lower]);
            let gap = |s: f64| a.offset.at(s) - b.offset.at(s);
            let style = match a.direction == b.direction {
                true => MarkingStyle::Dashed,
                false => MarkingStyle::Centre,
            };
            // Where one lane eases into or out of the other, nothing is painted.
            if style == MarkingStyle::Dashed && (gap(s0) - gap(s1)).abs() > 0.01 {
                continue;
            }
            let between = |i: usize| 0.5 * (edge(i, upper, -1.0) + edge(i, lower, 1.0));
            out.markings.push(Marking { points: along(&between), style });
        }
    }

    for intersection in &net.intersections {
        let (surface, curbs) = intersection_surface(net, polylines, &out.roads, intersection);
        out.intersections.push(surface);
        out.markings
            .extend(curbs.into_iter().map(|points| Marking { points, style: MarkingStyle::Edge }));
        approaches(net, polylines, intersection, &mut out);
    }
    out
}

/// On each lane coming into an intersection: a stop line across its end, and
/// an arrow for the ways its connections turn, as SUMO classes a link's
/// direction by the angle between its lanes. No arrow on a road too short to
/// fit one.
fn approaches(
    net: &RoadNetwork,
    polylines: &[Polyline],
    intersection: &Intersection,
    out: &mut RenderGeometry,
) {
    let lane = |l: LaneRef| &net.roads[l.road.0].lanes[l.lane];
    // Each lane's direction of travel where it meets the intersection: along
    // its road's end segment, which lies on the tangent there.
    let end_dir = |l: LaneRef, arriving: bool| {
        let points = &polylines[l.road.0].points;
        let n = points.len();
        let at_end = (lane(l).direction == Direction::Forward) == arriving;
        let dir = match at_end {
            true => points[n - 1] - points[n - 2],
            false => points[1] - points[0],
        };
        if lane(l).direction == Direction::Forward {
            dir
        } else {
            -dir
        }
    };
    let mut froms: Vec<LaneRef> = Vec::new();
    for &(from, _) in &intersection.connections {
        if !froms.contains(&from) {
            froms.push(from);
        }
    }
    for from in froms {
        let road_lane = lane(from);
        let line = &polylines[from.road.0];
        let forward = road_lane.direction == Direction::Forward;
        let arriving = end_dir(from, true);

        // Across the lane where it ends, set back by half its width so that
        // it's painted on the road.
        let k = if forward { line.points.len() - 1 } else { 0 };
        let back = -0.5 * STOP_LINE_WIDTH * arriving.normalize();
        let across = |side: f64| {
            let s = line.s[k];
            let t = road_lane.offset.at(s) + side * 0.5 * road_lane.width.at(s);
            line.points[k] + t * line.miters[k] + back
        };
        out.markings.push(Marking {
            points: vec![across(1.0), across(-1.0)],
            style: MarkingStyle::Stop,
        });

        let (mut left, mut straight, mut right) = (false, false, false);
        for &(_, to) in intersection.connections.iter().filter(|c| c.0 == from) {
            match arriving.angle_to(end_dir(to, false)) {
                t if t.abs() <= STRAIGHT_ON => straight = true,
                t if t > 0.0 => left = true,
                _ => right = true,
            }
        }
        let length = *line.s.last().unwrap();
        if length < 2.0 * ARROW_SETBACK {
            continue;
        }
        let s = if forward {
            length - ARROW_SETBACK
        } else {
            ARROW_SETBACK
        };
        // The segment it falls on, and how far along it.
        let i = line.s.partition_point(|&x| x <= s).clamp(1, line.s.len() - 1);
        let t = (s - line.s[i - 1]) / (line.s[i] - line.s[i - 1]);
        let centre = |k: usize| line.points[k] + road_lane.offset.at(line.s[k]) * line.miters[k];
        let pos = centre(i - 1).lerp(centre(i), t);
        let dir = if forward {
            centre(i) - centre(i - 1)
        } else {
            centre(i - 1) - centre(i)
        };
        out.arrows.push(Arrow { pos, heading: dir.to_angle(), left, straight, right });
    }
}

/// Where a road meets an intersection: its corners, left and right as seen
/// facing into the intersection, and that direction.
struct Mouth {
    left: DVec2,
    right: DVec2,
    inward: DVec2,
}

/// An intersection's surface, and its curbs: around the mouths of the roads
/// that meet there, counterclockwise, joined corner to corner by cubic Béziers
/// that carry on along each road's edge and turn out along the next one's.
fn intersection_surface(
    net: &RoadNetwork,
    polylines: &[Polyline],
    strips: &[Strip],
    intersection: &Intersection,
) -> (Vec<DVec2>, Vec<Vec<DVec2>>) {
    // The road ends it touches: a lane's travel ends at the road's end if it
    // runs forward, and starts there if it runs backward.
    let mut ends: Vec<(usize, bool)> = Vec::new();
    for &(from, to) in &intersection.connections {
        let forward =
            |l: LaneRef| net.roads[l.road.0].lanes[l.lane].direction == Direction::Forward;
        for end in [(from.road.0, forward(from)), (to.road.0, !forward(to))] {
            if !ends.contains(&end) {
                ends.push(end);
            }
        }
    }
    let mut mouths: Vec<Mouth> = ends
        .iter()
        .map(|&(road, at_end)| {
            let (strip, points) = (&strips[road], &polylines[road].points);
            let n = points.len();
            match at_end {
                true => Mouth {
                    left: strip.left[n - 1],
                    right: strip.right[n - 1],
                    inward: (points[n - 1] - points[n - 2]).normalize(),
                },
                false => Mouth {
                    left: strip.right[0],
                    right: strip.left[0],
                    inward: (points[0] - points[1]).normalize(),
                },
            }
        })
        .collect();
    if mouths.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let centre = mouths.iter().map(|m| m.left + m.right).sum::<DVec2>() / (2 * mouths.len()) as f64;
    let angle = |m: &Mouth| (0.5 * (m.left + m.right) - centre).to_angle();
    mouths.sort_by(|a, b| angle(a).total_cmp(&angle(b)));

    let mut surface = Vec::new();
    let mut curbs = Vec::new();
    for (i, mouth) in mouths.iter().enumerate() {
        let next = &mouths[(i + 1) % mouths.len()];
        surface.extend([mouth.left, mouth.right]);
        let (p0, p3) = (mouth.right, next.left);
        let reach = p0.distance(p3) / 3.0;
        let p = [p0, p0 + reach * mouth.inward, p3 + reach * next.inward, p3];
        let n = 16;
        let curb: Vec<DVec2> = (0..=n)
            .map(|k| {
                let t = k as f64 / n as f64;
                let u = 1.0 - t;
                u * u * u * p[0]
                    + 3.0 * u * u * t * p[1]
                    + 3.0 * u * t * t * p[2]
                    + t * t * t * p[3]
            })
            .collect();
        surface.extend(&curb[1..n]);
        curbs.push(curb);
    }
    (surface, curbs)
}
