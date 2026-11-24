//! What to draw for a road network: road and intersection surfaces, lane
//! markings, stop lines and turn arrows.

use glam::DVec2;

use crate::arms::{self, StopLine};
use crate::reference::Polyline;
use crate::{Direction, Intersection, KeepClear, LaneRef, Road, RoadNetwork, Turn};

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
    /// A keep-clear zone's outline, closed, or one of its diagonal hatches.
    KeepClear,
}

/// How far apart a keep-clear zone's hatches are, along its road, in meters.
const HATCH_SPACING: f64 = 2.0;

pub(crate) fn build(
    net: &RoadNetwork,
    polylines: &[Polyline],
    turns: &[Vec<Turn>],
) -> RenderGeometry {
    let mut out = RenderGeometry::default();
    let ends = arms::road_ends(net);
    // Each road's edge lines, by road, before they are cut where siblings overlap.
    let mut edges = Vec::new();
    for (r, (road, line)) in net.roads.iter().zip(polylines).enumerate() {
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
        edges.push([along(&left), along(&right)]);

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
            // Beside a lane coming into an intersection, it stops at the stop
            // line, where it crosses it.
            let n = line.points.len();
            let [from, to] = [false, true].map(|at_end| {
                let arm = ends[r][at_end as usize].map(|(_, arm)| arm);
                let coming_in = [a, b].iter().any(|l| arms::arriving(l, at_end));
                let back = |arm| arms::back(&arm, at_end, between(if at_end { n - 1 } else { 0 }));
                let back = arm.filter(|_| coming_in).map_or(0.0, back);
                if at_end { s1 - back } else { s0 + back }
            });
            let points = clip(&along(&between), &line.s, from, to);
            out.markings.push(Marking { points, style });
        }
    }

    // A road's edges where they don't run over a road it forks from another
    // with, or merges with into another, so a fork paints a gore where the two
    // part, as at a freeway's exit, rather than one road's lines across the
    // other.
    let siblings = |r: usize| {
        let joined = |&(from, to): &(crate::RoadId, crate::RoadId)| (from.0, to.0);
        net.joins.iter().map(joined).flat_map(move |(from, to)| {
            net.joins.iter().map(joined).filter_map(move |(other_from, other_to)| {
                (other_from == from && to == r && other_to != r)
                    .then_some(other_to)
                    .or((other_to == to && from == r && other_from != r).then_some(other_from))
            })
        })
    };
    for (r, road_edges) in edges.into_iter().enumerate() {
        let others: Vec<usize> = siblings(r).collect();
        for edge in road_edges {
            let mut pieces = vec![edge];
            for &other in &others {
                let surface = &out.roads[other];
                pieces = pieces.iter().flat_map(|p| outside(p, surface)).collect();
            }
            let pieces = pieces.into_iter().filter(|p| p.len() >= 2);
            out.markings
                .extend(pieces.map(|points| Marking { points, style: MarkingStyle::Edge }));
        }
    }

    for (road, line) in net.roads.iter().zip(polylines) {
        for zone in &road.keep_clear {
            keep_clear(road, line, zone, &mut out);
        }
    }

    for (intersection, turns) in net.intersections.iter().zip(turns) {
        let (surface, curbs) = intersection_surface(net, polylines, &out.roads, intersection);
        out.intersections.push(surface);
        out.markings
            .extend(curbs.into_iter().map(|points| Marking { points, style: MarkingStyle::Edge }));
        approaches(net, polylines, intersection, turns, &mut out);
    }
    out
}

/// How far inside a surface, in meters, a line must be to count as over it,
/// so lines along its edge, as a fork's shared outer edge is, stay.
const OVER: f64 = 0.01;

/// The pieces of `line` not over `surface`.
fn outside(line: &[DVec2], surface: &Strip) -> Vec<Vec<DVec2>> {
    let outline: Vec<DVec2> =
        surface.left.iter().chain(surface.right.iter().rev()).copied().collect();
    let edges = || outline.iter().zip(outline.iter().cycle().skip(1)).map(|(&a, &b)| (a, b));
    let over = |p: DVec2| {
        // Even-odd, and clear of the outline by more than [`OVER`].
        let crossings = edges()
            .filter(|&(a, b)| (a.y > p.y) != (b.y > p.y))
            .filter(|&(a, b)| p.x < a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x))
            .count();
        crossings % 2 == 1 && edges().all(|(a, b)| segment_distance(p, a, b) > OVER)
    };
    let mut pieces = Vec::new();
    let mut piece = Vec::new();
    for w in line.windows(2) {
        let (p, q) = (w[0], w[1]);
        // Where the segment crosses the outline, in order along it.
        let mut cuts: Vec<f64> = edges().filter_map(|(a, b)| crossing(p, q, a, b)).collect();
        cuts.push(1.0);
        cuts.sort_by(f64::total_cmp);
        let mut from = 0.0;
        for &to in &cuts {
            if to <= from {
                continue;
            }
            let (a, b) = (p.lerp(q, from), p.lerp(q, to));
            if over(a.lerp(b, 0.5)) {
                if piece.len() >= 2 {
                    pieces.push(std::mem::take(&mut piece));
                }
                piece.clear();
            } else {
                if piece.last() != Some(&a) {
                    piece.push(a);
                }
                piece.push(b);
            }
            from = to;
        }
    }
    if piece.len() >= 2 {
        pieces.push(piece);
    }
    pieces
}

/// Where along `p`–`q`, from 0 to 1, it crosses `a`–`b`, if it does.
fn crossing(p: DVec2, q: DVec2, a: DVec2, b: DVec2) -> Option<f64> {
    let (r, s) = (q - p, b - a);
    let denom = r.perp_dot(s);
    if denom.abs() < 1e-12 {
        return None;
    }
    let t = (a - p).perp_dot(s) / denom;
    let u = (a - p).perp_dot(r) / denom;
    ((0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)).then_some(t)
}

/// How far `p` is from the segment `a`–`b`.
fn segment_distance(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-12)).clamp(0.0, 1.0);
    p.distance(a + t * ab)
}

/// A keep-clear zone's outline, across the lanes of `road` going its way, and
/// its hatches, at 45° across it.
fn keep_clear(road: &Road, line: &Polyline, zone: &KeepClear, out: &mut RenderGeometry) {
    let lanes: Vec<_> = road.lanes.iter().filter(|l| l.direction == zone.direction).collect();
    if lanes.is_empty() {
        return;
    }
    // Its edges, by offset from the reference line, at `s`.
    let left = |s: f64| {
        lanes
            .iter()
            .map(|l| l.offset.at(s) + 0.5 * l.width.at(s))
            .fold(f64::MIN, f64::max)
    };
    let right = |s: f64| {
        lanes
            .iter()
            .map(|l| l.offset.at(s) - 0.5 * l.width.at(s))
            .fold(f64::MAX, f64::min)
    };
    // The point offset `t` from the reference line at `s`, between vertices.
    let at = |s: f64, t: f64| {
        let i = line.s.partition_point(|&x| x <= s).clamp(1, line.s.len() - 1);
        let u = ((s - line.s[i - 1]) / (line.s[i] - line.s[i - 1])).clamp(0.0, 1.0);
        let point = line.points[i - 1].lerp(line.points[i], u);
        let miter = line.miters[i - 1].lerp(line.miters[i], u);
        point + t * miter
    };
    let inside: Vec<f64> =
        line.s.iter().copied().filter(|&s| s > zone.from && s < zone.to).collect();
    let stations: Vec<f64> = std::iter::once(zone.from)
        .chain(inside)
        .chain(std::iter::once(zone.to))
        .collect();
    let mut outline: Vec<DVec2> = stations.iter().map(|&s| at(s, left(s))).collect();
    outline.extend(stations.iter().rev().map(|&s| at(s, right(s))));
    outline.push(outline[0]);
    out.markings.push(Marking { points: outline, style: MarkingStyle::KeepClear });
    // Each hatch from its right edge forward to its left, as far along as the
    // zone is wide, clipped to the zone.
    let width = left(zone.from) - right(zone.from);
    let mut s0 = zone.from - width;
    while s0 < zone.to {
        let (a, b) = (s0.max(zone.from), (s0 + width).min(zone.to));
        if b > a {
            let t = |s: f64| right(s) + (s - s0) / width * (left(s) - right(s));
            out.markings.push(Marking {
                points: vec![at(a, t(a)), at(b, t(b))],
                style: MarkingStyle::KeepClear,
            });
        }
        s0 += HATCH_SPACING;
    }
}

/// The stretch of `points`, at stations `s`, from `from` to `to`.
fn clip(points: &[DVec2], s: &[f64], from: f64, to: f64) -> Vec<DVec2> {
    let at = |x: f64| {
        let i = s.partition_point(|&y| y <= x).clamp(1, s.len() - 1);
        let u = (x - s[i - 1]) / (s[i] - s[i - 1]);
        points[i - 1].lerp(points[i], u)
    };
    let inside = s.iter().zip(points).filter(|&(&x, _)| x > from && x < to).map(|(_, &p)| p);
    std::iter::once(at(from)).chain(inside).chain(std::iter::once(at(to))).collect()
}

/// On each lane coming into an intersection: a stop line across its end, and
/// an arrow for the ways its connections turn, as SUMO classes a link's
/// direction by the angle between its lanes. No arrow on a road too short to
/// fit one.
fn approaches(
    net: &RoadNetwork,
    polylines: &[Polyline],
    intersection: &Intersection,
    turns: &[Turn],
    out: &mut RenderGeometry,
) {
    let lane = |l: LaneRef| &net.roads[l.road.0].lanes[l.lane];
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

        // Along its road end's stop line across the lane, set back by half its
        // width so that it's painted behind the line.
        let arm = intersection.arm(from.road);
        let stop = StopLine::new(line, &arm, forward);
        let total = *line.s.last().unwrap();
        let s_end = if forward { total } else { 0.0 };
        let offset = road_lane.offset.at(s_end);
        let across = |side: f64| {
            let t = offset + side * 0.5 * road_lane.width.at(s_end);
            stop.at(t) + 0.5 * STOP_LINE_WIDTH * stop.normal
        };
        out.markings.push(Marking {
            points: vec![across(1.0), across(-1.0)],
            style: MarkingStyle::Stop,
        });

        let (mut left, mut straight, mut right) = (false, false, false);
        let ways = intersection.connections.iter().zip(turns).filter(|(c, _)| c.0 == from);
        for (_, turn) in ways {
            match turn {
                Turn::Straight => straight = true,
                Turn::Left => left = true,
                Turn::Right => right = true,
            }
        }
        // Back from where its centre crosses the stop line.
        let back = arms::back(&arm, forward, offset) + ARROW_SETBACK;
        if total < back + ARROW_SETBACK {
            continue;
        }
        let s = if forward { total - back } else { back };
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

#[cfg(test)]
mod test {
    use super::*;
    use crate::{compile, scenarios};

    #[test]
    fn no_edge_line_runs_over_a_road() {
        let speed = 50.0 / 3.6;
        for net in [
            scenarios::ramps(100.0 / 3.6, 60.0 / 3.6),
            scenarios::grid(2, 2, 2, 100.0, speed),
            scenarios::signalised_crossroads(2, speed),
        ] {
            let render = compile(&net).unwrap().render;
            let edges = render.markings.iter().filter(|m| m.style == MarkingStyle::Edge);
            for marking in edges {
                for road in &render.roads {
                    let whole = outside(&marking.points, road);
                    let kept: usize = whole.iter().map(Vec::len).sum();
                    // Nothing of it is cut away: every piece is the whole line.
                    assert!(
                        whole.len() == 1 && kept >= marking.points.len(),
                        "an edge line from {:?} runs over a road",
                        marking.points[0]
                    );
                }
            }
        }
    }
}
