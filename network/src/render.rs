//! What to draw for a road network: road surfaces and lane markings.
//!
//! Drawn from the network's description, sampled at a tolerance of the
//! caller's choosing, not from the simulation's lanes, so how finely roads are
//! drawn is independent of how finely they are simulated.

use glam::DVec2;
use traffic_sim::SAME_POINT;

use crate::{CompileError, Network, RoadLane, cuts};

/// A road network's drawable geometry, in world meters.
#[derive(Clone, Debug, Default)]
pub struct RenderGeometry {
    /// Each carriageway's surface, between the outermost edges of a run of
    /// lanes side by side.
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

/// How far inside a related road's surface a marking has to be for it to be
/// left out, in meters: lines along its edge, as two roads' outer edges are
/// once they have merged, stay.
const INSIDE: f64 = 0.05;

/// What to draw for `net`, its curves sampled to within `tolerance` meters.
/// Where roads on the same ground overlap, as where they merge or fork, each
/// road's markings stop where they run onto the other's surface, as a merge's
/// inner edge lines end where the roads meet; roads at other grades keep
/// theirs.
pub fn render(net: &Network, tolerance: f64) -> Result<RenderGeometry, CompileError> {
    // Each road's surfaces, one for each lane, and markings.
    let mut roads: Vec<(Vec<Strip>, Vec<Marking>)> = Vec::new();
    for (r, road) in net.roads().iter().enumerate() {
        let eases: Vec<f64> = road.lanes().flat_map(|(lane, _)| lane.vertices(tolerance)).collect();
        let line = road
            .path
            .sample(tolerance, &eases)
            .map_err(|error| CompileError::Path { road: r, error })?;
        let lanes: Vec<(&RoadLane, bool)> = road.lanes().collect();
        // Each lane's edges at each vertex, right then left of the path.
        let edges: Vec<Vec<[f64; 2]>> = lanes
            .iter()
            .map(|&(lane, reverse)| line.s.iter().map(|&s| lane.edges(s, reverse)).collect())
            .collect();
        let edge =
            |i: usize, side: usize| -> Vec<f64> { edges[i].iter().map(|e| e[side]).collect() };
        let strips = (0..lanes.len())
            .map(|i| Strip {
                left: line.offset(&edge(i, 1)),
                right: line.offset(&edge(i, 0)),
            })
            .collect();
        let mut markings = Vec::new();
        for i in 0..lanes.len() {
            for side in [0, 1] {
                // Each segment's line, if it has one, by its middle.
                let styles: Vec<Option<MarkingStyle>> = (0..line.points.len() - 1)
                    .map(|k| {
                        let mid = |j: usize| -> [f64; 2] {
                            let [a, b] = [edges[j][k], edges[j][k + 1]];
                            [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]
                        };
                        let at = mid(i)[side];
                        let others = (0..lanes.len()).filter(|&j| j != i);
                        style(
                            at,
                            side,
                            lanes[i].1,
                            i,
                            others.map(|j| (j, mid(j), lanes[j].1)),
                        )
                    })
                    .collect();
                let offsets = edge(i, side);
                let points = line.offset(&offsets);
                let mut k = 0;
                while k < styles.len() {
                    let Some(style) = styles[k] else {
                        k += 1;
                        continue;
                    };
                    let end = (k..styles.len())
                        .find(|&j| styles[j] != Some(style))
                        .unwrap_or(styles.len());
                    markings.push(Marking { points: points[k..=end].to_vec(), style });
                    k = end;
                }
            }
        }
        roads.push((strips, markings));
    }
    // Each road's markings, but where they are on a related road's surface.
    let mut out = RenderGeometry::default();
    let related = cuts::related(net);
    let others = |r: usize| {
        related
            .iter()
            .filter_map(move |&[a, b]| match (a == r, b == r) {
                (true, false) => Some(b),
                (false, true) => Some(a),
                _ => None,
            })
            .filter(|&q| q < roads.len())
    };
    for (r, (_, markings)) in roads.iter().enumerate() {
        for marking in markings {
            let mut pieces = vec![marking.points.clone()];
            for q in others(r) {
                for strip in &roads[q].0 {
                    let area: Vec<DVec2> =
                        strip.left.iter().chain(strip.right.iter().rev()).copied().collect();
                    pieces = pieces.iter().flat_map(|line| outside(line, &area)).collect();
                }
            }
            let style = marking.style;
            out.markings.extend(pieces.into_iter().map(|points| Marking { points, style }));
        }
    }
    out.roads = roads.into_iter().flat_map(|(strips, _)| strips).collect();
    Ok(out)
}

/// The line along a lane's edge `at`, an offset from the road's path, on its
/// right (`side` 0) or left (1), on a lane running in `reverse` or not, the
/// `i`th of its road, beside its road's other lanes, each with its index, its
/// edges and whether it runs in reverse. None where it is inside another lane,
/// as where two converge; a line between them, drawn from the lane on the
/// right, where it touches another beside it, dashed between lanes running the
/// same way and a centre line between opposite ones; and an edge line
/// elsewhere, drawn once where two lanes' edges coincide.
fn style(
    at: f64,
    side: usize,
    reverse: bool,
    i: usize,
    others: impl Iterator<Item = (usize, [f64; 2], bool)>,
) -> Option<MarkingStyle> {
    let mut edge = Some(MarkingStyle::Edge);
    for (j, [right, left], other_reverse) in others {
        if right + SAME_POINT < at && at < left - SAME_POINT {
            return None;
        }
        // Touching it, it beyond the edge.
        let beyond = match side {
            0 => (left - at).abs() < SAME_POINT && right < at - SAME_POINT,
            _ => (right - at).abs() < SAME_POINT && left > at + SAME_POINT,
        };
        if beyond {
            return match side {
                0 => None,
                _ if reverse == other_reverse => Some(MarkingStyle::Dashed),
                _ => Some(MarkingStyle::Centre),
            };
        }
        // The same edge as one before it.
        if j < i && ([right, left][side] - at).abs() < SAME_POINT {
            edge = None;
        }
    }
    edge
}

/// The stretches of `line` not inside the polygon `area` by more than
/// [`INSIDE`]: it is split wherever it crosses the polygon's edges, and each
/// piece kept or left out by its middle. A stretch along the polygon's edge,
/// within [`INSIDE`] of it, goes as the stretch before it did, or at the
/// line's start as the stretch after it: so the outer edges of two roads
/// merging run on to where they meet, and of two forking start where they
/// part, while an inner edge that goes inside the other road doesn't reappear
/// along its far side.
fn outside(line: &[DVec2], area: &[DVec2]) -> Vec<Vec<DVec2>> {
    let edges = || area.iter().zip(area.iter().cycle().skip(1));
    // Each piece, and whether it is kept, if it isn't along the edge.
    let mut pieces: Vec<(DVec2, DVec2, Option<bool>)> = Vec::new();
    for w in line.windows(2) {
        let (p, d) = (w[0], w[1] - w[0]);
        // Where along it it crosses the polygon's edges.
        let mut ts = vec![0.0, 1.0];
        for (&a, &b) in edges() {
            let e = b - a;
            let denom = d.perp_dot(e);
            if denom.abs() < 1e-12 {
                continue;
            }
            let t = (a - p).perp_dot(e) / denom;
            let u = (a - p).perp_dot(d) / denom;
            if (0.0..=1.0).contains(&u) && t > 0.0 && t < 1.0 {
                ts.push(t);
            }
        }
        ts.sort_by(f64::total_cmp);
        for t in ts.windows(2) {
            let (from, to) = (p + t[0] * d, p + t[1] * d);
            let mid = 0.5 * (from + to);
            let kept = match boundary_distance(area, mid) <= INSIDE {
                true => None,
                false => Some(!contains(area, mid)),
            };
            pieces.push((from, to, kept));
        }
    }
    let first = pieces.iter().find_map(|p| p.2).unwrap_or(true);
    let mut out: Vec<Vec<DVec2>> = Vec::new();
    let mut run: Vec<DVec2> = Vec::new();
    let mut last = first;
    for (from, to, kept) in pieces {
        last = kept.unwrap_or(last);
        match last {
            true => {
                if run.is_empty() {
                    run.push(from);
                }
                run.push(to);
            }
            false => {
                if run.len() > 1 {
                    out.push(std::mem::take(&mut run));
                }
                run.clear();
            }
        }
    }
    if run.len() > 1 {
        out.push(run);
    }
    out
}

/// Whether `point` is inside the polygon `area`, by the even-odd rule.
fn contains(area: &[DVec2], point: DVec2) -> bool {
    let mut inside = false;
    for (&a, &b) in area.iter().zip(area.iter().cycle().skip(1)) {
        if (a.y > point.y) != (b.y > point.y) {
            let x = a.x + (point.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if point.x < x {
                inside = !inside;
            }
        }
    }
    inside
}

/// How far `point` is from the polygon `area`'s edges.
fn boundary_distance(area: &[DVec2], point: DVec2) -> f64 {
    area.iter()
        .zip(area.iter().cycle().skip(1))
        .map(|(&a, &b)| {
            let e = b - a;
            let t = ((point - a).dot(e) / e.length_squared().max(1e-18)).clamp(0.0, 1.0);
            point.distance(a + t * e)
        })
        .fold(f64::MAX, f64::min)
}

#[cfg(test)]
mod test {
    use glam::DVec2;
    use traffic_sim::Side;

    use super::*;
    use crate::{Path, Road, Station, kmh};

    fn lane(offset: f64) -> RoadLane {
        RoadLane::new(offset, 3.5)
    }

    /// How many lines of each style it has: centre, dashed and edge.
    fn styles(geometry: &RenderGeometry) -> [usize; 3] {
        let count = |style| geometry.markings.iter().filter(|m| m.style == style).count();
        [
            count(MarkingStyle::Centre),
            count(MarkingStyle::Dashed),
            count(MarkingStyle::Edge),
        ]
    }

    #[test]
    fn a_two_way_road_has_a_centre_line_and_dashes_between_lanes_the_same_way() {
        let mut net = Network::new(Side::Left);
        net.add_road(Road {
            path: Path::new(DVec2::ZERO).line_to(DVec2::new(100.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(-1.75), lane(-5.25)],
            reverse: vec![lane(1.75)],
        });
        let geometry = render(&net, 0.01).unwrap();
        // A surface for each lane, together from 3.5 m left of the path to
        // 7 m right of it.
        assert_eq!(geometry.roads.len(), 3);
        let ys = geometry.roads.iter().flat_map(|r| r.left.iter().chain(&r.right)).map(|p| p.y);
        let (lo, hi) = ys.fold((f64::MAX, f64::MIN), |(lo, hi), y| (lo.min(y), hi.max(y)));
        assert_eq!((lo, hi), (-7.0, 3.5));
        assert_eq!(styles(&geometry), [1, 1, 2]);
        let line = |style| geometry.markings.iter().find(|m| m.style == style).unwrap();
        assert_eq!(line(MarkingStyle::Centre).points[0].y, 0.0);
        assert_eq!(line(MarkingStyle::Dashed).points[0].y, -3.5);
    }

    #[test]
    fn lanes_apart_are_carriageways_of_their_own() {
        let mut net = Network::new(Side::Left);
        net.add_road(Road {
            path: Path::new(DVec2::ZERO).line_to(DVec2::new(100.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(-3.0)],
            reverse: vec![lane(3.0)],
        });
        let geometry = render(&net, 0.01).unwrap();
        assert_eq!(geometry.roads.len(), 2);
        assert_eq!(styles(&geometry), [0, 0, 4]);
    }

    /// A road whose outer lane eases in over its inner one, from 100 m to
    /// 180 m along, until the two are one.
    fn converging() -> Network {
        let mut net = Network::new(Side::Left);
        let easing = RoadLane::through(vec![
            Station::new(100.0, -5.25, 3.5),
            Station::new(180.0, -1.75, 3.5),
        ]);
        net.add_road(Road {
            path: Path::new(DVec2::ZERO).line_to(DVec2::new(200.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(-1.75), easing],
            reverse: vec![],
        });
        net
    }

    #[test]
    fn lanes_converging_in_a_road_have_no_line_between_them_where_they_overlap() {
        let geometry = render(&converging(), 0.005).unwrap();
        for marking in &geometry.markings {
            assert!(!runs_inside(&geometry, marking), "{:?}", marking.points);
        }
        // Dashed between them while they are side by side, and no more.
        let dashed: Vec<_> =
            geometry.markings.iter().filter(|m| m.style == MarkingStyle::Dashed).collect();
        assert_eq!(dashed.len(), 1);
        let end = dashed[0].points.last().unwrap();
        assert!((end.x - 100.0).abs() < 1.0 && end.y == -3.5, "{end}");
        // The edges either side run all the way.
        let reach = |y: f64| {
            geometry
                .markings
                .iter()
                .any(|m| m.style == MarkingStyle::Edge && m.points[0].y == y)
        };
        assert!(reach(0.0) && reach(-7.0));
    }

    /// Two one-lane roads curving together from 60 m apart to meet end on,
    /// joined to a third: related, as they join a road in common.
    fn merging(joined: bool) -> Network {
        let mut net = Network::new(Side::Left);
        let p = DVec2::new;
        let one_lane = |path| Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0)],
            reverse: vec![],
        };
        let curve_in =
            |y: f64| Path::new(p(0.0, y)).cubic_to(p(75.0, y), p(75.0, 0.0), p(150.0, 0.0));
        let north = net.add_road(one_lane(curve_in(30.0)));
        let south = net.add_road(one_lane(curve_in(-30.0)));
        let merged = net.add_road(one_lane(Path::new(p(150.0, 0.0)).line_to(p(250.0, 0.0))));
        if joined {
            net.join(north, merged);
            net.join(south, merged);
        }
        net
    }

    /// Whether a line drawn is somewhere well inside a road's surface.
    fn runs_inside(geometry: &RenderGeometry, marking: &Marking) -> bool {
        geometry.roads.iter().any(|strip| {
            let area: Vec<DVec2> =
                strip.left.iter().chain(strip.right.iter().rev()).copied().collect();
            marking.points.windows(2).any(|w| {
                let mid = 0.5 * (w[0] + w[1]);
                contains(&area, mid) && boundary_distance(&area, mid) > 2.0 * INSIDE
            })
        })
    }

    #[test]
    fn where_roads_merge_their_inner_edges_end_where_they_meet() {
        let geometry = render(&merging(true), 0.005).unwrap();
        assert_eq!(
            geometry.markings.len(),
            6,
            "each road's two edges, the inner ones shortened"
        );
        for marking in &geometry.markings {
            assert!(!runs_inside(&geometry, marking), "{:?}", marking.points);
        }
        // The outer edges run on to where the roads meet, the inner ones stop
        // short, at the gore.
        let ends: Vec<DVec2> =
            geometry.markings[..4].iter().map(|m| *m.points.last().unwrap()).collect();
        let at = |y: f64| ends.iter().any(|e| e.distance(DVec2::new(150.0, y)) < 1e-6);
        assert!(at(1.75) && at(-1.75), "{ends:?}");
        assert!(ends.iter().filter(|e| e.x < 140.0).count() == 2, "{ends:?}");
    }

    #[test]
    fn where_roads_fork_their_inner_edges_start_where_they_part() {
        // The merge the other way round: one road forking into two.
        let mut net = Network::new(Side::Left);
        let p = DVec2::new;
        let one_lane = |path| Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0)],
            reverse: vec![],
        };
        let branch =
            |y: f64| Path::new(p(100.0, 0.0)).cubic_to(p(175.0, 0.0), p(175.0, y), p(250.0, y));
        let before = net.add_road(one_lane(Path::new(p(0.0, 0.0)).line_to(p(100.0, 0.0))));
        for y in [30.0, -30.0] {
            let to = net.add_road(one_lane(branch(y)));
            net.join(before, to);
        }
        let geometry = render(&net, 0.005).unwrap();
        assert_eq!(geometry.markings.len(), 6);
        for marking in &geometry.markings {
            assert!(!runs_inside(&geometry, marking), "{:?}", marking.points);
        }
        let starts: Vec<DVec2> = geometry.markings[2..].iter().map(|m| m.points[0]).collect();
        let at = |y: f64| starts.iter().any(|e| e.distance(DVec2::new(100.0, y)) < 1e-6);
        assert!(at(1.75) && at(-1.75), "{starts:?}");
        assert!(
            starts.iter().filter(|e| e.x > 110.0).count() == 2,
            "{starts:?}"
        );
    }

    #[test]
    fn roads_unrelated_where_they_overlap_keep_their_lines() {
        let geometry = render(&merging(false), 0.005).unwrap();
        assert_eq!(geometry.markings.len(), 6);
        // Each runs on over the other.
        let inside = geometry.markings.iter().filter(|m| runs_inside(&geometry, m)).count();
        assert_eq!(inside, 2);
    }

    #[test]
    fn drawing_is_sampled_at_its_own_tolerance() {
        let mut net = Network::new(Side::Left);
        let k = 0.5523 * 50.0;
        let path = Path::new(DVec2::ZERO).cubic_to(
            DVec2::new(k, 0.0),
            DVec2::new(50.0, 50.0 - k),
            DVec2::new(50.0, 50.0),
        );
        net.add_road(Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0)],
            reverse: vec![],
        });
        let vertices = |tolerance| render(&net, tolerance).unwrap().roads[0].left.len();
        assert!(vertices(0.001) > 2 * vertices(0.01));
    }
}
