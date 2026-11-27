//! What to draw for a road network: road surfaces and lane markings.
//!
//! Drawn from the network's description, sampled at a tolerance of the
//! caller's choosing, not from the simulation's lanes, so how finely roads are
//! drawn is independent of how finely they are simulated.

use glam::DVec2;
use traffic_sim::SAME_POINT;

use crate::{CompileError, Network, RoadLane};

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

/// What to draw for `net`, its curves sampled to within `tolerance` meters.
pub fn render(net: &Network, tolerance: f64) -> Result<RenderGeometry, CompileError> {
    let mut out = RenderGeometry::default();
    for (r, road) in net.roads().iter().enumerate() {
        let line = road
            .path
            .sample(tolerance)
            .map_err(|error| CompileError::Path { road: r, error })?;
        let along = |offset: f64| line.offset(offset);
        // Its lanes across it, from its path's left to its right.
        let mut lanes: Vec<(&RoadLane, bool)> = road.lanes().collect();
        lanes.sort_by(|a, b| b.0.offset.total_cmp(&a.0.offset));
        // Runs of lanes side by side, each a carriageway: its surface, edge
        // lines either side, and a line between each two of its lanes.
        let mut start = 0;
        for i in 0..lanes.len() {
            let next = lanes.get(i + 1);
            let [right, _] = lanes[i].0.edges(lanes[i].1);
            let touches = next.is_some_and(|n| (n.0.edges(n.1)[1] - right).abs() < SAME_POINT);
            if let Some(&(_, reverse)) = next.filter(|_| touches) {
                let style = match reverse == lanes[i].1 {
                    true => MarkingStyle::Dashed,
                    false => MarkingStyle::Centre,
                };
                out.markings.push(Marking { points: along(right), style });
                continue;
            }
            let left = lanes[start].0.edges(lanes[start].1)[1];
            out.roads.push(Strip { left: along(left), right: along(right) });
            for edge in [left, right] {
                out.markings.push(Marking { points: along(edge), style: MarkingStyle::Edge });
            }
            start = i + 1;
        }
    }
    Ok(out)
}

#[cfg(test)]
mod test {
    use glam::DVec2;
    use traffic_sim::Side;

    use super::*;
    use crate::{Path, Road, kmh};

    fn lane(offset: f64) -> RoadLane {
        RoadLane { offset, left: 1.75, right: 1.75 }
    }

    fn styles(geometry: &RenderGeometry) -> Vec<MarkingStyle> {
        geometry.markings.iter().map(|m| m.style).collect()
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
        assert_eq!(geometry.roads.len(), 1);
        assert_eq!(
            geometry.roads[0].left,
            [DVec2::new(0.0, 3.5), DVec2::new(100.0, 3.5)]
        );
        assert_eq!(
            geometry.roads[0].right,
            [DVec2::new(0.0, -7.0), DVec2::new(100.0, -7.0)]
        );
        use MarkingStyle::*;
        assert_eq!(styles(&geometry), [Centre, Dashed, Edge, Edge]);
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
        assert_eq!(styles(&geometry), [MarkingStyle::Edge; 4]);
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
