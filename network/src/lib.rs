//! Road networks described at road level, compiled to a [`SimNetwork`] for the
//! simulation and to [`RenderGeometry`] for drawing. A proof of concept: the
//! shared format will change, and the generators in [`scenarios`] build it.
//!
//! The description follows OpenDRIVE's shape: each road has a reference line,
//! a start pose followed by pieces that each continue from the last, and lanes
//! placed beside it by an offset and a width that may vary along it. Roads are
//! joined end to start, and their lanes are matched where they meet, so merges
//! and diverges need no declaring: two lanes meeting one merge, one meeting two
//! diverges. Intersections list which lanes connect to which, and get a cubic
//! Bézier connector for each.

mod compile;
mod reference;
mod render;
pub mod scenarios;

use glam::DVec2;
use traffic_sim::{LaneId, SimNetwork};

pub use compile::{CompileError, compile};
pub use render::{Marking, MarkingStyle, RenderGeometry, Strip};

/// A position and a heading, in radians counterclockwise from +x.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    pub pos: DVec2,
    pub heading: f64,
}

impl Pose {
    pub fn new(x: f64, y: f64, heading: f64) -> Self {
        Self { pos: DVec2::new(x, y), heading }
    }

    /// Unit vector along the heading.
    pub fn dir(&self) -> DVec2 {
        DVec2::from_angle(self.heading)
    }
}

/// A piece of a road's reference line, continuing from where the last ended.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Piece {
    /// Straight on, in meters.
    Line { length: f64 },
    /// A circular arc, in meters, curving left for positive curvature (1/m).
    Arc { length: f64, curvature: f64 },
    /// A cubic Bézier from the end of the last piece, with absolute control
    /// points. It continues smoothly only if `c1` lies straight ahead.
    Bezier { c1: DVec2, c2: DVec2, end: DVec2 },
}

/// How a lane's offset or width varies along its road, in meters, by distance
/// along the reference line.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Profile {
    Constant(f64),
    /// From `from` until `start`, then easing (smoothstep) to `to` by `end`.
    Ease {
        from: f64,
        to: f64,
        start: f64,
        end: f64,
    },
}

impl Profile {
    pub fn at(&self, s: f64) -> f64 {
        match *self {
            Profile::Constant(v) => v,
            Profile::Ease { from, to, start, end } => {
                let x = ((s - start) / (end - start)).clamp(0.0, 1.0);
                from + (to - from) * x * x * (3.0 - 2.0 * x)
            }
        }
    }
}

/// Which way a lane runs along its road.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    /// From the road's start to its end.
    Forward,
    /// From the road's end to its start.
    Backward,
}

/// A lane of a road.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RoadLane {
    /// From the reference line to the lane's centre, positive to the left.
    pub offset: Profile,
    pub width: Profile,
    pub direction: Direction,
}

impl RoadLane {
    /// A lane of constant width at a constant offset.
    pub fn new(offset: f64, width: f64, direction: Direction) -> Self {
        Self {
            offset: Profile::Constant(offset),
            width: Profile::Constant(width),
            direction,
        }
    }
}

/// A road: a reference line and lanes beside it, with one speed limit.
#[derive(Clone, Debug, PartialEq)]
pub struct Road {
    pub start: Pose,
    pub pieces: Vec<Piece>,
    pub lanes: Vec<RoadLane>,
    /// In meters per second.
    pub speed_limit: f64,
}

impl Road {
    /// Where the reference line ends, and its heading there.
    pub fn end(&self) -> Pose {
        reference::end_pose(self.start, &self.pieces)
    }

    /// A road whose reference line ends at `end`, its start worked back from
    /// the pieces.
    pub fn ending_at(
        end: Pose,
        pieces: Vec<Piece>,
        lanes: Vec<RoadLane>,
        speed_limit: f64,
    ) -> Self {
        // Where the pieces end from the origin, facing +x.
        let rel = reference::end_pose(Pose::new(0.0, 0.0, 0.0), &pieces);
        let heading = end.heading - rel.heading;
        let pos = end.pos - DVec2::from_angle(heading).rotate(rel.pos);
        Self { start: Pose { pos, heading }, pieces, lanes, speed_limit }
    }
}

/// Identifies a road: its index in [`RoadNetwork::roads`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct RoadId(pub usize);

/// A lane of a road.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LaneRef {
    pub road: RoadId,
    /// Its index in the road's lanes.
    pub lane: usize,
}

/// Where lanes cross, merge and diverge freely: each connection is a lane
/// from the end of one road lane to the start of another.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Intersection {
    pub connections: Vec<(LaneRef, LaneRef)>,
}

/// A road network at road level.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RoadNetwork {
    pub roads: Vec<Road>,
    /// Each joins the end of the first road to the start of the second, which
    /// should start where and as the first ends. A road may join itself.
    pub joins: Vec<(RoadId, RoadId)>,
    pub intersections: Vec<Intersection>,
}

impl RoadNetwork {
    pub fn add_road(&mut self, road: Road) -> RoadId {
        self.roads.push(road);
        RoadId(self.roads.len() - 1)
    }

    pub fn join(&mut self, from: RoadId, to: RoadId) {
        self.joins.push((from, to));
    }
}

/// A road network compiled for the simulation and for drawing.
pub struct Compiled {
    pub sim: SimNetwork,
    pub render: RenderGeometry,
    /// Each road's lanes in the simulation, by road and lane index.
    pub road_lanes: Vec<Vec<LaneId>>,
    /// Each intersection's connectors in the simulation, in connection order.
    pub connectors: Vec<Vec<LaneId>>,
}
