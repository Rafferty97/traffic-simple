//! Road networks described at road level, compiled to a
//! [`SimNetwork`](traffic_sim::SimNetwork) for the
//! simulation, and drawn as [`RenderGeometry`], each sampled at its own
//! tolerance.
//!
//! A network is roads, each laid along a [`Path`] with lanes beside it, going
//! forward along the path or in reverse, and joins, each one road's end running
//! on into another's start, their lanes connecting where they meet. The
//! simulation sees only the lanes.

mod compile;
mod path;
mod render;

use traffic_sim::{LaneId, Side};

pub use compile::{CompileError, compile, simulate};
pub use path::{MIN_LENGTH, Path, PathError, Segment};
pub use render::{Arrow, Marking, MarkingStyle, RenderGeometry, Strip, render};

/// A road network.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct Network {
    /// Which side of the road traffic drives on.
    pub drives_on: Side,
    roads: Vec<Road>,
    /// Each the end of one road running on into the start of another (see
    /// [`Network::join`]).
    #[cfg_attr(feature = "serde", serde(default))]
    joins: Vec<[RoadId; 2]>,
}

impl Network {
    /// An empty network, its traffic driving on `drives_on`.
    pub fn new(drives_on: Side) -> Self {
        Self { drives_on, roads: Vec::new(), joins: Vec::new() }
    }

    /// Adds a road, and returns its id.
    pub fn add_road(&mut self, road: Road) -> RoadId {
        self.roads.push(road);
        self.road_id(self.roads.len() - 1).expect("it was just added")
    }

    /// Joins the end of `from` to the start of `to`, which should start where
    /// `from` ends: `from`'s forward lanes run on into `to`'s, and `to`'s
    /// reverse lanes into `from`'s. Lanes connect wherever one ends where
    /// another starts, so two lanes meeting one merge, and one meeting two
    /// diverge. A road may join itself, as a ring does.
    ///
    /// Compiling fails if a lane at the join meets none across it.
    pub fn join(&mut self, from: RoadId, to: RoadId) {
        self.joins.push([from, to]);
    }

    /// Its roads, in the order they were added.
    pub fn roads(&self) -> &[Road] {
        &self.roads
    }

    /// Its joins, each from one road's end to another's start, in the order
    /// they were made.
    pub fn joins(&self) -> &[[RoadId; 2]] {
        &self.joins
    }

    /// The id of its `index`th road, if it has one.
    pub fn road_id(&self, index: usize) -> Option<RoadId> {
        let road = self.roads.get(index)?;
        let first_lane = self.roads[..index].iter().map(Road::lane_count).sum::<usize>();
        Some(RoadId {
            index: index as u32,
            first_lane: first_lane as u32,
            lanes: road.lane_count() as u32,
        })
    }
}

/// A road: a path, lanes beside it, and a speed limit.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct Road {
    /// The line the road is laid along, which its lanes are offset from.
    pub path: Path,
    /// In meters per second (see [`kmh`] and [`mph`]).
    pub speed_limit: f64,
    /// The lanes running the way the path does.
    pub forward: Vec<RoadLane>,
    /// The lanes running the other way, if any.
    #[cfg_attr(feature = "serde", serde(default))]
    pub reverse: Vec<RoadLane>,
}

impl Road {
    fn lane_count(&self) -> usize {
        self.forward.len() + self.reverse.len()
    }

    /// Its lanes, forward ones first, each with whether it runs in reverse.
    fn lanes(&self) -> impl Iterator<Item = (&RoadLane, bool)> {
        let forward = self.forward.iter().map(|l| (l, false));
        forward.chain(self.reverse.iter().map(|l| (l, true)))
    }
}

/// A lane of a road, in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RoadLane {
    /// From the road's path to the lane's centreline, positive to the left of
    /// the path's direction, whichever way the lane runs.
    pub offset: f64,
    /// From the centreline to the lane's left edge, the left of the way the lane
    /// runs.
    pub left: f64,
    /// From the centreline to the lane's right edge, the right of the way the
    /// lane runs.
    pub right: f64,
}

impl RoadLane {
    /// Its edges, right then left of the path's direction, as offsets from it.
    fn edges(&self, reverse: bool) -> [f64; 2] {
        match reverse {
            false => [self.offset - self.right, self.offset + self.left],
            true => [self.offset - self.left, self.offset + self.right],
        }
    }
}

/// Identifies a road of a [`Network`], and through it its lanes in the
/// simulation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(rename_all = "camelCase"))]
pub struct RoadId {
    index: u32,
    first_lane: u32,
    lanes: u32,
}

impl RoadId {
    /// Its `i`th lane in the simulation: forward lanes first, in the order
    /// they were given, then reverse ones.
    ///
    /// Panics if it has no `i`th lane, as indexing a slice does.
    pub fn lane(self, i: usize) -> LaneId {
        assert!(
            i < self.lane_count(),
            "road {} has {} lanes, not {}",
            self.index,
            self.lanes,
            i + 1
        );
        LaneId(self.first_lane + i as u32)
    }

    /// How many lanes it has.
    pub fn lane_count(self) -> usize {
        self.lanes as usize
    }

    /// Its index among the network's roads.
    pub fn index(self) -> usize {
        self.index as usize
    }
}

/// A speed in km/h, in meters per second.
pub fn kmh(speed: f64) -> f64 {
    speed / 3.6
}

/// A speed in mph, in meters per second.
pub fn mph(speed: f64) -> f64 {
    speed * 0.44704
}
