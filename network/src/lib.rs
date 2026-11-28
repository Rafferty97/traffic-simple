//! Road networks described at road level, compiled to a
//! [`SimNetwork`](traffic_sim::SimNetwork) for the
//! simulation, and drawn as [`RenderGeometry`], each sampled at its own
//! tolerance.
//!
//! A network is roads, each laid along a [`Path`] with lanes beside it, going
//! forward along the path or in reverse, and joins, each one road's end running
//! on into another's start, their lanes connecting where they meet. Roads are
//! unrelated where they overlap, as at different grades, unless they join a
//! road in common or are declared to overlap: then their lanes are cut where
//! they start or stop overlapping, and vehicles on them keep apart. The
//! simulation sees only the lanes.

mod compile;
mod cuts;
mod lane;
mod path;
mod render;

use traffic_sim::Side;

pub use compile::{CompileError, Compiled, DROPPED_COST, PlaceError, compile};
pub use lane::{RoadLane, Station};
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
    /// Pairs of roads declared to overlap (see [`Network::overlap`]).
    #[cfg_attr(feature = "serde", serde(default))]
    overlaps: Vec<[RoadId; 2]>,
}

impl Network {
    /// An empty network, its traffic driving on `drives_on`.
    pub fn new(drives_on: Side) -> Self {
        Self {
            drives_on,
            roads: Vec::new(),
            joins: Vec::new(),
            overlaps: Vec::new(),
        }
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

    /// Declares that `a` and `b` overlap where their lanes' outlines do, on
    /// the same ground, so vehicles on them keep apart there: as where a road's
    /// lanes squeeze in beside another's. Otherwise roads are unrelated where
    /// they overlap, as at different grades, unless they join a road in
    /// common, as two roads merging into one or forking from one do; for
    /// those, declaring it changes nothing.
    pub fn overlap(&mut self, a: RoadId, b: RoadId) {
        self.overlaps.push([a, b]);
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

    /// Its declared overlaps, in the order they were made.
    pub fn overlaps(&self) -> &[[RoadId; 2]] {
        &self.overlaps
    }

    /// The id of its `index`th road, if it has one.
    pub fn road_id(&self, index: usize) -> Option<RoadId> {
        let road = self.roads.get(index)?;
        Some(RoadId { index: index as u32, lanes: road.lane_count() as u32 })
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

/// Identifies a road of a [`Network`]. Its lanes are numbered forward lanes
/// first, in the order they were given, then reverse ones; where they are in
/// the simulation, [`Compiled`] says.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RoadId {
    index: u32,
    lanes: u32,
}

impl RoadId {
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
