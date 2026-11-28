//! Lanes and their links: successors, neighbours and the side drivers keep to.

use arrayvec::ArrayVec;
use glam::DVec2;

use super::geometry::{LaneError, LaneGeometry};
use super::{Barrier, JunctionId};
use crate::class::Classes;

/// The most successors a lane can have: enough for a lane into a large
/// intersection that goes straight on, turns into up to three lanes either
/// way, and turns back. Inline, as the network is built once and read often.
pub const MAX_SUCCESSORS: usize = 8;

/// The most predecessors a lane can have: a lane leaving a junction collects
/// connectors from several approaches.
pub const MAX_PREDECESSORS: usize = 8;

/// Identifies a lane: its index in [`SimNetwork`](super::SimNetwork)'s lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LaneId(pub u32);

/// Identifies a road, for routing: lanes of the same road are equal for
/// finding a way, as a road's lanes are to a driver choosing a route, but for
/// the lane changes they need. Opaque to the simulation, and the network's to
/// set: a road's two directions are different roads, and so are the ways
/// through an intersection from one road into another. A lane that stands
/// alone is a road of its own, with an id no other lane has.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RoadId(pub u32);

/// A side of a lane, relative to its direction of travel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub fn opposite(self) -> Side {
        match self {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
        }
    }
}

/// How nearly, in meters, two neighbouring lanes' ends must be square across
/// them to count as together: snapping their points to the grid can tilt a
/// short first or last segment, so this is looser than [`SAME_POINT`](super::geometry::SAME_POINT).
pub const ALIGNED: f64 = 0.01;

/// The lane beside another, whether vehicles may change into it, and where
/// along the lane it is stored on the two run side by side: from where they
/// start together to where they end together, or to where the first of them
/// ends, square across both, as lanes coming into an intersection at a skewed
/// stop line do, each ending at its own.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Neighbor {
    pub lane: LaneId,
    /// Whether vehicles may change from the lane this is stored on into `lane`.
    pub can_change: bool,
    /// Where along the lane this is stored on the two start side by side, in
    /// meters: 0 where they start together.
    pub from: f64,
    /// Where along it they stop being side by side: its length where they end
    /// together. No lane change runs past it.
    pub until: f64,
}

/// Which side drivers on a lane keep to when they can: an incentive to change
/// lanes toward it, LMRS's keep-right bias. In Australia, keep left applies on
/// roads with limits above 80 km/h; the network decides where.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Keep {
    #[default]
    NoPreference,
    Left,
    Right,
}

impl Keep {
    /// The side to keep to, if any.
    pub fn side(self) -> Option<Side> {
        match self {
            Keep::NoPreference => None,
            Keep::Left => Some(Side::Left),
            Keep::Right => Some(Side::Right),
        }
    }
}

/// A lane: its geometry and speed limit, plus its links to other lanes, which
/// [`SimNetwork`](super::SimNetwork) sets.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lane {
    pub(super) geometry: LaneGeometry,
    /// Speed limit in meters per second.
    pub(super) speed_limit: f64,
    /// The lanes this one connects to, end to start.
    pub(super) successors: ArrayVec<LaneId, MAX_SUCCESSORS>,
    /// The lanes that connect to this one, end to start: the inverse of `successors`.
    pub(super) predecessors: ArrayVec<LaneId, MAX_PREDECESSORS>,
    /// The lane beside this one on the left, if any.
    pub(super) adj_left: Option<Neighbor>,
    /// The lane beside this one on the right, if any.
    pub(super) adj_right: Option<Neighbor>,
    /// The junction it belongs to, if any, and its index among that junction's lanes.
    pub(super) junction: Option<(JunctionId, u16)>,
    /// Which side drivers keep to.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(super) keep: Keep,
    /// What driving it costs routing beyond its time, in seconds.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(super) cost: f64,
    /// The road it is part of, for routing.
    pub(super) road: RoadId,
    /// Its barriers, sorted by `s`.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(super) barriers: Vec<Barrier>,
    /// Whether it is to be kept clear: a vehicle that can no longer stop before
    /// it is committed to its junction (see `docs/DESIGN.md`, Committing).
    #[cfg_attr(feature = "serde", serde(default))]
    pub(super) keep_clear: bool,
    /// The vehicle classes that may use it: all but trams by default.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(super) allows: Classes,
}

impl Lane {
    /// Builds a lane of `road` from its centerline polyline, a constant width
    /// in meters, centered on it, and a speed limit in meters per second.
    ///
    /// Points are snapped to the nearest millimeter, then consecutive duplicate
    /// points are removed.
    pub fn new(
        points: &[DVec2],
        width: f64,
        speed_limit: f64,
        road: RoadId,
    ) -> Result<Self, LaneError> {
        Self::with_half_widths(
            points,
            &vec![[0.5 * width; 2]; points.len()],
            speed_limit,
            road,
        )
    }

    /// Builds a lane of `road` from its centerline polyline, how far its left
    /// and right edges lie from it at each point, in meters, and a speed limit
    /// in meters per second. Between points, the half-widths vary linearly.
    ///
    /// Points and half-widths are snapped to the nearest millimeter, then
    /// consecutive duplicate points are removed.
    pub fn with_half_widths(
        points: &[DVec2],
        half_widths: &[[f64; 2]],
        speed_limit: f64,
        road: RoadId,
    ) -> Result<Self, LaneError> {
        let geometry = LaneGeometry::new(points, half_widths)?;
        Ok(Self {
            geometry,
            speed_limit,
            successors: ArrayVec::new(),
            predecessors: ArrayVec::new(),
            adj_left: None,
            adj_right: None,
            junction: None,
            keep: Keep::NoPreference,
            cost: 0.0,
            road,
            barriers: Vec::new(),
            keep_clear: false,
            allows: Classes::default(),
        })
    }

    /// Which side drivers keep to on it.
    pub fn keep(&self) -> Keep {
        self.keep
    }

    /// As [`Lane::keep`], to set.
    pub fn with_keep(self, keep: Keep) -> Self {
        Self { keep, ..self }
    }

    /// What driving it costs routing beyond its time, in seconds: a way
    /// through it costs that much more, as a dropped lane's taper does, so
    /// drivers change out of it where they can, and where they can't, go
    /// through it.
    pub fn cost(&self) -> f64 {
        self.cost
    }

    /// As [`Lane::cost`], to set.
    pub fn with_cost(self, cost: f64) -> Self {
        Self { cost, ..self }
    }

    /// The vehicle classes that may use it.
    pub fn allows(&self) -> Classes {
        self.allows
    }

    /// It, allowing `allows`: a tram track allows trams.
    pub fn with_allows(self, allows: Classes) -> Self {
        Self { allows, ..self }
    }

    /// The road it is part of, for routing.
    pub fn road(&self) -> RoadId {
        self.road
    }

    /// As [`Lane::road`], to set.
    pub fn with_road(self, road: RoadId) -> Self {
        Self { road, ..self }
    }

    /// Its shape, and the mapping between world positions and lane coordinates.
    pub fn geometry(&self) -> &LaneGeometry {
        &self.geometry
    }

    /// Speed limit in meters per second.
    pub fn speed_limit(&self) -> f64 {
        self.speed_limit
    }

    /// The lanes this one connects to, end to start, in the order they were connected.
    pub fn successors(&self) -> &[LaneId] {
        &self.successors
    }

    /// The lanes that connect to this one, end to start, in the order they were connected.
    pub fn predecessors(&self) -> &[LaneId] {
        &self.predecessors
    }

    /// The lane beside this one on the left, if any.
    pub fn adj_left(&self) -> Option<Neighbor> {
        self.adj_left
    }

    /// The lane beside this one on the right, if any.
    pub fn adj_right(&self) -> Option<Neighbor> {
        self.adj_right
    }

    /// Its barriers, sorted by `s`.
    pub fn barriers(&self) -> &[Barrier] {
        &self.barriers
    }

    /// Whether it is to be kept clear.
    pub fn keep_clear(&self) -> bool {
        self.keep_clear
    }

    /// The junction this lane belongs to, if any.
    pub fn junction(&self) -> Option<JunctionId> {
        self.junction.map(|(id, _)| id)
    }

    /// The lane beside this one on the given side, if any.
    pub fn adj(&self, side: Side) -> Option<Neighbor> {
        match side {
            Side::Left => self.adj_left,
            Side::Right => self.adj_right,
        }
    }
}
