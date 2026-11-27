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

mod arms;
mod compile;
mod reference;
mod render;
pub mod scenarios;
mod turns;

use glam::DVec2;
use traffic_sim::{Classes, Keep, LaneId, Leaving, Side, SimNetwork};

pub use compile::{CompileError, compile, fit_arms};
pub use render::{Arrow, Marking, MarkingStyle, RenderGeometry, Strip};
pub use turns::Turn;

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
    /// Whether a vehicle on it may change into its neighbour on the left, or
    /// on the right, of its direction of travel, if it has one: SUMO's
    /// `changeLeft` and `changeRight`.
    pub change_left: bool,
    pub change_right: bool,
    /// How drivers leave it, if it ends by merging into its neighbour, as an
    /// acceleration lane does: `None` if it doesn't.
    pub leaving: Option<Leaving>,
    /// The vehicle classes that may use it, as SUMO's `allow` and `disallow`
    /// have it: all but trams by default, trams alone on a tram track of its
    /// own, or every class on one shared with general traffic.
    pub allows: Classes,
}

impl RoadLane {
    /// A lane of constant width at a constant offset, which vehicles may change
    /// out of either way.
    pub fn new(offset: f64, width: f64, direction: Direction) -> Self {
        Self {
            offset: Profile::Constant(offset),
            width: Profile::Constant(width),
            direction,
            change_left: true,
            change_right: true,
            leaving: None,
            allows: Classes::ALL_BUT_TRAMS,
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
    /// Stretches its traffic keeps clear.
    pub keep_clear: Vec<KeepClear>,
}

/// A stretch of a road that its traffic going one way keeps clear, as a box
/// painted across a side road's mouth asks: a vehicle enters it only if there
/// is room for it beyond, so a queue leaves it empty.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KeepClear {
    /// Where it starts and ends along the road's reference line, in meters,
    /// `from` before `to`.
    pub from: f64,
    pub to: f64,
    /// The way the lanes it is across run.
    pub direction: Direction,
}

impl Road {
    /// How long its reference line is, in meters.
    pub fn length(&self) -> f64 {
        let line = reference::Polyline::new(self.start, &self.pieces, &[], [&[], &[]]);
        *line.s.last().expect("a line has its ends")
    }

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
        Self {
            start: Pose { pos, heading },
            pieces,
            lanes,
            speed_limit,
            keep_clear: Vec::new(),
        }
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
    pub control: Control,
    /// How wide its connectors are, in meters. By default each is as wide as
    /// the widest vehicle sweeps along it, with the room one needs to pass
    /// another on each side: narrower than a road lane, so connectors side by
    /// side at a mouth don't touch and conflict there, but wide enough that a
    /// vehicle's body stays within its own.
    pub connector_width: Option<f64>,
    /// How far each far-side turn runs straight on from its lane's end before
    /// its curve begins, in meters: as drivers turning across oncoming traffic
    /// pull into the intersection to wait, rather than turning from the line.
    pub far_side_straight: f64,
    /// How each road meets it, where that isn't simply at its end with a stop
    /// line square across it there (see [`Arm`]).
    pub arms: Vec<Arm>,
    /// At signals, movements that may turn on red besides those the network
    /// allows ([`RoadNetwork::turn_on_red`]), each the road it comes in from and
    /// which way it goes: turning left on red from one one-way street into
    /// another, say.
    pub turn_on_red: Vec<(RoadId, Turn)>,
    /// At signals, the roads whose far-side turns are hook turns, as in
    /// Melbourne: each runs from its lane into the intersection, drifting
    /// toward the near side, to wait in front of the road on that side, beside
    /// its stop line, angled toward the way out, out of the way of the traffic
    /// going straight on behind it; then turns when that road's traffic goes
    /// straight on, ahead of it (see `docs/DESIGN.md`, Hook turns).
    pub hook_turns: Vec<RoadId>,
    /// How its hook turns are laid out.
    pub hook_geometry: HookGeometry,
}

/// How an intersection's hook turns are laid out, in meters (see
/// `docs/DESIGN.md`, Hook turns): where each waits, relative to its lane in
/// and to the road on its near side, and how it gets there and turns.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HookGeometry {
    /// How far a turner moves over toward the near side from its lane in to
    /// wait: a lane, so the traffic going straight on passes it.
    pub shift: f64,
    /// How far short of the near-side road's lane, the edge toward its
    /// middle, its waiting line is, so that road's traffic passes in front of
    /// it and it has room to turn.
    pub across: f64,
    /// The least it waits from the near-side road's stop line, along that
    /// road: a cap on `shift`, for a tight intersection.
    pub along: f64,
    /// How far it runs straight on to where it waits: as long as a car waiting
    /// there and its gap to the line, so it waits facing the way it came in,
    /// parallel with the traffic going straight on. It moves over before this.
    pub queue: f64,
    /// How far along it moves over, as late as it can before its straight run:
    /// shorter moves it over more sharply, keeping it off the corner it comes
    /// in by.
    pub drift: f64,
    /// How far across its turn runs: from where it waits, it turns onto the
    /// line of its way out this far across, then runs on along it.
    pub turn_run: f64,
}

impl Default for HookGeometry {
    fn default() -> Self {
        Self {
            shift: 5.5,
            across: -0.2,
            along: 3.5,
            queue: 9.5,
            drift: 10.0,
            turn_run: 6.0,
        }
    }
}

/// How a road meets an intersection: where its stop line is, and how far its
/// lanes are part of the intersection. Its lanes coming in end where their stop
/// line is furthest back, and its lanes going out start `run_out` from its end:
/// those stretches belong to the intersection's connectors, so a vehicle's body
/// sweeping across them as it turns is in the junction, not on a road lane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Arm {
    pub road: RoadId,
    /// How far back from the road's end its stop line crosses its reference
    /// line, in meters.
    pub stop_line: f64,
    /// The stop line's angle from square across the road, in radians,
    /// counterclockwise: as it lies along a road crossing at an angle.
    pub skew: f64,
    /// How far into the road, from its end, turns into it may finish, in
    /// meters: a turn into one of its lanes other than the nearest on its side
    /// drifts across to it over that stretch, as drivers do.
    pub run_out: f64,
}

impl Arm {
    /// A road meeting an intersection at its end, its stop line square there.
    pub fn new(road: RoadId) -> Self {
        Arm { road, stop_line: 0.0, skew: 0.0, run_out: 0.0 }
    }
}

impl Intersection {
    /// How `road` meets it.
    pub fn arm(&self, road: RoadId) -> Arm {
        self.arms.iter().find(|a| a.road == road).copied().unwrap_or(Arm::new(road))
    }
}

/// How an intersection is controlled (see `docs/DESIGN.md`, Intersection
/// control).
#[derive(Clone, Debug, Default, PartialEq)]
pub enum Control {
    /// Nothing but the junction backstop: whoever can no longer stop goes first.
    #[default]
    Uncontrolled,
    /// A fixed-time signal plan, its phases run in turn.
    Signals(Vec<SignalPhase>),
    /// An all-way stop, as in the US: every approach stops at its line, and
    /// vehicles go in the order they came to rest there, every movement of
    /// the same priority.
    AllWayStop,
    /// Priority roads, `major`, and every other approach with a sign, the one
    /// `signs` gives its road, or a give-way sign. Connections have levels,
    /// the higher going first: straight on or near-side from a priority road,
    /// 3; far-side from one, 2; straight on or near-side from any other road,
    /// 1; far-side from any other, 0. A far-side turn from a priority road
    /// waits inside the intersection for a gap in its oncoming traffic, and
    /// every other approach gives way at its line to every connection of a
    /// higher level its own crosses or merges with.
    Priority { major: Vec<RoadId>, signs: Vec<(RoadId, Sign)> },
}

/// The sign on an approach without priority.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Sign {
    /// Give way: go once there is a gap in the traffic with priority.
    #[default]
    GiveWay,
    /// Stop: come to rest at the line, then go once there is a gap.
    Stop,
}

/// One phase of an intersection's signal plan.
#[derive(Clone, Debug, PartialEq)]
pub struct SignalPhase {
    /// The movements green in it, each the road its connections come in from
    /// and which way they go.
    pub green: Vec<(RoadId, Turn)>,
    /// How long it lasts once they are all green, in seconds.
    pub duration: f64,
}

/// Which side of the road traffic drives on: the left, as in Australia, by
/// default.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DrivesOn {
    #[default]
    Left,
    Right,
}

impl DrivesOn {
    pub fn side(self) -> Side {
        match self {
            DrivesOn::Left => Side::Left,
            DrivesOn::Right => Side::Right,
        }
    }
}

/// A road network at road level.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RoadNetwork {
    /// Which side of the road traffic drives on.
    pub drives_on: DrivesOn,
    /// Which side drivers keep to on roads with speed limits above
    /// `keep_above`: in Australia, the left, above 80 km/h.
    pub keep: Keep,
    /// In meters per second.
    pub keep_above: f64,
    /// Whether near-side turns at signals may turn on red, coming to rest at
    /// their line and then going through a gap, as in much of North America.
    /// Not by default, as in Australia.
    pub turn_on_red: bool,
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
    /// Which way each intersection's connections go, in connection order.
    pub turns: Vec<Vec<Turn>>,
}
