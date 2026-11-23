use std::ops::Range;

use arrayvec::ArrayVec;
use glam::DVec2;

use crate::lane::{LaneError, LaneGeometry, LanePos, SAME_POINT};

/// The most successors a lane can have: enough for a lane into a large
/// intersection that goes straight on, turns into up to three lanes either
/// way, and turns back. Inline, as the network is built once and read often.
pub const MAX_SUCCESSORS: usize = 8;

/// The most predecessors a lane can have: a lane leaving a junction collects
/// connectors from several approaches.
pub const MAX_PREDECESSORS: usize = 8;

/// Identifies a lane: its index in [`SimNetwork`]'s lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LaneId(pub u32);

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

/// The lane beside another, and whether vehicles may change into it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Neighbor {
    pub lane: LaneId,
    /// Whether vehicles may change from the lane this is stored on into `lane`.
    pub can_change: bool,
}

/// Why a network breaks an assumption the simulation makes about it.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum NetworkError {
    #[error("{lane:?} has {neighbor:?} on its {side:?}, but not the other way round")]
    OneWayNeighbor { lane: LaneId, side: Side, neighbor: LaneId },
    #[error(
        "{successor:?} follows {lane:?}, but its {side:?} neighbor {beside:?} doesn't \
         follow {lane:?}'s {side:?} neighbor {neighbor:?}"
    )]
    SectionMismatch {
        lane: LaneId,
        side: Side,
        neighbor: LaneId,
        successor: LaneId,
        beside: LaneId,
    },
    #[error("{successor:?} follows {lane:?} but starts {gap} m from its end")]
    Disconnected { lane: LaneId, successor: LaneId, gap: f64 },
    #[error("{neighbor:?} is beside {lane:?} but its {end:?} is {offset} m along from {lane:?}'s")]
    NotAligned { lane: LaneId, neighbor: LaneId, end: End, offset: f64 },
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

/// How drivers leave a lane that ends by merging into its neighbour, as an
/// acceleration lane does.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Leaving {
    /// They stay on it to the merge, where they take turns: a zipper.
    #[default]
    Zipper,
    /// They change out of it along the way, wanting to more the nearer its end:
    /// LMRS's route desire.
    Early,
}

/// A lane: its geometry and speed limit, plus its links to other lanes, which
/// [`SimNetwork`] sets.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Lane {
    geometry: LaneGeometry,
    /// Speed limit in meters per second.
    speed_limit: f64,
    /// The lanes this one connects to, end to start.
    successors: ArrayVec<LaneId, MAX_SUCCESSORS>,
    /// The lanes that connect to this one, end to start: the inverse of `successors`.
    predecessors: ArrayVec<LaneId, MAX_PREDECESSORS>,
    /// The lane beside this one on the left, if any.
    adj_left: Option<Neighbor>,
    /// The lane beside this one on the right, if any.
    adj_right: Option<Neighbor>,
    /// The junction it belongs to, if any, and its index among that junction's lanes.
    junction: Option<(JunctionId, u16)>,
    /// Which side drivers keep to.
    #[cfg_attr(feature = "serde", serde(default))]
    keep: Keep,
    /// How drivers leave it, if it ends by merging into its neighbour: `None`
    /// if it doesn't.
    #[cfg_attr(feature = "serde", serde(default))]
    leaving: Option<Leaving>,
    /// Its barriers, sorted by `s`.
    #[cfg_attr(feature = "serde", serde(default))]
    barriers: Vec<Barrier>,
    /// Whether it is to be kept clear: a vehicle that can no longer stop before
    /// it is committed to its junction (see `docs/DESIGN.md`, Committing).
    #[cfg_attr(feature = "serde", serde(default))]
    keep_clear: bool,
}

impl Lane {
    /// Builds a lane from its centerline polyline, a constant width in meters,
    /// centered on it, and a speed limit in meters per second.
    ///
    /// Points are snapped to the nearest millimeter, then consecutive duplicate
    /// points are removed.
    pub fn new(points: &[DVec2], width: f64, speed_limit: f64) -> Result<Self, LaneError> {
        Self::with_half_widths(points, &vec![[0.5 * width; 2]; points.len()], speed_limit)
    }

    /// Builds a lane from its centerline polyline, how far its left and right
    /// edges lie from it at each point, in meters, and a speed limit in meters
    /// per second. Between points, the half-widths vary linearly.
    ///
    /// Points and half-widths are snapped to the nearest millimeter, then
    /// consecutive duplicate points are removed.
    pub fn with_half_widths(
        points: &[DVec2],
        half_widths: &[[f64; 2]],
        speed_limit: f64,
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
            leaving: None,
            barriers: Vec::new(),
            keep_clear: false,
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

    /// How drivers leave it, if it ends by merging into its neighbour.
    pub fn leaving(&self) -> Option<Leaving> {
        self.leaving
    }

    /// As [`Lane::leaving`], to set.
    pub fn with_leaving(self, leaving: Option<Leaving>) -> Self {
        Self { leaving, ..self }
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

    /// Its place among its junction's lanes, if it is in one.
    pub(crate) fn junction_slot(&self) -> Option<u16> {
        self.junction.map(|(_, i)| i)
    }

    /// The lane beside this one on the given side, if any.
    pub fn adj(&self, side: Side) -> Option<Neighbor> {
        match side {
            Side::Left => self.adj_left,
            Side::Right => self.adj_right,
        }
    }
}

/// A point on a lane that vehicles stop at until they commit to crossing it (see
/// `docs/DESIGN.md`, Barriers and signals).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Barrier {
    /// Where it is along its lane, in meters: the order barriers are met in,
    /// and where one is passed. With a line, no vehicle crossing the line
    /// along the lane reaches it before here, so stopping short of it is safe.
    pub s: f64,
    pub kind: BarrierKind,
    /// The line itself, in world space, if it isn't square across the lane at
    /// `s`: vehicles stop by how far their nearest corner is from it.
    #[cfg_attr(feature = "serde", serde(default))]
    pub line: Option<Line>,
    /// The lanes it gives way to, if any (see [`SimNetwork::yields`]): a
    /// give-way line's gap is judged against vehicles on them, and a signal's
    /// lists the turners' next stretches it lets go first.
    #[cfg_attr(feature = "serde", serde(default))]
    pub yields: Option<YieldsId>,
}

impl Barrier {
    /// A barrier square across its lane at `s`, giving way to nothing.
    pub fn new(s: f64, kind: BarrierKind) -> Self {
        Barrier { s, kind, line: None, yields: None }
    }
}

/// Identifies a set of lanes a barrier gives way to: its index in the
/// network's sets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct YieldsId(pub u32);

/// A line in world space, as a half plane: the side `normal` points to is
/// short of it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line {
    /// A point on it.
    pub point: DVec2,
    /// Its unit normal, pointing back toward the vehicles coming up to it.
    pub normal: DVec2,
}

impl Line {
    /// How far `point` is short of the line, negative once over it.
    pub fn short_of(&self, point: DVec2) -> f64 {
        (point - self.point).dot(self.normal)
    }
}

/// What a barrier is, and so when a vehicle commits to crossing it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BarrierKind {
    /// A signal for a movement: crossed once a vehicle can no longer stop
    /// comfortably before it, while the light is green. With `on_red`, the
    /// lanes it gives way to turning on red, it is a stop line on red: crossed
    /// once the vehicle has come to rest there and there is a gap in the
    /// traffic on them.
    Signal {
        movement: MovementId,
        #[cfg_attr(feature = "serde", serde(default))]
        on_red: Option<YieldsId>,
    },
    /// A give-way line: crossed once there is a gap in the traffic with
    /// priority.
    GiveWay,
    /// A stop line: crossed once the vehicle has come to rest there and there
    /// is a gap in the traffic with priority.
    Stop,
    /// An all-way stop line: crossed once the vehicle has come to rest there,
    /// no vehicle at a conflicting all-way stop line of its junction came to
    /// rest earlier, and there is a gap in the traffic that has committed.
    AllWayStop,
    /// The start of a keep-clear zone `length` meters long, in meters along
    /// the vehicle's way: crossed once there is room beyond its far end, and
    /// it can no longer stop comfortably before it.
    KeepClear { length: f64 },
}

/// Identifies a signal movement: its index among every controller's movements,
/// in the order they were added, which is also its light's index in the state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MovementId(pub u32);

/// A fixed-time signal plan (see `docs/DESIGN.md`, Barriers and signals).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SignalPlan {
    /// At most 64, so that a mask of them fits a `u64`.
    pub movements: Vec<Movement>,
    /// Run in turn, round and round.
    pub phases: Vec<Phase>,
}

/// A group of lanes whose signals go and stop together.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Movement {
    /// The movements of its plan it conflicts with, by bit.
    pub conflicts: u64,
    /// How long it shows amber, in seconds.
    pub amber: f64,
    /// How long it must have been red before a movement it conflicts with turns
    /// green, in seconds: time for its traffic to clear.
    pub clearance: f64,
}

/// One phase of a signal plan.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Phase {
    /// The movements green in it, by bit.
    pub green: u64,
    /// How long it lasts once they are all green, in seconds.
    pub duration: f64,
}

/// Identifies a junction: its index in [`SimNetwork`]'s junctions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JunctionId(pub u32);

/// A group of lanes whose vehicles a single queue keeps apart, and where those
/// lanes conflict.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Junction {
    /// Its lanes. A lane's index here is its row and column in `conflicts`.
    lanes: Vec<LaneId>,
    /// Where each ordered pair of its lanes conflicts, if they do: `lanes.len()²`
    /// entries, the row the lane it is seen from. Each pair is stored both ways
    /// round.
    conflicts: Vec<Option<Conflict>>,
    /// Where each of its lanes' first conflict with the rest of it starts along
    /// it, if it has one, by its index in `lanes`: derived from `conflicts`, and
    /// so not saved but worked out again on restoring.
    #[cfg_attr(feature = "serde", serde(skip))]
    starts: Vec<Option<f64>>,
}

impl Junction {
    /// Its lanes.
    pub fn lanes(&self) -> &[LaneId] {
        &self.lanes
    }

    /// Works out where each of its lanes' first conflict starts, from its
    /// conflicts.
    fn derive_starts(&mut self) {
        let n = self.lanes.len();
        self.starts = (0..n)
            .map(|i| {
                let row = &self.conflicts[i * n..(i + 1) * n];
                row.iter().flatten().map(|c| c.mine.start).reduce(f64::min)
            })
            .collect();
    }
}

/// Where two lanes in a junction conflict, seen from one of them.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Conflict {
    pub kind: ConflictKind,
    /// The stretch of the lane it is seen from that overlaps the other, in `s`.
    pub mine: Range<f64>,
    /// The stretch of the other lane that overlaps it.
    pub theirs: Range<f64>,
}

/// How two lanes conflict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ConflictKind {
    /// They lead into the same lane: vehicles from both take turns.
    Merge,
    /// They cross: vehicles on both must never be in the overlap together.
    Cross,
}

/// One end of a lane.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum End {
    Start,
    End,
}

/// The compiled network the simulation runs on.
///
/// Each lane holds its own successors and neighbors. Adjacent lanes start and
/// end together, like the lanes of a SUMO edge or an OpenDRIVE lane section, so
/// the neighbor of a lane's successor is the successor of its neighbor.
/// [`SimNetwork::validate`] checks this.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimNetwork {
    lanes: Vec<Lane>,
    junctions: Vec<Junction>,
    /// The signal plans, and where each one's movements start among all of them.
    #[cfg_attr(feature = "serde", serde(default))]
    signals: Vec<(SignalPlan, u32)>,
    /// The sets of lanes barriers give way to.
    #[cfg_attr(feature = "serde", serde(default))]
    yields: Vec<Vec<LaneId>>,
}

/// The most a rounded corner of a vehicle's front axle's course reaches either
/// side of its vertex, in meters (see [`SimNetwork::smooth_point_along`]):
/// enough to round the polyline's kinks, short enough to keep a lane change's
/// offset, and shorter than a wheelbase, so that the front axle is never in a
/// corner needing a lane its path has left behind.
pub(crate) const FILLET: f64 = 2.0;

impl SimNetwork {
    /// Builds a network from its lanes, with no links. Lane `i` gets id `LaneId(i)`.
    pub fn new(lanes: Vec<Lane>) -> Self {
        Self {
            lanes,
            junctions: Vec::new(),
            signals: Vec::new(),
            yields: Vec::new(),
        }
    }

    /// Adds a set of lanes for barriers to give way to, returning its id.
    pub fn add_yields(&mut self, lanes: &[LaneId]) -> YieldsId {
        self.yields.push(lanes.to_vec());
        YieldsId(self.yields.len() as u32 - 1)
    }

    /// The lanes `barrier` gives way to: none if it has no set.
    pub fn yields(&self, barrier: &Barrier) -> &[LaneId] {
        barrier.yields.map_or(&[], |id| &self.yields[id.0 as usize])
    }

    /// Adds a barrier to `lane`, keeping its barriers sorted.
    pub fn add_barrier(&mut self, lane: LaneId, barrier: Barrier) {
        let barriers = &mut self.lanes[lane.0 as usize].barriers;
        let at = barriers.partition_point(|b| b.s <= barrier.s);
        barriers.insert(at, barrier);
    }

    /// Marks a lane to be kept clear, or not: a vehicle that can no longer stop
    /// before it is committed to its junction.
    pub fn set_keep_clear(&mut self, lane: LaneId, keep_clear: bool) {
        self.lanes[lane.0 as usize].keep_clear = keep_clear;
    }

    /// Adds a signal plan, returning the id of its first movement: the rest
    /// follow in order.
    ///
    /// Panics if it has more than 64 movements.
    pub fn add_signal(&mut self, plan: SignalPlan) -> MovementId {
        assert!(
            plan.movements.len() <= 64,
            "a signal plan has at most 64 movements"
        );
        let first = self.movement_count() as u32;
        self.signals.push((plan, first));
        MovementId(first)
    }

    /// The signal plans, each with the id of its first movement.
    pub fn signals(&self) -> impl Iterator<Item = (&SignalPlan, MovementId)> {
        self.signals.iter().map(|(plan, first)| (plan, MovementId(*first)))
    }

    /// How many signal movements there are, over every plan.
    pub fn movement_count(&self) -> usize {
        self.signals.iter().map(|(plan, _)| plan.movements.len()).sum()
    }

    /// The side of `lane` toward which `toward`, a lane starting level with it, lies
    /// or departs. A neighbor's side is known; otherwise it is the side of the
    /// first of `toward`'s vertices off `lane`'s centerline by more than rounding
    /// to the grid, before `lane` ends, or failing that, the side the last was.
    /// Lanes that never part are one lane in all but name, and left is as good a
    /// side as any for them.
    fn side_toward(&self, lane: LaneId, toward: LaneId) -> Side {
        let beside = self.lane(lane);
        let sides = [Side::Left, Side::Right];
        let neighbor = |side: Side| beside.adj(side).is_some_and(|n| n.lane == toward);
        if let Some(side) = sides.into_iter().find(|&side| neighbor(side)) {
            return side;
        }
        let (from, to) = (beside.geometry(), self.lane(toward).geometry());
        let side_of = |t: f64| if t < 0.0 { Side::Right } else { Side::Left };
        let mut at = from.lane_pos(0.0, 0.0);
        let mut last = 0.0;
        for point in to.centerline() {
            at = from.world_to_lane_near(point, at);
            if at.s > from.length() {
                break;
            }
            last = at.t;
            if at.t.abs() >= SAME_POINT {
                return side_of(at.t);
            }
        }
        side_of(last)
    }

    /// Connects the end of `from` to the start of `to`. A lane may connect to itself.
    ///
    /// A lane's successors are kept in order from left to right, by the way they
    /// part, so a vehicle taking one knows at once which side of the others it is
    /// heading. Its predecessors keep the order they were connected in.
    ///
    /// The end of `from` and the start of `to` should coincide; [`SimNetwork::validate`]
    /// checks this.
    /// Panics if `from` already has [`MAX_SUCCESSORS`] successors or `to` already
    /// has [`MAX_PREDECESSORS`] predecessors.
    pub fn connect(&mut self, from: LaneId, to: LaneId) {
        assert!((to.0 as usize) < self.lanes.len(), "no lane {to:?}");
        // Before the first it lies to the left of.
        let successors = self.lanes[from.0 as usize].successors();
        let index = successors
            .iter()
            .position(|&other| self.side_toward(other, to) == Side::Left)
            .unwrap_or(successors.len());
        let successors = &mut self.lanes[from.0 as usize].successors;
        assert!(
            !successors.is_full(),
            "{from:?} already has {MAX_SUCCESSORS} successors"
        );
        successors.insert(index, to);
        let predecessors = &mut self.lanes[to.0 as usize].predecessors;
        assert!(
            !predecessors.is_full(),
            "{to:?} already has {MAX_PREDECESSORS} predecessors"
        );
        predecessors.push(from);
    }

    /// Makes `left` and `right` neighbors, `left` on the left of `right`, with a
    /// lane-change permission for each direction.
    ///
    /// Replaces any neighbors they already had on those sides.
    pub fn set_adjacent(
        &mut self,
        left: LaneId,
        right: LaneId,
        left_to_right: bool,
        right_to_left: bool,
    ) {
        assert!(left != right, "a lane can't neighbor itself");
        let to_right = Neighbor { lane: right, can_change: left_to_right };
        let to_left = Neighbor { lane: left, can_change: right_to_left };
        self.lanes[left.0 as usize].adj_right = Some(to_right);
        self.lanes[right.0 as usize].adj_left = Some(to_left);
    }

    /// Checks the assumptions the simulation makes about a network, returning the
    /// first one broken:
    ///
    /// - Neighbors are mutual: a lane's neighbor on one side has it as its
    ///   neighbor on the other.
    /// - Neighboring lanes start and end together, within a centimeter.
    /// - A lane's successors start where it ends, within a centimeter.
    /// - Lane sections line up: the neighbor of a lane's successor, if it has
    ///   one, is a successor of the lane's neighbor. A successor may have no
    ///   neighbor where the lane beside ends, as at a lane drop.
    ///
    /// The simulation relies on these to map a vehicle's path to the lanes beside
    /// it. A network that breaks them gives wrong leaders rather than an error, so
    /// check networks once they are built. It scans every lane, so it is not for
    /// hot paths.
    pub fn validate(&self) -> Result<(), NetworkError> {
        let point =
            |lane: &Lane, s: f64| lane.geometry().lane_to_world(lane.geometry().lane_pos(s, 0.0));
        for (id, lane) in self.lanes() {
            for side in [Side::Left, Side::Right] {
                let Some(neighbor) = lane.adj(side) else {
                    continue;
                };
                let beside = self.lane(neighbor.lane);
                if beside.adj(side.opposite()).map(|n| n.lane) != Some(id) {
                    return Err(NetworkError::OneWayNeighbor {
                        lane: id,
                        side,
                        neighbor: neighbor.lane,
                    });
                }
                // Where the neighbor's ends fall along this lane.
                for (end, s) in [(End::Start, 0.0), (End::End, lane.geometry().length())] {
                    let at = match end {
                        End::Start => point(beside, 0.0),
                        End::End => point(beside, beside.geometry().length()),
                    };
                    let along =
                        lane.geometry().world_to_lane_near(at, lane.geometry().lane_pos(s, 0.0)).s;
                    if (along - s).abs() >= SAME_POINT {
                        let offset = along - s;
                        return Err(NetworkError::NotAligned {
                            lane: id,
                            neighbor: neighbor.lane,
                            end,
                            offset,
                        });
                    }
                }
                for &successor in lane.successors() {
                    let Some(next) = self.lane(successor).adj(side) else {
                        continue;
                    };
                    if !beside.successors().contains(&next.lane) {
                        return Err(NetworkError::SectionMismatch {
                            lane: id,
                            side,
                            neighbor: neighbor.lane,
                            successor,
                            beside: next.lane,
                        });
                    }
                }
            }
            for &successor in lane.successors() {
                let gap = point(lane, lane.geometry().length())
                    .distance(point(self.lane(successor), 0.0));
                if gap >= SAME_POINT {
                    return Err(NetworkError::Disconnected { lane: id, successor, gap });
                }
            }
        }
        Ok(())
    }

    /// Groups `lanes` into a junction, which keeps vehicles on them apart, and
    /// finds where they conflict. Connect the lanes first: whether two lanes
    /// merge, diverge or follow one another comes from their links.
    ///
    /// Two lanes conflict if they merge, leading into the same lane, or if their
    /// outlines overlap and they are not linked: not diverging from the same
    /// lane, one following the other, or neighbors. Vehicles on diverging lanes
    /// are kept apart by joining every branch, and on linked or neighboring lanes
    /// by their leader searches.
    ///
    /// Panics if a lane is already in a junction, or if `lanes` has more than
    /// `u16::MAX` lanes. It compares every pair of lanes, segment by segment, so
    /// it is for building networks, not for hot paths.
    pub fn add_junction(&mut self, lanes: &[LaneId]) -> JunctionId {
        let id = JunctionId(self.junctions.len() as u32);
        let n = lanes.len();
        assert!(n <= u16::MAX as usize, "too many lanes in one junction");
        for (i, &lane) in lanes.iter().enumerate() {
            let lane = &mut self.lanes[lane.0 as usize];
            assert!(
                lane.junction.is_none(),
                "a lane belongs to at most one junction"
            );
            lane.junction = Some((id, i as u16));
        }
        let mut conflicts = vec![None; n * n];
        for i in 0..n {
            for j in i + 1..n {
                let Some((kind, [mine, theirs])) = self.conflict(lanes[i], lanes[j]) else {
                    continue;
                };
                conflicts[i * n + j] =
                    Some(Conflict { kind, mine: mine.clone(), theirs: theirs.clone() });
                conflicts[j * n + i] = Some(Conflict { kind, mine: theirs, theirs: mine });
            }
        }
        let mut junction = Junction { lanes: lanes.to_vec(), conflicts, starts: Vec::new() };
        junction.derive_starts();
        self.junctions.push(junction);
        id
    }

    /// Works out again what each junction derives from its conflicts, which a
    /// saved network leaves out.
    pub(crate) fn derive_conflict_starts(&mut self) {
        for junction in &mut self.junctions {
            junction.derive_starts();
        }
    }

    /// Where a lane's first conflict with the rest of its junction starts along
    /// it, if it is in one and has one.
    pub fn conflict_start(&self, lane: LaneId) -> Option<f64> {
        let (id, i) = self.lane(lane).junction?;
        self.junction(id).starts[i as usize]
    }

    /// How two lanes conflict, if they do, and the stretch of each in which they
    /// overlap. A merge where the outlines don't overlap gets the lanes' ends.
    fn conflict(&self, a: LaneId, b: LaneId) -> Option<(ConflictKind, [Range<f64>; 2])> {
        let (lane_a, lane_b) = (self.lane(a), self.lane(b));
        let shares = |x: &[LaneId], y: &[LaneId]| x.iter().any(|l| y.contains(l));
        let overlap = lane_a.geometry().overlap(lane_b.geometry());
        if shares(lane_a.successors(), lane_b.successors()) {
            let ends = [lane_a, lane_b].map(|l| l.geometry().length()..l.geometry().length());
            return Some((ConflictKind::Merge, overlap.unwrap_or(ends)));
        }
        // A connector may be split into several lanes, one after another, its
        // first perhaps a lead-in before the junction, so branches of a fork
        // are traced back to the first lane of each.
        let (first_a, first_b) = (self.first_of_connector(a), self.first_of_connector(b));
        let linked = first_a == first_b
            || shares(
                self.lane(first_a).predecessors(),
                self.lane(first_b).predecessors(),
            )
            || self.leads_to(a, b)
            || self.leads_to(b, a)
            || lane_a.successors().contains(&b)
            || lane_b.successors().contains(&a)
            || [Side::Left, Side::Right]
                .iter()
                .any(|&side| lane_a.adj(side).is_some_and(|n| n.lane == b));
        if linked {
            return None;
        }
        overlap.map(|ranges| (ConflictKind::Cross, ranges))
    }

    /// The first lane of the run of lanes in `lane`'s junction that it ends,
    /// each the only lane leading into the next: where a connector split into
    /// pieces starts.
    pub(crate) fn first_in_junction(&self, mut lane: LaneId) -> LaneId {
        let junction = self.lane(lane).junction;
        while let [p] = self.lane(lane).predecessors()[..]
            && junction.is_some()
            && self.lane(p).junction.map(|j| j.0) == junction.map(|j| j.0)
            && p != lane
        {
            lane = p;
        }
        lane
    }

    /// Whether a vehicle following `now`, its body on `lane` though it isn't
    /// headed there, has left it behind: `lane` comes before `now` in the same
    /// connector, or `now` is out of `lane`'s junction, so its body lingers as
    /// it leaves. Otherwise its body only overlaps `lane` beside its own, whose
    /// conflicts are its own lanes'.
    pub(crate) fn left_behind(&self, now: LaneId, lane: LaneId) -> bool {
        let junction = self.lane(lane).junction.map(|j| j.0);
        self.lane(now).junction.map(|j| j.0) != junction || self.leads_to(now, lane)
    }

    /// The first lane of the connector `lane` is part of: as
    /// [`first_in_junction`](Self::first_in_junction), and then back through a
    /// lead-in before the junction, a lane outside any junction whose only
    /// successor is that one, if there is one.
    pub(crate) fn first_of_connector(&self, lane: LaneId) -> LaneId {
        let first = self.first_in_junction(lane);
        match self.lane(first).predecessors()[..] {
            [p] if self.lane(p).junction.is_none() && self.lane(p).successors() == [first] => p,
            _ => first,
        }
    }

    /// Whether `from` leads to `to` through that run of lanes: `to` comes
    /// before it, in the same connector.
    pub(crate) fn leads_to(&self, from: LaneId, to: LaneId) -> bool {
        let first = self.first_in_junction(from);
        let mut lane = from;
        while lane != first {
            lane = self.lane(lane).predecessors()[0];
            if lane == to {
                return true;
            }
        }
        false
    }

    /// The junction with the given id. Panics if it doesn't exist.
    pub fn junction(&self, id: JunctionId) -> &Junction {
        &self.junctions[id.0 as usize]
    }

    /// All junctions with their ids.
    pub fn junctions(&self) -> impl Iterator<Item = (JunctionId, &Junction)> {
        self.junctions.iter().enumerate().map(|(i, j)| (JunctionId(i as u32), j))
    }

    /// Where two lanes in the same junction conflict, seen from `mine`, if they do.
    pub fn conflict_between(&self, mine: LaneId, theirs: LaneId) -> Option<&Conflict> {
        let (id, i) = self.lane(mine).junction?;
        let (other, j) = self.lane(theirs).junction?;
        if other != id {
            return None;
        }
        let junction = self.junction(id);
        junction.conflicts[i as usize * junction.lanes.len() + j as usize].as_ref()
    }

    /// The lane with the given id. Panics if it doesn't exist.
    pub fn lane(&self, id: LaneId) -> &Lane {
        &self.lanes[id.0 as usize]
    }

    /// The point `s` along `path`, `offset(s)` across it, with each corner of the
    /// path's polyline rounded: within `d` of a vertex, half the shorter of the
    /// segments either side but at most [`FILLET`], the course is the quadratic Bézier from `d` before
    /// the vertex to `d` after, its control point the vertex, all at their own
    /// offsets. Its direction doesn't jump at vertices, it needs no square roots,
    /// and, symmetric about the vertex, it is parameterised all but evenly by
    /// `s`. Where the path runs out, the corner isn't rounded.
    pub(crate) fn smooth_point_along(
        &self,
        path: &[LaneId],
        mut s: f64,
        offset: impl Fn(f64) -> f64,
    ) -> DVec2 {
        // The lane `s` is on, and the path's length before it.
        let mut k = 0;
        let mut base = 0.0;
        while k + 1 < path.len() {
            let length = self.lane(path[k]).geometry().length();
            if s < length {
                break;
            }
            s -= length;
            base += length;
            k += 1;
        }
        let g = self.lane(path[k]).geometry();
        let i = g.seg_index(s);
        let (start, end) = g.segment_span(i);
        let point = |lane: LaneId, seg: usize, at: f64, base: f64| {
            self.lane(lane).geometry().segment_point(seg, at, offset(base + at))
        };
        // The segment before its start vertex, on this lane or the one before,
        // and the one after its end vertex, on this lane or the one after: each a
        // lane, its segment, that segment's span, and the path's length before
        // that lane.
        let before = if i > 0 {
            Some((path[k], i - 1, g.segment_span(i - 1), base))
        } else if k > 0 {
            let p = self.lane(path[k - 1]).geometry();
            let last = p.segment_count() - 1;
            Some((path[k - 1], last, p.segment_span(last), base - p.length()))
        } else {
            None
        };
        let after = if i + 1 < g.segment_count() {
            Some((path[k], i + 1, g.segment_span(i + 1), base))
        } else if k + 1 < path.len() {
            let n = self.lane(path[k + 1]).geometry();
            Some((path[k + 1], 0, n.segment_span(0), base + g.length()))
        } else {
            None
        };
        // Rounding the corner at the vertex between `(a, at the end of its
        // segment)` and `(b, at the start of its)`, if `s` is within it.
        let fillet = |prev: (LaneId, usize, (f64, f64), f64),
                      next: (LaneId, usize, (f64, f64), f64),
                      from_vertex: f64| {
            let (pl, pi, (ps, pe), pb) = prev;
            let (nl, ni, (ns, ne), nb) = next;
            let d = (0.5 * (pe - ps).min(ne - ns)).min(FILLET);
            (from_vertex.abs() < d).then(|| {
                let p0 = point(pl, pi, pe - d, pb);
                let p1 = point(nl, ni, ns, nb);
                let p2 = point(nl, ni, ns + d, nb);
                let u = 0.5 * (from_vertex + d) / d;
                let w = 1.0 - u;
                w * w * p0 + 2.0 * u * w * p1 + u * u * p2
            })
        };
        let here = (path[k], i, (start, end), base);
        let rounded = match (s - start < end - s, before, after) {
            (true, Some(prev), _) => fillet(prev, here, s - start),
            (false, _, Some(next)) => fillet(here, next, s - end),
            _ => None,
        };
        rounded.unwrap_or_else(|| point(path[k], i, s, base))
    }

    /// The point `s` along `path` and `t` across it, as [`SimNetwork::point_along`]
    /// gives it, with the unit direction there of a curve whose offset changes by
    /// `slope` (see [`LaneGeometry::direction_at`]).
    pub(crate) fn pose_along(
        &self,
        path: impl IntoIterator<Item = LaneId>,
        s: f64,
        t: f64,
        slope: f64,
    ) -> (DVec2, DVec2) {
        let (lane, pos) = self.lane_pos_along(path, s, t);
        let geometry = self.lane(lane).geometry();
        (
            geometry.lane_to_world(pos),
            geometry.direction_at(pos, slope),
        )
    }

    /// The lane `s` along `path` falls on, and the position there, `t` across
    /// it. Past the end of the last lane, it is on that lane, beyond its end.
    fn lane_pos_along(
        &self,
        path: impl IntoIterator<Item = LaneId>,
        mut s: f64,
        t: f64,
    ) -> (LaneId, LanePos) {
        let mut path = path.into_iter().peekable();
        while let Some(id) = path.next() {
            let geometry = self.lane(id).geometry();
            if s < geometry.length() || path.peek().is_none() {
                return (id, geometry.lane_pos(s, t));
            }
            s -= geometry.length();
        }
        unreachable!("a vehicle's path always holds its current lane")
    }

    /// All lanes with their ids.
    pub fn lanes(&self) -> impl Iterator<Item = (LaneId, &Lane)> {
        self.lanes.iter().enumerate().map(|(i, lane)| (LaneId(i as u32), lane))
    }
}

#[cfg(test)]
mod test {
    use glam::DVec2;

    use super::*;

    fn network(lanes: usize) -> SimNetwork {
        let lane = || Lane::new(&[DVec2::ZERO, DVec2::X * 10.0], 3.5, 14.0).unwrap();
        SimNetwork::new((0..lanes).map(|_| lane()).collect())
    }

    #[test]
    fn adjacency_is_symmetric_and_permissions_are_ordered() {
        let mut net = network(2);
        let (a, b) = (LaneId(0), LaneId(1));
        net.set_adjacent(a, b, true, false);
        assert_eq!(
            net.lane(a).adj_right(),
            Some(Neighbor { lane: b, can_change: true })
        );
        assert_eq!(
            net.lane(b).adj_left(),
            Some(Neighbor { lane: a, can_change: false })
        );
        assert_eq!(net.lane(a).adj_left(), None);
        assert_eq!(net.lane(b).adj_right(), None);
        assert_eq!(net.lane(a).adj(Side::Right), net.lane(a).adj_right());
    }

    #[test]
    fn successors_are_ordered_left_to_right_and_predecessors_as_connected() {
        // Lane 0 forks three ways at x = 10: lane 1 bears left, lane 2 runs
        // straight on and lane 3 bears right. Connected out of order.
        let lane = |points: &[DVec2]| Lane::new(points, 3.5, 14.0).unwrap();
        let from = DVec2::new(10.0, 0.0);
        let mut net = SimNetwork::new(vec![
            lane(&[DVec2::ZERO, from]),
            lane(&[from, DVec2::new(30.0, 6.0)]),
            lane(&[from, DVec2::new(30.0, 0.0)]),
            lane(&[from, DVec2::new(30.0, -6.0)]),
        ]);
        let [a, left, straight, right] = [0, 1, 2, 3].map(LaneId);
        net.connect(a, straight);
        net.connect(a, right);
        net.connect(a, left);
        assert_eq!(net.lane(a).successors(), [left, straight, right]);
        net.connect(left, left);
        net.connect(right, left);
        assert_eq!(net.lane(left).predecessors(), [a, left, right]);
        assert!(net.lane(straight).successors().is_empty());
    }

    #[test]
    #[should_panic(expected = "successors")]
    fn too_many_successors_panics() {
        let mut net = network(1);
        for _ in 0..=MAX_SUCCESSORS {
            net.connect(LaneId(0), LaneId(0));
        }
    }

    /// A straight lane along y, from x0 to x1.
    fn straight(x0: f64, x1: f64, y: f64) -> Lane {
        Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], 3.5, 14.0).unwrap()
    }

    /// Two lanes side by side in two sections: lanes 0 and 1 from x = 0 to 50, 2
    /// and 3 from 50 to 100, with 0 and 2 on the left.
    fn two_sections() -> SimNetwork {
        let mut net = SimNetwork::new(vec![
            straight(0.0, 50.0, 3.5),
            straight(0.0, 50.0, 0.0),
            straight(50.0, 100.0, 3.5),
            straight(50.0, 100.0, 0.0),
        ]);
        net.set_adjacent(LaneId(0), LaneId(1), true, true);
        net.set_adjacent(LaneId(2), LaneId(3), true, true);
        net
    }

    #[test]
    fn a_well_formed_network_is_valid() {
        let mut net = two_sections();
        net.connect(LaneId(0), LaneId(2));
        net.connect(LaneId(1), LaneId(3));
        assert_eq!(net.validate(), Ok(()));
    }

    #[test]
    fn a_lane_that_ends_is_valid() {
        // Lane 1 ends at x = 50, so lane 0's successor, 2, has no neighbor: a lane
        // drop, which vehicles leave by changing lanes or through a merge.
        let mut net = SimNetwork::new(vec![
            straight(0.0, 50.0, 3.5),
            straight(0.0, 50.0, 0.0),
            straight(50.0, 100.0, 3.5),
        ]);
        net.set_adjacent(LaneId(0), LaneId(1), true, true);
        net.connect(LaneId(0), LaneId(2));
        assert_eq!(net.validate(), Ok(()));
    }

    #[test]
    fn replaced_neighbors_are_caught() {
        // Making 2 lane 0's right neighbor leaves 1 still pointing at 0.
        let mut net = network(3);
        net.set_adjacent(LaneId(0), LaneId(1), true, true);
        net.set_adjacent(LaneId(0), LaneId(2), true, true);
        assert!(matches!(
            net.validate(),
            Err(NetworkError::OneWayNeighbor { lane: LaneId(1), side: Side::Left, .. })
        ));
    }

    #[test]
    fn sections_that_dont_line_up_are_caught() {
        // Both lanes of the first section lead to lane 2, but lane 2's neighbor,
        // 3, doesn't follow lane 0's neighbor.
        let mut net = two_sections();
        net.connect(LaneId(0), LaneId(2));
        net.connect(LaneId(1), LaneId(2));
        assert!(matches!(
            net.validate(),
            Err(NetworkError::SectionMismatch {
                lane: LaneId(0),
                successor: LaneId(2),
                beside: LaneId(3),
                ..
            })
        ));
    }

    #[test]
    fn a_gap_between_lanes_is_caught() {
        let mut net = SimNetwork::new(vec![straight(0.0, 50.0, 0.0), straight(51.0, 100.0, 0.0)]);
        net.connect(LaneId(0), LaneId(1));
        let Err(NetworkError::Disconnected { gap, .. }) = net.validate() else {
            panic!("expected a gap");
        };
        assert!((gap - 1.0).abs() < 1e-9);
    }

    #[test]
    fn neighbors_that_dont_start_together_are_caught() {
        let mut net = SimNetwork::new(vec![straight(0.0, 50.0, 3.5), straight(5.0, 50.0, 0.0)]);
        net.set_adjacent(LaneId(0), LaneId(1), true, true);
        let Err(NetworkError::NotAligned { end: End::Start, offset, .. }) = net.validate() else {
            panic!("expected misaligned starts");
        };
        assert!(offset.abs() > 1.0);
    }

    #[test]
    fn opposite_sides() {
        assert_eq!(Side::Left.opposite(), Side::Right);
        assert_eq!(Side::Right.opposite(), Side::Left);
    }
}
