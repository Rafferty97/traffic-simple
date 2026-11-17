use arrayvec::ArrayVec;
use glam::DVec2;

use crate::lane::{LaneError, LaneGeometry};

/// The most successors a lane can have.
pub const MAX_SUCCESSORS: usize = 4;

/// The most predecessors a lane can have. More than successors, since a lane
/// leaving a junction collects connectors from several approaches.
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

/// How far apart two points may be and still count as the same, in meters: a
/// lane's end and its successor's start, or the ends of neighboring lanes.
const COINCIDENT: f64 = 0.01;

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
}

impl Lane {
    /// Builds a lane from its centerline polyline, a constant width in meters,
    /// centered on it, and a speed limit in meters per second.
    ///
    /// Consecutive duplicate points are removed before the lane is built.
    pub fn new(points: &[DVec2], width: f64, speed_limit: f64) -> Result<Self, LaneError> {
        let geometry = LaneGeometry::new(points, &vec![[0.5 * width; 2]; points.len()])?;
        Ok(Self {
            geometry,
            speed_limit,
            successors: ArrayVec::new(),
            predecessors: ArrayVec::new(),
            adj_left: None,
            adj_right: None,
        })
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

    /// The lane beside this one on the given side, if any.
    pub fn adj(&self, side: Side) -> Option<Neighbor> {
        match side {
            Side::Left => self.adj_left,
            Side::Right => self.adj_right,
        }
    }
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
}

impl SimNetwork {
    /// Builds a network from its lanes, with no links. Lane `i` gets id `LaneId(i)`.
    pub fn new(lanes: Vec<Lane>) -> Self {
        Self { lanes }
    }

    /// The side of `lane` toward which `toward`, a lane starting level with it, lies
    /// or departs. A neighbor's side is known; otherwise `toward`'s centerline is
    /// walked, a meter at a time, until it is clearly beside `lane`, more than a
    /// quarter of its half-width from its centerline, and failing that, the side
    /// it last was before `lane` ends. Lanes that never part are one lane in all
    /// but name, and left is as good a side as any for them.
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
        for i in 0..=to.length().ceil() as usize {
            let point = to.lane_to_world(to.lane_pos((i as f64).min(to.length()), 0.0));
            at = from.world_to_lane_near(point, at);
            if at.s > from.length() {
                break;
            }
            last = at.t;
            let side = side_of(at.t);
            if at.t.abs() > 0.25 * from.half_width(at, side) {
                return side;
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
                    if (along - s).abs() > COINCIDENT {
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
                if gap > COINCIDENT {
                    return Err(NetworkError::Disconnected { lane: id, successor, gap });
                }
            }
        }
        Ok(())
    }

    /// The lane with the given id. Panics if it doesn't exist.
    pub fn lane(&self, id: LaneId) -> &Lane {
        &self.lanes[id.0 as usize]
    }

    /// The point `s` meters along a path from the start of its first lane, `t`
    /// meters left of the centerline. Past the end of the path, the last lane's
    /// final segment is extended.
    pub(crate) fn point_along(
        &self,
        path: impl IntoIterator<Item = LaneId>,
        mut s: f64,
        t: f64,
    ) -> DVec2 {
        let mut path = path.into_iter().peekable();
        while let Some(id) = path.next() {
            let lane = self.lane(id);
            if s < lane.geometry().length() || path.peek().is_none() {
                return lane.geometry().lane_to_world(lane.geometry().lane_pos(s, t));
            }
            s -= lane.geometry().length();
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
