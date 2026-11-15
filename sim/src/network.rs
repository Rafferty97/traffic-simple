use glam::DVec2;

use crate::lane::{Lane, MAX_PREDECESSORS, MAX_SUCCESSORS};

/// Identifies a lane: its index in [`SimNetwork`]'s lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LaneId(pub u32);

/// A side of a lane, relative to its direction of travel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
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
pub struct Neighbor {
    pub lane: LaneId,
    /// Whether vehicles may change from the lane this is stored on into `lane`.
    pub can_change: bool,
}

/// The compiled network the simulation runs on.
///
/// Each lane holds its own successors and neighbors. Adjacent lanes start and
/// end together, like the lanes of a SUMO edge or an OpenDRIVE lane section, so
/// the neighbor of a lane's successor is the successor of its neighbor. This
/// isn't checked.
#[derive(Clone, Debug)]
pub struct SimNetwork {
    lanes: Vec<Lane>,
}

impl SimNetwork {
    /// Builds a network from its lanes, with no links. Lane `i` gets id `LaneId(i)`.
    pub fn new(lanes: Vec<Lane>) -> Self {
        Self { lanes }
    }

    /// Connects the end of `from` to the start of `to`. A lane may connect to itself.
    ///
    /// The end of `from` and the start of `to` should coincide; this isn't checked.
    /// Panics if `from` already has [`MAX_SUCCESSORS`] successors or `to` already
    /// has [`MAX_PREDECESSORS`] predecessors.
    pub fn connect(&mut self, from: LaneId, to: LaneId) {
        assert!((to.0 as usize) < self.lanes.len(), "no lane {to:?}");
        let successors = &mut self.lanes[from.0 as usize].successors;
        assert!(
            !successors.is_full(),
            "{from:?} already has {MAX_SUCCESSORS} successors"
        );
        successors.push(to);
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

    /// The lane with the given id. Panics if it doesn't exist.
    pub fn lane(&self, id: LaneId) -> &Lane {
        &self.lanes[id.0 as usize]
    }

    /// The point `s` meters along a path from the start of its first lane, `t`
    /// meters left of the centerline. Past the end of the path, the last lane's
    /// final segment is extended.
    pub(crate) fn point_along(&self, path: &[LaneId], mut s: f64, t: f64) -> DVec2 {
        for (i, &id) in path.iter().enumerate() {
            let lane = self.lane(id);
            if s < lane.length() || i == path.len() - 1 {
                return lane.lane_to_world(lane.lane_pos(s, t));
            }
            s -= lane.length();
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
    fn successors_keep_the_order_they_were_connected_in() {
        let mut net = network(3);
        let (a, b, c) = (LaneId(0), LaneId(1), LaneId(2));
        net.connect(a, c);
        net.connect(b, b);
        net.connect(a, b);
        assert_eq!(net.lane(a).successors(), [c, b]);
        assert_eq!(net.lane(b).successors(), [b]);
        assert!(net.lane(c).successors().is_empty());
        assert!(net.lane(a).predecessors().is_empty());
        assert_eq!(net.lane(b).predecessors(), [b, a]);
        assert_eq!(net.lane(c).predecessors(), [a]);
    }

    #[test]
    #[should_panic(expected = "successors")]
    fn too_many_successors_panics() {
        let mut net = network(1);
        for _ in 0..=MAX_SUCCESSORS {
            net.connect(LaneId(0), LaneId(0));
        }
    }

    #[test]
    fn opposite_sides() {
        assert_eq!(Side::Left.opposite(), Side::Right);
        assert_eq!(Side::Right.opposite(), Side::Left);
    }
}
