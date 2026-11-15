//! Per-lane occupant lists, sorted by `s`, and leader search over them.
//!
//! A vehicle joins a lane when it advances onto it (its rear axle passes the end
//! of the previous lane) or changes lanes into it. It leaves once both rear
//! corners are outside the lane: past its end, or beside it. In between, its
//! record's `s` is the smaller of its rear corners' `s`. On a loop shorter than a
//! vehicle's rear overhang plus a lane, a vehicle can be in the same lane's list
//! twice, briefly, at two different `s`.
//!
//! A vehicle indicating a lane change also has a ghost record in the target
//! lane's list. Ghosts are kept up to date like any record but are only removed
//! explicitly, when the intent is cancelled or carried out.

use glam::DVec2;
use slotmap::SlotMap;

use crate::debug::PassCheck;
use crate::lane::LanePos;
use crate::models::lateral::LaneChange;
use crate::network::{LaneId, Side, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// How far ahead of its front bumper a vehicle looks for a leader, in meters.
const LEADER_HORIZON: f64 = 200.0;
/// The least room, in meters, a vehicle needs beside another to pass it.
const PASSING_CLEARANCE: f64 = 0.5;

/// A vehicle's record in one lane's occupant list.
#[derive(Clone, Copy, Debug)]
pub struct Occupant {
    /// The smaller of the vehicle's rear corners' `s` on this lane.
    pub s: f64,
    pub id: VehicleId,
    /// Lane coordinates of that rear corner, a hint for the next projection.
    hint: LanePos,
    /// Whether this is the ghost of a vehicle indicating a change into this lane.
    pub ghost: bool,
    /// Whether the vehicle is in this lane or moving out of it sideways.
    pub lateral: Lateral,
}

/// How a vehicle relates to a lane it has a record on, as followers there see it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Lateral {
    /// In the lane: following it, settled or still converging onto it after a
    /// lane change, lingering past the end of it after advancing, or the ghost of
    /// a vehicle indicating into it. A vehicle moving in is treated as already
    /// there.
    InLane,
    /// Moving out of the lane sideways after a lane change, toward this side.
    Diverging(Side),
}

/// The side of a vehicle ahead a follower might pass it on, or `None` if it may
/// never pass it, from how each relates to the lane searched.
///
/// The passing check is a snapshot taken as the follower reaches the other
/// vehicle's rear, so it is only sound when the follower isn't moving toward it
/// sideways: then that moment is the tightest, and the gap only grows. So:
///
/// - A vehicle in the lane can only be passed by a follower leaving the lane,
///   on the side it is leaving toward. Two in the same lane, settled or
///   converging, never pass.
/// - A vehicle diverging can be passed on the side it is leaving, unless the
///   follower is itself moving the same way.
///
/// `leaving` is the side the follower is leaving the searched lane toward, and
/// `moving` the side it is moving across toward, if any.
fn passing_side(leader: Lateral, leaving: Option<Side>, moving: Option<Side>) -> Option<Side> {
    match (leader, leaving) {
        (Lateral::InLane, leaving) => leaving,
        (Lateral::Diverging(side), _) if moving == Some(side) => None,
        (Lateral::Diverging(side), _) => Some(side.opposite()),
    }
}

/// How much room a follower would have passing a vehicle ahead on `side` of it,
/// if it drove straight ahead from `pos`, its rear axle, heading along `tangent`:
/// the lateral distance from its body's near edge to the nearer of the other
/// vehicle's rear corners, and that corner. Negative if they overlap.
///
/// Only the rear corners are measured, which assumes the other vehicle is
/// straight or turned away from the follower. That holds for the pairs
/// [`passing_side`] lets through: one diverging is turned away, and one in the
/// lane is passed only by a follower moving away from it.
fn clearance(
    side: Side,
    pos: DVec2,
    tangent: DVec2,
    half_width: f64,
    corners: [DVec2; 2],
) -> (f64, DVec2) {
    // Positive to the follower's left.
    let lateral = corners.map(|c| (c - pos).dot(tangent.perp()));
    match side {
        // Passing on its left, so it is to the follower's right.
        Side::Left => {
            let i = if lateral[0] >= lateral[1] { 0 } else { 1 };
            (-half_width - lateral[i], corners[i])
        }
        Side::Right => {
            let i = if lateral[0] <= lateral[1] { 0 } else { 1 };
            (lateral[i] - half_width, corners[i])
        }
    }
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`.
    pub fn new(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, false).0
    }

    /// As [`Occupant::new`], as the ghost of a vehicle indicating a change into `lane`.
    pub fn ghost(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, true).0
    }

    /// As [`Occupant::new`] or [`Occupant::ghost`], also saying where each rear
    /// corner is relative to the lane.
    fn project(
        network: &SimNetwork,
        lane_id: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
        ghost: bool,
    ) -> (Self, [Corner; 2]) {
        let lane = network.lane(lane_id);
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = lane.world_to_lane_near(rear_left, hint);
        let right = lane.world_to_lane_near(rear_right, hint);
        let corner = |p: LanePos| Corner {
            past_end: p.s >= lane.length(),
            beside: p.t.abs() > 0.5 * lane.width(),
        };
        let hint = if left.s <= right.s { left } else { right };
        // Diverging if this is an old lane it is moving out of sideways: the lane
        // beside it, away from the vehicle's old side, is the lane it follows or,
        // just after a section boundary, the lane before that.
        let following = vehicle.path[0];
        let toward = vehicle.old_side.map(Side::opposite);
        let moving_out = |side: Side| {
            lane.adj(side).is_some_and(|n| {
                n.lane == following || network.lane(n.lane).successors().contains(&following)
            })
        };
        let lateral = match toward {
            Some(side) if !ghost && lane_id != following && moving_out(side) => {
                Lateral::Diverging(side)
            }
            _ => Lateral::InLane,
        };
        let occupant = Self { s: hint.s, id, hint, ghost, lateral };
        (occupant, [corner(left), corner(right)])
    }
}

/// Where a rear corner is relative to a lane.
#[derive(Clone, Copy, Debug)]
struct Corner {
    past_end: bool,
    /// Further from the centerline than half the lane's width.
    beside: bool,
}

/// Refreshes every occupant's `s`, drops those whose rear corners are both
/// outside their lane, and restores the order.
///
/// - Ghosts are refreshed but never dropped here.
/// - A record on the lane a vehicle is following is never dropped for being
///   beside it, only for being past its end.
/// - Vehicles that lose a record on a lane they aren't following, by its end or
///   its side, have their ids pushed to `departed`, so their old side can be
///   checked once every list is up to date.
pub fn update(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &mut [Vec<Occupant>],
    departed: &mut Vec<VehicleId>,
) {
    for (i, list) in occupants.iter_mut().enumerate() {
        let lane_id = LaneId(i as u32);
        list.retain_mut(|occupant| {
            let vehicle = &vehicles[occupant.id];
            let (id, hint, ghost) = (occupant.id, occupant.hint, occupant.ghost);
            let (updated, corners) = Occupant::project(network, lane_id, id, vehicle, hint, ghost);
            *occupant = updated;
            if occupant.ghost {
                return true;
            }
            let following = vehicle.path[0] == lane_id;
            let outside = |c: &Corner| c.past_end || (c.beside && !following);
            if !corners.iter().all(outside) {
                return true;
            }
            if !following {
                departed.push(occupant.id);
            }
            false
        });
        insertion_sort(list);
    }
}

/// Sorts by `s`. Linear on input that is already sorted, which it nearly always
/// is: vehicles rarely pass each other within a lane between substeps.
fn insertion_sort(list: &mut [Occupant]) {
    for i in 1..list.len() {
        let mut j = i;
        while j > 0 && list[j - 1].s > list[j].s {
            list.swap(j - 1, j);
            j -= 1;
        }
    }
}

/// Where a searching vehicle plans to be: its lane change's cubic, or else the
/// centerline, along the lanes it follows. Pure pursuit tracks that path
/// closely, so it says where the vehicle will be when it gets somewhere ahead.
#[derive(Clone, Copy, Debug)]
pub struct Course<'a> {
    /// The lanes it follows, the one it is on first.
    path: &'a [LaneId],
    /// The lane change it is following, in `s` along `path`.
    change: Option<LaneChange>,
    /// Its rear axle's `s` along `path`.
    s: f64,
}

impl<'a> Course<'a> {
    pub fn new(path: &'a [LaneId], change: Option<LaneChange>, s: f64) -> Self {
        Self { path, change, s }
    }

    /// A vehicle's own planned path.
    pub fn of(vehicle: &'a Vehicle) -> Self {
        Self::new(&vehicle.path, vehicle.lane_change, vehicle.lane_pos.s)
    }

    /// Its rear axle's position and heading, as a unit tangent, `ahead` meters
    /// further along the course. The heading follows the cubic's slope as well as
    /// the lanes' bends.
    fn pose(&self, network: &SimNetwork, ahead: f64) -> (DVec2, DVec2) {
        let offset_at = |at: f64| self.change.map_or(0.0, |c| c.offset_at(at));
        let at = self.s + ahead.max(0.0);
        let pos = network.point_along(self.path, at, offset_at(at));
        let next = network.point_along(self.path, at + 1.0, offset_at(at + 1.0));
        (pos, (next - pos).normalize())
    }

    /// The side it is moving across toward during its lane change, if any: away
    /// from where the cubic starts.
    fn moving(&self) -> Option<Side> {
        let change = self.change?;
        match change.offset {
            o if o > 0.0 => Some(Side::Right),
            o if o < 0.0 => Some(Side::Left),
            _ => None,
        }
    }

    /// The side it is leaving `lane`, the `k`th lane searched, toward, if that
    /// lane is beside the course's `k`th lane rather than on it.
    fn leaving(&self, network: &SimNetwork, k: usize, lane: LaneId) -> Option<Side> {
        let level = *self.path.get(k)?;
        let searched = network.lane(lane);
        [Side::Left, Side::Right]
            .into_iter()
            .find(|&side| level != lane && searched.adj(side).is_some_and(|n| n.lane == level))
    }
}

/// How a leader search treats ghost records.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ghosts {
    /// Ghosts are passed over, but the nearest one before the leader is reported:
    /// the searching vehicle is that ghost's would-be follower.
    Report,
    /// Ghosts count as leaders, as when a vehicle weighs up changing lanes: a
    /// vehicle already indicating into the gap ahead goes first.
    Lead,
}

/// What a leader search found, each with the bumper-to-bumper gap to it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Search {
    pub leader: Option<(VehicleId, f64)>,
    /// The nearest ghost before the leader, with [`Ghosts::Report`].
    pub ghost: Option<(VehicleId, f64)>,
}

/// The nearest vehicle ahead along `lanes`, within the leader horizon, and the
/// bumper-to-bumper gap to it. `axle_s` is the vehicle's rear axle's `s` on the
/// first lane.
///
/// On the first lane, only occupants whose rear is ahead of the rear axle count.
/// On later lanes, every occupant counts. The vehicle's own records are skipped,
/// as are `skip`'s, for asking what if that vehicle weren't there.
///
/// Some records may be passed, by the rules in [`passing_side`]. One of those is
/// passed if, with this vehicle where its `course` puts it as its front reaches
/// that record's rear, driving straight ahead would leave at least
/// [`PASSING_CLEARANCE`] beside the other vehicle's rear corners. That reads the
/// other vehicle's corners from `vehicles`, the one place leader search does.
/// Each such check is reported to `on_pass`.
#[allow(clippy::too_many_arguments)]
pub fn find_leader(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    id: VehicleId,
    vehicle: &Vehicle,
    lanes: &[LaneId],
    axle_s: f64,
    course: Course<'_>,
    ghosts: Ghosts,
    skip: Option<VehicleId>,
    mut on_pass: impl FnMut(PassCheck),
) -> Search {
    let half_width = 0.5 * vehicle.spec.width;
    let front = axle_s + vehicle.spec.length - vehicle.spec.rear_axle;
    let mut found = Search::default();
    // Start of the current lane, measured along `lanes` from the start of the first.
    let mut offset = 0.0;
    let mut after = axle_s;
    for (k, &lane) in lanes.iter().enumerate() {
        if offset - front > LEADER_HORIZON {
            break;
        }
        let list = &occupants[lane.0 as usize];
        let start = list.partition_point(|o| o.s <= after);
        for o in &list[start..] {
            if o.id == id || Some(o.id) == skip {
                continue;
            }
            let gap = offset + o.s - front;
            if gap > LEADER_HORIZON {
                return found;
            }
            if o.ghost && ghosts == Ghosts::Report {
                found.ghost = found.ghost.or(Some((o.id, gap)));
                continue;
            }
            let leaving = course.leaving(network, k, lane);
            if let Some(side) = passing_side(o.lateral, leaving, course.moving()) {
                let (pos, tangent) = course.pose(network, gap);
                let [rear_left, rear_right, ..] = vehicles[o.id].corners();
                let (clearance, corner) =
                    clearance(side, pos, tangent, half_width, [rear_left, rear_right]);
                let passed = clearance >= PASSING_CLEARANCE;
                let leader = o.id;
                on_pass(PassCheck { leader, pos, tangent, side, corner, clearance, passed });
                if passed {
                    continue;
                }
            }
            found.leader = Some((o.id, gap));
            return found;
        }
        offset += network.lane(lane).length();
        after = f64::NEG_INFINITY;
    }
    found
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::lane::Lane;

    #[test]
    fn the_rules_only_let_through_followers_not_moving_toward_the_leader() {
        use Lateral::{Diverging, InLane};
        use Side::{Left, Right};
        // In the same lane, settled or converging: never.
        assert_eq!(passing_side(InLane, None, None), None);
        assert_eq!(passing_side(InLane, None, Some(Left)), None);
        // A follower leaving the lane passes on the side it leaves toward.
        assert_eq!(passing_side(InLane, Some(Left), Some(Left)), Some(Left));
        assert_eq!(passing_side(InLane, Some(Right), None), Some(Right));
        // A diverging vehicle is passed on the side it is leaving...
        assert_eq!(passing_side(Diverging(Left), None, None), Some(Right));
        assert_eq!(
            passing_side(Diverging(Left), Some(Right), Some(Right)),
            Some(Right)
        );
        // ...unless the follower is moving the same way.
        assert_eq!(passing_side(Diverging(Left), None, Some(Left)), None);
        assert_eq!(
            passing_side(Diverging(Right), Some(Right), Some(Right)),
            None
        );
    }

    #[test]
    fn clearance_is_measured_across_the_followers_heading() {
        let half_width = 0.95;
        // Heading along x, with the other car's rear corners 2.0 and 3.9 m to its
        // left: passing on that car's right leaves 1.05 m.
        let corners = [DVec2::new(5.0, 3.9), DVec2::new(5.0, 2.0)];
        let (room, corner) = clearance(Side::Right, DVec2::ZERO, DVec2::X, half_width, corners);
        assert!((room - 1.05).abs() < 1e-12);
        assert_eq!(corner, corners[1]);
        // Passing on its left instead would overlap it.
        assert!(clearance(Side::Left, DVec2::ZERO, DVec2::X, half_width, corners).0 < 0.0);
        // Heading 45 degrees to the left, toward it: much less room.
        let toward = DVec2::new(1.0, 1.0).normalize();
        let (room, _) = clearance(Side::Right, DVec2::ZERO, toward, half_width, corners);
        assert!(room < 0.0, "room {room}");
    }

    #[test]
    fn a_course_heads_along_its_cubic() {
        // Straight along x, changing from 3.5 m left of the centerline back onto
        // it over 15 m: halfway, it is 1.75 m left and heading right of x.
        let lane = Lane::new(&[DVec2::ZERO, DVec2::new(100.0, 0.0)], 3.5, 14.0).unwrap();
        let network = SimNetwork::new(vec![lane]);
        let path = [LaneId(0)];
        let change = LaneChange::new(0.0, 3.5, 0.0, 0.0);
        let course = Course::new(&path, Some(change), 0.0);
        assert_eq!(course.moving(), Some(Side::Right));
        let (pos, tangent) = course.pose(&network, 7.5);
        assert!((pos - DVec2::new(7.5, 1.75)).length() < 1e-9, "at {pos}");
        assert!(tangent.y < -0.2, "heading {tangent}");
        // Not changing, it heads straight along the centerline.
        let (pos, tangent) = Course::new(&path, None, 0.0).pose(&network, 7.5);
        assert!((pos - DVec2::new(7.5, 0.0)).length() < 1e-9);
        assert!((tangent - DVec2::X).length() < 1e-9);
    }

    #[test]
    fn insertion_sort_orders_by_s() {
        let mut ids = SlotMap::<VehicleId, ()>::with_key();
        let lane = Lane::new(&[glam::DVec2::ZERO, glam::DVec2::X * 10.0], 3.5, 14.0).unwrap();
        let mut list: Vec<_> = [3.0, 1.0, 2.0, 0.5]
            .into_iter()
            .map(|s| Occupant {
                s,
                id: ids.insert(()),
                hint: lane.lane_pos(s, 0.0),
                ghost: false,
                lateral: Lateral::InLane,
            })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
