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

use arrayvec::ArrayVec;
use glam::DVec2;
use slotmap::SlotMap;

use crate::debug::PassCheck;
use crate::lane::LanePos;
use crate::models::lateral::LaneChange;
use crate::network::{LaneId, Side, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// How far ahead of its front bumper a vehicle looks for a leader, in meters.
pub(crate) const LEADER_HORIZON: f64 = 200.0;
/// The most leaders a search finds. A lane rarely gives more than one or two;
/// past this many, the rest are beyond what the nearer ones already impose.
pub const MAX_LEADERS: usize = 8;
/// The least room, in meters, a vehicle needs beside another to pass it.
const PASSING_CLEARANCE: f64 = 0.5;
/// How far behind the other vehicle's rear, in meters, a passing check puts the
/// searching vehicle's nose, so a check that fails is drawn short of overlapping.
const NOSE_GAP: f64 = 0.1;

/// A vehicle's record in one lane's occupant list.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
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

    /// Where it will be, as its rear axle's position and heading, when its nose
    /// is [`NOSE_GAP`] short of `point`, another vehicle's rearmost corner, at `s`
    /// on `lane`, the `k`th lane searched. `nose` is how far its nose is ahead of
    /// its rear axle.
    ///
    /// On the lanes it follows, `gap`, how far its rear axle travels before its
    /// nose reaches `s`, is exact. On a lane beside them, distances differ from
    /// its own, so `point` is projected onto its own `k`th lane instead.
    ///
    /// If its nose is already there, there is no earlier point to check from, so
    /// it is where it is `now`, its actual pose, rather than where its course
    /// would put it, which pure pursuit lags behind.
    #[allow(clippy::too_many_arguments)]
    fn reach(
        &self,
        network: &SimNetwork,
        k: usize,
        lane: LaneId,
        gap: f64,
        point: DVec2,
        s: f64,
        nose: f64,
        now: (DVec2, DVec2),
    ) -> (DVec2, DVec2) {
        let ahead = match self.path.get(k) {
            Some(&own) if own != lane => {
                let (own_lane, searched) = (network.lane(own), network.lane(lane));
                let before: f64 = self.path[..k].iter().map(|&l| network.lane(l).length()).sum();
                // Adjacent lanes start and end together, so the same fraction is a good hint.
                let hint = own_lane.lane_pos(s / searched.length() * own_lane.length(), 0.0);
                before + own_lane.world_to_lane_near(point, hint).s - nose - self.s
            }
            _ => gap,
        };
        match ahead - NOSE_GAP {
            ahead if ahead > 0.0 => self.pose(network, ahead),
            _ => now,
        }
    }

    /// How far its rear axle has still to go to the end of its lane change, or zero
    /// if it isn't changing lanes.
    fn remaining(&self) -> f64 {
        self.change.map_or(0.0, |c| (c.end() - self.s).max(0.0))
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
    /// the searching vehicle may choose to make room for it.
    Pass,
    /// Ghosts count as leaders, as when a vehicle weighs up changing lanes: a
    /// vehicle already indicating into the gap ahead goes first.
    Lead,
}

/// What a leader search found, each with the bumper-to-bumper gap to it.
#[derive(Clone, Debug, Default)]
pub struct Search {
    /// Every vehicle ahead it must react to, nearest first: each is its own term
    /// in the longitudinal model, whose minimum binds.
    pub leaders: ArrayVec<(VehicleId, f64), MAX_LEADERS>,
    /// The nearest ghost before the first leader, with [`Ghosts::Pass`].
    pub ghost: Option<(VehicleId, f64)>,
}

/// The vehicles ahead along `lanes`, within the leader horizon, that a vehicle
/// must react to, each with the bumper-to-bumper gap to it. `axle_s` is the
/// vehicle's rear axle's `s` on the first lane.
///
/// On the first lane, only occupants whose rear is at or ahead of this vehicle's
/// rear count, `axle_s` less its rear overhang: [`find_follower`] takes those
/// behind, so between them they find every vehicle, one alongside included. On
/// later lanes, every occupant counts. The vehicle's own records are skipped,
/// as are `skip`'s, for asking what if that vehicle weren't there.
///
/// Searching lanes beside the course rather than along it, as the old-side search
/// does, the horizon is where the course's lane change ends instead: after that
/// the vehicle is wholly on the lanes it follows, so nothing it would only reach
/// later can be in its way there.
///
/// Some records may be passed, by the rules in [`passing_side`]. One of those is
/// passed if, with this vehicle where its `course` puts it as its front reaches
/// that record's rear, driving straight ahead would leave at least
/// [`PASSING_CLEARANCE`] beside the other vehicle's rear corners. That reads the
/// other vehicle's corners from `vehicles`, the one place leader search does.
/// Each such check is reported to `on_pass`.
///
/// Which records it needs, by transitivity: whatever constrains the first vehicle
/// in the lane ahead, R, reaches this vehicle through R, so it needn't look past
/// R, except in two ways. R may have passed a vehicle diverging from the lane
/// that this one can't: those lie between R and the next vehicle in the lane, R2,
/// which R can't pass. And R's path may leave this one's at a lane boundary, so
/// what lies beyond on this one's path never constrains R. So on each lane it
/// takes the first vehicle in the lane, and the diverging vehicles it can't pass
/// up to the next one, R2, which it stops at without taking; then it goes on to
/// the next lane of its path and does the same, as SUMO takes the nearest vehicle
/// on each lane ahead. Where R does carry on along its path, the next lane's
/// vehicles are beyond R, and their terms are weaker.
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
    let horizon = match course.path.first() == lanes.first() {
        true => LEADER_HORIZON,
        // A gap is how far the rear axle travels before the nose reaches a rear.
        false => course.remaining().min(LEADER_HORIZON),
    };
    let mut found = Search::default();
    // Start of the current lane, measured along `lanes` from the start of the first.
    let mut offset = 0.0;
    let rear = axle_s - vehicle.spec.rear_axle;
    'lanes: for (k, &lane) in lanes.iter().enumerate() {
        if offset - front > horizon {
            break;
        }
        let list = &occupants[lane.0 as usize];
        let start = if k == 0 {
            list.partition_point(|o| o.s < rear)
        } else {
            0
        };
        // Past the first vehicle in the lane on this lane, R: only vehicles
        // diverging from it still count, up to the next one in the lane.
        let mut past_r = false;
        for o in &list[start..] {
            if o.id == id || Some(o.id) == skip {
                continue;
            }
            let gap = offset + o.s - front;
            if gap > horizon {
                break 'lanes;
            }
            if o.ghost && ghosts == Ghosts::Pass {
                if found.leaders.is_empty() {
                    found.ghost = found.ghost.or(Some((o.id, gap)));
                }
                continue;
            }
            let in_lane = o.lateral == Lateral::InLane;
            if past_r && in_lane {
                // R2: whatever lies beyond reaches it through R.
                break;
            }
            let leaving = course.leaving(network, k, lane);
            if let Some(side) = passing_side(o.lateral, leaving, course.moving()) {
                // The record sits at the other vehicle's rearmost corner.
                let rearmost = network.lane(lane).lane_to_world(o.hint);
                let nose = vehicle.spec.length - vehicle.spec.rear_axle;
                let now = (vehicle.pos, DVec2::from_angle(vehicle.heading));
                let (pos, tangent) = course.reach(network, k, lane, gap, rearmost, o.s, nose, now);
                let [rear_left, rear_right, ..] = vehicles[o.id].corners();
                let corners = [rear_left, rear_right];
                let (clearance, corner) = clearance(side, pos, tangent, half_width, corners);
                let passed = clearance >= PASSING_CLEARANCE;
                let leader = o.id;
                on_pass(PassCheck { leader, pos, tangent, side, corner, clearance, passed });
                if passed {
                    continue;
                }
            }
            if found.leaders.try_push((o.id, gap)).is_err() {
                break 'lanes;
            }
            past_r |= in_lane;
        }
        offset += network.lane(lane).length();
    }
    found
}

/// The nearest vehicle behind `s` on `lane` that would end up behind a vehicle
/// there, and the gap from its front to `s`: searching back along `lane`'s
/// occupants, then the lanes leading into it, up to the leader horizon.
///
/// Every vehicle on `lane` itself counts, whether settled, moving in or moving
/// out, since any of them can be in the way; on the lanes leading into it, one
/// counts if its path goes on through `lane`. Ghosts and `skip` are passed over.
/// One further back than the leader horizon is too far away to be affected, and
/// its own leader search wouldn't reach `s` either.
///
/// It splits the lane at `s` with [`find_leader`]: a record whose rear is at or
/// ahead of `s` is a leader, and one whose rear is behind is a follower, so a
/// vehicle alongside is found by one or the other, with a gap of zero or less.
///
/// This is the backward search: intermittent decisions and insertion use it to
/// find the vehicles they would affect, while every per-frame search looks only
/// forward.
pub fn find_follower(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    lane: LaneId,
    s: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    behind(network, occupants, vehicles, lane, lane, s, 0.0, skip)
}

/// [`find_follower`]'s search of `lane`, from `s` back, where `s` is `travelled`
/// back from the point searched from, on `target`.
#[allow(clippy::too_many_arguments)]
fn behind(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    vehicles: &SlotMap<VehicleId, Vehicle>,
    target: LaneId,
    lane: LaneId,
    s: f64,
    travelled: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    let list = &occupants[lane.0 as usize];
    let end = list.partition_point(|o| o.s < s);
    let counts = |o: &Occupant| {
        let vehicle = &vehicles[o.id];
        // Its path is short of `target` only if it is full and `target` lies beyond.
        let through = vehicle.path.contains(&target) || vehicle.path.is_full();
        !o.ghost && o.id != skip && (lane == target || through)
    };
    if let Some(o) = list[..end].iter().rev().find(|o| counts(o)) {
        let gap = travelled + s - (o.s + vehicles[o.id].spec.length);
        return (gap <= LEADER_HORIZON).then_some((o.id, gap));
    }
    // None on this lane: on to the ends of the lanes leading into it. On a loop a
    // lane leads into itself, which the horizon ends.
    let travelled = travelled + s;
    if travelled > LEADER_HORIZON {
        return None;
    }
    network
        .lane(lane)
        .predecessors()
        .iter()
        .filter_map(|&p| {
            let length = network.lane(p).length();
            behind(
                network, occupants, vehicles, target, p, length, travelled, skip,
            )
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
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

    /// Two concentric half-circle lanes, curving left: lane 0 inside, of radius
    /// 150 m, and lane 1 outside it.
    fn bend() -> SimNetwork {
        let arc = |radius: f64| {
            let points: Vec<_> = (0..=400)
                .map(|i| {
                    let a = std::f64::consts::PI * i as f64 / 400.0 - std::f64::consts::FRAC_PI_2;
                    radius * DVec2::new(a.cos(), a.sin())
                })
                .collect();
            Lane::new(&points, 3.5, 14.0).unwrap()
        };
        let mut network = SimNetwork::new(vec![arc(150.0), arc(153.5)]);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network
    }

    /// Where a predicted rear axle at `pos`, heading along `tangent`, puts its
    /// nose, `nose` ahead, as `s` along `lane`.
    fn nose_along(
        network: &SimNetwork,
        lane: LaneId,
        pos: DVec2,
        tangent: DVec2,
        nose: f64,
        near: f64,
    ) -> f64 {
        let lane = network.lane(lane);
        lane.world_to_lane_near(pos + tangent * nose, lane.lane_pos(near, 0.0)).s
    }

    const NOSE: f64 = 3.6;
    /// Unused: `reach` shouldn't fall back to where the vehicle is.
    const NOWHERE: (DVec2, DVec2) = (DVec2::new(1e9, 1e9), DVec2::X);

    #[test]
    fn on_its_own_lane_the_nose_stops_just_short_of_the_other_rear() {
        let network = bend();
        let path = [LaneId(1)];
        let course = Course::new(&path, None, 10.0);
        let lane = network.lane(LaneId(1));
        // A rear corner 60 m along, off the centerline.
        let corner = lane.lane_to_world(lane.lane_pos(60.0, -0.9));
        let gap = 60.0 - (10.0 + NOSE);
        let (pos, tangent) = course.reach(&network, 0, LaneId(1), gap, corner, 60.0, NOSE, NOWHERE);
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, 60.0);
        assert!((nose - (60.0 - NOSE_GAP)).abs() < 0.02, "nose at {nose}");
    }

    #[test]
    fn on_the_old_side_the_other_rear_is_projected_onto_its_own_lane() {
        // Searching lane 0 from a course along lane 1: lane 0 is shorter round
        // the bend, so its distances aren't the course's, and the gap is ignored.
        let network = bend();
        let path = [LaneId(1)];
        let course = Course::new(&path, None, 10.0);
        let beside = network.lane(LaneId(0));
        let corner = beside.lane_to_world(beside.lane_pos(58.0, -1.2));
        let (pos, tangent) =
            course.reach(&network, 0, LaneId(0), 999.0, corner, 58.0, NOSE, NOWHERE);
        let own = network.lane(LaneId(1));
        let target = own.world_to_lane_near(corner, own.lane_pos(58.0, 0.0)).s;
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, target);
        assert!(
            (nose - (target - NOSE_GAP)).abs() < 0.02,
            "nose at {nose}, corner at {target}"
        );
    }

    #[test]
    fn mid_change_the_nose_still_stops_short() {
        // Changing onto lane 1 from lane 0, so heading across the lane: the nose,
        // ahead along that heading, falls a little short, never past.
        let network = bend();
        let path = [LaneId(1)];
        let change = LaneChange::new(10.0, 3.5, 0.0, 0.0);
        let course = Course::new(&path, Some(change), 10.0);
        let lane = network.lane(LaneId(1));
        let corner = lane.lane_to_world(lane.lane_pos(25.0, 0.0));
        let gap = 25.0 - (10.0 + NOSE);
        let (pos, tangent) = course.reach(&network, 0, LaneId(1), gap, corner, 25.0, NOSE, NOWHERE);
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, 25.0);
        assert!(
            nose < 25.0 && nose > 25.0 - NOSE_GAP - 0.3,
            "nose at {nose}"
        );
    }

    #[test]
    fn already_alongside_it_is_checked_where_it_is_now() {
        let network = bend();
        let path = [LaneId(1)];
        let course = Course::new(&path, None, 10.0);
        let lane = network.lane(LaneId(1));
        let now = (DVec2::new(1.0, 2.0), DVec2::Y);
        // The other rear is behind its nose already, and just short of it.
        for s in [11.0, 13.0, 10.0 + NOSE + 0.5 * NOSE_GAP] {
            let corner = lane.lane_to_world(lane.lane_pos(s, 0.0));
            let gap = s - (10.0 + NOSE);
            let reached = course.reach(&network, 0, LaneId(1), gap, corner, s, NOSE, now);
            assert_eq!(reached, now, "rear at {s}");
        }
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
