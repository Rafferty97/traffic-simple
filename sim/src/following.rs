//! Multi-lane car following, as `docs/multilane-car-following.md` has it:
//! how vehicles on lanes that overlap, merge and diverge, and changing lanes
//! between them, find every vehicle that could be in their way, and pass only
//! those they can prove they won't hit.
//!
//! Here, the occupancy records. Each lane keeps a list of its occupants,
//! sorted by `s`. A vehicle joins a lane when it advances onto it (its rear
//! axle passes the end of the previous lane) or changes lanes into it. It
//! leaves once both rear corners are outside the lane: past its end, or beside
//! it. In between, its record's `s` is the smaller of its rear corners' `s`. On
//! a loop shorter than a vehicle's rear overhang plus a lane, a vehicle can be
//! in the same lane's list twice, briefly, at two different `s`.
//!
//! A vehicle indicating a lane change also has a ghost record in the target
//! lane's list. Ghosts are kept up to date like any record but are only removed
//! explicitly, when the intent is cancelled or carried out.
//!
//! The rest is in the submodules, one for each part of the design doc:
//! [`overlaps`](mod@overlaps) (Preprocessing), [`classify`](mod@classify) (Classifying occupancy records),
//! [`join`] (joining lanes, invariant 1), [`leaders`] (the leader search and Old-lane search), and [`pass`] (Pass
//! check).

mod classify;
mod join;
mod leaders;
mod overlaps;
mod pass;

pub(crate) use classify::{around, classify, record};
pub(crate) use join::{advance, join_overlaps, reclassify, remove_ghost};
pub use leaders::{Ahead, Along, Found, find_leaders, for_each_leader};
pub use overlaps::{Overlaps, overlaps};
pub(crate) use pass::body;
pub use pass::{MIN_PASSING_CLEARANCE, PASSING_CLEARANCE, passing_leader};

use glam::DVec2;
use slotmap::SlotMap;

use crate::network::geometry::LanePos;
use crate::network::{LaneId, Side, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// A vehicle's record in one lane's occupant list.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Occupant {
    pub id: VehicleId,
    /// Lane coordinates of the vehicle's rearmost corner on this lane: the
    /// record's position, and a hint for the next projection.
    pub(crate) hint: LanePos,
    /// Whether the vehicle is in this lane, converging on it, diverging from it
    /// or indicating into it.
    pub lateral: Lateral,
    /// How long, in seconds, the vehicle has been in this lane's traffic: since
    /// it changed lanes into it, entered a merge onto it or joined it from
    /// beside. Advancing from one lane to the next carries it over, and an
    /// inserted vehicle starts settled.
    /// Leader terms relax with it (see the longitudinal model's `relaxation`).
    #[cfg_attr(feature = "serde", serde(default = "settled"))]
    pub age: f64,
}

/// Whether `a` and `b`, the `s` of two of one vehicle's records on a lane
/// `length` long, are the same record. Projections of the same rear corners
/// are either at much the same `s` or a whole loop apart, so half the lane's
/// length tells them apart.
pub(crate) fn same_record(length: f64, a: f64, b: f64) -> bool {
    (a - b).abs() <= 0.5 * length
}

/// The age of a record from before records had one: long settled.
#[cfg(feature = "serde")]
fn settled() -> f64 {
    crate::models::acceleration::SETTLED
}

impl Occupant {
    /// The smaller of the vehicle's rear corners' `s` on this lane.
    pub fn s(&self) -> f64 {
        self.hint.s
    }

    /// Whether this is the ghost of a vehicle indicating a change into this lane.
    pub fn is_ghost(&self) -> bool {
        self.lateral == Lateral::Ghost
    }

    /// Whether this is `id`'s record, not its ghost.
    pub fn is_record_of(&self, id: VehicleId) -> bool {
        self.id == id && !self.is_ghost()
    }
}

/// How a vehicle relates to a lane it has a record on, as followers there see it
/// (see `docs/multilane-car-following.md`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Lateral {
    /// In the lane: following it, settled, or lingering past the end of it
    /// after advancing.
    InLane,
    /// Its course may yet bring it into the lane, or closer to it: changing lanes
    /// into it, merging into it from a lane that leads into the same one, or
    /// leaving it on a course that may come back. It turns diverging toward
    /// `side` once its `s` reaches `turn`, which is `f64::MAX` if it never does
    /// (see `following::classify`); finite, so snapshots serialize.
    Converging { turn: f64, side: Side },
    /// Moving away from the lane sideways, toward this side, and never coming
    /// back to it.
    Diverging(Side),
    /// The ghost of a vehicle indicating a change into the lane.
    Ghost,
}

impl Lateral {
    /// Converging, until classified again.
    pub const CONVERGING: Self = Self::Converging { turn: f64::MAX, side: Side::Left };

    pub fn is_converging(self) -> bool {
        matches!(self, Self::Converging { .. })
    }

    pub fn is_diverging(self) -> bool {
        matches!(self, Self::Diverging(_))
    }
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`, for a
    /// record that is `lateral` to it. A record's lateral status is fixed when it
    /// is made, from how the lanes lie, not from where the vehicle is: on a lane
    /// it is joining sideways, its corners start out level with the centerline,
    /// where their side says nothing.
    pub fn new(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
        lateral: Lateral,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, lateral).0
    }

    /// As [`Occupant::new`], as the ghost of a vehicle indicating a change into `lane`.
    pub fn ghost(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, Lateral::Ghost).0
    }

    /// As [`Occupant::new`] or [`Occupant::ghost`], also saying where each rear
    /// corner is relative to the lane.
    fn project(
        network: &SimNetwork,
        lane_id: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
        lateral: Lateral,
    ) -> (Self, [Corner; 2]) {
        let geometry = network.lane(lane_id).geometry();
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = geometry.world_to_lane_near(rear_left, hint);
        let right = geometry.world_to_lane_near(rear_right, hint);
        let corner = |p: LanePos| Corner {
            past_end: p.s >= geometry.length(),
            beside: !geometry.between_edges(p),
        };
        let hint = if left.s <= right.s { left } else { right };
        let occupant = Self { id, hint, lateral, age: 0.0 };
        (occupant, [corner(left), corner(right)])
    }
}

/// Whether `point` is on `geometry`'s lane, between its ends and its edges,
/// projecting from `hint`.
pub(crate) fn inside(
    geometry: &crate::network::geometry::LaneGeometry,
    point: DVec2,
    hint: LanePos,
) -> bool {
    let p = geometry.world_to_lane_near(point, hint);
    p.s >= 0.0 && p.s < geometry.length() && geometry.between_edges(p)
}

/// Where a rear corner is relative to a lane.
#[derive(Clone, Copy, Debug)]
struct Corner {
    past_end: bool,
    /// Outside the lane's edges, where it is along the lane.
    beside: bool,
}

/// Refreshes every occupant's `s`, drops those whose rear corners are both
/// outside their lane, and restores the order.
///
/// - Ghosts are refreshed but never dropped here.
/// - A record on the lane a vehicle follows is in the lane, or converging while
///   it changes lanes. Elsewhere, a converging record turns diverging once its
///   `s` reaches where its classification said it would.
/// - Only a diverging record is dropped for being beside its lane. Any record is
///   dropped once both rear corners are past the lane's end.
/// - Vehicles that lose a record on a lane they aren't following, by its end or
///   its side, have their ids pushed to `departed`, so their old side can be
///   checked once every list is up to date.
pub fn update(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &mut [Vec<Occupant>],
    departed: &mut Vec<VehicleId>,
    dt: f64,
) {
    for (i, list) in occupants.iter_mut().enumerate() {
        let lane_id = LaneId(i as u32);
        list.retain_mut(|occupant| {
            occupant.age += dt;
            let vehicle = &vehicles[occupant.id];
            let Occupant { id, hint, lateral, .. } = *occupant;
            let (updated, corners) =
                Occupant::project(network, lane_id, id, vehicle, hint, lateral);
            occupant.hint = updated.hint;
            let following = vehicle.path[0] == lane_id;
            match occupant.lateral {
                Lateral::Ghost => return true,
                _ if following => {
                    occupant.lateral = match vehicle.lane_change {
                        Some(_) => Lateral::CONVERGING,
                        None => Lateral::InLane,
                    };
                }
                Lateral::Converging { turn, side } if occupant.hint.s >= turn => {
                    occupant.lateral = Lateral::Diverging(side);
                }
                _ => {}
            }
            // Only a vehicle diverging from the lane is ever dropped for being
            // beside it: one converging may yet come back.
            let diverging = matches!(occupant.lateral, Lateral::Diverging(_));
            let outside = |c: &Corner| c.past_end || (c.beside && diverging);
            if !corners.iter().all(outside) {
                return true;
            }
            // Beside the lane, the whole body must be clear of it: where the
            // lane bends toward it, a vehicle leaving it points back across it,
            // and its front can still be in the lane after its rear has left.
            if diverging && !corners.iter().all(|c| c.past_end) {
                let geometry = network.lane(lane_id).geometry();
                let [.., front_right, front_left] = vehicle.corners();
                let in_lane = |c: DVec2| inside(geometry, c, occupant.hint);
                if in_lane(front_left) || in_lane(front_right) {
                    return true;
                }
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
/// is: vehicles rarely pass each other within a lane between steps.
fn insertion_sort(list: &mut [Occupant]) {
    for i in 1..list.len() {
        let mut j = i;
        while j > 0 && list[j - 1].s() > list[j].s() {
            list.swap(j - 1, j);
            j -= 1;
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::network::Lane;

    #[test]
    fn insertion_sort_orders_by_s() {
        let mut ids = SlotMap::<VehicleId, ()>::with_key();
        let lane = Lane::new(
            &[glam::DVec2::ZERO, glam::DVec2::X * 10.0],
            3.5,
            14.0,
            crate::simulation::test::alone(),
        )
        .unwrap();
        let mut list: Vec<_> = [3.0, 1.0, 2.0, 0.5]
            .into_iter()
            .map(|s| Occupant {
                id: ids.insert(()),
                hint: lane.geometry().lane_pos(s, 0.0),
                lateral: Lateral::InLane,
                age: 0.0,
            })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s()).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
