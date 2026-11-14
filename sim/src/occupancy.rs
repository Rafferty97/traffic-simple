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

use slotmap::SlotMap;

use crate::lane::{Lane, LanePos};
use crate::network::{LaneId, SimNetwork};
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
    /// The lateral extent of the vehicle's rear on this lane: its rear corners'
    /// `t`, smaller first.
    pub extent: [f64; 2],
    /// Whether the vehicle has left this lane and is only straddling it. Kept
    /// up to date by `update`; false on a new record, which is the cautious way.
    pub departing: bool,
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`.
    pub fn new(lane: &Lane, id: VehicleId, vehicle: &Vehicle, hint: LanePos) -> Self {
        Self::project(lane, id, vehicle, hint).0
    }

    /// As [`Occupant::new`], as the ghost of a vehicle indicating a change into `lane`.
    pub fn ghost(lane: &Lane, id: VehicleId, vehicle: &Vehicle, hint: LanePos) -> Self {
        Self { ghost: true, ..Self::new(lane, id, vehicle, hint) }
    }

    /// As [`Occupant::new`], also saying where each rear corner is relative to the lane.
    fn project(
        lane: &Lane,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> (Self, [Corner; 2]) {
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = lane.world_to_lane_near(rear_left, hint);
        let right = lane.world_to_lane_near(rear_right, hint);
        let corner = |p: LanePos| Corner {
            past_end: p.s >= lane.length(),
            beside: p.t.abs() > 0.5 * lane.width(),
        };
        let hint = if left.s <= right.s { left } else { right };
        let extent = [left.t.min(right.t), left.t.max(right.t)];
        // Set by `update`, which knows which lane this is.
        let occupant = Self { s: hint.s, id, hint, ghost: false, extent, departing: false };
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
/// - Vehicles whose rear passes the end of the lane they follow, when it has no
///   successors, have left the network; their ids are pushed to `leavers`.
/// - Vehicles whose rear has moved off a lane sideways have finished leaving an
///   old lane after a lane change; their ids are pushed to `left_sideways`.
pub fn update(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &mut [Vec<Occupant>],
    leavers: &mut Vec<VehicleId>,
    left_sideways: &mut Vec<VehicleId>,
) {
    for (i, list) in occupants.iter_mut().enumerate() {
        let lane_id = LaneId(i as u32);
        let lane = network.lane(lane_id);
        let dead_end = lane.successors().is_empty();
        list.retain_mut(|occupant| {
            let vehicle = &vehicles[occupant.id];
            let (updated, corners) = Occupant::project(lane, occupant.id, vehicle, occupant.hint);
            *occupant = Occupant { ghost: occupant.ghost, ..updated };
            if occupant.ghost {
                return true;
            }
            let following = vehicle.path[0] == lane_id;
            occupant.departing = !following;
            let outside = |c: &Corner| c.past_end || (c.beside && !following);
            if !corners.iter().all(outside) {
                return true;
            }
            // Only the lane it follows: an old lane may dead-end beside a lane that goes on.
            if following && dead_end && corners.iter().all(|c| c.past_end) {
                leavers.push(occupant.id);
            } else if corners.iter().all(|c| c.beside) {
                left_sideways.push(occupant.id);
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
/// A departing vehicle, one only straddling a lane it has left, is passed if its
/// rear leaves room beside it: at least [`PASSING_CLEARANCE`] between its extent
/// and this vehicle's body, centered `axle_t` from the centerline.
#[allow(clippy::too_many_arguments)]
pub fn find_leader(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lanes: &[LaneId],
    axle_s: f64,
    axle_t: f64,
    ghosts: Ghosts,
    skip: Option<VehicleId>,
) -> Search {
    let half_width = 0.5 * vehicle.spec.width;
    let body = [axle_t - half_width, axle_t + half_width];
    let room_to_pass = |extent: [f64; 2]| {
        extent[0] - body[1] >= PASSING_CLEARANCE || body[0] - extent[1] >= PASSING_CLEARANCE
    };
    let front = axle_s + vehicle.spec.length - vehicle.spec.rear_axle;
    let mut found = Search::default();
    // Start of the current lane, measured along `lanes` from the start of the first.
    let mut offset = 0.0;
    let mut after = axle_s;
    for &lane in lanes {
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
            if o.departing && room_to_pass(o.extent) {
                continue;
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
                extent: [-1.0, 1.0],
                departing: false,
            })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
