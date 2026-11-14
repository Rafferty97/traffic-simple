//! Per-lane occupant lists, sorted by `s`, and leader search over them.
//!
//! A vehicle joins a lane when it advances onto it (its rear axle passes the end
//! of the previous lane) or changes lanes into it. It leaves once both rear
//! corners are outside the lane: past its end, or beside it. In between, its
//! record's `s` is the smaller of its rear corners' `s`. On a loop shorter than a
//! vehicle's rear overhang plus a lane, a vehicle can be in the same lane's list
//! twice, briefly, at two different `s`.

use slotmap::SlotMap;

use crate::lane::{Lane, LanePos};
use crate::network::{LaneId, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// How far ahead of its front bumper a vehicle looks for a leader, in metres.
const LEADER_HORIZON: f64 = 200.0;

/// A vehicle's record in one lane's occupant list.
#[derive(Clone, Copy, Debug)]
pub struct Occupant {
    /// The smaller of the vehicle's rear corners' `s` on this lane.
    pub s: f64,
    pub id: VehicleId,
    /// Lane coordinates of that rear corner, a hint for the next projection.
    hint: LanePos,
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`.
    pub fn new(lane: &Lane, id: VehicleId, vehicle: &Vehicle, hint: LanePos) -> Self {
        Self::project(lane, id, vehicle, hint).0
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
        (Self { s: hint.s, id, hint }, [corner(left), corner(right)])
    }
}

/// Where a rear corner is relative to a lane.
#[derive(Clone, Copy, Debug)]
struct Corner {
    past_end: bool,
    /// Further from the centreline than half the lane's width.
    beside: bool,
}

/// Refreshes every occupant's `s`, drops those whose rear corners are both
/// outside their lane, and restores the order.
///
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
            *occupant = updated;
            let following = vehicle.path[0] == lane_id;
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

/// The nearest vehicle ahead along `vehicle`'s path, within the leader horizon,
/// and the bumper-to-bumper gap to it.
///
/// On the vehicle's own lane, only occupants whose rear is ahead of its rear axle
/// count. On later lanes in its path, every occupant counts.
pub fn find_leader(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
) -> Option<(VehicleId, f64)> {
    let front = vehicle.lane_pos.s + vehicle.spec.length - vehicle.spec.rear_axle;
    // Start of the current path lane, measured along the path from the start of
    // the vehicle's own lane.
    let mut offset = 0.0;
    let mut after = vehicle.lane_pos.s;
    for &lane in &vehicle.path {
        if offset - front > LEADER_HORIZON {
            break;
        }
        let list = &occupants[lane.0 as usize];
        let start = list.partition_point(|o| o.s <= after);
        if let Some(o) = list[start..].iter().find(|o| o.id != id) {
            let gap = offset + o.s - front;
            return (gap <= LEADER_HORIZON).then_some((o.id, gap));
        }
        offset += network.lane(lane).length();
        after = f64::NEG_INFINITY;
    }
    None
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
            .map(|s| Occupant { s, id: ids.insert(()), hint: lane.lane_pos(s, 0.0) })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
