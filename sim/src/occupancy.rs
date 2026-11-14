//! Per-lane occupant lists, sorted by `s`, and leader search over them.
//!
//! A vehicle joins a lane when it advances onto it (its rear axle passes the end
//! of the previous lane) and leaves once both rear corners are past the lane's
//! end. In between, its record's `s` is the smaller of its rear corners' `s`.
//! On a loop shorter than a vehicle's rear overhang plus a lane, a vehicle can
//! be in the same lane's list twice, briefly, at two different `s`.

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
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = lane.world_to_lane_near(rear_left, hint);
        let right = lane.world_to_lane_near(rear_right, hint);
        let hint = if left.s <= right.s { left } else { right };
        Self { s: hint.s, id, hint }
    }
}

/// Refreshes every occupant's `s`, drops those that have cleared their lane and
/// restores the order.
///
/// Vehicles that clear a lane with no successors have left the network; their
/// ids are pushed to `leavers`.
pub fn update(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &mut [Vec<Occupant>],
    leavers: &mut Vec<VehicleId>,
) {
    for (i, list) in occupants.iter_mut().enumerate() {
        let id = LaneId(i as u32);
        let lane = network.lane(id);
        let dead_end = network.lane(id).successors().is_empty();
        for occupant in list.iter_mut() {
            *occupant = Occupant::new(lane, occupant.id, &vehicles[occupant.id], occupant.hint);
        }
        list.retain(|occupant| {
            let cleared = occupant.s >= lane.length();
            if cleared && dead_end {
                leavers.push(occupant.id);
            }
            !cleared
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
