//! Joining and leaving lanes' occupant lists: a vehicle is a registered
//! occupant of every lane it is on or overlaps (invariant 1), each record
//! classified, and its ghost is where it is indicating to.

use arrayvec::ArrayVec;

use super::{Lateral, Occupant, Overlaps, around, classify, record, same_record};
use crate::network::{LaneId, MAX_SUCCESSORS, SimNetwork};
use crate::vehicle::{LaneChangeIntent, Vehicle, VehicleId};

/// Whether `id` has a record, not a ghost, on `lane` near `s` (see
/// [`same_record`]). On a loop, a vehicle can have a second one,
/// lingering past the end, at the other end.
fn has_record_at(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    lane: LaneId,
    id: VehicleId,
    s: f64,
) -> bool {
    let length = network.lane(lane).geometry().length();
    occupants[lane.0 as usize]
        .iter()
        .any(|o| o.is_record_of(id) && same_record(length, o.s(), s))
}

/// Joins `vehicle`, which has just joined `lane`, to every lane overlapping it
/// that it isn't on yet, each record classified and kept in order.
pub(crate) fn join_overlaps(
    network: &SimNetwork,
    overlaps: &Overlaps,
    occupants: &mut [Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lane: LaneId,
) {
    for p in &overlaps[lane.0 as usize] {
        let occupant = record(network, overlaps, p.lane, id, vehicle);
        if has_record_at(network, occupants, p.lane, id, occupant.s()) {
            continue;
        }
        let list = &mut occupants[p.lane.0 as usize];
        let index = list.partition_point(|o| o.s() <= occupant.s());
        list.insert(index, occupant);
    }
}

/// Classifies afresh `vehicle`'s records around `lane` (see
/// [`around`]), other than on the lane it follows: all of them, or,
/// if `converging_only`, those still converging.
pub(crate) fn reclassify(
    network: &SimNetwork,
    overlaps: &Overlaps,
    occupants: &mut [Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lane: LaneId,
    converging_only: bool,
) {
    for lane in around(network, overlaps, lane) {
        if lane == vehicle.path[0] {
            continue;
        }
        for o in occupants[lane.0 as usize].iter_mut() {
            if o.is_record_of(id) && (!converging_only || o.lateral.is_converging()) {
                classify(network, overlaps, vehicle, lane, o);
            }
        }
    }
}

/// Removes a vehicle's ghost from a lane's occupants.
pub(crate) fn remove_ghost(occupants: &mut [Vec<Occupant>], lane: LaneId, id: VehicleId) {
    occupants[lane.0 as usize].retain(|o| !(o.id == id && o.is_ghost()));
}

/// Joins `vehicle`, which has just advanced from `previous` onto the lane it
/// now follows, to that lane and every lane overlapping it, such as the other
/// branches where lanes part and the other lanes leading into the same one,
/// each classified from its course; and, while it straddles after a lane
/// change, to the lanes its old lane leads to as well, so followers there keep
/// seeing it. Its record on the lane it follows carries over its age on
/// `previous`, unless it `enters_merge`. Its ghost, if it is indicating, moves
/// beside the new lane, or the intent is cancelled if there it may change
/// into none. Joins unsorted: the next occupancy update sorts.
pub(crate) fn advance(
    network: &SimNetwork,
    overlaps: &Overlaps,
    occupants: &mut [Vec<Occupant>],
    id: VehicleId,
    vehicle: &mut Vehicle,
    previous: LaneId,
    enters_merge: bool,
) {
    // It joins the lane it now follows, and every lane overlapping
    // it, such as the other branches where lanes part and the other
    // lanes leading into the same one, each classified from its
    // course. Joins unsorted; `update_occupancy` sets its `s` and
    // sorts.
    let followed = vehicle.path[0];
    // Carried over from the lane before, unless it enters a merge, where it
    // joins another lane's traffic.
    let carried = match enters_merge {
        true => 0.0,
        false => occupants[previous.0 as usize]
            .iter()
            .filter(|o| o.is_record_of(id))
            .map(|o| o.age)
            .fold(0.0, f64::max),
    };
    let join = |occupants: &mut [Vec<Occupant>], lane: LaneId| {
        let geometry = network.lane(lane).geometry();
        let hint = match lane == followed {
            true => vehicle.lane_pos,
            false => geometry.world_to_lane(vehicle.pos),
        };
        let mut occupant = Occupant::new(network, lane, id, vehicle, hint, Lateral::InLane);
        // Already there, as one lane it joins can overlap another,
        // and, straddling into a merge, the lane its old lane leads
        // to overlaps the one it follows: once is enough. If that is
        // the lane it follows, the record there is its own now.
        let list = &mut occupants[lane.0 as usize];
        let length = geometry.length();
        let near = |o: &Occupant| o.is_record_of(id) && same_record(length, o.s(), occupant.s());
        if let Some(existing) = list.iter_mut().find(|o| near(o)) {
            if lane == followed {
                existing.age = carried;
            }
            return;
        }
        classify(network, overlaps, vehicle, lane, &mut occupant);
        if lane == followed {
            occupant.age = carried;
        }
        occupants[lane.0 as usize].push(occupant);
    };
    join(occupants, followed);
    // Still straddling after a lane change: also join the lanes its old
    // lane leads to, so followers there keep seeing it.
    let old = vehicle.old_side.and_then(|side| network.lane(previous).adj(side));
    // Every lane it enters, it also joins those overlapping it.
    let mut entered = ArrayVec::<LaneId, { 1 + MAX_SUCCESSORS }>::new();
    entered.push(followed);
    if let Some(old) = old {
        for &lane in network.lane(old.lane).successors() {
            join(occupants, lane);
            entered.push(lane);
        }
    }
    for &lane in &entered {
        for p in &overlaps[lane.0 as usize] {
            join(occupants, p.lane);
        }
    }
    // What it knows of its course has grown: records that were
    // converging for want of it are classified again.
    reclassify(network, overlaps, occupants, id, vehicle, previous, true);
    // Indicating: the ghost moves to the new lane's neighbor, or it stops
    // indicating if there's none to change into, still wanting to.
    if let Some(side) = vehicle.intent.indicating() {
        if let Some(old) = network.lane(previous).adj(side) {
            remove_ghost(occupants, old.lane, id);
        }
        match network.lane(followed).adj(side).filter(|n| n.can_change) {
            Some(n) => {
                let target = network.lane(n.lane);
                let hint = target.geometry().lane_pos(vehicle.lane_pos.s, 0.0);
                occupants[n.lane.0 as usize]
                    .push(Occupant::ghost(network, n.lane, id, vehicle, hint));
            }
            None => vehicle.intent = LaneChangeIntent::Desire { side, check: None },
        }
    }
}
