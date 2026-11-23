//! A vehicle's acceleration searching lanes other than its own.

use slotmap::SlotMap;

use crate::following::{Ahead, Along, Found, for_each_leader};
use crate::models::acceleration;
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// A vehicle's acceleration under the longitudinal model searching `along` some
/// lanes, against the free road there, its leaders and the end of its path.
/// Ghosts count as leaders if `ghosts_lead`, as when
/// weighing up a change into their lane: a vehicle already indicating into the
/// gap goes first. Its own ghost, `id`'s, is passed over. It keeps at most
/// `headway`, a share, of its time headway to each leader.
pub(crate) fn accel_along(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    along: Along,
    ghosts_lead: bool,
    headway: f64,
) -> f64 {
    let lane = along.lane(&state.network, vehicle, 0).unwrap_or(vehicle.path[0]);
    let desired = vehicle.spec.desired_speed(state.network.lane(lane).speed_limit());
    let mut accel = acceleration::free_road(&vehicle.spec, vehicle.speed, desired);
    for_each_leader(state, vehicle, along, 0.0, |found| match found {
        Found::Ahead(Ahead::Ghost(owner), ..) if owner == id || !ghosts_lead => {}
        Found::Ahead(ahead, gap, relaxed) => {
            let speed = speed_of(&state.vehicles, ahead);
            let share = relaxed.min(headway);
            let term = acceleration::relaxed(&vehicle.spec, vehicle.speed, gap, speed, share);
            accel = accel.min(term);
        }
        Found::Pass(_) => {}
    });
    accel
}

/// The speed of something a leader search found: a vehicle's, a ghost's owner's,
/// or zero for a stop line.
pub(crate) fn speed_of(vehicles: &SlotMap<VehicleId, Vehicle>, ahead: Ahead) -> f64 {
    match ahead {
        Ahead::Vehicle(id) | Ahead::Ghost(id) => vehicles[id].speed,
        Ahead::PathEnd | Ahead::Barrier { .. } => 0.0,
    }
}
