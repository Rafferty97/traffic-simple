//! The speed of what a leader search found.

use slotmap::SlotMap;

use crate::following::Ahead;
use crate::vehicle::{Vehicle, VehicleId};

/// The speed of something a leader search found: a vehicle's, a ghost's owner's,
/// or zero for a stop line.
pub(crate) fn speed_of(vehicles: &SlotMap<VehicleId, Vehicle>, ahead: Ahead) -> f64 {
    match ahead {
        Ahead::Vehicle(id) | Ahead::Ghost(id) => vehicles[id].speed,
        Ahead::PathEnd | Ahead::Barrier { .. } => 0.0,
    }
}
