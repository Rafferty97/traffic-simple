use slotmap::{new_key_type, SlotMap};
pub use simulation::Simulation;
pub use vehicle::Vehicle;
pub use link::Link;

pub mod math;
mod util;
mod simulation;
mod link;
mod vehicle;
mod obstacle;

new_key_type! {
    /// Unique ID of a [Link].
    pub struct LinkId;
    /// Unique ID of a [Vehicle].
    pub struct VehicleId;
}

type LinkSet = SlotMap<LinkId, Link>;
type VehicleSet = SlotMap<VehicleId, Vehicle>;