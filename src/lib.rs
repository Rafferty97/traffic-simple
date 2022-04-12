pub use cgmath;
pub use link::{Link, LinkAttributes, TrafficControl};
pub use simulation::Simulation;
use slotmap::{new_key_type, SlotMap};
pub use slotmap::{Key, KeyData};
pub use util::Interval;
pub use vehicle::{Vehicle, VehicleAttributes};

mod conflict;
mod link;
pub mod math;
mod obstacle;
mod simulation;
mod util;
mod vehicle;

new_key_type! {
    /// Unique ID of a [Link].
    pub struct LinkId;
    /// Unique ID of a [Vehicle].
    pub struct VehicleId;
}

type LinkSet = SlotMap<LinkId, Link>;
type VehicleSet = SlotMap<VehicleId, Vehicle>;
