pub use cgmath;
pub use group::LinkGroup;
pub use light::TrafficLight;
pub use link::{Link, LinkAttributes, TrafficControl};
pub use simulation::Simulation;
use slotmap::{new_key_type, SlotMap};
pub use slotmap::{Key, KeyData};
pub use util::Interval;
pub use vehicle::{Vehicle, VehicleAttributes};

mod conflict;
mod group;
mod light;
mod link;
pub mod math;
mod simulation;
mod util;
mod vehicle;

new_key_type! {
    /// Unique ID of a [Link].
    pub struct LinkId;
    /// Unique ID of a [Vehicle].
    pub struct VehicleId;
    /// Unique ID of a [TrafficLight].
    pub struct TrafficLightId;
}

type LinkSet = SlotMap<LinkId, Link>;
type VehicleSet = SlotMap<VehicleId, Vehicle>;
