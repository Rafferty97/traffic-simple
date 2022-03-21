use slotmap::{new_key_type, SlotMap};
use vehicle::Vehicle;
use link::Link;

mod util;
mod math;
mod simulation;
mod link;
mod vehicle;
mod curve;
mod obstacle;

new_key_type! {
    pub struct LinkId;
    pub struct VehicleId;
}

type LinkSet = SlotMap<LinkId, Link>;
type VehicleSet = SlotMap<VehicleId, Vehicle>;