use slotmap::{new_key_type, SlotMap};
use util::Interval;
use math::Point2d;
use vehicle::Vehicle;
use link::Link;

mod util;
mod math;
mod link;
mod vehicle;

new_key_type! {
    pub struct LinkId;
    pub struct VehicleId;
}

type LinkSet = SlotMap<LinkId, Link>;
type VehicleSet = SlotMap<VehicleId, Vehicle>;