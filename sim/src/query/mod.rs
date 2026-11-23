//! Searches of the traffic beyond multi-lane car following's, for the models
//! and the core: the follower a vehicle would have, how fast each lane's
//! traffic goes, and a vehicle's acceleration searching other lanes.

mod accel;
mod follower;
mod lane_speeds;

pub(crate) use accel::{accel_along, speed_of};
pub use follower::{axle_beside, find_follower, record_index};
pub(crate) use lane_speeds::{ANTICIPATION, LaneSpeeds, friction};
