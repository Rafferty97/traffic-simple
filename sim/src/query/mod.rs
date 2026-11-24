//! Searches of the traffic beyond multi-lane car following's, for the models
//! and the core: the follower a vehicle would have, how fast each lane's
//! traffic goes, and what a lane change would do.

mod follower;
mod lane_speeds;
mod prospect;
mod speed;

pub use follower::{axle_beside, find_follower, record_index};
pub(crate) use lane_speeds::{ANTICIPATION, LaneSpeeds, friction};
pub use prospect::{Gap, Prospect};
pub(crate) use prospect::{asked, measure};
pub(crate) use speed::speed_of;
