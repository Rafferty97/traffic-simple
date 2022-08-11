#![warn(missing_docs)]

//! This is a simple traffic simulation module with a focus on performance and realism.
//!
//! This is a personal project, with currently poor documentation and is not quite feature complete.
//!
//! ## Goals
//!
//! The goal of this package is to be capable of simulating any road network in a realistic manner,
//! with reasonable performance. This includes simulating things such as:
//!
//! * Car following
//! * Lane-changing decisions and dynamics
//! * Yielding at give way/stop signs
//! * Merging and diverging
//! * Simple path finding
//!
//! The primary gap right now is the lack of a lane-changing.
//! There are pieces in place, but it is in general a very hard problem to solve.
//!
//! # Hello world
//! This sample code demonstrates how to simulate a single vehicle on a straight, single-lane road.
//!
//! ```
//! use traffic_sim::{Simulation, LinkAttributes, VehicleAttributes};
//! use traffic_sim::math::{LineSegment2d, Point2d};
//!
//! // Create a simulation
//! let mut simulation = Simulation::new();
//!
//! // Add a link, which is a single lane of traffic
//! // This one is a straight line 100m long
//! let link_id = simulation.add_link(&LinkAttributes {
//!     curve: &LineSegment2d::from_ends(Point2d::new(0.0, 0.0), Point2d::new(100.0, 0.0)),
//!     speed_limit: 16.6667, // m/s (60km/h)
//! });
//!
//! // Add a vehicle to the start of the link we just created
//! let veh_id = simulation.add_vehicle(&VehicleAttributes {
//!     width: 2.0, // m
//!     length: 5.0, // m
//!     wheel_base: 1.5, // m
//!     headway: 1.5, // s
//!     max_acc: 2.0, // m
//!     comf_dec: 2.0, // m
//! }, link_id);
//!
//! // Simulate 10 frames, each advancing time by 0.1s.
//! // Each frame, print out the coordinates of our single vehicle
//! for _ in 0..10 {
//!     simulation.step(0.1);
//!     println!("Vehicle is at {:?}", simulation.get_vehicle(veh_id).position());
//! }
//! ```

pub use cgmath;
pub use group::LinkGroup;
pub use light::{LightState, TrafficLight};
pub use link::{Link, LinkAttributes, TrafficControl};
pub use simulation::Simulation;
use slotmap::{new_key_type, SlotMap};
pub use slotmap::{Key, KeyData};
pub use util::Interval;
pub use vehicle::{Vehicle, VehicleAttributes};

mod conflict;
mod debug;
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
