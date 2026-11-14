//! Core traffic simulation.
//!
//! See `docs/DESIGN.md` for the design. This crate depends on nothing else in
//! the workspace.

mod debug;
mod kinematics;
mod lane;
mod math;
mod models;
mod network;
mod occupancy;
mod simulation;
mod vehicle;

pub use debug::{DebugEvent, DebugSink, NoopSink};
pub use lane::{Lane, LaneError, LanePos};
pub use network::{LaneId, SimNetwork};
pub use occupancy::Occupant;
pub use simulation::Simulation;
pub use vehicle::{Leader, PATH_LEN, Placement, Vehicle, VehicleId, VehicleSpec};
