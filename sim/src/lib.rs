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

pub use debug::{AccelSource, DebugEvent, DebugSink, LaneChangeEval, NoopSink, Outcome};
pub use lane::{Lane, LaneError, LanePos, MAX_SUCCESSORS};
pub use models::acceleration::SAFE_BRAKING;
pub use models::lateral::LaneChange;
pub use network::{LaneId, Neighbor, Side, SimNetwork};
pub use occupancy::Occupant;
pub use simulation::{InsertError, LaneChangeError, Simulation};
pub use vehicle::{Intent, PATH_LEN, Placement, Response, Vehicle, VehicleId, VehicleSpec};
