//! Core traffic simulation.
//!
//! See `docs/DESIGN.md` for the design. This crate depends on nothing else in
//! the workspace.

mod debug;
mod following;
mod kinematics;
mod lane;
mod math;
mod merge;
mod models;
mod network;
mod occupancy;
mod simulation;
mod vehicle;

pub use debug::{AccelSource, DebugEvent, DebugSink, NoopSink, PassCheck};
pub use lane::{LaneError, LaneGeometry, LanePos, ParseLaneError};
pub use models::acceleration::SAFE_BRAKING;
pub use models::lane_change::{
    INTENT as LANE_CHANGE_INTENT, LaneChangeEval, MIN_INDICATING, Outcome,
};
pub use models::lateral::LaneChange;
pub use models::routing::Route;
pub use network::{
    Conflict, ConflictKind, End, Junction, JunctionId, Lane, LaneId, MAX_PREDECESSORS,
    MAX_SUCCESSORS, Neighbor, NetworkError, Side, SimNetwork,
};
pub use occupancy::{Lateral, Occupant};
pub use simulation::{InsertError, LaneChangeError, Simulation, Snapshot};
pub use vehicle::{Intent, Placement, Vehicle, VehicleId, VehicleSpec};
