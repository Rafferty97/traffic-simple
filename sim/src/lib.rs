//! Core traffic simulation.
//!
//! See `docs/DESIGN.md` for the design. This crate depends on nothing else in
//! the workspace.

mod debug;
mod following;
mod junction;
mod kinematics;
mod lane;
mod lane_changing;
mod math;
mod merge;
mod models;
mod network;
mod occupancy;
mod path;
mod simulation;
mod vehicle;

pub use debug::{AccelSource, DebugEvent, DebugSink, NoopSink, PassCheck};
pub use lane::{LaneError, LaneGeometry, LanePos, ParseLaneError};
pub use models::acceleration::SAFE_BRAKING;
pub use models::lane_change::{
    D_COOP, D_FREE, D_SYNC, Incentives, LaneChangeEval, MIN_INDICATING, Outcome,
};
pub use models::lateral::LaneChange;
pub use models::routing::Route;
pub use network::{
    Conflict, ConflictKind, End, Junction, JunctionId, Keep, Lane, LaneId, Leaving,
    MAX_PREDECESSORS, MAX_SUCCESSORS, Neighbor, NetworkError, Side, SimNetwork,
};
pub use occupancy::{Lateral, Occupant};
pub use simulation::{Config, InsertError, LaneChangeError, Simulation, Snapshot};
pub use vehicle::{Intent, Placement, Vehicle, VehicleId, VehicleSpec};
