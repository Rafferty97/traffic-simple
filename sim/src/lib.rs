//! Core traffic simulation.
//!
//! See `docs/DESIGN.md` for the design. This crate depends on nothing else in
//! the workspace.

mod barrier;
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

pub use barrier::{AMBER_DECEL, Decision as BarrierDecision};
pub use debug::{AccelSource, DebugEvent, DebugSink, NoopSink, PassCheck};
pub use junction::Entry;
pub use lane::{GRID, LaneError, LaneGeometry, LanePos, ParseLaneError, SAME_POINT};
pub use models::acceleration::SAFE_BRAKING;
pub use models::lane_change::{
    D_COOP, D_FREE, D_SYNC, Incentives, LaneChangeEval, MIN_INDICATING, Outcome,
};
pub use models::lateral::LaneChange;
pub use models::routing::Route;
pub use models::signal::{Light, MovementLight};
pub use network::{
    Barrier, BarrierKind, Conflict, ConflictKind, End, Junction, JunctionId, Keep, Lane, LaneId,
    Leaving, Line, MAX_PREDECESSORS, MAX_SUCCESSORS, Movement, MovementId, Neighbor, NetworkError,
    Phase, Side, SignalPlan, SimNetwork,
};
pub use occupancy::{Lateral, Occupant, PASSING_CLEARANCE};
pub use simulation::{Config, InsertError, LaneChangeError, SimState, Simulation};
pub use vehicle::{Intent, Placement, Vehicle, VehicleId, VehicleSpec};
