//! Core traffic simulation.
//!
//! See `docs/DESIGN.md` for the design. This crate depends on nothing else in
//! the workspace.

mod barrier;
mod debug;
mod following;
mod junction;
mod merge;
mod models;
mod motion;
mod network;
mod query;
mod simulation;
mod vehicle;

pub use barrier::{AMBER_DECEL, Decision as BarrierDecision};
pub use debug::{AccelSource, DebugEvent, DebugSink, NoopSink, PassCheck};
pub use following::{Lateral, Occupant, PASSING_CLEARANCE};
pub use junction::Entry;
pub use models::acceleration::{SAFE_BRAKING, STOPPING_DECEL, STOPPING_SPEED};
pub use models::lane_change::{
    D_COOP, D_FREE, D_SYNC, Incentives, LaneChangeEval, MIN_INDICATING, Outcome,
};
pub use models::routing::Route;
pub use models::signal::{Light, MovementLight};
pub use motion::kinematics::trail;
pub use motion::lateral::LaneChange;
pub use network::geometry::{GRID, LaneError, LaneGeometry, LanePos, ParseLaneError, SAME_POINT};
pub use network::{
    ALIGNED, Barrier, BarrierKind, Conflict, ConflictKind, End, Junction, JunctionId, Keep, Lane,
    LaneId, Leaving, Line, MAX_PREDECESSORS, MAX_SUCCESSORS, Movement, MovementId, Neighbor,
    NetworkError, Phase, Side, SignalPlan, SimNetwork, YieldsId,
};
pub use simulation::{Config, InsertError, LaneChangeError, SimState, Simulation};
pub use vehicle::{Intent, Placement, Vehicle, VehicleId, VehicleSpec};
