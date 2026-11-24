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
pub use debug::{AccelSource, DebugEvent, Field, PassCheck};
pub use following::{Lateral, Occupant, PASSING_CLEARANCE};
pub use junction::Entry;
pub use models::acceleration::{MOVING_OFF, SAFE_BRAKING, STOPPING_DECEL, STOPPING_SPEED};
pub use models::lane_change::MIN_INDICATING;
pub use models::routing::{DestinationId, Route};
pub use models::signal::{Light, MovementLight};
pub use motion::kinematics::trail;
pub use motion::lateral::LaneChange;
pub use network::geometry::{GRID, LaneError, LaneGeometry, LanePos, ParseLaneError, SAME_POINT};
pub use network::{
    ALIGNED, Barrier, BarrierKind, Conflict, ConflictKind, End, Junction, JunctionId, Keep, Lane,
    LaneId, Leaving, Line, MAX_PREDECESSORS, MAX_SUCCESSORS, Movement, MovementId, Neighbor,
    NetworkError, Phase, RoadId, Side, SignalPlan, SimNetwork, YieldsId,
};
pub use simulation::{Config, InsertError, LaneChangeError, SimState, Simulation};
pub use vehicle::{LaneChangeIntent, Placement, Vehicle, VehicleId, VehicleSpec};
