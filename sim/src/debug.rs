use glam::DVec2;

use crate::models::lane_change::LaneChangeEval;
use crate::network::{ConflictKind, Side};
use crate::vehicle::VehicleId;

/// Receives debug events from the simulation.
///
/// Events are built inside a closure, so a sink that ignores them costs nothing.
///
/// Events are grouped by the update that produces them. Each update first records
/// its group's reset event, then that group's events, so a sink that accumulates
/// events and clears a group on its reset always holds what the last update of
/// that group used.
///
/// Lane-change decisions are staggered across vehicles, so their reset is per
/// vehicle, and [`DebugEvent::VehicleRemoved`] says when to drop a vehicle's.
pub trait DebugSink {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent);
}

/// What the simulation's algorithms are doing, for the debug view.
#[derive(Clone, Copy, Debug)]
pub enum DebugEvent {
    /// A longitudinal update is starting: earlier [`DebugEvent::AccelTerm`]s are stale.
    RecomputeAccels,
    /// One term the longitudinal model considered for a vehicle. The vehicle's
    /// acceleration is the minimum of its terms.
    AccelTerm { vehicle: VehicleId, source: AccelSource, accel: f64 },
    /// A motion update is starting: earlier [`DebugEvent::Lookahead`]s are stale.
    RecomputeMotion,
    /// The point a vehicle's path tracker is steering toward.
    Lookahead { vehicle: VehicleId, point: DVec2 },
    /// A vehicle's lane-change decision is being made: its earlier
    /// [`DebugEvent::LaneChangeEval`]s are stale.
    RecomputeDecision { vehicle: VehicleId },
    /// One side a vehicle weighed up for a lane change.
    LaneChangeEval { vehicle: VehicleId, eval: LaneChangeEval },
    /// A vehicle has left the simulation.
    VehicleRemoved { vehicle: VehicleId },
    /// A vehicle's leader search, in the longitudinal update, weighed up passing
    /// another vehicle on a lane. Stale after the next
    /// [`DebugEvent::RecomputeAccels`].
    PassCheck { vehicle: VehicleId, check: PassCheck },
}

/// Whether a vehicle can pass another ahead of it on a lane, checked in world
/// space: where the searching vehicle will be as its front reaches the other's
/// rear, against the other's rear corners, as if it drove straight ahead.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PassCheck {
    /// The vehicle that might be passed.
    pub leader: VehicleId,
    /// Where the searching vehicle's rear axle is predicted to be then.
    pub pos: DVec2,
    /// Its predicted heading there, as a unit vector.
    pub tangent: DVec2,
    /// The side of the other vehicle it would pass on.
    pub side: Side,
    /// The other vehicle's rear corner nearer that side.
    pub corner: DVec2,
    /// Room between that corner and the searching vehicle's near edge, in meters:
    /// negative if they overlap.
    pub clearance: f64,
    /// Whether that is enough to pass.
    pub passed: bool,
}

/// What an acceleration term reacts to.
#[derive(Clone, Copy, Debug)]
pub enum AccelSource {
    /// The free road, up to the desired speed.
    FreeRoad { desired_speed: f64 },
    /// A vehicle ahead, with the bumper-to-bumper gap along the follower's path,
    /// and the share of its time headway the follower keeps to it as relaxation
    /// has it: less than 1 just after either appeared in the lane.
    Leader { id: VehicleId, gap: f64, headway: f64 },
    /// A maximum speed imposed on the vehicle, in m/s: 0 holds it to a stop.
    Cap { max_speed: f64 },
    /// Making room for a vehicle indicating into its lane ahead, as if that
    /// vehicle's ghost were a leader, with the gap to it.
    Yield { id: VehicleId, gap: f64 },
    /// The end of the lanes the vehicle knows it will take, as a stop line, with
    /// the gap to it.
    PathEnd { gap: f64 },
    /// A vehicle ahead of it in a junction's queue, and the conflict between
    /// them: at a crossing, the gap to the vehicle's own side of the overlap,
    /// which it waits short of; at a merge, the gap to the other vehicle as if it
    /// were on the vehicle's own lane.
    Junction { id: VehicleId, kind: ConflictKind, gap: f64 },
    /// The vehicle before it in a merge's order, with the gap between them as
    /// each is from the merge point: negative while it is the nearer.
    Merge { id: VehicleId, gap: f64 },
}

/// A sink that discards everything.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoopSink;

impl DebugSink for NoopSink {
    #[inline(always)]
    fn record(&mut self, _event: impl FnOnce() -> DebugEvent) {}
}
