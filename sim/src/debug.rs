use glam::DVec2;

use crate::models::lane_change::Outcome;
use crate::network::{LaneId, Side};
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
    /// a vehicle diverging from a lane. Stale after the next
    /// [`DebugEvent::RecomputeAccels`].
    PassCheck { vehicle: VehicleId, check: PassCheck },
}

/// Whether a vehicle can pass one diverging from a lane: that vehicle's trailing
/// rear corner against where the searching vehicle's body will be beside it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PassCheck {
    /// The diverging vehicle.
    pub leader: VehicleId,
    /// The lane it is diverging from, where the check was made.
    pub lane: LaneId,
    /// Its record's `s` on `lane`: where its trailing corner is.
    pub s: f64,
    /// The side it is leaving toward. It can only be passed on the other.
    pub leaving: Side,
    /// Its trailing rear corner's `t` on `lane`.
    pub trailing: f64,
    /// The `t` span on `lane` the searching vehicle's body will cover while
    /// passing it, right edge first.
    pub body: [f64; 2],
    /// Room between them on the passing side, in meters: negative if they overlap.
    pub clearance: f64,
    /// Whether that is enough to pass.
    pub passed: bool,
}

/// How a vehicle weighed up changing lanes to one side, under MOBIL.
#[derive(Clone, Copy, Debug)]
pub struct LaneChangeEval {
    pub side: Side,
    /// Its own acceleration in the target lane.
    pub own_after: f64,
    /// Its own acceleration there less its acceleration now.
    pub own_gain: f64,
    /// The change for its would-be new follower, once that follower has replied.
    pub new_follower: Option<f64>,
    /// The change for its current follower, once that follower has replied.
    pub old_follower: Option<f64>,
    /// MOBIL's net gain, weighing the followers' changes by politeness.
    pub incentive: f64,
    /// Whether it and its new follower would brake gently enough.
    pub safe: bool,
    pub outcome: Outcome,
}

/// What an acceleration term reacts to.
#[derive(Clone, Copy, Debug)]
pub enum AccelSource {
    /// The free road, up to the desired speed.
    FreeRoad { desired_speed: f64 },
    /// A vehicle ahead, with the bumper-to-bumper gap along the follower's path.
    Leader { id: VehicleId, gap: f64 },
    /// The vehicle is being held to a stop.
    Hold,
}

/// A sink that discards everything.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoopSink;

impl DebugSink for NoopSink {
    #[inline(always)]
    fn record(&mut self, _event: impl FnOnce() -> DebugEvent) {}
}
