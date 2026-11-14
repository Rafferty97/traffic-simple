use glam::DVec2;

use crate::vehicle::VehicleId;

/// Receives debug events from the simulation.
///
/// Events are built inside a closure, so a sink that ignores them costs nothing.
///
/// Events are grouped by the update that produces them. Each update first records
/// its group's reset event, then that group's events, so a sink that accumulates
/// events and clears a group on its reset always holds what the last update of
/// that group used.
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
    /// The point a vehicle's path tracker is steering towards.
    Lookahead { vehicle: VehicleId, point: DVec2 },
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
