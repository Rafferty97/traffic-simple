use glam::DVec2;

use crate::vehicle::VehicleId;

/// Receives debug events from the simulation.
///
/// Events are built inside a closure, so a sink that ignores them costs nothing.
pub trait DebugSink {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent);
}

/// What the simulation's algorithms are doing, for the debug view.
#[derive(Clone, Debug)]
pub enum DebugEvent {
    /// The point a vehicle's path tracker is steering towards.
    Lookahead { vehicle: VehicleId, point: DVec2 },
}

/// A sink that discards everything.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoopSink;

impl DebugSink for NoopSink {
    #[inline(always)]
    fn record(&mut self, _event: impl FnOnce() -> DebugEvent) {}
}

/// Records every event. The caller clears it between steps.
impl DebugSink for Vec<DebugEvent> {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        self.push(event());
    }
}
