use glam::DVec2;

use crate::BarrierKind;
use crate::barrier::Decision as BarrierDecision;
use crate::network::{ConflictKind, LaneId, Side};
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
pub(crate) trait DebugSink {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent);
}

/// What the simulation's algorithms are doing, for the debug view.
#[derive(Clone, Debug)]
pub enum DebugEvent {
    /// A longitudinal update is starting: earlier [`DebugEvent::AccelTerm`]s are stale.
    RecomputeAccels,
    /// One term the longitudinal model considered for a vehicle. The vehicle's
    /// acceleration is the minimum of its terms.
    AccelTerm { vehicle: VehicleId, source: AccelSource, accel: f64 },
    /// A motion update is starting: earlier [`DebugEvent::FrontAxle`]s are stale.
    RecomputeMotion,
    /// Where a vehicle's front axle was placed on its course, its rear axle
    /// trailing.
    FrontAxle { vehicle: VehicleId, point: DVec2 },
    /// A model's decision for a vehicle is being made: its earlier
    /// [`DebugEvent::ModelEval`]s are stale.
    RecomputeDecision { vehicle: VehicleId },
    /// Something a model weighed up for a vehicle, such as one side the
    /// lane-change model weighed up a change to, described as named fields, in
    /// order, so the debug view can show it knowing nothing of the model.
    ModelEval {
        vehicle: VehicleId,
        model: &'static str,
        fields: Vec<(&'static str, Field)>,
    },
    /// A vehicle has left the simulation.
    VehicleRemoved { vehicle: VehicleId },
    /// A vehicle's leader search, in the longitudinal update, weighed up passing
    /// another vehicle on a lane. Stale after the next
    /// [`DebugEvent::RecomputeAccels`].
    PassCheck { vehicle: VehicleId, check: PassCheck },
    /// What a vehicle, the head on its way to a barrier, the `index`th of
    /// `lane`'s, did about it in the longitudinal update. Stale after the next
    /// [`DebugEvent::RecomputeAccels`].
    Barrier {
        vehicle: VehicleId,
        lane: LaneId,
        index: u16,
        gap: f64,
        decision: BarrierDecision,
        /// The vehicle it gives way to, held for one.
        yields_to: Option<VehicleId>,
    },
}

/// The value of a field a model describes something it weighed up by (see
/// [`DebugEvent::ModelEval`]).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Field {
    Num(f64),
    Flag(bool),
    Text(&'static str),
    /// Nothing there: not measured, say.
    None,
}

impl From<f64> for Field {
    fn from(x: f64) -> Self {
        Field::Num(x)
    }
}

impl From<bool> for Field {
    fn from(b: bool) -> Self {
        Field::Flag(b)
    }
}

impl From<&'static str> for Field {
    fn from(s: &'static str) -> Self {
        Field::Text(s)
    }
}

impl<T: Into<Field>> From<Option<T>> for Field {
    fn from(x: Option<T>) -> Self {
        x.map_or(Field::None, Into::into)
    }
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
    /// A lower speed limit on a lane ahead along its path, the speed it would go
    /// there, in m/s, and the gap from its front to that lane's start: zero
    /// once its front is on it.
    LimitAhead { lane: LaneId, limit: f64, gap: f64 },
    /// A maximum speed imposed on the vehicle, in m/s: 0 holds it to a stop.
    Cap { max_speed: f64 },
    /// Slow and short of moving off, it finishes its stop or stays at rest (see
    /// `models::acceleration::stopping`).
    FinishingStop,
    /// The end of the lanes the vehicle knows it will take, as a stop line, with
    /// the gap to it.
    PathEnd { gap: f64 },
    /// A vehicle ahead of it in a junction's queue, and the conflict between
    /// them: at a crossing, the gap to the vehicle's own side of the overlap,
    /// which it waits short of; at a merge, the gap to the other vehicle as if it
    /// were on the vehicle's own lane.
    Junction { id: VehicleId, kind: ConflictKind, gap: f64 },
    /// Held out of a junction's queue, short of its first conflict there, with
    /// the gap to it: it would be both blocked by a vehicle queued only part of
    /// its way and block its next stretch.
    JunctionHold { gap: f64 },
    /// A term a model adds, named by the model, as the lane-change model's
    /// synchronisation is: the vehicle it is for and the gap to it, if any.
    Model {
        model: &'static str,
        term: &'static str,
        id: Option<VehicleId>,
        gap: Option<f64>,
    },
    /// The vehicle before it in a merge's order, with the gap between them as
    /// each is from the merge point: negative while it is the nearer.
    Merge { id: VehicleId, gap: f64 },
    /// A barrier holding the vehicle back, the `index`th of `lane`'s, with the
    /// gap from its front to the line.
    Barrier { lane: LaneId, index: u16, kind: BarrierKind, gap: f64 },
}

/// A sink that discards everything: the simulation's own, outside a debug step.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct NoopSink;

impl DebugSink for NoopSink {
    #[inline(always)]
    fn record(&mut self, _event: impl FnOnce() -> DebugEvent) {}
}

/// The sink a debug step collects into, for its caller.
impl DebugSink for Vec<DebugEvent> {
    #[inline]
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        self.push(event());
    }
}
