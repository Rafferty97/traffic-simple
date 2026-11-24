//! How lanes are controlled: barriers, and the signal plans that drive them.

use glam::DVec2;

/// A point on a lane that vehicles stop at until they commit to crossing it (see
/// `docs/DESIGN.md`, Barriers and signals).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Barrier {
    /// Where it is along its lane, in meters: the order barriers are met in,
    /// and where one is passed. With a line, no vehicle crossing the line
    /// along the lane reaches it before here, so stopping short of it is safe.
    pub s: f64,
    pub kind: BarrierKind,
    /// The line itself, in world space, if it isn't square across the lane at
    /// `s`: vehicles stop by how far their nearest corner is from it.
    #[cfg_attr(feature = "serde", serde(default))]
    pub line: Option<Line>,
    /// The lanes it gives way to, if any (see [`SimNetwork::yields`](super::SimNetwork::yields)): a
    /// give-way line's gap is judged against vehicles on them, and a signal's
    /// lists the turners' next stretches it lets go first.
    #[cfg_attr(feature = "serde", serde(default))]
    pub yields: Option<YieldsId>,
}

impl Barrier {
    /// A barrier square across its lane at `s`, giving way to nothing.
    pub fn new(s: f64, kind: BarrierKind) -> Self {
        Barrier { s, kind, line: None, yields: None }
    }
}

/// Identifies a set of lanes a barrier gives way to: its index in the
/// network's sets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct YieldsId(pub u32);

/// A line in world space, as a half plane: the side `normal` points to is
/// short of it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Line {
    /// A point on it.
    pub point: DVec2,
    /// Its unit normal, pointing back toward the vehicles coming up to it.
    pub normal: DVec2,
}

impl Line {
    /// How far `point` is short of the line, negative once over it.
    pub fn short_of(&self, point: DVec2) -> f64 {
        (point - self.point).dot(self.normal)
    }
}

/// What a barrier is, and so when a vehicle commits to crossing it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum BarrierKind {
    /// A signal for a movement: crossed once a vehicle can no longer stop
    /// comfortably before it, while the light is green. With `on_red`, the
    /// lanes it gives way to turning on red, it is a stop line on red: crossed
    /// once the vehicle has come to rest there and there is a gap in the
    /// traffic on them.
    Signal {
        movement: MovementId,
        #[cfg_attr(feature = "serde", serde(default))]
        on_red: Option<YieldsId>,
    },
    /// A give-way line: crossed once there is a gap in the traffic with
    /// priority.
    GiveWay,
    /// A stop line: crossed once the vehicle has come to rest there and there
    /// is a gap in the traffic with priority.
    Stop,
    /// An all-way stop line: crossed once the vehicle has come to rest there,
    /// no vehicle at a conflicting all-way stop line of its junction came to
    /// rest earlier, and there is a gap in the traffic that has committed.
    AllWayStop,
    /// The start of a keep-clear zone `length` meters long, in meters along
    /// the vehicle's way: crossed once there is room beyond its far end, and
    /// it can no longer stop comfortably before it.
    KeepClear { length: f64 },
}

/// Identifies a signal movement: its index among every controller's movements,
/// in the order they were added, which is also its light's index in the state.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MovementId(pub u32);

/// A fixed-time signal plan (see `docs/models/signals.md`).
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SignalPlan {
    /// At most 64, so that a mask of them fits a `u64`.
    pub movements: Vec<Movement>,
    /// Run in turn, round and round.
    pub phases: Vec<Phase>,
}

/// A group of lanes whose signals go and stop together.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Movement {
    /// The movements of its plan it conflicts with, by bit.
    pub conflicts: u64,
    /// How long it shows amber, in seconds.
    pub amber: f64,
    /// How long it must have been red before a movement it conflicts with turns
    /// green, in seconds: time for its traffic to clear.
    pub clearance: f64,
}

/// One phase of a signal plan.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Phase {
    /// The movements green in it, by bit.
    pub green: u64,
    /// How long it lasts once they are all green, in seconds.
    pub duration: f64,
}
