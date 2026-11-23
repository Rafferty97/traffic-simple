//! Junctions: groups of lanes that cross or merge, and where each pair conflicts.

use std::ops::Range;

use super::LaneId;

/// Identifies a junction: its index in [`SimNetwork`](super::SimNetwork)'s junctions.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct JunctionId(pub u32);

/// A group of lanes whose vehicles a single queue keeps apart, and where those
/// lanes conflict.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Junction {
    /// Its lanes. A lane's index here is its row and column in `conflicts`.
    pub(super) lanes: Vec<LaneId>,
    /// Where each ordered pair of its lanes conflicts, if they do: `lanes.len()²`
    /// entries, the row the lane it is seen from. Each pair is stored both ways
    /// round.
    pub(super) conflicts: Vec<Option<Conflict>>,
    /// Where each of its lanes' first conflict with the rest of it starts along
    /// it, if it has one, by its index in `lanes`: derived from `conflicts`, and
    /// so not saved but worked out again on restoring.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(super) starts: Vec<Option<f64>>,
}

impl Junction {
    /// Its lanes.
    pub fn lanes(&self) -> &[LaneId] {
        &self.lanes
    }

    /// Works out where each of its lanes' first conflict starts, from its
    /// conflicts.
    pub(super) fn derive_starts(&mut self) {
        let n = self.lanes.len();
        self.starts = (0..n)
            .map(|i| {
                let row = &self.conflicts[i * n..(i + 1) * n];
                row.iter().flatten().map(|c| c.mine.start).reduce(f64::min)
            })
            .collect();
    }
}

/// Where two lanes in a junction conflict, seen from one of them.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Conflict {
    pub kind: ConflictKind,
    /// The stretch of the lane it is seen from that overlaps the other, in `s`.
    pub mine: Range<f64>,
    /// The stretch of the other lane that overlaps it.
    pub theirs: Range<f64>,
}

/// How two lanes conflict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum ConflictKind {
    /// They lead into the same lane: vehicles from both take turns.
    Merge,
    /// They cross: vehicles on both must never be in the overlap together.
    Cross,
}
