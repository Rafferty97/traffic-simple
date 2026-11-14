use crate::lane::Lane;

/// Identifies a lane: its index in [`SimNetwork`]'s lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LaneId(pub u32);

/// The compiled network the simulation runs on.
#[derive(Clone, Debug)]
pub struct SimNetwork {
    lanes: Vec<Lane>,
}

impl SimNetwork {
    /// Builds a network from its lanes. Lane `i` gets id `LaneId(i)`.
    pub fn new(lanes: Vec<Lane>) -> Self {
        Self { lanes }
    }

    /// The lane with the given id. Panics if it doesn't exist.
    pub fn lane(&self, id: LaneId) -> &Lane {
        &self.lanes[id.0 as usize]
    }

    /// All lanes with their ids.
    pub fn lanes(&self) -> impl Iterator<Item = (LaneId, &Lane)> {
        self.lanes.iter().enumerate().map(|(i, lane)| (LaneId(i as u32), lane))
    }
}
