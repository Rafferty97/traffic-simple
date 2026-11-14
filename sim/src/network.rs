use crate::lane::Lane;

/// Identifies a lane: its index in [`SimNetwork`]'s lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct LaneId(pub u32);

/// The compiled network the simulation runs on.
#[derive(Clone, Debug)]
pub struct SimNetwork {
    lanes: Vec<Lane>,
    /// The lanes each lane connects to, end to start, indexed by lane.
    successors: Vec<Vec<LaneId>>,
}

impl SimNetwork {
    /// Builds a network from its lanes, with no connections. Lane `i` gets id `LaneId(i)`.
    pub fn new(lanes: Vec<Lane>) -> Self {
        let successors = vec![Vec::new(); lanes.len()];
        Self { lanes, successors }
    }

    /// Connects the end of `from` to the start of `to`. A lane may connect to itself.
    ///
    /// The end of `from` and the start of `to` should coincide; this isn't checked.
    pub fn connect(&mut self, from: LaneId, to: LaneId) {
        assert!((to.0 as usize) < self.lanes.len(), "no lane {to:?}");
        self.successors[from.0 as usize].push(to);
    }

    /// The lane with the given id. Panics if it doesn't exist.
    pub fn lane(&self, id: LaneId) -> &Lane {
        &self.lanes[id.0 as usize]
    }

    /// The lanes that `id` connects to, in the order they were connected.
    pub fn successors(&self, id: LaneId) -> &[LaneId] {
        &self.successors[id.0 as usize]
    }

    /// All lanes with their ids.
    pub fn lanes(&self) -> impl Iterator<Item = (LaneId, &Lane)> {
        self.lanes.iter().enumerate().map(|(i, lane)| (LaneId(i as u32), lane))
    }
}
