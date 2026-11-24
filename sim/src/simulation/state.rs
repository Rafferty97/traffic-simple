//! What a simulation is at one instant: its configuration, its state, and
//! what is derived from the state rather than saved.

use slotmap::SlotMap;

use crate::following::{self, Occupant, Overlaps};
use crate::junction::Queue;
use crate::merge::{self, Merges};
use crate::models::acceleration;
use crate::models::driver;
use crate::models::lane_change;
use crate::models::routing;
use crate::models::signal::{MovementLight, SignalState};
use crate::motion::path::{Through, recompute_through};
use crate::network::{LaneId, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// How a simulation steps, fixed when it is made.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Config {
    /// The duration of a simulation step.
    pub dt: f64,
    /// Expensive simulation logic runs at a reduced rate,
    /// every `macro_step` steps.
    pub macro_step: u32,
}

/// A simulation's whole state at one instant, network included, without its
/// debug sink or anything derived from the rest: a freeze frame.
/// [`Simulation::state`](super::Simulation::state) lends it, and [`Simulation::restore`](super::Simulation::restore) turns it back
/// into a simulation, exactly as it was. With the `serde` feature, it
/// serializes, for saving to a file.
///
/// The searches read it, kept apart from the rest of the simulation, so that
/// an update can lend it to them while writing elsewhere, such as to the debug
/// sink.
///
/// A restored simulation continues bit for bit like the original, through steps,
/// holds, lane changes and removals, until it inserts a vehicle into a slot freed
/// before the snapshot. The vehicle store's free slots come back in a different
/// order, so that vehicle can get a different id from the one it would have had,
/// and the update order can differ from then on.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SimState {
    pub(crate) network: SimNetwork,
    pub(crate) vehicles: SlotMap<VehicleId, Vehicle>,
    /// The driver model: each driver's traits, and the state they vary with.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) drivers: driver::Model,
    /// The longitudinal model, with its state for each vehicle.
    pub(crate) longitudinal: acceleration::Model,
    /// The lane-change model, with its state for each vehicle.
    pub(crate) lane_changing: lane_change::Model,
    /// The routing model: how each vehicle chooses the lanes it adds to its
    /// path, with any state that needs.
    pub(crate) routing: routing::Model,
    /// Each lane's occupants sorted by `s`, indexed by lane.
    pub(crate) occupants: Vec<Vec<Occupant>>,
    /// Each junction's queue, by junction.
    pub(crate) queues: Vec<Queue>,
    /// Vehicles with a speed imposed on them, and the most each may go, in m/s: 0
    /// holds one to a stop. Few, so a list beats a field on every vehicle.
    pub(crate) caps: Vec<(VehicleId, f64)>,
    /// Vehicles to remove: those that left the network in a step's motion,
    /// removed at its end, and those [`Simulation::remove`](super::Simulation::remove)
    /// queued, removed first thing in the next step.
    pub(crate) removals: Vec<VehicleId>,
    /// How it steps.
    pub(crate) config: Config,
    /// Number of simulation steps that have occured since the simulation began.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) frame: u64,
    /// Time [`Simulation::step_variable`](super::Simulation::step_variable) was given but hasn't stepped yet, in
    /// seconds: less than a step.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) residual: f64,
    /// Each signal controller's place in its plan, by controller.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) signals: Vec<SignalState>,
    /// Every signal movement's light, by [`MovementId`](crate::network::MovementId).
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) lights: Vec<MovementLight>,
    /// The destinations registered, each a set of lanes, by
    /// [`DestinationId`](routing::DestinationId).
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) destinations: Vec<Vec<LaneId>>,
}

impl SimState {
    /// Simulated time since the simulation was created, in seconds.
    pub fn time(&self) -> f64 {
        self.time_since(0)
    }

    /// Simulated time since the give `frame`.
    pub(crate) fn time_since(&self, frame: u64) -> f64 {
        self.config.dt * self.frame.saturating_sub(frame) as f64
    }
}

#[cfg(test)]
impl SimState {
    pub(crate) fn macro_step_dt(&self) -> f64 {
        self.config.dt * (self.config.macro_step as f64)
    }

    pub(crate) fn set_time(&mut self, time: f64) {
        self.frame = (time / self.config.dt) as u64;
    }
}

/// What the simulation derives from its state, rebuilt rather than saved.
pub(crate) struct Derived {
    /// Each lane's overlapping lanes outside junctions, from the network.
    pub overlaps: Overlaps,
    /// Which merge each lane belongs to, from the network and the overlaps.
    pub merges: Merges,
    /// How far beyond each lane's end the paths through it reach (see [`Through`]).
    pub through: Through,
    /// Each destination's route table, by destination.
    pub routes: routing::Tables,
}

impl Derived {
    pub(super) fn new(state: &SimState) -> Self {
        let network = &state.network;
        let overlaps = following::overlaps(network);
        let merges = merge::merges(network, &overlaps);
        let mut through = Through::new();
        let paths = state.vehicles.values().map(|v| v.path.as_slice());
        recompute_through(network, &mut through, paths);
        let routes = routing::tables(network, &state.destinations);
        Self { overlaps, merges, through, routes }
    }
}
