//! Loads a freeze frame saved by the UI's Freeze frame button: a snapshot of the
//! simulation's whole state at one instant, and the vehicle selected then. Needs
//! the `serde` feature.
//!
//! To turn a freeze frame into a test, copy it into `sim/tests/captures/` and:
//!
//! ```ignore
//! let (mut sim, selected) = frame::load(include_str!("captures/frame-123.json"));
//! let mut sim = common::Watched::new(sim, Terms::default());
//! // …step on from that instant and assert what should happen.
//! ```
//!
//! It continues exactly as the original would have, until it inserts a vehicle
//! (see [`traffic_sim::SimState`]).

use traffic_sim::{SimState, Simulation, VehicleId};

/// The simulation in a freeze frame, and the vehicle selected.
pub fn load(json: &str) -> (Simulation, Option<VehicleId>) {
    let mut frame: serde_json::Value = serde_json::from_str(json).expect("a freeze frame is JSON");
    let snapshot: SimState =
        serde_json::from_value(frame["snapshot"].take()).expect("a freeze frame has a snapshot");
    let selected =
        serde_json::from_value(frame["selected"].take()).expect("a freeze frame has a selection");
    (Simulation::restore(snapshot), selected)
}
