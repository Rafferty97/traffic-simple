use std::fs;
use std::path::Path;

use crate::simulation::Simulation;

pub mod debug;
mod geometry;
mod math;
pub mod network;
pub mod simulation;
mod util;

pub fn load_network_file(path: impl AsRef<Path>) -> anyhow::Result<Simulation> {
    let network = fs::read(path)?;
    let network = serde_json::from_slice(&network)?;

    let mut sim = Simulation::new();
    sim.insert_network(&network)?;

    Ok(sim)
}
