use std::time::{Duration, Instant};

use lz4_flex::decompress_size_prepended;
use traffic_sim::Simulation;

fn main() {
    let content = std::fs::read("C:/Users/alexa/Downloads/network.stn").unwrap();
    let content = decompress_size_prepended(&content).unwrap();
    let mut sim: Simulation = bson::from_slice(&content).unwrap();

    println!("Simulating...");
    let NUM_FRAMES = 1000;
    loop {
        let start = Instant::now();
        for _ in 0..NUM_FRAMES {
            sim.step(0.05);
        }
        let frame = start.elapsed() / NUM_FRAMES;
        println!(
            "Avg. frame: {:?} --> {}x speedup ({:.0} vehs --> {:.0} vehs)",
            frame,
            0.1 / frame.as_secs_f32(),
            sim.iter_vehicles().count(),
            0.1 / frame.as_secs_f32() * (sim.iter_vehicles().count() as f32),
        )
    }
}
