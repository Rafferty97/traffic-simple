//! Headless benchmark: one simulation step of compiled scenarios with junctions
//! and merges, with the no-op debug sink. Cars keep arriving at every entry
//! lane, so traffic stays as it was when measuring started.

use std::hint::black_box;

use criterion::{Criterion, criterion_group, criterion_main};
use traffic_network::{RoadNetwork, compile, scenarios};
use traffic_sim::{LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};

const DT: f64 = 1.0 / 60.0;

/// A simulation of `net`, and its entry lanes.
fn build(net: &RoadNetwork) -> (Simulation, Vec<LaneId>) {
    let sim = Simulation::new(compile(net).unwrap().sim, NoopSink);
    let entries = sim
        .network()
        .lanes()
        .filter(|(_, lane)| lane.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    (sim, entries)
}

/// One step, after trying a car every `every` steps at the entries in turn.
fn step(sim: &mut Simulation, entries: &[LaneId], every: u64, n: &mut u64) {
    if n.is_multiple_of(every) {
        let lane = entries[(*n / every) as usize % entries.len()];
        let speed = sim.network().lane(lane).speed_limit();
        let at = Placement { lane, s: 5.0, speed, route: Route::random(*n) };
        let _ = sim.try_insert(VehicleSpec::passenger_car(), at);
    }
    *n += 1;
    sim.step_variable(black_box(DT));
}

fn junctions(c: &mut Criterion) {
    let mut group = c.benchmark_group("junction_step");
    let cases = [
        // More than it can take, so queues build up on every way in.
        ("crossroads_2", scenarios::crossroads(2, 50.0 / 3.6), 20),
        (
            "signals_2",
            scenarios::signalised_crossroads(2, 50.0 / 3.6),
            20,
        ),
        ("ramps", scenarios::ramps(100.0 / 3.6, 60.0 / 3.6), 30),
    ];
    for (name, net, every) in cases {
        let (mut sim, entries) = build(&net);
        let mut n = 0;
        // Into steady traffic before measuring.
        for _ in 0..(120.0 / DT) as usize {
            step(&mut sim, &entries, every, &mut n);
        }
        group.bench_function(name, |b| b.iter(|| step(&mut sim, &entries, every, &mut n)));
    }
    group.finish();
}

criterion_group!(benches, junctions);
criterion_main!(benches);
