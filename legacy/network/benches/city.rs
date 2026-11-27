//! Headless benchmarks at city scale, with the no-op debug sink: building a
//! simulation of a large grid of crossroads, and one step of a small grid
//! flooded from its edges and of a large one kept at 10,000 vehicles.

use std::hint::black_box;

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use traffic_network_legacy::{RoadNetwork, compile, scenarios};
use traffic_sim::{Driver, LaneId, Placement, Route, Simulation, VehicleSpec};

/// The length of a step, in seconds: 30 Hz.
const DT: f64 = 1.0 / 30.0;

/// A simulation of `network` stepping at 30 Hz, with the slower updates at 10 Hz.
fn sim(network: traffic_sim::SimNetwork) -> Simulation {
    let config = traffic_sim::Config { dt: DT, macro_step: 3 };
    Simulation::with_config(network, config)
}

const SPEED: f64 = 50.0 / 3.6;

/// The small grid: 4 by 4 crossroads 100 m apart, as the stress test has it.
fn small() -> RoadNetwork {
    scenarios::grid(4, 4, 2, 100.0, SPEED)
}

/// The large grid: 16 by 16 crossroads 200 m apart, city blocks of Melbourne's
/// size, with room for 10,000 vehicles to keep moving. With blocks half as
/// long, it gridlocks at that many.
fn large() -> RoadNetwork {
    scenarios::grid(16, 16, 2, 200.0, SPEED)
}

/// How many vehicles the large grid is kept at.
const VEHICLES: usize = 10_000;

fn build(c: &mut Criterion) {
    let mut group = c.benchmark_group("city_build");
    group.sample_size(10);
    let net = large();
    group.bench_function("compile_16x16", |b| {
        b.iter(|| compile(black_box(&net)).unwrap())
    });
    // The lanes' overlaps, merges and the rest derived as a simulation is built.
    group.bench_function("simulation_16x16", |b| {
        b.iter_batched(|| compile(&net).unwrap().sim, sim, BatchSize::LargeInput)
    });
    group.finish();
}

/// A grid flooded from its edges: a car tried every `every` steps at its entry
/// lanes in turn.
struct Flooded {
    sim: Simulation,
    entries: Vec<LaneId>,
    every: u64,
    n: u64,
}

impl Flooded {
    fn new(net: &RoadNetwork, every: u64) -> Self {
        let sim = sim(compile(net).unwrap().sim);
        let entries = sim
            .network()
            .lanes()
            .filter(|(_, lane)| lane.predecessors().is_empty())
            .map(|(id, _)| id)
            .collect();
        Flooded { sim, entries, every, n: 0 }
    }

    fn step(&mut self) {
        if self.n.is_multiple_of(self.every) {
            let lane = self.entries[(self.n / self.every) as usize % self.entries.len()];
            let speed = self.sim.network().lane(lane).speed_limit();
            let at = Placement { lane, s: 5.0, speed, route: Route::random(self.n) };
            let _ = self.sim.try_insert(
                VehicleSpec::passenger_car(),
                Driver { seed: self.n, ..Driver::default() },
                at,
            );
        }
        self.n += 1;
        self.sim.step_variable(black_box(DT));
    }
}

/// A grid kept at `target` vehicles: as many as leave it are tried again, at
/// rest, anywhere on its roads.
struct Kept {
    sim: Simulation,
    roads: Vec<LaneId>,
    target: usize,
    /// Xorshift64, so every run is the same.
    rng: u64,
}

impl Kept {
    fn new(net: &RoadNetwork, target: usize) -> Self {
        let sim = sim(compile(net).unwrap().sim);
        let roads = sim
            .network()
            .lanes()
            .filter(|(_, lane)| lane.junction().is_none())
            .map(|(id, _)| id)
            .collect();
        Kept { sim, roads, target, rng: 0x9E37_79B9_7F4A_7C15 }
    }

    fn next(&mut self) -> u64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        self.rng
    }

    /// Tops it up, a few tries at most, then steps.
    fn step(&mut self) {
        let missing = self.target.saturating_sub(self.sim.vehicles().count());
        for _ in 0..missing.min(200) {
            let i = self.next() % self.roads.len() as u64;
            let lane = self.roads[i as usize];
            let length = self.sim.network().lane(lane).geometry().length();
            let s = (self.next() % 1000) as f64 / 1000.0 * length;
            let seed = self.next();
            let at = Placement { lane, s, speed: 0.0, route: Route::random(seed) };
            let driver = Driver { seed, ..Driver::default() };
            let _ = self.sim.try_insert(VehicleSpec::passenger_car(), driver, at);
        }
        self.sim.step_variable(black_box(DT));
    }
}

fn step(c: &mut Criterion) {
    let mut group = c.benchmark_group("city_step");
    // More than it can take, so queues back up from one crossroads into the
    // next.
    let mut flooded = Flooded::new(&small(), (1.0 / 3.0 / DT).round() as u64);
    for _ in 0..(120.0 / DT) as usize {
        flooded.step();
    }
    group.bench_function("flooded_4x4", |b| b.iter(|| flooded.step()));
    let mut kept = Kept::new(&large(), VEHICLES);
    // Into steady traffic before measuring.
    for _ in 0..(120.0 / DT) as usize {
        kept.step();
    }
    group.bench_function("10k_16x16", |b| b.iter(|| kept.step()));
    group.finish();
}

criterion_group!(benches, build, step);
criterion_main!(benches);
