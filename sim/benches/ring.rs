//! Headless benchmark: one simulation step of a ring at Sugiyama's density
//! (22 cars per 230 m), at several sizes, with the no-op debug sink.

use std::hint::black_box;

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use traffic_sim::NoopSink;

#[path = "../tests/common/mod.rs"]
mod common;

use common::DT;
const METERS_PER_CAR: f64 = 230.0 / 22.0;

fn ring_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("ring_step");
    for cars in [100, 1_000, 10_000] {
        let network = common::ring(cars as f64 * METERS_PER_CAR, 50.0 / 3.6);
        let mut sim = common::sim(network, NoopSink);
        common::fill_ring(&mut sim, cars);
        // Settle into moving traffic before measuring.
        for _ in 0..(20.0 / DT) as usize {
            sim.step_variable(DT);
        }
        group.bench_with_input(BenchmarkId::from_parameter(cars), &cars, |b, _| {
            b.iter(|| sim.step_variable(black_box(DT)))
        });
    }
    group.finish();
}

criterion_group!(benches, ring_step);
criterion_main!(benches);
