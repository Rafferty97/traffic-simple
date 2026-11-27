//! A grid of signalised and priority crossroads joined by short roads, as a
//! city has: traffic goes through one crossroads after another, and queues
//! back up from one into the next.

use traffic_network_legacy::{Control, compile, scenarios};

mod common;

const SPEED: f64 = 50.0 / 3.6;

/// The grid the tests run: 4 by 4 crossroads, 100 m apart.
fn grid(lanes: usize) -> traffic_network_legacy::RoadNetwork {
    scenarios::grid(4, 4, lanes, 100.0, SPEED)
}

#[test]
fn a_grid_compiles_into_a_junction_per_crossroads_alternating_signals_and_priority() {
    for lanes in 1..=2 {
        let net = grid(lanes);
        let compiled = compile(&net).unwrap();
        assert_eq!(compiled.sim.junctions().count(), 16);
        let signals = net
            .intersections
            .iter()
            .filter(|x| matches!(x.control, Control::Signals(_)))
            .count();
        assert_eq!(signals, 8);
        // In from each road out from the edge, every lane: 4 sides of 4.
        let entries = compiled.sim.lanes().filter(|(_, l)| l.predecessors().is_empty()).count();
        assert_eq!(entries, 16 * lanes);
    }
}

#[test]
fn a_grid_flooded_with_traffic_has_no_collisions() {
    for lanes in 1..=2 {
        common::flood(&grid(lanes), 150.0);
    }
}
