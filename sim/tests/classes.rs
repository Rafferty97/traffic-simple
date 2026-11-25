//! Vehicle classes and lane use: a vehicle keeps to the lanes its class may use,
//! and one on rails keeps to its track.

use glam::DVec2;
use traffic_sim::{
    Classes, InsertError, Lane, LaneChangeError, LaneId, Placement, Route, Side, SimNetwork,
    VehicleSpec,
};

mod common;
use common::{car, ring_road};

const SPEED_LIMIT: f64 = 14.0;

/// A straight lane from `a` to `b`.
fn lane(a: DVec2, b: DVec2) -> Lane {
    Lane::new(&[a, b], 3.5, SPEED_LIMIT, common::alone()).unwrap()
}

/// A lane, 100 m, forking into two: lane 1 for any class but trams, lane 2 a
/// tram track.
fn fork() -> SimNetwork {
    let p = DVec2::new;
    let mut network = SimNetwork::new(vec![
        lane(p(0.0, 0.0), p(100.0, 0.0)).with_allows(Classes::ALL),
        lane(p(100.0, 0.0), p(200.0, 10.0)),
        lane(p(100.0, 0.0), p(200.0, -10.0)).with_allows(Classes::TRAMS),
    ]);
    network.connect(LaneId(0), LaneId(1));
    network.connect(LaneId(0), LaneId(2));
    network
}

#[test]
fn every_vehicle_takes_only_a_branch_its_class_may_use() {
    for seed in 0..40 {
        for (spec, branch) in [(car(), LaneId(1)), (VehicleSpec::tram(), LaneId(2))] {
            let mut sim = common::sim(fork());
            let at = Placement {
                lane: LaneId(0),
                s: 20.0,
                speed: 0.0,
                route: Route::random(seed),
            };
            let id = sim.insert(spec, common::driver(), at);
            assert_eq!(
                sim.get(id).unwrap().path(),
                [LaneId(0), branch],
                "seed {seed}"
            );
        }
    }
}

#[test]
fn a_vehicle_isnt_inserted_on_a_lane_its_class_may_not_use() {
    let mut sim = common::sim(fork());
    let at = |lane| Placement { lane, s: 20.0, speed: 0.0, route: Route::default() };
    assert_eq!(
        sim.try_insert(car(), common::driver(), at(LaneId(2))),
        Err(InsertError::NotAllowed)
    );
    assert_eq!(
        sim.try_insert(VehicleSpec::tram(), common::driver(), at(LaneId(1))),
        Err(InsertError::NotAllowed)
    );
    assert!(sim.try_insert(VehicleSpec::tram(), common::driver(), at(LaneId(2))).is_ok());
}

#[test]
fn nothing_on_rails_changes_lanes() {
    // A three-lane ring every class may use, so only the rails stop it.
    let mut network = ring_road(1000.0, 3, SPEED_LIMIT);
    let lanes: Vec<LaneId> = network.lanes().map(|(id, _)| id).collect();
    for lane in lanes {
        let allowed = network.lane(lane).clone().with_allows(Classes::ALL);
        network = rebuilt_with(network, lane, allowed);
    }
    let mut sim = common::sim(network);
    let at = Placement {
        lane: LaneId(1),
        s: 0.0,
        speed: 10.0,
        route: Route::default(),
    };
    let tram = sim.insert(VehicleSpec::tram(), common::driver(), at);
    assert_eq!(
        sim.change_lane(tram, Side::Left),
        Err(LaneChangeError::NotPermitted)
    );
    for _ in 0..(60.0 / common::DT) as usize {
        sim.step();
        assert_eq!(
            sim.get(tram).unwrap().lane(),
            LaneId(1),
            "it left its track"
        );
    }
}

/// `network` with `lane` replaced by `with`, its links as they were.
fn rebuilt_with(network: SimNetwork, lane: LaneId, with: Lane) -> SimNetwork {
    let mut lanes: Vec<Lane> = network.lanes().map(|(_, l)| l.clone()).collect();
    lanes[lane.0 as usize] = with;
    let mut rebuilt = SimNetwork::new(lanes);
    for (id, l) in network.lanes() {
        for &next in l.successors() {
            rebuilt.connect(id, next);
        }
        if let Some(n) = l.adj(Side::Right) {
            let back = network.lane(n.lane).adj(Side::Left).is_some_and(|m| m.can_change);
            rebuilt.set_adjacent(id, n.lane, n.can_change, back);
        }
    }
    rebuilt
}

#[test]
fn a_vehicle_on_rails_keeps_its_rear_bogie_on_its_track_round_a_bend() {
    // A ring 30 m across, tighter than any tram track.
    let mut network = ring_road(94.0, 1, SPEED_LIMIT);
    network = rebuilt_with(
        network.clone(),
        LaneId(0),
        network.lane(LaneId(0)).clone().with_allows(Classes::ALL),
    );
    let mut sim = common::sim(network);
    let at = Placement { lane: LaneId(0), s: 0.0, speed: 5.0, route: Route::default() };
    let tram = sim.insert(VehicleSpec::tram(), common::driver(), at);
    let radius = 94.0 / std::f64::consts::TAU;
    for _ in 0..(30.0 / common::DT) as usize {
        sim.step();
        let rear = sim.get(tram).unwrap().pos();
        assert!(
            (rear.length() - radius).abs() < 0.05,
            "its rear bogie is {:.2} m off its track",
            rear.length() - radius
        );
    }
}
