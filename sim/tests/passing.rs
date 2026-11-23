//! Passing checks, made as the follower's nose reaches the other vehicle's rear.
//! Where the nose is put is checked by snapshot tests of `Course::reach`, in
//! `occupancy`.

use traffic_sim::{
    DebugEvent, DebugSink, Lane, LaneId, PassCheck, Placement, Route, Side, SimNetwork, Simulation,
    VehicleId,
};

mod common;
use common::car;

const DT: f64 = traffic_sim::STEP;
const SPEED_LIMIT: f64 = 14.0;

/// Keeps every passing check made.
#[derive(Default)]
struct Checks(Vec<(VehicleId, PassCheck)>);

impl DebugSink for Checks {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        if let DebugEvent::PassCheck { vehicle, check } = event() {
            self.0.push((vehicle, check));
        }
    }
}

/// A car held in lane 1, `ahead` meters ahead of another moving at 10 m/s that
/// changes into lane 0, its change taking 40 m. Returns how many passing checks
/// the changer makes against the held car while straddling.
fn checks_against_a_held_car(ahead: f64) -> usize {
    let lane = |y: f64| {
        let points = [glam::DVec2::new(0.0, y), glam::DVec2::new(600.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    // Lane 0 is on the left of lane 1.
    let mut network = SimNetwork::new(vec![lane(3.5), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    let mut sim = Simulation::new(network, Checks::default());
    let changer = sim.insert(
        car(),
        Placement {
            lane: LaneId(1),
            s: 100.0,
            speed: 10.0,
            route: Route::default(),
        },
    );
    let held = sim.insert(
        car(),
        Placement {
            lane: LaneId(1),
            s: 100.0 + ahead,
            speed: 0.0,
            route: Route::default(),
        },
    );
    sim.hold(held);
    sim.change_lane(changer, Side::Left).unwrap();
    for _ in 0..(10.0 / DT) as usize {
        sim.step_variable(DT);
    }
    sim.sink().0.iter().filter(|(id, c)| *id == changer && c.leader == held).count()
}

#[test]
fn a_straddling_car_ignores_old_lane_cars_beyond_its_lane_change() {
    // The change ends about 40 m on: a car 100 m ahead on the old lane can't be
    // reached before then, so it isn't even checked.
    assert_eq!(checks_against_a_held_car(100.0), 0);
    // One 20 m ahead can, so it is.
    assert!(checks_against_a_held_car(20.0) > 0);
}
