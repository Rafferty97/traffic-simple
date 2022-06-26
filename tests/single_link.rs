//! Tests that involve the simulation of a single link.

use traffic_sim::{
    math::{LineSegment2d, Point2d},
    LinkAttributes, Simulation, VehicleAttributes,
};

/// Test that a vehicle's position increases monotonically.
#[test]
fn vehicle_drives_forward() {
    let mut sim = Simulation::new();
    let link = sim.add_link(&LinkAttributes {
        curve: &LineSegment2d::from_ends(Point2d::new(0.0, 0.0), Point2d::new(100.0, 0.0)),
        speed_limit: 16.66,
    });
    let veh = sim.add_vehicle(
        &VehicleAttributes {
            length: 5.0,
            width: 2.0,
            max_acc: 2.0,
            comf_dec: 2.0,
            wheel_base: 1.5,
        },
        link,
    );

    let mut pos = sim.get_vehicle(veh).pos_mid();
    for _ in 0..100 {
        sim.step(0.1);
        let next_pos = sim.get_vehicle(veh).pos_mid();
        assert!(next_pos > pos);
        pos = next_pos;
    }
}
