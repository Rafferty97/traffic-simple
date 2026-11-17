//! Every example network compiles to a valid simulation network.

use traffic_network::{compile, scenarios};

#[test]
fn a_crossroads_with_two_lanes_each_way_compiles() {
    let compiled = compile(&scenarios::crossroads(2, 14.0)).unwrap();
    // Four arms of four lanes, and per arm two straight on, one right and one left.
    assert_eq!(compiled.road_lanes.iter().flatten().count(), 16);
    assert_eq!(compiled.connectors[0].len(), 16);
    assert_eq!(compiled.render.intersections.len(), 1);
    let junction = compiled.sim.junctions().next().unwrap().1;
    assert_eq!(junction.lanes().len(), 16);
}

#[test]
fn crossroads_of_one_and_three_lanes_each_way_compile() {
    for (lanes, connectors) in [(1, 12), (3, 20)] {
        let compiled = compile(&scenarios::crossroads(lanes, 14.0)).unwrap();
        // Per arm, one straight on from each lane, one right and one left.
        assert_eq!(compiled.connectors[0].len(), connectors);
    }
}

#[test]
fn the_freeway_merges_at_the_lane_drop_and_forks_at_the_off_ramp() {
    let compiled = compile(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6)).unwrap();
    let sim = &compiled.sim;
    // Roads: approach, parallel, drop, middle, after, on-ramp, off-ramp.
    let lanes = &compiled.road_lanes;
    let (on_ramp, accel, drop_accel, drop_left) =
        (lanes[5][0], lanes[1][0], lanes[2][0], lanes[2][1]);
    assert_eq!(sim.lane(on_ramp).successors(), [accel]);
    // Alongside, a driver can change into the left lane; on the drop, it merges.
    assert_eq!(
        sim.lane(accel).adj_right().map(|n| n.lane),
        Some(lanes[1][1])
    );
    assert!(sim.lane(drop_accel).adj_right().is_none());
    assert_eq!(
        sim.lane(drop_accel).successors(),
        sim.lane(drop_left).successors()
    );
    let junction = sim.lane(drop_accel).junction().expect("the drop is a junction");
    assert_eq!(sim.lane(drop_left).junction(), Some(junction));
    // The middle's left lane forks, left up the off-ramp and right along the freeway.
    assert_eq!(
        sim.lane(lanes[3][0]).successors(),
        [lanes[6][0], lanes[4][0]]
    );
}

#[test]
fn a_ring_compiles_into_lanes_that_lead_into_themselves() {
    let compiled = compile(&scenarios::ring(1000.0, 3, 14.0)).unwrap();
    for (id, lane) in compiled.sim.lanes() {
        assert_eq!(lane.successors(), [id]);
    }
    assert!(compiled.sim.lane(compiled.road_lanes[0][1]).adj_left().is_some());
}

/// Whether two convex bodies, given by their corners, overlap: by the separating
/// axis theorem, along the edges of each.
fn overlap(a: [glam::DVec2; 4], b: [glam::DVec2; 4]) -> bool {
    let separated = |axis: glam::DVec2| {
        let span = |c: [glam::DVec2; 4]| {
            let dots = c.map(|q| axis.dot(q));
            (
                dots.iter().copied().fold(f64::MAX, f64::min),
                dots.iter().copied().fold(f64::MIN, f64::max),
            )
        };
        let ((a0, a1), (b0, b1)) = (span(a), span(b));
        a1 < b0 || b1 < a0
    };
    let axes = [a[1] - a[0], a[2] - a[1], b[1] - b[0], b[2] - b[1]];
    !axes.iter().any(|&axis| separated(axis.perp()))
}

/// Floods a network with traffic for `seconds`, a car tried every third of a
/// second at its entry lanes in turn, asserting that no two bodies ever overlap.
fn flood(net: &traffic_network::RoadNetwork, seconds: f64) {
    use traffic_sim::{LaneId, NoopSink, Placement, Route, Simulation, VehicleSpec};
    let car = VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        speed_factor: 1.0,
        max_accel: 1.0,
        comfy_decel: 1.5,
        time_headway: 1.0,
        min_gap: 1.5,
        rear_axle: 1.0,
        front_axle: 3.7,
        courtesy: 1.0,
        politeness: 0.2,
    };
    {
        let compiled = compile(net).unwrap();
        let mut sim = Simulation::new(compiled.sim, NoopSink);
        let entries: Vec<LaneId> = sim
            .network()
            .lanes()
            .filter(|(_, lane)| lane.predecessors().is_empty())
            .map(|(id, _)| id)
            .collect();
        // A car tried every third of a second, more than the junction can take, so
        // queues build up and drivers change lanes on the way in.
        for step in 0..(seconds * 60.0) as u64 {
            if step % 20 == 0 {
                let lane = entries[(step / 20 * 7) as usize % entries.len()];
                let speed = sim.network().lane(lane).speed_limit();
                let at = Placement { lane, s: 5.0, speed, route: Route::random(step) };
                let _ = sim.try_insert(car, at);
            }
            sim.step(1.0 / 60.0);
            // Collisions last for many frames, so a check every tenth of a second
            // catches them.
            if step % 6 != 0 {
                continue;
            }
            let bodies: Vec<_> = sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
            for (i, &(a, body_a)) in bodies.iter().enumerate() {
                for &(b, body_b) in &bodies[i + 1..] {
                    assert!(
                        !overlap(body_a, body_b),
                        "{a:?} and {b:?} collide at {:.2} s",
                        sim.time()
                    );
                }
            }
        }
    }
}

#[test]
fn crossroads_flooded_with_traffic_have_no_collisions() {
    for lanes in 1..=3 {
        flood(&scenarios::crossroads(lanes, 50.0 / 3.6), 100.0);
    }
}

#[test]
fn a_freeway_with_ramps_flooded_with_traffic_has_no_collisions() {
    flood(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6), 200.0);
}
