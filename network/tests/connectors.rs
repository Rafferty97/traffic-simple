//! Junction connectors: as wide as the widest vehicle sweeps along them, plus
//! the room one needs to pass another on each side, unless the network says
//! otherwise.

mod common;

use glam::DVec2;
use traffic_network::{Turn, compile, scenarios};
use traffic_sim::{LaneId, NoopSink, PASSING_CLEARANCE, Placement, Route, SimNetwork, VehicleSpec};

/// Whether `point` lies within `lane`'s outline, or just beyond an end: where
/// a curve's first or last segment turns a little from the lane it meets, their
/// square ends leave a sliver between them on the outside.
fn within(net: &SimNetwork, lane: LaneId, point: DVec2) -> bool {
    const SEAM: f64 = 0.1;
    let geometry = net.lane(lane).geometry();
    let pos = geometry.world_to_lane(point);
    (-SEAM..=geometry.length() + SEAM).contains(&pos.s) && geometry.between_edges(pos)
}

#[test]
fn a_car_keeps_its_body_within_every_connector_it_takes() {
    let car = VehicleSpec::passenger_car();
    for lanes in 1..=3 {
        let compiled = compile(&scenarios::crossroads(lanes, 50.0 / 3.6)).unwrap();
        for &connector in &compiled.connectors[0] {
            let net = compiled.sim.clone();
            let lane = net.lane(connector);
            let (before, after) = (lane.predecessors()[0], lane.successors()[0]);
            let limit = lane.speed_limit();
            // Briskly and slowly, since pure pursuit looks further ahead the
            // faster it goes.
            for speed in [limit, 2.0] {
                let mut sim = common::sim(net.clone(), NoopSink);
                let at = Placement { lane: connector, s: 0.0, speed, route: Route::random(0) };
                let id = sim.insert(car, at);
                sim.cap_speed(id, speed);
                while sim.get(id).is_some_and(|v| v.lane() == connector) {
                    sim.step();
                    let Some(v) = sim.get(id) else { break };
                    for corner in v.corners() {
                        assert!(
                            [before, connector, after].iter().any(|&l| within(&net, l, corner)),
                            "{lanes} lanes: on {connector:?} at {speed:.1} m/s, a corner at \
                             {corner:?} is outside its lanes, {:.2} m along",
                            v.lane_pos().s,
                        );
                    }
                }
            }
        }
    }
}

#[test]
fn a_straight_connector_is_a_vehicle_wide_plus_room_to_pass_each_side() {
    let car = VehicleSpec::passenger_car();
    let compiled = compile(&scenarios::crossroads(2, 50.0 / 3.6)).unwrap();
    let half = 0.5 * car.width + PASSING_CLEARANCE;
    for (&connector, &turn) in compiled.connectors[0].iter().zip(&compiled.turns[0]) {
        let widths = compiled.sim.lane(connector).geometry().half_widths();
        if turn == Turn::Straight {
            assert!(
                widths.iter().all(|w| (w[0] - half).abs() < 1e-3 && (w[1] - half).abs() < 1e-3)
            );
        } else {
            // A turn widens most on its inner side, where its rear axle cuts
            // inside its front's course, and less on its outer side, where its
            // front corner swings out.
            let inner = if turn == Turn::Left { 0 } else { 1 };
            let most = |e: usize| widths.iter().map(|w| w[e]).fold(0.0, f64::max);
            assert!(most(inner) > half + 0.2, "{turn:?}: {widths:?}");
            assert!(most(inner) > most(1 - inner), "{turn:?}: {widths:?}");
        }
    }
}

#[test]
fn connectors_side_by_side_at_a_mouth_do_not_conflict() {
    // Two lanes each way: every connector from an approach lane is beside
    // those from the other, 3.5 m apart, at the mouth.
    let compiled = compile(&scenarios::signalised_crossroads(2, 50.0 / 3.6)).unwrap();
    let net = &compiled.sim;
    for (&a, &b) in compiled.connectors[0]
        .iter()
        .flat_map(|a| compiled.connectors[0].iter().map(move |b| (a, b)))
    {
        let (la, lb) = (net.lane(a), net.lane(b));
        let side_by_side = la.predecessors()[0] != lb.predecessors()[0]
            && la
                .geometry()
                .centerline()
                .next()
                .unwrap()
                .distance(lb.geometry().centerline().next().unwrap())
                < 3.6;
        if side_by_side && let Some(conflict) = net.conflict_between(a, b) {
            assert!(
                conflict.mine.start > 1.0,
                "{a:?} and {b:?} conflict from {:?}",
                conflict.mine
            );
        }
    }
}

#[test]
fn a_network_may_set_its_connectors_width() {
    let mut net = scenarios::crossroads(1, 50.0 / 3.6);
    net.intersections[0].connector_width = Some(3.2);
    let compiled = compile(&net).unwrap();
    for &connector in &compiled.connectors[0] {
        let widths = compiled.sim.lane(connector).geometry().half_widths();
        assert!(widths.iter().all(|w| *w == [1.6, 1.6]), "{widths:?}");
    }
}

#[test]
fn far_side_turns_run_straight_on_before_turning_and_nothing_else_does() {
    // Arms fitted for the longer run, and kept for both.
    let mut fitted = scenarios::crossroads(2, 50.0 / 3.6);
    fitted.intersections[0].far_side_straight = 5.0;
    let some = traffic_network::fit_arms(&mut fitted).unwrap();
    fitted.intersections[0].far_side_straight = 0.0;
    let none = compile(&fitted).unwrap();
    for (i, &turn) in some.turns[0].iter().enumerate() {
        let geometry =
            |c: &traffic_network::Compiled| c.sim.lane(c.connectors[0][i]).geometry().clone();
        let (plain, run) = (geometry(&none), geometry(&some));
        let points: Vec<DVec2> = run.centerline().collect();
        if turn == Turn::Right {
            // Driving on the left, right turns cross oncoming traffic: from its
            // lane's end, along the stretch to the intersection and then the
            // length given, it runs straight on along its approach.
            let from = some.sim.lane(some.connectors[0][i]).predecessors()[0];
            let ahead: Vec<DVec2> = some.sim.lane(from).geometry().centerline().collect();
            let along = (ahead[ahead.len() - 1] - ahead[ahead.len() - 2]).normalize();
            let road = some.road_lanes.iter().position(|ids| ids.contains(&from)).unwrap();
            let stretch = fitted.intersections[0].arm(traffic_network::RoadId(road)).stop_line;
            let on_line = |p: DVec2| (p - points[0]).perp_dot(along).abs() < 1e-3;
            let straight = points.iter().take_while(|&&p| on_line(p)).last().unwrap();
            let length = straight.distance(points[0]);
            // At least that far: the quintic after it leaves along the same line,
            // within a millimetre of it for a while.
            assert!(length > stretch + 5.0 - 2e-3, "it runs on {length:.3} m");
            assert!(run.length() > plain.length(), "it isn't any longer");
        } else {
            assert_eq!(run.to_mm_string(), plain.to_mm_string(), "{turn:?} changed");
        }
    }
}
