//! Stop lines: a vehicle always comes to rest at its line, then goes once there
//! is a gap in the traffic with priority, as at a give-way line.

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierDecision, BarrierKind, DebugEvent, DebugSink, Lane, LaneId, NoopSink,
    Placement, Route, SimNetwork, Simulation, VehicleId,
};

mod common;
use common::car;

const LIMIT: f64 = 14.0;

fn p(x: f64, y: f64) -> DVec2 {
    DVec2::new(x, y)
}

/// A crossroads: lane 0 comes from the west and lane 1 from the south, each to
/// the junction 10 m from the origin; connectors 2 and 3 cross it to lanes 4
/// and 5. Connector 3, from the south, starts with a stop line giving way to
/// connector 2.
fn stop() -> SimNetwork {
    crossroads(false)
}

/// As [`stop`], or, with `lead_in`, with connector 3 cut 2 m in: a lead-in,
/// outside the junction, carrying the stop line, and lane 6 on from it in the
/// junction, as a skewed stop line cuts a connector.
fn crossroads(lead_in: bool) -> SimNetwork {
    let lane = |points: &[DVec2]| Lane::new(points, 3.5, LIMIT).unwrap();
    let cut = if lead_in { -8.0 } else { 10.0 };
    let mut lanes = vec![
        lane(&[p(-100.0, 0.0), p(-10.0, 0.0)]),
        lane(&[p(0.0, -100.0), p(0.0, -10.0)]),
        lane(&[p(-10.0, 0.0), p(10.0, 0.0)]),
        lane(&[p(0.0, -10.0), p(0.0, cut)]),
        lane(&[p(10.0, 0.0), p(100.0, 0.0)]),
        lane(&[p(0.0, 10.0), p(0.0, 100.0)]),
    ];
    if lead_in {
        lanes.push(lane(&[p(0.0, cut), p(0.0, 10.0)]));
    }
    let mut net = SimNetwork::new(lanes);
    let links: &[(u32, u32)] = match lead_in {
        true => &[(0, 2), (1, 3), (2, 4), (3, 6), (6, 5)],
        false => &[(0, 2), (1, 3), (2, 4), (3, 5)],
    };
    for &(from, to) in links {
        net.connect(LaneId(from), LaneId(to));
    }
    let entry = if lead_in { 6 } else { 3 };
    net.add_junction(&[LaneId(2), LaneId(entry)]);
    let yields = Some(net.add_yields(&[LaneId(2)]));
    net.add_barrier(
        LaneId(3),
        Barrier { yields, ..Barrier::new(0.0, BarrierKind::Stop) },
    );
    net
}

fn at(lane: u32, s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(lane), s, speed, route: Route::random(0) }
}

/// At rest, from the south, where a car stops for its line.
fn at_line() -> Placement {
    let spec = car();
    at(
        1,
        90.0 - (spec.length - spec.rear_axle) - spec.line_gap,
        0.0,
    )
}

/// How far north the front of the car from the south is: past its line at
/// `y > -10`.
fn front_y(sim: &Simulation, id: VehicleId) -> f64 {
    let v = sim.get(id).unwrap();
    let spec = v.spec();
    (v.pos() + (spec.length - spec.rear_axle) * DVec2::from_angle(v.heading())).y
}

#[test]
fn a_car_comes_to_rest_at_a_clear_stop_line_then_goes() {
    let mut sim = Simulation::new(stop(), NoopSink);
    let car_id = sim.insert(car(), at(1, 20.0, LIMIT));
    let mut came_to_rest = false;
    let mut crossed = None;
    while sim.time() < 30.0 {
        sim.step();
        let Some(v) = sim.get(car_id) else { break };
        if front_y(&sim, car_id) <= -10.0 {
            came_to_rest |= v.speed() == 0.0;
        } else if crossed.is_none() {
            crossed = Some(sim.time());
            assert!(came_to_rest, "it crossed without coming to rest");
        }
    }
    assert!(crossed.is_some(), "it never went");
}

#[test]
fn a_car_at_a_stop_line_waits_for_traffic_with_priority_then_goes() {
    waits_for_priority_then_goes(false);
}

#[test]
fn a_car_at_a_stop_line_on_a_lead_in_waits_for_traffic_with_priority_then_goes() {
    waits_for_priority_then_goes(true);
}

/// Each barrier decision of the last longitudinal update, and the vehicle it
/// gives way to.
#[derive(Default)]
struct Decisions(Vec<(VehicleId, BarrierDecision, Option<VehicleId>)>);

impl DebugSink for Decisions {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::RecomputeAccels => self.0.clear(),
            DebugEvent::Barrier { vehicle, decision, yields_to, .. } => {
                self.0.push((vehicle, decision, yields_to))
            }
            _ => {}
        }
    }
}

#[test]
fn a_stop_line_on_a_lead_in_gives_way_to_the_lanes_in_the_junction_beyond() {
    let mut sim = Simulation::new(crossroads(true), Decisions::default());
    let yielding = sim.insert(car(), at_line());
    let priority = sim.insert(car(), at(0, 30.0, LIMIT));
    // Long enough to have come to rest and decided again.
    while sim.time() < 0.5 {
        sim.step();
    }
    let &(_, decision, yields_to) = sim.sink().0.iter().find(|d| d.0 == yielding).unwrap();
    assert!(matches!(decision, BarrierDecision::Hold(_)), "{decision:?}");
    assert_eq!(yields_to, Some(priority));
}

fn waits_for_priority_then_goes(lead_in: bool) {
    let mut sim = Simulation::new(crossroads(lead_in), NoopSink);
    let yielding = sim.insert(car(), at_line());
    // About 5 s away: too close to go ahead of.
    let priority = sim.insert(car(), at(0, 30.0, LIMIT));
    let mut waited = true;
    let mut slowest = f64::INFINITY;
    while sim.time() < 30.0 {
        sim.step();
        if let Some(v) = sim.get(priority) {
            slowest = slowest.min(v.speed());
        }
        // It may go behind the car with priority, but not reach their crossing
        // before that car's rear has cleared it.
        let across = sim.get(priority).is_none_or(|v| v.pos().x - v.spec().rear_axle > 1.75);
        if !across && sim.get(yielding).is_some() && front_y(&sim, yielding) > -1.75 {
            waited = false;
        }
    }
    assert!(waited, "it went ahead of the car with priority");
    assert!(
        slowest > LIMIT - 0.5,
        "the car with priority slowed to {slowest:.2} m/s"
    );
    let gone = sim.get(yielding).is_none_or(|_| front_y(&sim, yielding) > 10.0);
    assert!(gone, "it never went once the way was clear");
}

#[test]
fn a_car_at_rest_at_a_stop_line_goes_through_a_wide_gap() {
    let mut sim = Simulation::new(stop(), NoopSink);
    let yielding = sim.insert(car(), at_line());
    // Far off: about 90 m from their crossing at 10 m/s, 9 s.
    let _far = sim.insert(car(), at(0, 5.0, 10.0));
    while sim.time() < 4.0 {
        sim.step();
    }
    assert!(
        front_y(&sim, yielding) > -5.0,
        "it waited for a car far off"
    );
}

#[test]
fn a_car_arriving_at_a_stop_line_records_when_it_came_to_rest() {
    let mut sim = Simulation::new(stop(), NoopSink);
    let car_id = sim.insert(car(), at(1, 60.0, LIMIT));
    // A car with priority keeps it waiting after it comes to rest.
    let _priority = sim.insert(car(), at(0, 0.0, 6.0));
    let mut recorded = false;
    while sim.time() < 12.0 && !recorded {
        sim.step();
        let v = sim.get(car_id).unwrap();
        if let Some(step) = v.stopped_at() {
            assert_eq!(
                v.speed(),
                0.0,
                "it came to rest when recorded at step {step}"
            );
            recorded = true;
        }
    }
    assert!(recorded, "it never recorded coming to rest");
}

/// A driver kept waiting grows impatient, and in time cuts in front of a car
/// with priority that has stopped short of the junction.
#[test]
fn a_car_kept_waiting_in_time_goes_ahead_of_a_stopped_car_with_priority() {
    let mut sim = Simulation::new(stop(), NoopSink);
    let yielding = sim.insert(car(), at_line());
    // At rest 5 m short of the junction, held there.
    let spec = car();
    let priority = sim.insert(spec, at(0, 85.0 - (spec.length - spec.rear_axle), 0.0));
    sim.hold(priority);
    while sim.time() < 3.0 {
        sim.step();
    }
    assert!(front_y(&sim, yielding) < -10.0, "it went at once");
    while sim.time() < 40.0 {
        sim.step();
    }
    assert!(
        sim.get(yielding).is_none_or(|_| front_y(&sim, yielding) > 10.0),
        "it never went"
    );
}
