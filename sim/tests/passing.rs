//! Passing checks, made as the follower's nose reaches the other vehicle's rear.

use traffic_sim::{
    DebugEvent, DebugSink, Lane, LaneId, PassCheck, Placement, Side, SimNetwork, Simulation,
    VehicleId, VehicleSpec,
};

mod common;
use common::{car, ring_road};

const DT: f64 = 1.0 / 60.0;
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

#[test]
fn the_predicted_nose_stops_just_short_of_the_other_cars_rear() {
    // A busy three-lane ring, so lanes curve and plenty of vehicles change lanes,
    // with checks from both the lanes they follow and the old side.
    let mut sim = Simulation::new(ring_road(1000.0, 3, SPEED_LIMIT), Checks::default());
    let (mut checks, mut worst, mut furthest) = (0, f64::INFINITY, 0.0f64);
    for i in 0..30_000 {
        sim.step(DT);
        if i % 30 == 0 {
            let factor = 0.8 + 0.4 * ((i / 30) % 7) as f64 / 6.0;
            let spec = VehicleSpec { speed_factor: factor, ..car() };
            let at = Placement { lane: LaneId(2), s: 250.0, speed: 12.0 };
            let _ = sim.try_insert(spec, at);
        }
        for (id, check) in std::mem::take(&mut sim.sink_mut().0) {
            let v = sim.get(id).unwrap();
            let lane = sim.network().lane(v.lane());
            // Away from the seam, where `s` wraps round.
            if v.lane_pos().s > lane.length() - 100.0 {
                continue;
            }
            // Along the follower's own lane: its predicted nose, and the other
            // car's rearmost corner.
            let along = |p| lane.world_to_lane_near(p, v.lane_pos()).s;
            let nose = along(check.pos + check.tangent * (v.spec().length - v.spec().rear_axle));
            let [rear_left, rear_right, ..] = sim.get(check.leader).unwrap().corners();
            let rear = along(rear_left).min(along(rear_right));
            let short = rear - nose;
            worst = worst.min(short);
            checks += 1;
            // Close up, the projection is the whole story.
            if rear - v.lane_pos().s < 30.0 {
                furthest = furthest.max(short);
            }
        }
    }
    assert!(checks > 10_000, "only {checks} checks");
    assert!(
        worst >= 0.0,
        "a nose went {:.3} m past the other car's rear",
        -worst
    );
    assert!(
        furthest < 0.5,
        "a nose stopped {furthest:.2} m short close up"
    );
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
    let changer = sim.insert(car(), Placement { lane: LaneId(1), s: 100.0, speed: 10.0 });
    let held = sim.insert(
        car(),
        Placement { lane: LaneId(1), s: 100.0 + ahead, speed: 0.0 },
    );
    sim.hold(held);
    sim.change_lane(changer, Side::Left).unwrap();
    for _ in 0..(10.0 / DT) as usize {
        sim.step(DT);
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
