//! Lane-change decisions under MOBIL, through intents and ghost records.

use glam::DVec2;
use std::collections::HashMap;

use traffic_sim::{
    AccelSource, DebugEvent, DebugSink, Lane, LaneId, Placement, SAFE_BRAKING, SimNetwork,
    Simulation, VehicleId,
};

mod common;
use common::{Terms, car};

const DT: f64 = 1.0 / 60.0;
const SPEED_LIMIT: f64 = 14.0;

type Sim = Simulation<Terms>;

/// Two straight 500 m lanes, lane 0 on the left of lane 1, with changes allowed
/// both ways.
fn two_lanes() -> Sim {
    two_lanes_with(Terms::default())
}

fn two_lanes_with<D: DebugSink>(sink: D) -> Simulation<D> {
    let lane = |y: f64| {
        let points = [DVec2::new(0.0, y), DVec2::new(500.0, y)];
        Lane::new(&points, 3.5, SPEED_LIMIT).unwrap()
    };
    let mut network = SimNetwork::new(vec![lane(3.5), lane(0.0)]);
    network.set_adjacent(LaneId(0), LaneId(1), true, true);
    Simulation::new(network, sink)
}

fn place(lane: u32, s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(lane), s, speed }
}

fn run(sim: &mut Sim, seconds: f64, mut each: impl FnMut(&Sim)) {
    for _ in 0..(seconds / DT) as usize {
        sim.step(DT);
        each(sim);
    }
}

#[test]
fn overtakes_a_stopped_car_after_indicating() {
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), place(1, 100.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), place(1, 20.0, 10.0));

    let (mut indicated_at, mut changed_at, mut time) = (None, None, 0.0);
    run(&mut sim, 15.0, |sim| {
        time += DT;
        let v = sim.get(id).unwrap();
        if let Some(intent) = v.intent() {
            indicated_at.get_or_insert(intent.since);
        }
        if v.lane() == LaneId(0) {
            changed_at.get_or_insert(time);
        }
    });
    let indicated = indicated_at.expect("never indicated");
    let changed = changed_at.expect("never changed lanes");
    assert!(
        changed - indicated >= 1.0,
        "indicated for only {} s",
        changed - indicated
    );
    // It passed the stopped car.
    assert!(sim.get(id).unwrap().lane_pos().s > 110.0);
}

#[test]
fn waits_for_a_fast_car_in_the_target_lane() {
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), place(1, 120.0, 0.0));
    sim.hold(blocker);
    // Closing on the stopped car, so it has every reason to change at once and
    // politeness alone won't hold it back.
    let id = sim.insert(car(), place(1, 85.0, 8.0));
    // A fast car in the free lane, just behind: changing now would cut it off.
    let fast = sim.insert(car(), place(0, 65.0, SPEED_LIMIT));

    let mut hardest = 0.0f64;
    let mut changed_behind_fast = None;
    run(&mut sim, 20.0, |sim| {
        hardest = hardest.min(sim.get(fast).unwrap().accel());
        let v = sim.get(id).unwrap();
        if v.lane() == LaneId(0) && changed_behind_fast.is_none() {
            let f = sim.get(fast).unwrap();
            changed_behind_fast = Some(f.lane_pos().s > v.lane_pos().s);
        }
    });
    assert_eq!(
        changed_behind_fast,
        Some(true),
        "should change once the fast car is past"
    );
    assert!(
        hardest >= -SAFE_BRAKING,
        "the fast car braked at {hardest} m/s²"
    );
}

#[test]
fn ghosts_brake_nobody() {
    let mut sim = two_lanes();
    let blocker = sim.insert(car(), place(1, 100.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), place(1, 40.0, 6.0));
    // Close behind in the target lane, so the change waits and the ghost stays.
    let follower = sim.insert(car(), place(0, 30.0, 8.0));

    let mut indicating_frames = 0;
    run(&mut sim, 6.0, |sim| {
        if sim.get(id).unwrap().intent().is_none() {
            return;
        }
        indicating_frames += 1;
        let reacts_to_ghost = sim.sink().0.iter().any(|&(vehicle, source, _)| {
            vehicle == follower && matches!(source, AccelSource::Leader { id: l, .. } if l == id)
        });
        assert!(!reacts_to_ghost, "the follower braked for a ghost");
    });
    assert!(indicating_frames > 0, "never indicated");
}

/// Counts each vehicle's lane-change decisions.
#[derive(Default)]
struct Decisions(HashMap<VehicleId, usize>);

impl DebugSink for Decisions {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        if let DebugEvent::RecomputeDecision { vehicle } = event() {
            *self.0.entry(vehicle).or_default() += 1;
        }
    }
}

#[test]
fn every_vehicle_decides_about_five_times_a_second() {
    let mut sim = two_lanes_with(Decisions::default());
    let ids: Vec<VehicleId> =
        (0..7).map(|i| sim.insert(car(), place(i % 2, 50.0 * i as f64, 0.0))).collect();
    for _ in 0..(10.0 / DT) as usize {
        sim.step(DT);
    }
    for id in ids {
        let count = sim.sink().0[&id];
        assert!(
            (49..=51).contains(&count),
            "{id:?} decided {count} times in 10 s"
        );
    }
}

#[test]
fn a_car_far_behind_in_the_target_lane_doesnt_hold_up_a_change() {
    // A car held 300 m back in lane 0, beyond the reach of its leader search, so
    // it never replies to the ghost; and it is too far back to be affected.
    let mut sim = two_lanes();
    let far = sim.insert(car(), place(0, 50.0, 0.0));
    sim.hold(far);
    // A stopped car ahead in lane 1 makes changing left worth it.
    let blocker = sim.insert(car(), place(1, 420.0, 0.0));
    sim.hold(blocker);
    let id = sim.insert(car(), place(1, 350.0, 10.0));
    let mut changed = false;
    run(&mut sim, 5.0, |sim| {
        changed |= sim.get(id).unwrap().lane() == LaneId(0)
    });
    assert!(changed, "it waited for a reply that could never come");
}
