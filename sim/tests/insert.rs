//! Checked insertion with `try_insert`.

use traffic_sim::{InsertError, LaneId, NoopSink, Placement, Simulation};

mod common;
use common::{car, ring};

const SPEED_LIMIT: f64 = 14.0;

fn at(s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(0), s, speed }
}

#[test]
fn inserts_into_an_empty_lane() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    let id = sim.try_insert(car(), at(10.0, SPEED_LIMIT)).unwrap();
    assert_eq!(sim.get(id).unwrap().speed(), SPEED_LIMIT);
}

#[test]
fn refuses_to_overlap() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    sim.insert(car(), at(50.0, 0.0));
    assert_eq!(
        sim.try_insert(car(), at(52.0, 0.0)),
        Err(InsertError::NoRoom)
    );
    assert_eq!(
        sim.try_insert(car(), at(48.0, 0.0)),
        Err(InsertError::NoRoom)
    );
}

#[test]
fn refuses_to_cut_in_front_of_a_fast_follower() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    // A follower at the limit, 10 m behind the spot: it would have to brake hard.
    sim.insert(car(), at(40.0, SPEED_LIMIT));
    assert_eq!(
        sim.try_insert(car(), at(55.0, 0.0)),
        Err(InsertError::NoRoom)
    );
    // Far enough ahead, and moving, it fits.
    assert!(sim.try_insert(car(), at(120.0, SPEED_LIMIT)).is_ok());
}

#[test]
fn refuses_to_appear_just_behind_a_stopped_leader_at_speed() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    sim.insert(car(), at(100.0, 0.0));
    // Its speed is capped at the leader's, so it arrives stopped and fits.
    let id = sim.try_insert(car(), at(90.0, SPEED_LIMIT)).unwrap();
    assert_eq!(sim.get(id).unwrap().speed(), 0.0);
}

#[test]
fn sees_followers_across_the_seam() {
    let mut sim = Simulation::new(ring(230.0, SPEED_LIMIT), NoopSink);
    let length = sim.network().lane(LaneId(0)).length();
    // A fast follower just before the seam; the spot is just after it.
    sim.insert(car(), at(length - 8.0, SPEED_LIMIT));
    assert_eq!(
        sim.try_insert(car(), at(4.0, 0.0)),
        Err(InsertError::NoRoom)
    );
}
