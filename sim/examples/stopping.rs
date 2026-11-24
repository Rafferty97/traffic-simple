//! Measures how vehicles finish their stops behind a stationary leader, and
//! move off again, for tuning the stopping state (see "Finishing a stop" in
//! `docs/models/longitudinal.md`): where they come to rest against IDM's
//! minimum gap, whether they creep on after coming to rest, how long the last
//! few km/h take, and how soon a queue moves off once its head goes.
//!
//! `cargo run --release -p traffic-sim --example stopping`

use glam::DVec2;
use traffic_sim::{
    Barrier, BarrierKind, Config, Driver, Lane, LaneId, Movement, Phase, Placement, RoadId, Route,
    SignalPlan, SimNetwork, Simulation, VehicleId, VehicleSpec,
};

const DT: f64 = 1.0 / 30.0;
const LIMIT: f64 = 50.0 / 3.6;

fn sim() -> Simulation {
    let lane = Lane::new(
        &[DVec2::ZERO, DVec2::new(2000.0, 0.0)],
        3.5,
        LIMIT,
        RoadId(0),
    )
    .unwrap();
    Simulation::with_config(
        SimNetwork::new(vec![lane]),
        Config { dt: DT, macro_step: 3 },
    )
}

/// The lane, with a light at 400 m that stays red: its movement is never green.
fn red_light() -> Simulation {
    let lane = Lane::new(
        &[DVec2::ZERO, DVec2::new(2000.0, 0.0)],
        3.5,
        LIMIT,
        RoadId(0),
    )
    .unwrap();
    let mut network = SimNetwork::new(vec![lane]);
    let movement = Movement { conflicts: 0b10, amber: 3.0, clearance: 2.0 };
    let plan = SignalPlan {
        movements: vec![movement, Movement { conflicts: 0b01, ..movement }],
        phases: vec![Phase { green: 0b10, duration: 1.0e6 }],
    };
    let movement = network.add_signal(plan);
    network.add_barrier(
        LaneId(0),
        Barrier::new(400.0, BarrierKind::Signal { movement, on_red: None }),
    );
    Simulation::with_config(network, Config { dt: DT, macro_step: 3 })
}

fn car() -> VehicleSpec {
    VehicleSpec::passenger_car()
}

/// A driver of textbook habits, with its dawdling or without, its generator
/// seeded by `seed`.
fn driver(noise: bool, seed: u64) -> Driver {
    let driver = Driver::default();
    let accel_noise = if noise { driver.accel_noise } else { 0.0 };
    Driver { accel_noise, seed, ..driver }
}

fn place(s: f64, speed: f64) -> Placement {
    Placement { lane: LaneId(0), s, speed, route: Route::default() }
}

/// The bumper-to-bumper gap from `follower` to `leader`.
fn gap(sim: &Simulation, follower: VehicleId, leader: VehicleId) -> f64 {
    let (f, l) = (sim.get(follower).unwrap(), sim.get(leader).unwrap());
    let rear = |v: &traffic_sim::Vehicle| v.lane_pos().s - v.spec().rear_axle;
    rear(l) - (rear(f) + f.spec().length)
}

/// How one vehicle's stops went.
#[derive(Default)]
struct Stops {
    /// The gap at which it first came to rest.
    first_rest: Option<f64>,
    /// How many times it moved off and came to rest again with its leader still.
    creeps: u32,
    /// How far it crept in all.
    crept: f64,
    /// How long it took from below 3 km/h to rest, in seconds.
    finishing: f64,
    was_resting: bool,
    rest_s: f64,
    slow_since: Option<f64>,
}

impl Stops {
    fn observe(&mut self, sim: &Simulation, id: VehicleId, leader: VehicleId) {
        let v = sim.get(id).unwrap();
        let resting = v.speed() == 0.0;
        if v.speed() < 3.0 / 3.6 && self.slow_since.is_none() && self.first_rest.is_none() {
            self.slow_since = Some(sim.time());
        }
        if resting && !self.was_resting {
            match self.first_rest {
                None => {
                    self.first_rest = Some(gap(sim, id, leader));
                    self.finishing = sim.time() - self.slow_since.unwrap_or(sim.time());
                }
                Some(_) => {
                    self.creeps += 1;
                    self.crept += v.lane_pos().s - self.rest_s;
                }
            }
            self.rest_s = v.lane_pos().s;
        }
        self.was_resting = resting;
    }
}

fn main() {
    let min_gap = VehicleSpec::passenger_car().min_gap;
    println!(
        "Settings: stopping below {:.1} km/h, braking at least {} m/s², moving off at {} m/s²\n",
        traffic_sim::STOPPING_SPEED * 3.6,
        traffic_sim::STOPPING_DECEL,
        traffic_sim::MOVING_OFF,
    );
    for noise in [false, true] {
        println!("Acceleration noise {}:", if noise { "on" } else { "off" });
        // One vehicle coming up to a stationary leader from each speed.
        for speed in [2.0, 6.0, 14.0] {
            let mut sim = sim();
            let leader = sim.insert(car(), driver(false, 1), place(400.0, 0.0));
            sim.hold(leader);
            let id = sim.insert(
                car(),
                driver(noise, 7),
                place(400.0 - 4.0 * speed * speed - 30.0, speed),
            );
            let mut stops = Stops::default();
            for _ in 0..(90.0 / DT) as usize {
                sim.step();
                stops.observe(&sim, id, leader);
            }
            println!(
                "  from {:>4.1} km/h: first at rest {:+.2} m from the minimum gap after {:.1} s \
                 finishing; {} creeps, {:.2} m; finally {:+.2} m",
                speed * 3.6,
                stops.first_rest.map_or(f64::NAN, |g| g - min_gap),
                stops.finishing,
                stops.creeps,
                stops.crept,
                gap(&sim, id, leader) - min_gap,
            );
        }
        // One vehicle coming up to a red light from each speed: its front against
        // its own gap to lines, where IDM would settle.
        for (speed, line_gap) in [(2.0, 1.0), (6.0, 1.0), (14.0, 1.0), (2.0, 0.5), (14.0, 0.5)] {
            let mut sim = red_light();
            // Its gap to lines exactly so, no spread.
            let driver = Driver { line_gap, line_gap_spread: 0.0, ..driver(noise, 7) };
            let id = sim.insert(
                car(),
                driver,
                place(400.0 - 4.0 * speed * speed - 30.0, speed),
            );
            let short = |sim: &Simulation| {
                let v = sim.get(id).unwrap();
                let front = v.lane_pos().s - v.spec().rear_axle + v.spec().length;
                400.0 - front - sim.line_gap(id).unwrap()
            };
            let (mut first, mut creeps, mut was, mut last) = (None, 0, false, 0.0);
            let (mut slow, mut finishing) = (None, f64::NAN);
            for _ in 0..(90.0 / DT) as usize {
                sim.step();
                let speed_now = sim.get(id).unwrap().speed();
                if speed_now < 3.0 / 3.6 && slow.is_none() {
                    slow = Some(sim.time());
                }
                let resting = speed_now == 0.0;
                if resting && !was {
                    match first {
                        None => {
                            first = Some(short(&sim));
                            finishing = sim.time() - slow.unwrap_or(sim.time());
                        }
                        Some(_) => creeps += 1,
                    }
                }
                was = resting;
                last = short(&sim);
            }
            println!(
                "  at a red light from {:>4.1} km/h, {line_gap} m short: first at rest {:+.2} m from \
                 its line gap after {finishing:.1} s finishing; {creeps} creeps; finally {last:+.2} m",
                speed * 3.6,
                first.unwrap_or(f64::NAN),
            );
        }
        // A queue of ten arriving at the limit two seconds apart, then let go.
        let mut sim = sim();
        let head = sim.insert(car(), driver(false, 1), place(600.0, 0.0));
        sim.hold(head);
        let mut queue = vec![head];
        let mut stops: Vec<Stops> = Vec::new();
        let mut passed = [None; 11];
        let release = (120.0 / DT) as usize;
        for step in 0..(180.0 / DT) as usize {
            if step % (2.0 / DT) as usize == 0 && queue.len() < 11 {
                let id = sim.insert(car(), driver(noise, 100 + step as u64), place(0.0, LIMIT));
                queue.push(id);
                stops.push(Stops::default());
            }
            if step == release {
                sim.release(head);
            }
            sim.step();
            if step < release {
                for (k, s) in stops.iter_mut().enumerate() {
                    s.observe(&sim, queue[k + 1], queue[k]);
                }
            }
            // Moving off: when each passes 650 m, 50 m beyond where the head waited.
            for (k, &id) in queue.iter().enumerate() {
                if passed[k].is_none() && sim.get(id).is_some_and(|v| v.lane_pos().s > 650.0) {
                    passed[k] = Some(sim.time() - release as f64 * DT);
                }
            }
        }
        let creeps: u32 = stops.iter().map(|s| s.creeps).sum();
        let short = stops.iter().filter_map(|s| s.first_rest).map(|g| g - min_gap);
        let (lo, hi) = short.fold((f64::MAX, f64::MIN), |(lo, hi), g| (lo.min(g), hi.max(g)));
        println!(
            "  queue of 10: first at rest {lo:+.2} to {hi:+.2} m from the minimum gap; {creeps} creeps"
        );
        let times: Vec<String> =
            passed.iter().map(|t| t.map_or("–".into(), |t| format!("{t:.1}"))).collect();
        println!(
            "  moving off, each past 50 m beyond the head's start, s: {}\n",
            times.join(" ")
        );
    }
}
