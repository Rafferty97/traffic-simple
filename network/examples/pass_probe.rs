//! Experiment: runs several scenarios headless and reports collisions, hard
//! braking, throughput, passing checks and step time. Temporary.

use std::f64::consts::TAU;
use std::time::{Duration, Instant};

use glam::DVec2;
use traffic_network::{Direction, Piece, Pose, Road, RoadLane, RoadNetwork, compile, scenarios};
use traffic_sim::{
    DebugEvent, DebugSink, Keep, LaneId, NoopSink, Placement, Route, Side, Simulation, VehicleId,
    VehicleSpec,
};

/// The length of a step, in seconds: 30 Hz.
const DT: f64 = 1.0 / 30.0;

/// A simulation of `network` stepping at 30 Hz, with the slower updates at 10 Hz.
fn sim<D: traffic_sim::DebugSink>(network: traffic_sim::SimNetwork, sink: D) -> Simulation<D> {
    let config = traffic_sim::Config { dt: DT, macro_step: 3 };
    Simulation::with_config(network, config, sink)
}

/// SplitMix64, so the probe needs no dependencies.
struct Rng(u64);
impl Rng {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }
    fn f(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
}

#[derive(Default)]
struct Counts {
    checks: u64,
    passed: u64,
    /// Each vehicle's terms from the last longitudinal update.
    terms: std::collections::HashMap<VehicleId, Vec<(traffic_sim::AccelSource, f64)>>,
}
impl DebugSink for Counts {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::PassCheck { check, .. } => {
                self.checks += 1;
                self.passed += check.passed as u64;
            }
            DebugEvent::RecomputeAccels => self.terms.clear(),
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.terms.entry(vehicle).or_default().push((source, accel));
            }
            _ => {}
        }
    }
}

fn car(rng: &mut Rng) -> VehicleSpec {
    let u = 1.0 - rng.f();
    let v = rng.f();
    let normal = (-2.0 * u.ln()).sqrt() * (TAU * v).cos();
    VehicleSpec {
        speed_factor: (1.0 + 0.1 * normal).clamp(0.8, 1.2),
        courtesy: 2.0 * rng.f(),
        politeness: 0.1 + 0.3 * rng.f(),
        assertiveness: rng.f().powi(3),
        ..VehicleSpec::passenger_car()
    }
}

#[derive(Clone, Copy)]
enum Demand {
    /// Keep this many on a closed network, spawning anywhere.
    Count(usize),
    /// Vehicles per hour into each entry lane.
    Flow(f64),
}

struct Harness {
    demand: Demand,
    credit: f64,
    rng: Rng,
    entries: Vec<LaneId>,
}

impl Harness {
    fn step<D: DebugSink>(&mut self, sim: &mut Simulation<D>) {
        match self.demand {
            Demand::Count(target) => {
                if sim.vehicles().count() >= target {
                    return;
                }
                self.credit = (self.credit + 2.0 * DT).min(1.0);
                if self.credit < 1.0 {
                    return;
                }
                let lanes = sim.network().lanes().count() as u64;
                let lane = LaneId((self.rng.next() % lanes) as u32);
                let s = self.rng.f() * sim.network().lane(lane).geometry().length();
                self.spawn(sim, lane, s);
            }
            Demand::Flow(flow) => {
                let n = self.entries.len();
                self.credit = (self.credit + flow / 3600.0 * n as f64 * DT).min(n as f64);
                if self.credit >= 1.0 {
                    let lane = self.entries[(self.rng.next() % n as u64) as usize];
                    self.spawn(sim, lane, 5.0);
                }
            }
        }
    }

    fn spawn<D: DebugSink>(&mut self, sim: &mut Simulation<D>, lane: LaneId, s: f64) {
        let spec = car(&mut self.rng);
        let limit = sim.network().lane(lane).speed_limit();
        let at = Placement {
            lane,
            s,
            speed: spec.desired_speed(limit),
            route: Route::random(self.rng.next()),
        };
        if sim.try_insert(spec, at).is_ok() {
            self.credit -= 1.0;
        }
    }
}

fn overlap(a: [DVec2; 4], b: [DVec2; 4]) -> bool {
    for poly in [a, b] {
        for i in 0..4 {
            let n = (poly[(i + 1) % 4] - poly[i]).perp();
            let (amin, amax) = project(a, n);
            let (bmin, bmax) = project(b, n);
            if amax <= bmin || bmax <= amin {
                return false;
            }
        }
    }
    true
}
fn project(p: [DVec2; 4], n: DVec2) -> (f64, f64) {
    p.iter().fold((f64::MAX, f64::MIN), |(lo, hi), c| {
        let d = c.dot(n);
        (lo.min(d), hi.max(d))
    })
}

#[derive(Default, Debug)]
struct Stats {
    overlap_frames: u64,
    overlap_pairs: std::collections::BTreeSet<(VehicleId, VehicleId)>,
    hard: u64,
    min_accel: f64,
    exits: u64,
    changes: u64,
    vehicle_frames: u64,
    /// Vehicle-checks where a body was inside a lane it has no record on, not
    /// ahead on its path: cutting into a neighbour while not changing lanes, and
    /// anything else (a missing record).
    cut: u64,
    missing: u64,
    missing_examples: Vec<String>,
}

/// Counts bodies inside lanes they have no record on (invariant 1), every call.
fn check_records<D: DebugSink>(sim: &Simulation<D>, stats: &mut Stats) {
    const INSIDE: f64 = 0.05;
    let net = sim.network();
    for (id, v) in sim.vehicles() {
        let corners = v.corners();
        for (lane_id, lane) in net.lanes() {
            if lane.junction().is_some() || v.path().contains(&lane_id) {
                continue;
            }
            let g = lane.geometry();
            let inside = corners.iter().any(|&c| {
                let p = g.world_to_lane(c);
                p.s > INSIDE
                    && p.s < g.length() - INSIDE
                    && p.t < g.half_width(p, Side::Left) - INSIDE
                    && -p.t < g.half_width(p, Side::Right) - INSIDE
                    && g.lane_to_world(p).distance(c) < 0.01
            });
            if !inside {
                continue;
            }
            let has = sim.occupants(lane_id).iter().any(|o| o.is_record_of(id));
            if has {
                continue;
            }
            // Its nose over the start of a lane after one it is on: still ahead
            // of it, found from the lane before.
            let ahead = lane
                .predecessors()
                .iter()
                .any(|&p| sim.occupants(p).iter().any(|o| o.is_record_of(id)));
            if ahead {
                continue;
            }
            let own = net.lane(v.path()[0]);
            let neighbour = [Side::Left, Side::Right]
                .iter()
                .any(|&side| own.adj(side).is_some_and(|n| n.lane == lane_id));
            if neighbour && v.lane_change().is_none() {
                stats.cut += 1;
            } else {
                stats.missing += 1;
                if stats.missing_examples.len() < 3 {
                    stats.missing_examples.push(format!(
                        "t={:.1} {id:?} on {:?} path {:?} change {} in {lane_id:?}",
                        sim.time(),
                        v.lane(),
                        v.path(),
                        v.lane_change().is_some()
                    ));
                }
            }
        }
    }
}

fn run<D: DebugSink>(
    net: &RoadNetwork,
    demand: Demand,
    seed: u64,
    minutes: f64,
    sink: D,
    measure: bool,
) -> (Stats, Duration, Simulation<D>) {
    let compiled = compile(net).expect("compiles");
    let mut sim = sim(compiled.sim, sink);
    let entries = sim
        .network()
        .lanes()
        .filter(|(_, l)| l.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut harness = Harness { demand, credit: 0.0, rng: Rng(seed), entries };
    let mut stats = Stats { min_accel: 0.0, ..Default::default() };
    let mut time = Duration::ZERO;
    let mut prev: std::collections::HashMap<VehicleId, bool> = Default::default();
    for _ in 0..(minutes * 60.0 / DT) as usize {
        let before = sim.vehicles().count();
        let t = Instant::now();
        sim.step_variable(DT);
        time += t.elapsed();
        let after = sim.vehicles().count();
        stats.exits += before.saturating_sub(after) as u64;
        harness.step(&mut sim);
        if !measure {
            continue;
        }
        // Collisions: sweep by x.
        let mut bodies: Vec<_> = sim.vehicles().map(|(id, v)| (v.corners(), id)).collect();
        bodies.sort_by(|a, b| a.0[0].x.total_cmp(&b.0[0].x));
        let mut any = false;
        for i in 0..bodies.len() {
            for j in i + 1..bodies.len() {
                if bodies[j].0[0].x - bodies[i].0[0].x > 12.0 {
                    break;
                }
                if bodies[i].0[0].distance(bodies[j].0[0]) < 12.0
                    && overlap(bodies[i].0, bodies[j].0)
                {
                    any = true;
                    let (a, b) = (bodies[i].1, bodies[j].1);
                    stats.overlap_pairs.insert(if a < b { (a, b) } else { (b, a) });
                }
            }
        }
        stats.overlap_frames += any as u64;
        if std::env::var("RECORDS").is_ok()
            && ((sim.time() * 60.0).round() as u64).is_multiple_of(10)
        {
            check_records(&sim, &mut stats);
        }
        for (id, v) in sim.vehicles() {
            stats.vehicle_frames += 1;
            if v.accel() < -3.0 {
                stats.hard += 1;
            }
            stats.min_accel = stats.min_accel.min(v.accel());

            let changing = v.lane_change().is_some();
            if changing && !prev.get(&id).copied().unwrap_or(true) {
                stats.changes += 1;
            }
            prev.insert(id, changing);
        }
    }
    (stats, time, sim)
}

/// What binds vehicles braking harder than 6 m/s², by kind of term and lane.
fn why(net: &RoadNetwork, demand: Demand, seed: u64, minutes: f64) {
    use traffic_sim::AccelSource as S;
    let compiled = compile(net).expect("compiles");
    let mut sim = sim(compiled.sim, Counts::default());
    let entries = sim
        .network()
        .lanes()
        .filter(|(_, l)| l.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut harness = Harness { demand, credit: 0.0, rng: Rng(seed), entries };
    let mut why: std::collections::BTreeMap<String, u64> = Default::default();
    for _ in 0..(minutes * 60.0 / DT) as usize {
        sim.step_variable(DT);
        harness.step(&mut sim);
        for (id, v) in sim.vehicles() {
            if v.accel() >= -6.0 {
                continue;
            }
            let terms = sim.sink().terms.get(&id);
            let binding = terms.and_then(|t| t.iter().find(|(_, a)| *a == v.accel()));
            let kind = match binding.map(|b| b.0) {
                Some(S::Leader { gap, .. }) => {
                    format!("leader gap{}", if gap < 0.0 { "<0" } else { ">=0" })
                }
                Some(S::Merge { .. }) => "merge".into(),
                Some(S::Junction { .. }) => "junction".into(),
                Some(S::Yield { .. }) => "yield".into(),
                Some(S::PathEnd { .. }) => "path end".into(),
                Some(other) => format!("{other:?}").split(' ').next().unwrap().into(),
                None => "old side?".into(),
            };
            *why.entry(format!("{kind:<14} on {:?}", v.lane())).or_default() += 1;
        }
    }
    for (k, n) in why {
        println!("    {k}: {n}");
    }
}

fn lane(offset: f64) -> RoadLane {
    RoadLane::new(offset, 3.5, Direction::Forward)
}

/// A straight one-way freeway of `n` lanes.
fn freeway(n: usize, length: f64) -> RoadNetwork {
    let mut net = RoadNetwork {
        keep: Keep::Left,
        keep_above: 80.0 / 3.6,
        ..RoadNetwork::default()
    };
    let lanes = (0..n).map(|i| lane(3.5 * (i as f64 + 0.5))).collect();
    net.add_road(Road {
        start: Pose::new(0.0, 0.0, 0.0),
        pieces: vec![Piece::Line { length }],
        lanes,
        speed_limit: 100.0 / 3.6,
        keep_clear: Vec::new(),
    });
    net
}

/// A winding one-way freeway of `n` lanes: alternating bends of `radius`.
fn winding(n: usize, radius: f64) -> RoadNetwork {
    let mut net = RoadNetwork {
        keep: Keep::Left,
        keep_above: 80.0 / 3.6,
        ..RoadNetwork::default()
    };
    let lanes = (0..n).map(|i| lane(3.5 * (i as f64 + 0.5))).collect();
    let mut pieces = vec![Piece::Line { length: 100.0 }];
    for k in 0..6 {
        let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
        pieces.push(Piece::Arc { length: 0.5 * radius, curvature: sign / radius });
        pieces.push(Piece::Line { length: 50.0 });
    }
    net.add_road(Road {
        start: Pose::new(0.0, 0.0, 0.0),
        pieces,
        lanes,
        speed_limit: 100.0 / 3.6,
        keep_clear: Vec::new(),
    });
    net
}

/// A targeted case on a straight three-lane road, lanes left to right L, M, R:
/// A behind, B `d` ahead, each at a fixed speed, each changing lanes once.
#[derive(Clone, Copy, Debug)]
enum Case {
    /// B changes L→M, A changes M→R: B moves toward A, which is leaving.
    IntoLeaving,
    /// B changes M→R, A changes L→M: both move right, A into B's old lane.
    SameWay,
    /// B changes M→L, A changes M→R: they part.
    Apart,
    /// B changes M→R, A stays on M.
    Leaving,
}

struct Outcome {
    collided: bool,
    passed: bool,
    hard: bool,
}

#[allow(clippy::too_many_arguments)]
fn targeted(
    case: Case,
    va: f64,
    vb: f64,
    d: f64,
    ta: f64,
    tb: f64,
    a_changes: bool,
    b_changes: bool,
) -> Outcome {
    let compiled = compile(&freeway3()).unwrap();
    let ids = compiled.road_lanes[0].clone();
    let (l, m, r) = (ids[0], ids[1], ids[2]);
    let mut sim = sim(compiled.sim, NoopSink);
    let mut rng = Rng(1);
    let spec = || VehicleSpec { speed_factor: 1.0, ..car(&mut Rng(7)) };
    let _ = &mut rng;
    let (la, lb, sa, sb) = match case {
        Case::IntoLeaving => (m, l, Side::Right, Side::Right),
        Case::SameWay => (l, m, Side::Right, Side::Right),
        Case::Apart => (m, m, Side::Right, Side::Left),
        Case::Leaving => (m, m, Side::Right, Side::Right),
    };
    let a = sim.insert(
        spec(),
        Placement { lane: la, s: 100.0, speed: va, route: Route::default() },
    );
    let b = sim.insert(
        spec(),
        Placement { lane: lb, s: 100.0 + d, speed: vb, route: Route::default() },
    );
    sim.cap_speed(a, va);
    sim.cap_speed(b, vb);
    let _ = r;
    let (mut done_a, mut done_b) = (false, matches!(case, Case::Leaving) && false);
    let mut out = Outcome { collided: false, passed: false, hard: false };
    let a_changes = a_changes && !matches!(case, Case::Leaving);
    for frame in 0..(15.0 / DT) as usize {
        let t = frame as f64 * DT;
        if a_changes && !done_a && t >= ta {
            done_a = sim.change_lane(a, sa).is_ok() || t > ta + 1.0;
        }
        if b_changes && !done_b && t >= tb {
            done_b = sim.change_lane(b, sb).is_ok() || t > tb + 1.0;
        }
        sim.step_variable(DT);
        let (va_, vb_) = (sim.get(a).unwrap(), sim.get(b).unwrap());
        if overlap(va_.corners(), vb_.corners()) {
            out.collided = true;
        }
        if va_.accel() < -3.0 {
            out.hard = true;
        }
        let rear_b = vb_.pos() - vb_.spec().rear_axle * DVec2::from_angle(vb_.heading());
        let front_a = va_.pos()
            + (va_.spec().length - va_.spec().rear_axle) * DVec2::from_angle(va_.heading());
        if (va_.pos() - vb_.pos()).x > 0.0 && front_a.x > rear_b.x + 5.0 {
            out.passed = true;
        }
    }
    out
}

fn freeway3() -> RoadNetwork {
    let mut net = RoadNetwork {
        keep: Keep::Left,
        keep_above: 80.0 / 3.6,
        ..RoadNetwork::default()
    };
    net.add_road(Road {
        start: Pose::new(0.0, 0.0, 0.0),
        pieces: vec![Piece::Line { length: 1500.0 }],
        lanes: vec![lane(3.5), lane(0.0), lane(-3.5)],
        speed_limit: 120.0 / 3.6,
        keep_clear: Vec::new(),
    });
    net
}

fn grid() {
    for case in [Case::IntoLeaving, Case::SameWay, Case::Apart, Case::Leaving] {
        {
            let (mut n, mut col, mut pass, mut hard) = (0, 0, 0, 0);
            let mut examples = Vec::new();
            for va in [8.0, 14.0, 20.0, 26.0] {
                for vb in [2.0, 6.0, 10.0, 16.0] {
                    for d in [6.0, 12.0, 20.0, 35.0, 50.0] {
                        for ta in [0.0, 0.5, 1.0, 2.0, 3.0] {
                            for tb in [0.0, 0.5, 1.0, 2.0, 3.0] {
                                // Unavoidable setups: they collide with no
                                // changes, or with only B's (a cut-in).
                                let still = targeted(case, va, vb, d, ta, tb, false, false);
                                let cut_in = targeted(case, va, vb, d, ta, tb, false, true);
                                if still.collided || cut_in.collided {
                                    continue;
                                }
                                let o = targeted(case, va, vb, d, ta, tb, true, true);
                                n += 1;
                                col += o.collided as u32;
                                pass += o.passed as u32;
                                hard += o.hard as u32;
                                if o.collided && examples.len() < 3 {
                                    examples.push((va, vb, d, ta, tb));
                                }
                            }
                        }
                    }
                }
            }
            println!(
                "{:<12} {:<6} runs {n:>5}  collided {col:>4}  A passed B {pass:>5}  A braked hard {hard:>5}  e.g. {examples:?}",
                format!("{case:?}"),
                "",
            );
        }
    }
}

/// At the lane drop: A on the acceleration lane, B `d` ahead on the freeway's
/// left lane, near the end of the stretch alongside. B changes right at `tb`
/// (or never), as it reaches the drop, where the acceleration lane tapers in
/// toward B's old lane. Returns (collided, invariant-1 violations).
fn at_the_drop(va: f64, vb: f64, d: f64, x: f64, tb: Option<f64>) -> (bool, u64) {
    let compiled = compile(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6)).unwrap();
    let parallel = compiled.road_lanes[1].clone();
    let (accel, left) = (parallel[0], parallel[1]);
    let mut sim = sim(compiled.sim, NoopSink);
    let spec = VehicleSpec { speed_factor: 1.0, ..car(&mut Rng(7)) };
    let b = sim.insert(
        spec,
        Placement { lane: left, s: x, speed: vb, route: Route::default() },
    );
    let a = sim.insert(
        spec,
        Placement { lane: accel, s: x - d, speed: va, route: Route::default() },
    );
    sim.cap_speed(a, va);
    sim.cap_speed(b, vb);
    let mut stats = Stats::default();
    let (mut collided, mut done) = (false, false);
    for frame in 0..(20.0 / DT) as usize {
        let t = frame as f64 * DT;
        if let Some(tb) = tb
            && !done
            && t >= tb
        {
            done = sim.change_lane(b, Side::Right).is_ok() || t > tb + 1.0;
        }
        sim.step_variable(DT);
        let (Some(va_), Some(vb_)) = (sim.get(a), sim.get(b)) else {
            if std::env::var("DEBUG").is_ok() {
                eprintln!(
                    "ended at {t:.2}: a {:?} b {:?}",
                    sim.get(a).map(|v| v.lane()),
                    sim.get(b).map(|v| v.lane())
                );
            }
            break;
        };
        collided |= overlap(va_.corners(), vb_.corners());
        if std::env::var("DEBUG").is_ok() && frame % 60 == 0 {
            eprintln!(
                "t {t:.0} A {:?} s {:.1} v {:.1} | B {:?} s {:.1} v {:.1} t {:.2} | dist {:.1}",
                va_.lane(),
                va_.lane_pos().s,
                va_.speed(),
                vb_.lane(),
                vb_.lane_pos().s,
                vb_.speed(),
                vb_.lane_pos().t,
                va_.pos().distance(vb_.pos())
            );
        }
        if frame % 5 == 0 {
            check_records(&sim, &mut stats);
        }
    }
    (collided, stats.missing)
}

fn grid2() {
    if std::env::var("DEBUG").is_ok() {
        at_the_drop(22.0, 12.0, 6.0, 175.0, Some(0.5));
        return;
    }
    for leaves in [true, false] {
        let (mut n, mut col, mut missing, mut ex) = (0, 0, 0, Vec::new());
        for va in [10.0, 16.0, 22.0, 28.0] {
            for vb in [6.0, 12.0, 18.0, 24.0] {
                for d in [-6.0, 0.0, 6.0, 12.0, 25.0, 40.0] {
                    for x in [120.0, 150.0, 175.0, 195.0] {
                        for tb in [0.0, 0.5, 1.5, 3.0] {
                            if !leaves && tb > 0.0 {
                                continue;
                            }
                            let (c, m) = at_the_drop(va, vb, d, x, leaves.then_some(tb));
                            n += 1;
                            col += c as u32;
                            missing += m;
                            if c && ex.len() < 4 {
                                ex.push((va, vb, d, x, tb));
                            }
                        }
                    }
                }
            }
        }
        println!(
            "{:<6} runs {n:>5}  collided {col:>4}  missing records {missing:>4}  e.g. {ex:?}",
            if leaves { "leave" } else { "merge" }
        );
    }
}

/// At the ramps' merge, at `flow` into each entry: how fast each lane runs,
/// how vehicles enter the merge, and which lane changes happen.
fn merge_probe(flow: f64, minutes: f64, seed: u64) {
    use std::collections::BTreeMap;
    let compiled = compile(&scenarios::ramps(100.0 / 3.6, 80.0 / 3.6)).expect("compiles");
    let mut sim = sim(compiled.sim, Counts::default());
    let entries = sim
        .network()
        .lanes()
        .filter(|(_, l)| l.predecessors().is_empty())
        .map(|(id, _)| id)
        .collect();
    let mut harness = Harness {
        demand: Demand::Flow(flow),
        credit: 0.0,
        rng: Rng(seed),
        entries,
    };
    let mut binding: BTreeMap<String, (u64, f64)> = BTreeMap::new();
    let mut speed: BTreeMap<u32, (f64, u64)> = BTreeMap::new();
    let mut changes: BTreeMap<(u32, u32), u64> = BTreeMap::new();
    let mut entering: BTreeMap<u32, (u64, f64, u64)> = BTreeMap::new();
    let mut last: std::collections::HashMap<VehicleId, LaneId> = Default::default();
    let mut entry_times: Vec<f64> = Vec::new();
    // Stops within 60 m of the merge point on the lanes leading to it: moving
    // above 1 m/s, then below 0.2 m/s.
    let mut moving: std::collections::HashMap<VehicleId, bool> = Default::default();
    let mut stops = 0u64;
    // Vehicle-frames spent partway through a lane change, crawling: straddling
    // two lanes, blocking both.
    let mut stalled = 0u64;
    let warm = 60.0;
    for _ in 0..(minutes * 60.0 / DT) as usize {
        sim.step_variable(DT);
        harness.step(&mut sim);
        if sim.time() < warm {
            for (id, v) in sim.vehicles() {
                last.insert(id, v.lane());
            }
            continue;
        }
        for (id, v) in sim.vehicles() {
            stalled += (v.lane_change().is_some() && v.speed() < 1.0) as u64;
            let lane = v.lane();
            let length = sim.network().lane(lane).geometry().length();
            if (lane.0 == 2 || lane.0 == 3) && v.lane_pos().s > length - 60.0 {
                let was = moving.get(&id).copied().unwrap_or(false);
                if was && v.speed() < 0.2 {
                    stops += 1;
                    moving.insert(id, false);
                } else if v.speed() > 1.0 {
                    moving.insert(id, true);
                }
            }
            if (lane.0 == 2 || lane.0 == 3) && v.lane_pos().s > length - 40.0 {
                use traffic_sim::AccelSource as S;
                let terms = sim.sink().terms.get(&id);
                let b = terms.and_then(|t| t.iter().find(|(_, a)| *a == v.accel()));
                let kind = match b.map(|b| b.0) {
                    Some(S::Leader { id: l, headway, .. }) => {
                        let other = sim.get(l).map(|o| o.lane());
                        format!(
                            "leader on {:?} headway<1:{}",
                            other.map(|o| o.0),
                            headway < 1.0
                        )
                    }
                    Some(S::Merge { .. }) => "merge".into(),
                    Some(S::FreeRoad { .. }) => "free".into(),
                    Some(other) => format!("{other:?}").split(' ').next().unwrap().into(),
                    None => "other".into(),
                };
                let e = binding.entry(format!("lane {} {kind}", lane.0)).or_default();
                e.0 += 1;
                e.1 += v.speed();
            }
            let e = speed.entry(lane.0).or_default();
            e.0 += v.speed();
            e.1 += 1;
            if let Some(&before) = last.get(&id)
                && before != lane
            {
                let net = sim.network();
                if net.lane(before).successors().contains(&lane) {
                    // Into the merge: lanes 5 and 6.
                    if lane.0 == 5 || lane.0 == 6 {
                        let e = entering.entry(lane.0).or_default();
                        e.0 += 1;
                        e.1 += v.speed();
                        e.2 += (v.speed() < 2.0) as u64;
                        entry_times.push(sim.time());
                    }
                } else {
                    *changes.entry((before.0, lane.0)).or_default() += 1;
                }
            }
            last.insert(id, lane);
        }
    }
    // A few seconds at the head of the queues, in detail.
    if std::env::var("TRACE").is_ok() {
        use traffic_sim::AccelSource as S;
        for frame in 0..(4.0 / DT) as usize {
            sim.step_variable(DT);
            harness.step(&mut sim);
            if frame % 60 != 0 {
                continue;
            }
            println!("t {:.2}", sim.time());
            let mut rows: Vec<_> = sim
                .vehicles()
                .filter_map(|(id, v)| {
                    let lane = v.lane().0;
                    let to_end = sim.network().lane(v.lane()).geometry().length()
                        - v.lane_pos().s
                        - (v.spec().length - v.spec().rear_axle);
                    let d = match lane {
                        2 | 3 => to_end,
                        5 | 6 => -(v.lane_pos().s),
                        _ => return None,
                    };
                    (d < 120.0 && d > -10.0 && (lane == 3 || lane == 6 || d < 10.0))
                        .then_some((d, id, lane, v))
                })
                .collect();
            rows.sort_by(|a, b| a.0.total_cmp(&b.0));
            for o in sim.occupants(LaneId(3)).iter().rev().take(8) {
                let v = sim.get(o.id).unwrap();
                let terms = sim.sink().terms.get(&o.id);
                let b = terms
                    .and_then(|t| t.iter().find(|(_, a)| *a == v.accel()))
                    .map(|b| format!("{:?}", b.0));
                println!(
                    "  on 3: {:?} s {:.1} {:?} age {:.1} | follows {:?} t {:.2} v {:.2} change {:?} intent {:?} | {}",
                    o.id,
                    o.s(),
                    o.lateral,
                    o.age,
                    v.lane(),
                    v.lane_pos().t,
                    v.speed(),
                    v.lane_change().map(|c| (c.s0 as i32, c.length as i32)),
                    v.intent().map(|i| i.side),
                    b.unwrap_or_default()
                );
            }
            rows.clear();
            for (d, id, lane, v) in rows {
                let terms = sim.sink().terms.get(&id);
                let b = terms.and_then(|t| t.iter().find(|(_, a)| *a == v.accel()));
                let why = match b.map(|b| b.0) {
                    Some(S::Leader { id, gap, headway }) => {
                        format!("leader {id:?} gap {gap:.1} h {headway:.2}")
                    }
                    Some(S::Merge { id, gap }) => format!("merge after {id:?} gap {gap:.1}"),
                    Some(other) => format!("{other:?}"),
                    None => "?".into(),
                };
                println!(
                    "  {id:?} lane {lane} d {d:6.1} v {:4.1} a {:5.2}  {why}",
                    v.speed(),
                    v.accel()
                );
            }
        }
        return;
    }
    println!("flow {flow} per entry, seed {seed}, {minutes} min (first minute left out)");
    println!("  mean speed by lane (m/s):");
    for (lane, (sum, n)) in &speed {
        println!(
            "    lane {lane:>2}: {:5.1}  ({:.0} vehicles on average)",
            sum / *n as f64,
            *n as f64 / ((minutes * 60.0 - warm) / DT)
        );
    }
    println!("  entering the merge:");
    for (lane, (n, v, slow)) in &entering {
        println!(
            "    onto lane {lane}: {n} vehicles, mean speed {:.1} m/s, {slow} below 2 m/s",
            v / *n as f64
        );
    }
    entry_times.sort_by(f64::total_cmp);
    let gaps: Vec<f64> = entry_times.windows(2).map(|w| w[1] - w[0]).collect();
    if !gaps.is_empty() {
        let mean = gaps.iter().sum::<f64>() / gaps.len() as f64;
        println!(
            "    mean time between entries {mean:.2} s ({:.0} veh/h through the merge)",
            3600.0 / mean
        );
    }
    println!("  lane changes (from -> to): {changes:?}");
    println!("  stops within 60 m of the merge point: {stops}");
    println!("  vehicle-frames stalled partway through a lane change: {stalled}");
    println!("  binding term in the last 40 m before the merge point (frames, mean speed):");
    for (k, (n, v)) in &binding {
        println!("    {k}: {n}, {:.1} m/s", v / *n as f64);
    }
}

fn main() {
    if std::env::args().nth(1).as_deref() == Some("merge") {
        let flow = std::env::args().nth(2).and_then(|a| a.parse().ok()).unwrap_or(1800.0);
        return merge_probe(flow, 10.0, 1);
    }
    if std::env::args().nth(1).as_deref() == Some("drop") {
        return grid2();
    }
    if std::env::args().nth(1).as_deref() == Some("grid") {
        return grid();
    }

    let minutes: f64 = std::env::args().nth(1).and_then(|a| a.parse().ok()).unwrap_or(10.0);
    let cases: Vec<(String, RoadNetwork, Demand)> = vec![
        (
            "ramps 800".into(),
            scenarios::ramps(100.0 / 3.6, 80.0 / 3.6),
            Demand::Flow(800.0),
        ),
        (
            "ramps 1500".into(),
            scenarios::ramps(100.0 / 3.6, 80.0 / 3.6),
            Demand::Flow(1500.0),
        ),
        (
            "ring3 150".into(),
            scenarios::ring(1000.0, 3, 50.0 / 3.6),
            Demand::Count(150),
        ),
        (
            "ring3 300".into(),
            scenarios::ring(1000.0, 3, 50.0 / 3.6),
            Demand::Count(300),
        ),
        (
            "straight4 1200".into(),
            freeway(4, 2000.0),
            Demand::Flow(1200.0),
        ),
        (
            "straight4 1800".into(),
            freeway(4, 2000.0),
            Demand::Flow(1800.0),
        ),
        (
            "winding3 r150 1500".into(),
            winding(3, 150.0),
            Demand::Flow(1500.0),
        ),
        (
            "ramps 2000".into(),
            scenarios::ramps(100.0 / 3.6, 80.0 / 3.6),
            Demand::Flow(2000.0),
        ),
        (
            "crossroads2 600".into(),
            scenarios::crossroads(2, 50.0 / 3.6),
            Demand::Flow(600.0),
        ),
        (
            "crossroads2 1200".into(),
            scenarios::crossroads(2, 50.0 / 3.6),
            Demand::Flow(1200.0),
        ),
        (
            "crossroads1 900".into(),
            scenarios::crossroads(1, 50.0 / 3.6),
            Demand::Flow(900.0),
        ),
    ];
    let only = std::env::var("CASE").ok();
    let cases: Vec<_> = cases
        .into_iter()
        .filter(|c| only.as_deref().is_none_or(|o| c.0.starts_with(o)))
        .collect();
    println!(
        "{:<20} {:>4} {:>7} {:>6} {:>8} {:>7} {:>6} {:>6} {:>9} {:>8} {:>8}",
        "case",
        "seed",
        "ovFrm",
        "ovPr",
        "hard",
        "minA",
        "exits",
        "chg",
        "checks",
        "pass%",
        "ms/step"
    );
    for (name, net, demand) in &cases {
        for seed in 1..=3 {
            let (stats, _, sim) = run(net, *demand, seed, minutes, Counts::default(), true);
            let (_, time, _) = run(net, *demand, seed, minutes, NoopSink, false);
            let steps = minutes * 60.0 / DT;
            let c = sim.sink();
            println!(
                "{:<20} {:>4} {:>7} {:>6} {:>8} {:>7.2} {:>6} {:>6} {:>9} {:>7.1}% {:>8.4}",
                name,
                seed,
                stats.overlap_frames,
                stats.overlap_pairs.len(),
                stats.hard,
                stats.min_accel,
                stats.exits,
                stats.changes,
                c.checks,
                100.0 * c.passed as f64 / c.checks.max(1) as f64,
                time.as_secs_f64() * 1e3 / steps,
            );
            if std::env::var("WHY").is_ok() {
                why(net, *demand, seed, minutes);
            }
            if std::env::var("RECORDS").is_ok() {
                println!(
                    "    cut {} missing {} {:?}",
                    stats.cut, stats.missing, stats.missing_examples
                );
            }
        }
    }
}
