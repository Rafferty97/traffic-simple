//! Checks shared by the network's tests.

#![allow(dead_code)]

use traffic_sim::{Config, ConflictKind, DebugEvent, Driver, LaneId, SimNetwork, Simulation};

/// The length of a step in the tests' standard simulation, in seconds: 30 Hz.
pub const DT: f64 = 1.0 / 30.0;

/// The tests' standard stepping: 30 Hz steps, with the slower updates every
/// third, at 10 Hz.
pub const CONFIG: Config = Config { dt: DT, macro_step: 3 };

/// A simulation of `network` stepping by [`CONFIG`].
pub fn sim(network: SimNetwork) -> Simulation {
    Simulation::with_config(network, CONFIG)
}

/// What a test collects from a simulation's debug events, as they come.
pub trait Collect {
    fn collect(&mut self, event: DebugEvent);
}

/// A simulation stepped with its debug events, each handed to `sink` as it
/// comes. It derefs to the simulation, so it reads and changes like one.
pub struct Watched<C> {
    sim: Simulation,
    sink: C,
    events: Vec<DebugEvent>,
}

impl<C: Collect> Watched<C> {
    pub fn new(sim: Simulation, sink: C) -> Self {
        Self { sim, sink, events: Vec::new() }
    }

    /// One step, as [`Simulation::step`], collecting its events.
    pub fn step(&mut self) {
        self.sim.step_debug(&mut self.events);
        self.drain();
    }

    /// As [`Simulation::step_variable`], collecting its events.
    pub fn step_variable(&mut self, dt: f64) -> u32 {
        let steps = self.sim.step_variable_debug(dt, &mut self.events);
        self.drain();
        steps
    }

    /// What it has collected.
    pub fn sink(&self) -> &C {
        &self.sink
    }

    fn drain(&mut self) {
        for event in self.events.drain(..) {
            self.sink.collect(event);
        }
    }
}

impl<C> std::ops::Deref for Watched<C> {
    type Target = Simulation;

    fn deref(&self) -> &Simulation {
        &self.sim
    }
}

impl<C> std::ops::DerefMut for Watched<C> {
    fn deref_mut(&mut self) -> &mut Simulation {
        &mut self.sim
    }
}

/// A simulation of `network` stepping by [`CONFIG`], watched by `sink`.
pub fn watched<C: Collect>(network: SimNetwork, sink: C) -> Watched<C> {
    Watched::new(sim(network), sink)
}

/// How many standard steps take `seconds`.
pub fn steps(seconds: f64) -> u64 {
    (seconds / DT).round() as u64
}

/// How far a vehicle's front may be into a conflict it shouldn't have entered,
/// in meters, before it counts: the backstop's margin lets one committing just
/// as another does stop a little way in.
const ENTERED: f64 = 0.5;

/// The first of the junctions' queue invariants that doesn't hold, if any (see
/// `docs/DESIGN.md`, Queue invariants), described:
///
/// 1. Of two queued vehicles following one lane, the one ahead is queued first.
/// 2. No queued vehicle is well into a crossing conflict with one queued above
///    it that hasn't cleared it, unless following it the same general way, as
///    the junction lets it.
/// 3. A vehicle's entries run in the order of its path, none after one that
///    covers it right through.
///
/// The fourth, that the queue closes no cycle, shows as a lockup.
#[allow(dead_code)]
pub fn queue_violation(sim: &Simulation) -> Option<String> {
    let net = sim.network();
    for (junction, _) in net.junctions() {
        let queue = sim.queue(junction);
        let place = |id| queue.iter().position(|e| e.id == id);
        // 3.
        for (id, v) in sim.vehicles() {
            let at = |l: LaneId| v.path().iter().position(|&p| p == l);
            let entries: Vec<_> = queue.iter().filter(|e| e.id == id).collect();
            for pair in entries.windows(2) {
                match (pair[0].until(net, junction), pair[1].until(net, junction)) {
                    (None, _) => return Some(format!("{id:?} queued again after its whole way")),
                    (Some(a), Some(b)) if at(a).zip(at(b)).is_some_and(|(a, b)| a >= b) => {
                        return Some(format!("{id:?} queued out of its path's order"));
                    }
                    _ => {}
                }
            }
        }
        // 1.
        for (lane, _) in net.lanes() {
            let mut on: Vec<_> = sim
                .occupants(lane)
                .iter()
                .filter(|o| !o.is_ghost())
                .filter(|o| sim.get(o.id).is_some_and(|v| v.lane() == lane))
                .filter_map(|o| place(o.id).map(|p| (o.s(), o.id, p)))
                .collect();
            on.sort_by(|a, b| a.0.total_cmp(&b.0));
            for pair in on.windows(2) {
                let ((_, behind, p_behind), (_, ahead, p_ahead)) = (pair[0], pair[1]);
                if p_behind < p_ahead {
                    return Some(format!(
                        "{behind:?} is queued before {ahead:?}, ahead of it on {lane:?}"
                    ));
                }
            }
        }
        // 2. Each by the entry that covers the lane it is on: the first that
        // covers it right through, or stops at a lane after it on its path.
        let covering = |id, v: &traffic_sim::Vehicle| {
            let at = |l: LaneId| v.path().iter().position(|&p| p == l);
            queue.iter().position(|e| {
                e.id == id && e.until(net, junction).is_none_or(|u| at(u).is_none_or(|u| u > 0))
            })
        };
        let queued: Vec<_> = sim
            .vehicles()
            .filter_map(|(id, v)| covering(id, v).map(|p| (p, id, v)))
            .collect();
        for &(p_b, b, vb) in &queued {
            for &(p_a, a, va) in &queued {
                if p_a >= p_b || a == b {
                    continue;
                }
                let (mine, theirs) = (vb.lane(), va.lane());
                let Some(conflict) = net.conflict_between(mine, theirs) else {
                    continue;
                };
                if conflict.kind != ConflictKind::Cross {
                    continue;
                }
                let front = vb.lane_pos().s - vb.spec().rear_axle + vb.spec().length;
                let rear = va.lane_pos().s - va.spec().rear_axle;
                let entered = front > conflict.mine.start + ENTERED
                    && front - vb.spec().length < conflict.mine.end;
                let cleared = rear >= conflict.theirs.end;
                let geometry = net.lane(mine).geometry();
                let along = geometry.tangent_at(geometry.lane_pos(conflict.mine.start, 0.0));
                let same_way = glam::DVec2::from_angle(va.heading()).dot(along) > 0.0;
                if entered && !cleared && !same_way {
                    return Some(format!(
                        "{b:?} is in its conflict with {a:?}, queued above it, before it has cleared: {b:?} on {mine:?} front {front:.2} in {:?} at {:.2} m/s, {a:?} on {theirs:?} rear {rear:.2} of {:?} at {:.2} m/s; queue {:?}",
                        conflict.mine,
                        vb.speed(),
                        conflict.theirs,
                        va.speed(),
                        queue.iter().map(|e| (e.id, e.until(net, junction))).collect::<Vec<_>>()
                    ));
                }
            }
        }
    }
    None
}

/// Whether two convex bodies, given by their corners, overlap: by the separating
/// axis theorem, along the edges of each.
pub fn overlap(a: [glam::DVec2; 4], b: [glam::DVec2; 4]) -> bool {
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

/// A vehicle whose body is well inside a lane outside junctions that it has no
/// record on, if any, other than lanes on its path and those its nose pokes
/// into past the end of a lane it is on: each vehicle has a record on every lane
/// it intersects (see `docs/multilane-car-following.md`).
pub fn uncovered(sim: &traffic_sim::Simulation) -> Option<String> {
    use traffic_sim::Side;
    const INSIDE: f64 = 0.05;
    let net = sim.network();
    let on = |lane, id| sim.occupants(lane).iter().any(|o| o.is_record_of(id));
    for (id, v) in sim.vehicles() {
        for (lane_id, lane) in net.lanes() {
            if lane.junction().is_some() || v.path().contains(&lane_id) || on(lane_id, id) {
                continue;
            }
            if lane.predecessors().iter().any(|&p| on(p, id)) {
                continue;
            }
            let g = lane.geometry();
            let inside = v.corners().iter().any(|&c| {
                let p = g.world_to_lane(c);
                p.s > INSIDE
                    && p.s < g.length() - INSIDE
                    && p.t < g.half_width(p, Side::Left) - INSIDE
                    && -p.t < g.half_width(p, Side::Right) - INSIDE
                    && g.lane_to_world(p).distance(c) < 0.01
            });
            if inside {
                return Some(format!("{id:?} on {:?} is in {lane_id:?}", v.lane()));
            }
        }
    }
    None
}

/// Floods a network with traffic for `seconds`, a car tried every third of a
/// second at its entry lanes in turn, asserting that no two bodies ever overlap,
/// and that every vehicle has a record on each lane its body is in.
#[allow(dead_code)]
pub fn flood(net: &traffic_network::RoadNetwork, seconds: f64) {
    use traffic_sim::{LaneId, Placement, Route, Simulation, VehicleSpec};
    let car = VehicleSpec::passenger_car();

    {
        let compiled = traffic_network::compile(net).unwrap();
        let mut sim = Simulation::new(compiled.sim);
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
                let _ = sim.try_insert(car, Driver { seed: step, ..Driver::default() }, at);
            }
            sim.step_variable(1.0 / 60.0);
            // Collisions last for many frames, so a check every tenth of a second
            // catches them.
            if step % 6 != 0 {
                continue;
            }
            if let Some(what) = uncovered(&sim) {
                panic!("{what} with no record there at {:.2} s", sim.time());
            }
            if let Some(what) = queue_violation(&sim) {
                panic!("{what} at {:.2} s", sim.time());
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
