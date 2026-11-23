//! The simulation loop: [`Simulation::step`] and each phase of it, in order.
//! Each phase reads the state, works out what every vehicle decides, then
//! applies the decisions; how a single vehicle's decision is made is in the
//! modules it calls.

use super::accel::{AccelUpdate, junction_terms, longitudinal, merge_terms, old_side, straddling};
use super::{Derived, SimState, Simulation};
use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink, NoopSink};
use crate::following;
use crate::junction;
use crate::models::acceleration;
use crate::models::lane_change::{self, Decision};
use crate::models::signal;
use crate::motion;
use crate::motion::path::{extend_path, raise_through, reach, recompute_through};
use crate::network::{JunctionId, LaneId};
use crate::query::LaneSpeeds;
use crate::vehicle::OldFollower;

impl Simulation {
    /// Advances the simulation by `dt` seconds of wall time, a frame's: as many
    /// whole steps as fit, with what is left over carried to the next call.
    /// Returns how many steps it took.
    pub fn step_variable(&mut self, dt: f64) -> u32 {
        self.step_variable_with(dt, &mut NoopSink)
    }

    /// As [`Simulation::step_variable`], appending what each step's algorithms
    /// did to `events`, in order (see [`Simulation::step_debug`]).
    pub fn step_variable_debug(&mut self, dt: f64, events: &mut Vec<DebugEvent>) -> u32 {
        self.step_variable_with(dt, events)
    }

    fn step_variable_with<S: DebugSink>(&mut self, dt: f64, sink: &mut S) -> u32 {
        self.state.residual += dt;
        let step = self.state.config.dt;
        let mut taken = 0;
        // A step's worth, give or take rounding, so frames of exactly a step
        // take exactly one.
        while self.state.residual >= step - 1e-9 * step {
            self.state.residual -= step;
            self.step_with(sink);
            taken += 1;
        }
        self.state.residual = self.state.residual.max(0.0);
        taken
    }

    /// How far into the next step the time given to [`Simulation::step_variable`]
    /// has reached, from 0 to 1: for drawing vehicles between where they were
    /// before the last step and where they are now.
    pub fn alpha(&self) -> f64 {
        (self.state.residual / self.state.config.dt).clamp(0.0, 1.0)
    }

    /// Advances the simulation by one step, [`Config::dt`](super::Config::dt) seconds.
    pub fn step(&mut self) {
        self.step_with(&mut NoopSink);
    }

    /// As [`Simulation::step`], appending what its algorithms did to `events`,
    /// in order, for a debug view. Events come in groups, each starting with
    /// its reset (see [`DebugEvent`]), so applying them in order to what earlier
    /// steps returned always leaves what each group's last update used.
    pub fn step_debug(&mut self, events: &mut Vec<DebugEvent>) {
        self.step_with(events);
    }

    /// One step, recording to `sink`: the whole pipeline, in order.
    pub(super) fn step_with<S: DebugSink>(&mut self, sink: &mut S) {
        let dt = self.state.config.dt;
        let every = self.state.config.macro_step;

        self.apply_removals(sink);

        if self.state.frame.is_multiple_of(every as u64) {
            let dt = (every as f64) * dt;
            self.update_queues();
            self.update_lane_changes(dt, sink);
            self.update_accels(sink);
        }

        self.update_motion(dt, sink);
        self.update_occupancy(dt);
        self.update_signals(dt);

        self.state.frame += 1;
    }

    /// Updates traffic lights.
    pub(super) fn update_signals(&mut self, dt: f64) {
        let SimState { network, signals, lights, .. } = &mut self.state;
        signal::step(network, signals, lights, dt);
    }

    /// Lane changes, for every vehicle, `dt` seconds after the last: the
    /// lane-change model decides from a read-only view of the state (see
    /// [`lane_change::decide`]), and its decisions are applied in order.
    pub(super) fn update_lane_changes<S: DebugSink>(&mut self, dt: f64, sink: &mut S) {
        let Self { state, derived, lane_decisions, asks, lane_speeds, .. } = self;
        // Exact again for the follower searches the gap checks make.
        let paths = state.vehicles.values().map(|v| v.path.as_slice());
        recompute_through(&state.network, &mut derived.through, paths);
        LaneSpeeds::refresh(lane_speeds, state);
        lane_change::decide(
            state,
            &derived.through,
            lane_speeds,
            sink,
            dt,
            lane_decisions,
            asks,
        );

        for i in 0..self.lane_decisions.len() {
            let (id, state, decision) = self.lane_decisions[i];
            self.state.lane_changers[id].lc = state;
            match decision {
                Decision::Nothing => {}
                Decision::Indicate(side) => self.indicate(id, side),
                Decision::Cancel => self.cancel_intent(id),
                // Already checked: only permitted changes are asked about.
                Decision::Change(side) => drop(self.change_lane(id, side)),
            }
        }
    }

    /// Longitudinal model: sets each vehicle's held acceleration (see
    /// [`longitudinal`]). Decisions are computed from a read-only view of the
    /// state, then applied.
    ///
    /// Vehicles are visited lane by lane, from the front of each lane back, at
    /// their record on the lane they follow, so each one's search there starts at
    /// its own record rather than looking for it. SUMO plans each lane's vehicles
    /// from the front back in the same way.
    pub(super) fn update_accels<S: DebugSink>(&mut self, sink: &mut S) {
        let Self { state, derived, decisions, lane_speeds, .. } = self;
        let SimState { network, vehicles, occupants, .. } = &*state;
        sink.record(|| DebugEvent::RecomputeAccels);
        decisions.clear();
        for (index, list) in occupants.iter().enumerate() {
            let lane = LaneId(index as u32);
            let length = network.lane(lane).geometry().length();
            for (i, o) in list.iter().enumerate().rev() {
                if o.is_ghost() {
                    continue;
                }
                let vehicle = &vehicles[o.id];
                if vehicle.path[0] == lane {
                    // On a loop, a vehicle can briefly have a second record on the
                    // lane it follows, lingering at its end, about a lane's length
                    // from its rear.
                    let rear = vehicle.lane_pos.s - vehicle.spec.rear_axle;
                    if !following::same_record(length, o.s(), rear) {
                        continue;
                    }
                    decisions.push(longitudinal(
                        state,
                        &derived.through,
                        lane_speeds,
                        sink,
                        o.id,
                        vehicle,
                        i,
                    ));
                } else if let Some(side) = straddling(network, vehicle, lane) {
                    let accel = old_side(state, sink, o.id, vehicle, side, i);
                    decisions.push(AccelUpdate::OldSide { id: o.id, accel });
                }
            }
        }
        debug_assert_eq!(
            decisions.iter().filter(|d| matches!(d, AccelUpdate::Own { .. })).count(),
            vehicles.len(),
            "each vehicle has one record on the lane it follows"
        );
        for (j, queue) in state.queues.iter().enumerate() {
            junction_terms(state, sink, JunctionId(j as u32), queue, decisions);
        }
        for m in 0..derived.merges.len() {
            merge_terms(state, &derived.merges, sink, m, decisions);
        }

        let noise_step = acceleration::NoiseStep::new(
            self.state.config.dt * f64::from(self.state.config.macro_step),
        );
        // What its old side and its speed cap allow first, then its own update,
        // so that whether a leader binds it is judged against all of them.
        for update in &self.decisions {
            if let AccelUpdate::Own { id, yielding, committed, arrived, .. } = *update {
                let commits = committed.saturating_sub(self.state.vehicles[id].committed);
                let released = committed < self.state.vehicles[id].committed;
                let vehicle = &mut self.state.vehicles[id];
                let driver = &mut self.state.drivers[id];
                // Heading for another barrier now, it draws how far short it stops;
                // past one it arrived at, it has yet to arrive at the next.
                if vehicle.committed != committed {
                    if committed > vehicle.committed {
                        vehicle.stopped_at = None;
                    }
                    vehicle.committed = committed;
                    acceleration::draw_line_gap(driver, &vehicle.spec);
                }
                if arrived {
                    vehicle.stopped_at = Some(self.state.frame);
                }
                // Committed past a barrier, it may now arrive at a junction beyond,
                // or be handed over further in one; released, handed back.
                if commits > 0 || released {
                    self.arrivals.push(id);
                }
                let spread = vehicle.spec.accel_noise;
                driver.noise =
                    acceleration::noise(driver.noise, spread, noise_step, &mut driver.rng);
                vehicle.accel = f64::INFINITY;
                let changer = &mut self.state.lane_changers[id];
                changer.lc.set_yielding(yielding);
                changer.old_follower = None;
            }
        }
        for update in &self.decisions {
            if let AccelUpdate::OldSide { id, accel }
            | AccelUpdate::Junction { id, accel }
            | AccelUpdate::Merge { id, accel } = *update
            {
                let vehicle = &mut self.state.vehicles[id];
                vehicle.accel = vehicle.accel.min(accel);
            }
        }
        for &(id, max_speed) in &self.state.caps {
            let vehicle = &mut self.state.vehicles[id];
            let term = acceleration::capped(vehicle.speed, max_speed);
            let source = AccelSource::Cap { max_speed };
            sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
            vehicle.accel = vehicle.accel.min(term);
        }
        self.held.clear();
        for update in &self.decisions {
            if let AccelUpdate::Own { id, accel, binding, .. } = *update {
                let vehicle = &mut self.state.vehicles[id];
                let others = vehicle.accel;
                vehicle.accel = accel.min(others);
                // Held up by that leader: without it, every other term would be left.
                if let Some(b) = binding {
                    let without = b.rest.min(others);
                    if b.term <= without {
                        let follower = OldFollower { id, without };
                        self.held.push((b.leader, follower));
                    }
                }
            }
        }
        // Each leader keeps the one it holds up most: the most it would gain.
        let SimState { vehicles, drivers, lane_changers, .. } = &mut self.state;
        for &(leader, follower) in &self.held {
            let gain = |f: &OldFollower| f.without - vehicles[f.id].accel;
            let changer = &mut lane_changers[leader];
            let better = changer.old_follower.is_none_or(|kept| gain(&follower) > gain(&kept));
            if better {
                changer.old_follower = Some(follower);
            }
        }
        // Slow and slowing, it finishes its stop, or stays put until it can
        // move off properly.
        for update in &self.decisions {
            if let AccelUpdate::Own { id, .. } = *update {
                let vehicle = &mut vehicles[id];
                let driver = &mut drivers[id];
                let (speed, accel) = (vehicle.speed, vehicle.accel);
                driver.stopping = acceleration::stopping(driver.stopping, speed, accel);
                if let Some(term) = driver.stopping.then_some(-acceleration::STOPPING_DECEL) {
                    let source = AccelSource::FinishingStop;
                    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                    vehicle.accel = vehicle.accel.min(term);
                }
            }
        }
    }

    /// Path tracking and integration: steers, moves, then advances each vehicle
    /// onto its next lane once its rear axle passes the end of its current one.
    /// A vehicle whose rear axle passes the end of a lane with no successors has
    /// left the network and is removed at the end, as [`Simulation::remove`] does.
    ///
    /// During a lane change, the target path is offset from the centerline along
    /// the change's cubic, and the change ends where the cubic does.
    pub(super) fn update_motion<S: DebugSink>(&mut self, dt: f64, sink: &mut S) {
        let Self {
            state: SimState { network, vehicles, occupants, routes, drivers, removals, .. },
            derived: Derived { overlaps, merges, through },
            arrivals,
            ..
        } = self;
        sink.record(|| DebugEvent::RecomputeMotion);
        for (id, vehicle) in vehicles.iter_mut() {
            let front = motion::drive(network, vehicle, dt);
            sink.record(|| DebugEvent::FrontAxle { vehicle: id, point: front });

            let lane = network.lane(vehicle.path[0]);
            // The barriers its rear axle passes, from where it was along the
            // lane it follows.
            let mut from = vehicle.lane_pos.s;
            let mut passed = 0;
            vehicle.lane_pos = lane.geometry().world_to_lane_near(vehicle.pos, vehicle.lane_pos);
            // Its path growing, or its entering a junction, may make it arrive at one.
            let mut arrives = false;
            while vehicle.path.len() > 1 {
                let length = network.lane(vehicle.path[0]).geometry().length();
                if vehicle.lane_pos.s < length {
                    break;
                }
                let previous = vehicle.path.remove(0);
                if vehicle.committed > 0 {
                    passed += barrier::between(network.lane(previous), from, f64::INFINITY);
                    from = f64::NEG_INFINITY;
                }
                let next = network.lane(vehicle.path[0]);
                arrives |= next.junction().is_some();
                let hint = next.geometry().lane_pos(vehicle.lane_pos.s - length, 0.0);
                vehicle.lane_pos = next.geometry().world_to_lane_near(vehicle.pos, hint);
                if let Some(change) = &mut vehicle.lane_change {
                    change.s0 -= length;
                }
                vehicle.front_s -= length;
                // Lanes can be shorter than a step's travel, so it may advance
                // again: it needs to know where it goes beyond this one.
                let s = vehicle.lane_pos.s;
                let needed = reach(vehicle, s, vehicle.lane_change);
                let lanes = vehicle.path.len();
                extend_path(network, &mut vehicle.path, &mut routes[id], s, needed);
                arrives |= vehicle.path.len() > lanes;
                // Carried over from the lane before, unless it enters a merge,
                // where it joins another lane's traffic.
                let followed = vehicle.path[0];
                let enters_merge =
                    matches!(merges.of(followed), Some(m) if merges.of(previous) != Some(m));
                following::advance(
                    network,
                    overlaps,
                    occupants,
                    id,
                    vehicle,
                    previous,
                    enters_merge,
                );
            }

            if vehicle.committed > 0 {
                passed += barrier::between(network.lane(vehicle.path[0]), from, vehicle.lane_pos.s);
                vehicle.committed = vehicle.committed.saturating_sub(passed.min(255) as u8);
                if passed > 0 {
                    acceleration::draw_line_gap(&mut drivers[id], &vehicle.spec);
                    vehicle.stopped_at = None;
                }
            }

            // Far enough ahead to know where it is going, at its new speed.
            let s = vehicle.lane_pos.s;
            let needed = reach(vehicle, s, vehicle.lane_change);
            let lanes = vehicle.path.len();
            extend_path(network, &mut vehicle.path, &mut routes[id], s, needed);
            arrives |= vehicle.path.len() > lanes;
            if arrives {
                raise_through(network, through, &vehicle.path);
            }
            if arrives {
                arrivals.push(id);
            }

            // Past the end with nowhere to advance to: it has left the network.
            let lane = network.lane(vehicle.path[0]);
            if vehicle.lane_pos.s >= lane.geometry().length() && lane.successors().is_empty() {
                removals.push(id);
            }

            if vehicle.lane_change.is_some_and(|change| vehicle.lane_pos.s >= change.end()) {
                vehicle.lane_change = None;
            }
        }
        // As a removal queued through the API would be, so ghosts and any
        // other records go too.
        self.apply_removals(sink);
    }

    /// Brings the junctions' queues up to date at the start of a macro step,
    /// just before the decisions that read them, from the motion and occupancy
    /// since the last: vehicles whose path grew or that entered a junction arrive
    /// where nothing holds them back (see [`arrive`](crate::junction::arrive)), committed vehicles clear
    /// of a junction leave its queue, those that can plainly stop are released
    /// (see `junction::may_release`), and each lane approached is scanned for vehicles
    /// to commit (see `junction::commit_along`).
    pub(super) fn update_queues(&mut self) {
        let Self { state, arrivals, scan_empty, scan_kept, .. } = self;
        junction::update(state, arrivals, scan_empty, scan_kept);
    }

    /// Refreshes the occupant lists, and clears the old side of vehicles that have
    /// finished leaving it.
    pub(super) fn update_occupancy(&mut self, dt: f64) {
        let Self {
            state: SimState { network, vehicles, occupants, .. },
            departed,
            ..
        } = self;
        following::update(network, vehicles, occupants, departed, dt);
        // A vehicle straddles while it still has a record on the lane beside the
        // one it follows, on its old side, or, just after a section boundary, on
        // the lane beside the one before: at a lane drop that is all there is.
        // Losing a record elsewhere, such as at the end of the lane before, leaves
        // it straddling.
        for id in departed.drain(..) {
            let vehicle = &mut vehicles[id];
            let Some(side) = vehicle.old_side else {
                continue;
            };
            let on = |lane: LaneId| {
                network.lane(lane).adj(side).is_some_and(|old| {
                    occupants[old.lane.0 as usize].iter().any(|o| o.is_record_of(id))
                })
            };
            let following = vehicle.path[0];
            let straddling =
                on(following) || network.lane(following).predecessors().iter().any(|&p| on(p));
            if !straddling {
                vehicle.old_side = None;
            }
        }
    }
}
