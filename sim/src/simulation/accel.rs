//! One vehicle's acceleration, as the longitudinal update puts it together:
//! the lowest of every term it is subject to. The free road and each leader
//! along its path, barriers as gap acceptance decides at them, and the
//! lane-change model's terms; while it straddles, its old lanes; and the terms
//! junctions and merges give it.

use slotmap::SlotMap;

use super::SimState;
use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink};
use std::ops::ControlFlow;

use crate::following::{Ahead, Along, Found, find_leaders, for_each_leader};
use crate::junction::{self, Queue};
use crate::merge::{self, Merges};
use crate::models::acceleration;
use crate::models::gap_acceptance::{self, Answer};
use crate::models::signal::Light;
use crate::network::{BarrierKind, JunctionId, LaneId, Side, SimNetwork};
use crate::query::{LaneSpeeds, friction, speed_of};
use crate::vehicle::{Vehicle, VehicleId, VehicleSpec};

/// What the longitudinal update decided for a vehicle, from one of its records.
#[derive(Clone, Copy, Debug)]
pub(super) enum AccelUpdate {
    /// From its record on the lane it follows: its acceleration, the leader
    /// whose term is the lowest there, if any, and the nearest ghost ahead, if
    /// it comes before anything else, with the gap to it.
    Own {
        id: VehicleId,
        accel: f64,
        binding: Option<Binding>,
        ghost: Option<(VehicleId, f64)>,
        /// How many barriers ahead it is committed to now.
        committed: u8,
        /// Whether it has just come to rest at the all-way stop line it heads for.
        arrived: bool,
    },
    /// From its record on its old lane, while it straddles it: the most its old
    /// side allows.
    OldSide { id: VehicleId, accel: f64 },
    /// From a junction whose queue it is in: the most the committed vehicles
    /// ahead of it there allow.
    Junction { id: VehicleId, accel: f64 },
    /// From a merge it is approaching: the most the vehicle before it in the
    /// merge's order allows.
    Merge { id: VehicleId, accel: f64 },
    /// From the lane-change model: the most its terms allow, and whose ghost it
    /// is making room for, if anyone's.
    LaneChange { id: VehicleId, accel: f64, yielding: Option<VehicleId> },
}

/// The leader whose term was the lowest in a vehicle's search along its path.
#[derive(Clone, Copy, Debug)]
pub(super) struct Binding {
    pub(super) leader: VehicleId,
    /// Its term.
    pub(super) term: f64,
    /// The lowest of every other term from that search: the free road, other
    /// leaders and the end of the path.
    pub(super) rest: f64,
}

/// The harshest of a vehicle's terms for lower speeds ahead along its path, on
/// its lanes' speed profiles (see [`acceleration::limit_ahead`]), as far ahead
/// as they can matter, each recorded; infinite if there are none.
fn limits_ahead<D: DebugSink>(
    state: &SimState,
    profiles: &acceleration::Profiles,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
) -> f64 {
    let network = &state.network;
    let spec = &vehicle.spec;
    let reach = acceleration::limit_reach(spec, vehicle.speed);
    let front = front_along(vehicle);
    let driver = state.drivers.traits(id);
    let mut harshest = f64::INFINITY;
    // Where each lane of its path starts, along the first.
    let mut start = 0.0;
    for &lane in &vehicle.path {
        if start - front > reach {
            break;
        }
        for &(s, speed) in profiles.of(lane) {
            let gap = start + s - front;
            if gap <= 0.0 || gap > reach {
                continue;
            }
            let limit = driver.desired_speed(spec, speed);
            if let Some(accel) = acceleration::limit_ahead(spec, vehicle.speed, gap, limit) {
                let source = AccelSource::LimitAhead { lane, limit, gap };
                sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel });
                harshest = harshest.min(accel);
            }
        }
        start += network.lane(lane).geometry().length();
    }
    harshest
}

/// How far along its path a vehicle's front is, from the start of the lane it
/// follows.
fn front_along(vehicle: &Vehicle) -> f64 {
    vehicle.lane_pos.s - vehicle.spec.rear_axle + vehicle.spec.length
}

/// The speed a vehicle's lanes' profiles allow at its front (see
/// [`acceleration::Profiles`]), or at the end of its path, beyond it.
fn speed_at_front(
    network: &SimNetwork,
    profiles: &acceleration::Profiles,
    vehicle: &Vehicle,
) -> f64 {
    let mut s = front_along(vehicle);
    for &lane in &vehicle.path {
        let length = network.lane(lane).geometry().length();
        if s <= length {
            return profiles.at(lane, s);
        }
        s -= length;
    }
    let last = *vehicle.path.last().expect("a path has a lane");
    profiles.at(last, f64::INFINITY)
}

/// A vehicle's acceleration under the longitudinal model along its own lanes,
/// and the nearest ghost ahead of it, for the lane-change model. `own` is the
/// index of its record in the occupants of the lane it follows.
///
/// The acceleration is the minimum over every term the model considers there:
/// the free road, each leader along its path, the end of its path and the
/// barriers on it. While it straddles, its old side adds more (see [`old_side`]). Each
/// term is recorded to the debug sink as it is computed.
#[allow(clippy::too_many_arguments)]
pub(super) fn longitudinal<D: DebugSink>(
    state: &SimState,
    through: &[f64],
    profiles: &acceleration::Profiles,
    speeds: &[LaneSpeeds],
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    own: usize,
) -> AccelUpdate {
    let limit = speed_at_front(&state.network, profiles, vehicle);
    let desired_speed = state.drivers.desired_speed(id, &vehicle.spec, limit);
    let desired_speed = desired_speed.min(friction(state, speeds, vehicle));
    // Drivers don't hold a speed exactly: their noise wanders it.
    let free = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed)
        + state.drivers.noise(id);
    let source = AccelSource::FreeRoad { desired_speed };
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: free });
    // The lowest of every term but the binding leader's, and that leader.
    let mut rest = free.min(limits_ahead(state, profiles, sink, id, vehicle));
    let mut binding: Option<Binding> = None;

    // The nearest ghost, if it comes before anything else ahead.
    let mut ghost = None;
    let mut first = true;
    // Wanting to change lanes, it keeps room to get halfway through a change
    // from rest.
    let halfway = acceleration::kept_room(&state.network, vehicle);
    // How many barriers ahead it is committed to, in order: those it is released
    // from drop out, with all after them. The nearest it hasn't committed to is
    // the head's to decide about, once the search is done; the search stops
    // there, unless the barrier may let it pass for now, a green light or a
    // keep-clear zone, when it goes on to find the leaders beyond, and stops at
    // the next.
    let mut committed = vehicle.committed;
    let mut head = None;
    let reach = halfway.map_or(0.0, |h| vehicle.spec.min_gap + h);
    let _ = find_leaders(state, vehicle, Along::Path(own), reach, |found| {
        if let Found::Ahead(Ahead::Barrier { lane, index, rank }, gap, _) = found {
            let barrier = state.network.lane(lane).barriers()[usize::from(index)];
            let gap = barrier::gap_to(vehicle, &barrier, gap);
            if rank < u16::from(committed) {
                if !barrier::released(state, vehicle, barrier, gap) {
                    return ControlFlow::Continue(());
                }
                committed = rank as u8;
            }
            // Found in turn, it is the head's; out of turn, beyond one shielded,
            // it waits.
            if head.is_none() && rank == u16::from(committed) {
                head = Some((lane, index, barrier, gap));
            }
            return match barrier.kind {
                BarrierKind::Signal { movement, .. }
                    if head.is_some_and(|(l, i, ..)| (l, i) == (lane, index))
                        && state.lights[movement.0 as usize].light == Light::Green =>
                {
                    ControlFlow::Continue(())
                }
                BarrierKind::KeepClear { .. }
                    if head.is_some_and(|(l, i, ..)| (l, i) == (lane, index)) =>
                {
                    ControlFlow::Continue(())
                }
                _ => ControlFlow::Break(()),
            };
        }
        if let Found::Ahead(ahead, gap, _) = found {
            if let (true, Ahead::Ghost(owner)) = (first, ahead) {
                ghost = Some((owner, gap));
            }
            first = false;
        }
        let Some(term) = react(&state.vehicles, sink, id, vehicle, found, halfway) else {
            return ControlFlow::Continue(());
        };
        match found {
            // On a loop shorter than its horizon it can find itself ahead, which
            // holds nobody up.
            Found::Ahead(Ahead::Vehicle(leader), ..) if leader != id => {
                if let Some(b) = binding.filter(|b| b.term <= term) {
                    binding = Some(b);
                    rest = rest.min(term);
                } else {
                    if let Some(b) = binding {
                        rest = rest.min(b.term);
                    }
                    binding = Some(Binding { leader, term, rest: 0.0 });
                }
            }
            _ => rest = rest.min(term),
        }
        ControlFlow::Continue(())
    });

    // The head's barrier: gap acceptance decides, at most one a step, and the
    // core carries it out. Committing, it sees beyond at the next update.
    let mut arrived = false;
    if let Some((lane, index, barrier, gap)) = head {
        let k = vehicle.path.iter().position(|&l| l == lane).unwrap_or(0);
        let answer = gap_acceptance::decide(state, through, profiles, id, vehicle, k, barrier, gap);
        let light = match barrier.kind {
            BarrierKind::Signal { movement, .. } => Some(state.lights[movement.0 as usize].light),
            _ => None,
        };
        debug_assert!(
            answer != Answer::Pass
                || light == Some(Light::Green)
                || matches!(barrier.kind, BarrierKind::KeepClear { .. }),
            "only a green light or a keep-clear zone lets a vehicle pass for now: {answer:?} at {barrier:?}"
        );
        let line_gap = state.drivers.line_gap(id);
        // At rest at a give-way, stop or all-way stop line, or a red light it
        // may turn on, as the head on its way to it: it has arrived there.
        // Not at rest further back, behind a vehicle headed elsewhere.
        let stops = match barrier.kind {
            BarrierKind::GiveWay | BarrierKind::Stop => true,
            BarrierKind::Signal { on_red, .. } => on_red.is_some() && light == Some(Light::Red),
            _ => false,
        };
        if stops
            && vehicle.speed == 0.0
            && gap <= line_gap + barrier::AT_LINE
            && vehicle.stopped_at.is_none()
        {
            arrived = true;
        }
        let (decision, yields_to) = match answer {
            Answer::Commit => (barrier::Decision::Commit, None),
            Answer::Pass => (barrier::Decision::Pass, None),
            Answer::Yield(_) | Answer::Stop => {
                let term = acceleration::hold(&vehicle.spec, line_gap, vehicle.speed, gap);
                let kind = barrier.kind;
                let source = AccelSource::Barrier { kind, lane, index, gap };
                sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                rest = rest.min(term);
                let yields_to = match answer {
                    Answer::Yield(other) => Some(other),
                    _ => None,
                };
                (barrier::Decision::Hold(term), yields_to)
            }
        };
        if answer == Answer::Commit {
            committed = committed.saturating_add(1);
        }
        sink.record(|| DebugEvent::Barrier { vehicle: id, lane, index, gap, decision, yields_to });
    }

    let binding = binding.map(|b| Binding { rest, ..b });
    let accel = binding.map_or(rest, |b| b.term.min(rest));
    AccelUpdate::Own { id, accel, binding, ghost, committed, arrived }
}

/// The terms the vehicles in a junction's queue impose on those below them,
/// at most one per vehicle, the lowest, each recorded to the debug sink: from
/// each one above it that it is a decided pair with (see
/// `junction::decided`), for every conflict between a lane of its path in the
/// junction and a lane of the junction's, crossing or merging alike (see
/// `junction::conflict_term`).
///
/// O(q²) in the number queued, which is small.
pub(super) fn junction_terms<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    queue: &Queue,
    decisions: &mut Vec<AccelUpdate>,
) {
    let order = &queue.order;
    for (k, &id) in order.iter().enumerate() {
        let mut accel = f64::INFINITY;
        for &other in &order[..k] {
            if junction::decided(state, junction, other, id) {
                accel = accel.min(junction::term(state, sink, junction, id, other));
            }
        }
        if accel < f64::INFINITY {
            decisions.push(AccelUpdate::Junction { id, accel });
        }
    }
}

/// Each vehicle in a merge's order takes a term from the one before it (see
/// [`merge::term`]), each recorded to the debug sink.
pub(super) fn merge_terms<D: DebugSink>(
    state: &SimState,
    merges: &Merges,
    sink: &mut D,
    merge: usize,
    decisions: &mut Vec<AccelUpdate>,
) {
    let SimState { network, vehicles, occupants, .. } = state;
    let mut before: Option<merge::Member> = None;
    merge::order(network, merges, vehicles, occupants, merge, |follower| {
        let leader = before.replace(follower);
        let Some((accel, gap)) = leader.and_then(|l| merge::term(vehicles, &follower, &l)) else {
            return;
        };
        let (id, source) = (
            follower.id(),
            AccelSource::Merge { id: leader.unwrap().id(), gap },
        );
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel });
        decisions.push(AccelUpdate::Merge { id, accel });
    });
}

/// The side of `lane` a vehicle is straddling it from, if it is partway through a
/// lane change out of it: `lane` is beside the lane it follows, on its old side.
pub(super) fn straddling(network: &SimNetwork, vehicle: &Vehicle, lane: LaneId) -> Option<Side> {
    let side = vehicle.old_side.filter(|_| vehicle.lane_change.is_some())?;
    let old = network.lane(vehicle.path[0]).adj(side)?;
    (old.lane == lane).then_some(side)
}

/// The most a vehicle straddling its old lane, on `side` of the lane it follows,
/// may accelerate for what is ahead of it there: a leader search from its record
/// there, `own` in the old lane's occupants, along the neighbors of its path, so it doesn't run into its old leader. Once the change is complete,
/// nothing on the old lanes can be in its way, so it isn't asked. Each term is
/// recorded to the debug sink as it is computed.
pub(super) fn old_side<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    side: Side,
    own: usize,
) -> f64 {
    let mut accel = f64::INFINITY;
    for_each_leader(state, vehicle, Along::Leaving(side, own), 0.0, |found| {
        if let Some(term) = react(&state.vehicles, sink, id, vehicle, found, None) {
            accel = accel.min(term);
        }
    });
    accel
}

/// Records what vehicle `id`'s leader search found and, for a leader or a stop
/// line, returns the term it imposes. Ghosts aren't leaders here: the nearest is
/// weighed up for making room. With `halfway`, the distance its rear axle needs
/// to get halfway through a lane change it wants to make, it keeps that much
/// room behind a leader before it would have to stop (as the lane-change model weighs room to finish),
/// so it isn't boxed in: as VISSIM's minimum clearance for changing lanes from
/// a standstill, and the room drivers are taught to leave in a queue.
fn react<D: DebugSink>(
    vehicles: &SlotMap<VehicleId, Vehicle>,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    found: Found,
    halfway: Option<f64>,
) -> Option<f64> {
    let (ahead, gap, headway) = match found {
        Found::Ahead(ahead, gap, headway) => (ahead, gap, headway),
        Found::Pass(check) => {
            sink.record(|| DebugEvent::PassCheck { vehicle: id, check });
            return None;
        }
    };
    let source = match ahead {
        Ahead::Vehicle(leader) => AccelSource::Leader { id: leader, gap, headway },
        Ahead::PathEnd => AccelSource::PathEnd { gap },
        Ahead::Ghost(_) | Ahead::Barrier { .. } => return None,
    };
    let speed = speed_of(vehicles, ahead);
    // Wanting to change lanes, it keeps room behind a leader to get halfway
    // through a change before it would have to stop, so it isn't boxed in: that
    // room is the minimum gap of a second term, which IDM settles on from above,
    // so the room it keeps is never short.
    let mut term = acceleration::relaxed(&vehicle.spec, vehicle.speed, gap, speed, headway);
    if let (Some(halfway), Ahead::Vehicle(leader)) = (halfway, ahead) {
        let short = halfway - vehicles[leader].comfortable_stop();
        if short > 0.0 {
            // A preference, not safety: never braking harder than comfortably.
            let min_gap = vehicle.spec.min_gap + short;
            let spec = VehicleSpec { min_gap, ..vehicle.spec };
            let room = acceleration::relaxed(&spec, vehicle.speed, gap, speed, headway);
            term = term.min(room.max(-vehicle.spec.comfy_decel));
        }
    }
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
    Some(term)
}
