//! One vehicle's acceleration, as the longitudinal update puts it together:
//! the lowest of every term it is subject to. The free road and each leader
//! along its path, barriers as gap acceptance decides at them, and the
//! lane-change model's terms; while it straddles, its old lanes; and the terms
//! junctions and merges give it.

use slotmap::SlotMap;

use super::SimState;
use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink};
use crate::following::{Ahead, Along, Found, for_each_leader};
use crate::junction::{self, Entry, Queue, junction_term};
use crate::merge::{self, Merges};
use crate::models::acceleration;
use crate::models::gap_acceptance;
use crate::models::lane_change;
use crate::models::signal::Light;
use crate::network::{BarrierKind, JunctionId, LaneId, Side, SimNetwork};
use crate::query::{LaneSpeeds, friction, speed_of};
use crate::vehicle::{Vehicle, VehicleId, VehicleSpec};

/// What the longitudinal update decided for a vehicle, from one of its records.
#[derive(Clone, Copy, Debug)]
pub(super) enum AccelUpdate {
    /// From its record on the lane it follows: its acceleration, whose ghost it is
    /// making room for, and the leader whose term is the lowest there, if any.
    Own {
        id: VehicleId,
        accel: f64,
        yielding: Option<VehicleId>,
        binding: Option<Binding>,
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
}

/// The leader whose term was the lowest in a vehicle's search along its path.
#[derive(Clone, Copy, Debug)]
pub(super) struct Binding {
    pub(super) leader: VehicleId,
    /// Its term.
    pub(super) term: f64,
    /// The lowest of every other term from that search: the free road, other
    /// leaders, the end of the path and the ghost it makes room for.
    pub(super) rest: f64,
}

/// A vehicle's acceleration under the longitudinal model, and whose ghost it is
/// making room for, if anyone's. `own` is the index of its record in the occupants of the lane it
/// follows.
///
/// The acceleration is the minimum over every term the model considers: the free
/// road, each leader along its path, the end of its path, and the ghost it makes
/// room for. While it straddles, its old side adds more (see [`old_side`]). Each
/// term is recorded to the debug sink as it is computed.
pub(super) fn longitudinal<D: DebugSink>(
    state: &SimState,
    through: &[f64],
    speeds: &[LaneSpeeds],
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    own: usize,
) -> AccelUpdate {
    let lane = vehicle.path[0];
    let desired_speed = vehicle.spec.desired_speed(state.network.lane(lane).speed_limit());
    let desired_speed = desired_speed.min(friction(state, speeds, vehicle));
    // Drivers don't hold a speed exactly: their noise wanders it.
    let free = acceleration::free_road(&vehicle.spec, vehicle.speed, desired_speed)
        + state.drivers[id].noise;
    let source = AccelSource::FreeRoad { desired_speed };
    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: free });
    // The lowest of every term but the binding leader's, and that leader.
    let mut rest = free;
    let mut binding: Option<Binding> = None;

    // The nearest ghost, if it comes before anything else ahead.
    let mut ghost = None;
    let mut first = true;
    // Wanting to change lanes, it keeps room to get halfway through a change
    // from rest.
    let halfway = lane_change::kept_room(state, id, vehicle);
    // How many barriers ahead it is committed to, in order: those it is released
    // from drop out, with all after them, and once one holds it, or one beyond is
    // found out of turn behind one shielded, the rest wait.
    let mut committed = vehicle.committed;
    let mut held = false;
    let mut arrived = false;
    let reach = halfway.map_or(0.0, |h| vehicle.spec.min_gap + h);
    for_each_leader(state, vehicle, Along::Path(own), reach, |found| {
        if let Found::Ahead(Ahead::Barrier { lane, index, rank }, gap, _) = found {
            let barrier = state.network.lane(lane).barriers()[usize::from(index)];
            let gap = barrier::gap_to(vehicle, &barrier, gap);
            if rank < u16::from(committed) {
                if !barrier::released(state, id, vehicle, lane, barrier, gap) {
                    return;
                }
                committed = rank as u8;
            }
            if held || rank != u16::from(committed) {
                held = true;
                return;
            }
            let k = vehicle.path.iter().position(|&l| l == lane).unwrap_or(0);
            let (decision, yields_to) =
                gap_acceptance::decide(state, through, id, vehicle, k, barrier, gap);
            // At rest at a give-way, stop or all-way stop line, or a red light it
            // may turn on, as the head on its way to it: it has arrived there.
            // Not at rest further back, behind a vehicle headed elsewhere.
            let stops = match barrier.kind {
                BarrierKind::GiveWay | BarrierKind::Stop | BarrierKind::AllWayStop => true,
                BarrierKind::Signal { movement, on_red } => {
                    on_red.is_some() && state.lights[movement.0 as usize].light == Light::Red
                }
                _ => false,
            };
            if stops
                && vehicle.speed == 0.0
                && gap <= state.drivers[id].line_gap + barrier::AT_LINE
                && vehicle.stopped_at.is_none()
            {
                arrived = true;
            }
            sink.record(|| DebugEvent::Barrier {
                vehicle: id,
                lane,
                index,
                gap,
                decision,
                yields_to,
            });
            match decision {
                barrier::Decision::Commit => committed = committed.saturating_add(1),
                barrier::Decision::Pass => held = true,
                barrier::Decision::Hold(term) => {
                    held = true;
                    let source = AccelSource::Barrier { lane, index, gap };
                    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                    rest = rest.min(term);
                }
            }
            return;
        }
        if let Found::Ahead(ahead, gap, _) = found {
            if let (true, Ahead::Ghost(owner)) = (first, ahead) {
                ghost = Some((owner, gap));
            }
            first = false;
        }
        let Some(term) = react(&state.vehicles, sink, id, vehicle, found, halfway) else {
            return;
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
    });

    // The lane-change model's terms: making room for the ghost, and
    // synchronising with the target lane.
    let (term, yielding) = lane_change::terms(state, sink, id, vehicle, ghost);
    rest = rest.min(term);
    let binding = binding.map(|b| Binding { rest, ..b });
    let accel = binding.map_or(rest, |b| b.term.min(rest));
    AccelUpdate::Own { id, accel, yielding, binding, committed, arrived }
}

/// The terms the vehicles committed at a junction impose on each one committed
/// after them, and on the vehicles at the front of the ways in, not yet
/// committed, at most one per follower, the lowest, each recorded to the debug
/// sink.
///
/// For each pair, ahead and behind, every conflict between a lane of the
/// follower's path in the junction and a lane of the junction's counts, crossing
/// or merging alike: see `junction::conflict_term`.
///
/// O(c·(c + f)) in the numbers committed and at the fronts, which are small.
pub(super) fn junction_terms<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    queue: &Queue,
    decisions: &mut Vec<AccelUpdate>,
) {
    let committed = &queue.committed;
    // Each entry for the stretch it covers: from where the vehicle's entry
    // before it stops covering it.
    let followers = committed.iter().enumerate().map(|(k, e)| {
        let lane = |b: &Entry| b.until(&state.network, junction);
        let from = committed[..k].iter().rev().find(|b| b.id == e.id).and_then(lane);
        (e.id, &committed[..k], [from, lane(e)])
    });
    // The vehicles at the front of the ways in, not yet committed, are behind
    // every committed one.
    let fronts = queue.fronts.iter().map(|&id| (id, &committed[..], [None; 2]));
    for (id, ahead, stretch) in followers.chain(fronts) {
        let accel = junction_term(state, sink, junction, id, ahead, stretch);
        if accel < f64::INFINITY {
            decisions.push(AccelUpdate::Junction { id, accel });
        }
    }
    // Those it holds out of its queue, short of their next conflicts.
    for &id in &queue.held {
        let vehicle = &state.vehicles[id];
        if let Some((accel, gap)) =
            junction::hold_term(&state.network, junction, committed, id, vehicle)
        {
            let source = AccelSource::JunctionHold { gap };
            sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel });
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
