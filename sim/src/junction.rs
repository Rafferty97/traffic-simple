//! The junction backstop (see "Conflicts, merging and junctions" in
//! `docs/DESIGN.md`): each junction's queue of the vehicles committed there,
//! which can no longer stop short of their conflicts, who arrives, commits and
//! is released, and the terms committed vehicles impose on those behind them.

use arrayvec::ArrayVec;
use glam::DVec2;
use slotmap::SlotMap;

use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink};
use crate::models::acceleration;
use crate::models::signal::MovementLight;
use crate::network::{Conflict, JunctionId, LaneId, SimNetwork};
use crate::occupancy::Occupant;
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// A vehicle's place in a junction's queue, for a stretch of its way through:
/// the stretch its barriers have handed it over for (see [`handed_over`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Entry {
    pub id: VehicleId,
    /// The lane of its path, in the junction, from which this entry no longer
    /// covers it, where its next barrier is: `None` if it covers it right
    /// through.
    pub until: Option<LaneId>,
}

impl Entry {
    /// Whether it covers `vehicle`, its vehicle, on `lane`: `lane` comes before
    /// its `until` on its path, or isn't on it any more, being behind it.
    pub(crate) fn covers(&self, vehicle: &Vehicle, lane: LaneId) -> bool {
        let Some(until) = self.until else {
            return true;
        };
        let at = |l: LaneId| vehicle.path.iter().position(|&p| p == l);
        match (at(until), at(lane)) {
            (Some(until), Some(lane)) => lane < until,
            _ => true,
        }
    }
}

/// A junction's queue: the vehicles committed there, which can no longer stop
/// short of their conflicts, in the order they committed, and the lanes that
/// vehicles not yet committed may be approaching it along.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Queue {
    /// Each holds back every vehicle committed after it, on the lanes it
    /// covers. A vehicle handed over in two stretches, as a turner waiting
    /// inside the junction for a gap is, has an entry for each, the second
    /// joining at the back when it commits to the barrier between them.
    pub committed: Vec<Entry>,
    /// Every lane on the way of a vehicle that has arrived, from the lane it
    /// was on to the first of the junction's lanes with a conflict on its path,
    /// that lane included, sorted and without repeats. The junction's scans
    /// for vehicles to commit start from its own lanes here and follow only
    /// lanes here (see [`commit_along`]). Vehicles only add lanes; the scans
    /// drop those they find no one on.
    pub approaching: Vec<LaneId>,
    /// The vehicles not yet committed that the last scans stopped at, each
    /// headed onto the lane scanned, so the first on its way: the committed
    /// vehicles hold these back too (see [`junction_term`]). Those behind them
    /// follow them. Refilled every step.
    pub fronts: Vec<VehicleId>,
}

/// Brings the junctions' queues up to date after a step's motion and
/// occupancy: vehicles whose path grew or that entered a junction arrive
/// where nothing holds them back (see [`arrive`]), committed vehicles clear
/// of a junction leave its queue, those that can plainly stop are released
/// (see [`may_release`]), and each lane approached is scanned for vehicles
/// to commit (see [`commit_along`]).
pub(crate) fn update(
    state: &mut SimState,
    arrivals: &mut Vec<VehicleId>,
    scan_empty: &mut Vec<LaneId>,
    scan_kept: &mut Vec<LaneId>,
) {
    let SimState { network, vehicles, occupants, queues, lights, .. } = state;
    // Some may have left the network since.
    for &id in arrivals.iter() {
        if let Some(vehicle) = vehicles.get(id) {
            arrive(network, occupants, queues, id, vehicle);
        }
    }
    arrivals.clear();
    for (j, queue) in queues.iter_mut().enumerate() {
        let junction = JunctionId(j as u32);
        // Headed into it, or still on it.
        queue.committed.retain(|e| {
            headed(network, &vehicles[e.id], junction)
                || entered(network, occupants, junction, e.id)
        });
        // From the back, so a lane's vehicles are released back to front in
        // one pass. A released vehicle's way in is marked again, as the
        // scans passed it over while it was committed.
        for k in (0..queue.committed.len()).rev() {
            if k < queue.committed.len()
                && may_release(network, vehicles, occupants, junction, &queue.committed, k)
            {
                let id = queue.committed[k].id;
                queue.committed.retain(|e| e.id != id);
                mark_way(
                    network,
                    &mut queue.approaching,
                    &vehicles[id].path,
                    junction,
                );
            }
        }
        queue.fronts.clear();
        if queue.approaching.is_empty() {
            continue;
        }
        let Queue { committed, approaching, fronts } = queue;
        let mut scan = Scan {
            network,
            vehicles,
            occupants,
            lights,
            junction,
            approaching,
            empty: scan_empty,
            kept: scan_kept,
            fronts,
        };
        scan.empty.clear();
        scan.kept.clear();
        for &lane in scan.approaching {
            if network.lane(lane).junction() == Some(junction) {
                commit_along(&mut scan, lane, committed);
            }
        }
        // A lane stays while some scan found someone on it or beyond, or none
        // reached it, so whoever is on it is behind someone found.
        approaching.retain(|lane| !scan_empty.contains(lane) || scan_kept.contains(lane));
    }
}

/// Makes a vehicle arrive at each junction on its path that nothing holds it
/// back from: up to and including the first junction it hasn't physically
/// entered (see [`entered`]), and short of any junction a barrier it hasn't
/// committed to holds it back from, at or before the junction's first conflict
/// on its path: it arrives there once it commits. At each, the lanes of its path up to the first of the
/// junction's lanes with a conflict, that lane included, join those it is
/// approached along, so the junction's scans find it.
pub(crate) fn arrive(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    queues: &mut [Queue],
    id: VehicleId,
    vehicle: &Vehicle,
) {
    let mut last = None;
    for &lane in &vehicle.path {
        let Some(j) = network.lane(lane).junction() else {
            continue;
        };
        if last == Some(j) {
            continue;
        }
        // In its queue already, it is handed over afresh.
        let queue = &mut queues[j.0 as usize].committed;
        if queue.iter().any(|e| e.id == id) {
            hand_over(network, queue, id, vehicle, j);
            last = Some(j);
            continue;
        }
        if let Some(previous) = last
            && !entered(network, occupants, previous, id)
        {
            return;
        }
        let into = |k: usize| {
            let lane = vehicle.path[k];
            network.lane(lane).junction() == Some(j) && conflict_start(network, lane).is_some()
        };
        if let Some(k) = (0..vehicle.path.len()).find(|&k| into(k)) {
            let start = conflict_start(network, vehicle.path[k]).unwrap_or(0.0);
            if barrier::holds_before(network, vehicle, k, start) {
                return;
            }
        }
        mark_way(
            network,
            &mut queues[j.0 as usize].approaching,
            &vehicle.path,
            j,
        );
        last = Some(j);
    }
}

/// Brings `id`'s entries in a junction's queue, `queue`, up to date with how
/// far its barriers now hand it over (see [`handed_over`]): further than its
/// last entry covers, having committed to the barrier there, it joins the back
/// again for the rest; released from a barrier it had committed to, the
/// entries from there on go.
fn hand_over(
    network: &SimNetwork,
    queue: &mut Vec<Entry>,
    id: VehicleId,
    vehicle: &Vehicle,
    junction: JunctionId,
) {
    let until = handed_over(network, vehicle, junction);
    let mine = || queue.iter().enumerate().filter(|(_, e)| e.id == id).map(|(k, _)| k);
    let Some(last) = mine().next_back() else {
        return;
    };
    // Released short of where an entry of it starts: that one goes, and those
    // after it.
    let mut start = None;
    let starts = mine().find(|&k| {
        let at = start;
        start = queue[k].until;
        at.is_some() && at == until
    });
    if let Some(k) = starts {
        let mut i = 0;
        queue.retain(|e| {
            i += 1;
            i <= k || e.id != id
        });
        return;
    }
    match (queue[last].until, until) {
        // Handed over further: it joins the back again for the rest.
        (Some(covered), _) if until != Some(covered) => queue.push(Entry { id, until }),
        // Released short of the end of its last entry, which covered it right
        // through: it covers it only so far now.
        (None, Some(_)) => queue[last].until = until,
        _ => {}
    }
}

/// How far `vehicle`'s barriers hand it over to `junction`: up to the lane
/// of its path where the next barrier it hasn't committed to is, if that is in
/// the junction, or right through (`None`).
fn handed_over(network: &SimNetwork, vehicle: &Vehicle, junction: JunctionId) -> Option<LaneId> {
    barrier::nth(network, vehicle, usize::from(vehicle.committed))
        .map(|(lane, _)| lane)
        .filter(|&lane| network.lane(lane).junction() == Some(junction))
}

/// Adds the lanes of `path` up to the first of `junction`'s lanes on it with a
/// conflict, that lane included, to `approaching`, kept sorted.
fn mark_way(
    network: &SimNetwork,
    approaching: &mut Vec<LaneId>,
    path: &[LaneId],
    junction: JunctionId,
) {
    let into = |&lane: &LaneId| {
        network.lane(lane).junction() == Some(junction) && conflict_start(network, lane).is_some()
    };
    let Some(k) = path.iter().position(into) else {
        return;
    };
    for &lane in &path[..=k] {
        if let Err(i) = approaching.binary_search_by_key(&lane.0, |l| l.0) {
            approaching.insert(i, lane);
        }
    }
}

/// What a junction's scans for vehicles to commit read, and where they note
/// which lanes they found anyone on.
struct Scan<'a> {
    network: &'a SimNetwork,
    vehicles: &'a SlotMap<VehicleId, Vehicle>,
    occupants: &'a [Vec<Occupant>],
    /// The signals' lights, for whether one holds a vehicle back.
    lights: &'a [MovementLight],
    junction: JunctionId,
    /// The lanes vehicles approach the junction along, sorted.
    approaching: &'a [LaneId],
    /// Lanes a scan found no one on, nor beyond.
    empty: &'a mut Vec<LaneId>,
    /// Lanes a scan found someone on, or beyond.
    kept: &'a mut Vec<LaneId>,
    /// The vehicles not yet committed the scans stopped at, headed onto the
    /// lane scanned.
    fronts: &'a mut Vec<VehicleId>,
}

/// How many lanes a scan back from a junction's lane holds at once: that lane
/// and those it has come back through.
const SCAN_DEPTH: usize = 32;

/// Commits the vehicles approaching `lane`, one of the scan's junction's lanes
/// with a conflict, that must commit now, nearest first, noting on the way
/// which lanes it found anyone on.
///
/// It scans back from the start of the lane's conflict, a lane being in at most
/// one junction and so having at most one conflict zone: along its occupants,
/// then the lanes leading into it, branch by branch, following only lanes
/// vehicles approach the junction along. On each branch it passes over ghosts,
/// committed vehicles and vehicles whose path has left the way to `lane` and
/// goes nowhere in the junction. At the first other one:
///
/// - one headed onto `lane` commits if it must (see [`must_commit`]) and the
///   scan goes on behind it; if it needn't yet, the scan stops there, since
///   nobody behind it may commit before it;
/// - one headed elsewhere in the junction, or whose path ends before it could
///   leave the way to `lane`, stops the scan: it may yet go in ahead of those
///   behind it, and another lane's scan, or a later one, commits it.
///
/// So vehicles commit in their order along each lane. IDM keeps a follower able
/// to stop behind its leader only roughly: one closing fast on a slow leader
/// could otherwise commit first.
fn commit_along(scan: &mut Scan, lane: LaneId, committed: &mut Vec<Entry>) {
    let Some(start) = conflict_start(scan.network, lane) else {
        return;
    };
    // A vehicle waiting part way through, its body on the lane but not handed
    // over for it, is in the way of everyone coming onto it.
    let blocked = scan.occupants[lane.0 as usize].iter().any(|o| {
        !o.is_ghost()
            && committed
                .iter()
                .rev()
                .find(|e| e.id == o.id)
                .is_some_and(|last| !handed_over_for(last, &scan.vehicles[o.id], lane))
    });
    if blocked {
        return;
    }
    let mut way = ArrayVec::<LaneId, SCAN_DEPTH>::new();
    scan_back(scan, lane, lane, start, &mut way, committed);
}

/// [`commit_along`]'s scan of `on`, from `s` back, having come back through
/// `way` from `target`. Returns whether it stopped at a vehicle that may yet
/// approach `target`, on `on` or beyond, and notes `on` as kept or empty.
fn scan_back(
    scan: &mut Scan,
    target: LaneId,
    on: LaneId,
    s: f64,
    way: &mut ArrayVec<LaneId, SCAN_DEPTH>,
    committed: &mut Vec<Entry>,
) -> bool {
    // So long a way back is unheard of: stop, as if at someone, to be safe.
    if way.try_push(on).is_err() {
        scan.kept.push(on);
        return true;
    }
    let stopped = stops_on(scan, target, on, s, way, committed) || {
        let mut stopped = false;
        for &p in scan.network.lane(on).predecessors() {
            if scan.approaching.binary_search_by_key(&p.0, |l| l.0).is_ok() {
                let length = scan.network.lane(p).geometry().length();
                stopped |= scan_back(scan, target, p, length, way, committed);
            }
        }
        stopped
    };
    way.pop();
    match stopped {
        true => scan.kept.push(on),
        false => scan.empty.push(on),
    }
    stopped
}

/// Whether [`scan_back`] stops on `on`, scanning its occupants from `s` back
/// and committing those that must as it goes. A vehicle headed onto `target`
/// that it stops at is noted as a front.
fn stops_on(
    scan: &mut Scan,
    target: LaneId,
    on: LaneId,
    s: f64,
    way: &[LaneId],
    committed: &mut Vec<Entry>,
) -> bool {
    let list = &scan.occupants[on.0 as usize];
    let end = list.partition_point(|o| o.s() < s);
    for o in list[..end].iter().rev() {
        if o.is_ghost() {
            continue;
        }
        let vehicle = &scan.vehicles[o.id];
        // Committed for the way to `target`, it is passed over; handed over
        // only part way, short of it, it may yet go on ahead of those behind
        // it, as a turner waiting for a gap may, so stops the scan.
        if let Some(last) = committed.iter().rev().find(|e| e.id == o.id) {
            if handed_over_for(last, vehicle, target) {
                continue;
            }
            return true;
        }
        if vehicle.path.contains(&target) {
            if must_commit(scan.network, scan.junction, vehicle) {
                let until = handed_over(scan.network, vehicle, scan.junction);
                committed.push(Entry { id: o.id, until });
                continue;
            }
            // A red or amber light holds it short of the conflict, so it will
            // surely stop there: the junction's terms can't matter to it, nor to
            // those behind it.
            let k = vehicle.path.iter().position(|&l| l == target).unwrap_or(0);
            let at = conflict_start(scan.network, target).unwrap_or(0.0);
            if barrier::held_by(scan.network, scan.lights, vehicle, k, at) {
                return true;
            }
            if !scan.fronts.contains(&o.id) {
                scan.fronts.push(o.id);
            }
            return true;
        }
        let into = |&l: &LaneId| scan.network.lane(l).junction() == Some(scan.junction);
        let undecided = vehicle.path.last().is_some_and(|last| way.contains(last));
        if vehicle.path.iter().any(into) || undecided {
            return true;
        }
    }
    false
}

/// Whether `vehicle`, its last entry in a junction's queue `last`, is handed
/// over for `lane`: right through, or for a stretch of its way that `lane` comes
/// before. One handed over only part way, short of `lane`, or not headed onto it
/// at all, may wait there, in the way of others coming onto it.
fn handed_over_for(last: &Entry, vehicle: &Vehicle, lane: LaneId) -> bool {
    let at = |l: LaneId| vehicle.path.iter().position(|&p| p == l);
    match (last.until, at(lane)) {
        (None, _) => true,
        (Some(until), Some(t)) => at(until).is_none_or(|u| t < u),
        (Some(_), None) => false,
    }
}

/// Whether a vehicle has physically entered a junction: it has a record, not a
/// ghost, on one of its lanes.
fn entered(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    id: VehicleId,
) -> bool {
    let lanes = network.junction(junction).lanes();
    lanes
        .iter()
        .any(|&lane| occupants[lane.0 as usize].iter().any(|o| o.is_record_of(id)))
}

/// How far a vehicle needs to stop from its speed, braking at `SAFE_BRAKING`:
/// whether it can stop, for committing to a junction and being released.
fn stopping_distance(vehicle: &Vehicle) -> f64 {
    vehicle.speed * vehicle.speed / (2.0 * acceleration::SAFE_BRAKING)
}

/// Where a lane's first conflict with the rest of its junction starts along it,
/// if it has one.
fn conflict_start(network: &SimNetwork, lane: LaneId) -> Option<f64> {
    let j = network.lane(lane).junction()?;
    let lanes = network.junction(j).lanes();
    let conflicts = lanes.iter().filter_map(|&theirs| network.conflict_between(lane, theirs));
    conflicts.map(|c| c.mine.start).reduce(f64::min)
}

/// How far a vehicle's front is from where it commits to a junction, negative
/// once past it, if its path has such a point there: the start of the first
/// of the junction's lanes on its path that is to be kept clear, or the start
/// of the first conflict, whichever comes first.
fn commit_distance(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> Option<f64> {
    let front = vehicle.lane_pos.s - vehicle.spec.rear_axle + vehicle.spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for &lane in &vehicle.path {
        let l = network.lane(lane);
        if l.junction() == Some(junction) {
            if l.keep_clear() {
                return Some(start);
            }
            if let Some(conflict) = conflict_start(network, lane) {
                return Some(start + conflict);
            }
        }
        start += l.geometry().length();
    }
    None
}

/// Whether a vehicle approaching a junction must commit there: it can no
/// longer stop, braking at `SAFE_BRAKING`, before the first of the junction's
/// lanes on its path to be kept clear, or before its first conflict there,
/// or its front is past that already (see [`commit_distance`]). So a vehicle
/// inside the junction, past its stop line, is always in the queue, never
/// unseen. One with neither on its path never needs to.
///
/// Vehicles at the front of the ways in already slow for committed vehicles in
/// their way, so committing late costs no harder braking, and keeps the order
/// vehicles commit in close to the order they reach the junction.
fn must_commit(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> bool {
    commit_distance(network, junction, vehicle)
        .is_some_and(|distance| distance <= stopping_distance(vehicle))
}

/// Whether the vehicle committed at `index` in a junction's committed list may
/// be released: it could now stop, at `SAFE_BRAKING`, a car length short of
/// where it commits (see [`commit_distance`]), so it plainly can stop, and no vehicle that committed after it is
/// behind it on its way in. The car length is the hysteresis against
/// [`must_commit`], and keeps a vehicle held at the line committed.
///
/// A vehicle committed while it couldn't stop may since have been stopped,
/// by the junction itself, with the margin it has up to `MAX_BRAKING`, or by
/// traffic ahead. Released, it no longer holds back those committed after it,
/// which breaks any cycle of waiting it was part of. Releasing from the back of
/// each lane keeps vehicles committed in their order along it.
fn may_release(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    committed: &[Entry],
    index: usize,
) -> bool {
    let id = committed[index].id;
    let vehicle = &vehicles[id];
    let Some(distance) = commit_distance(network, junction, vehicle) else {
        return false;
    };
    if stopping_distance(vehicle) > distance - vehicle.spec.length {
        return false;
    }
    let lane = vehicle.path[0];
    let records = &occupants[lane.0 as usize];
    let record = |id: VehicleId| records.iter().find(|o| o.is_record_of(id));
    let rear = record(id).map_or(vehicle.lane_pos.s - vehicle.spec.rear_axle, |o| o.s());
    // Behind it on its lane, or on its way onto it.
    let behind = |&later: &VehicleId| match record(later) {
        Some(o) => o.s() < rear,
        None => vehicles[later].path.contains(&lane),
    };
    !committed[index + 1..]
        .iter()
        .map(|e| e.id)
        .filter(|&e| e != id)
        .any(|e| behind(&e))
}

/// Whether a vehicle's path takes it onto any of a junction's lanes.
fn headed(network: &SimNetwork, vehicle: &Vehicle, junction: JunctionId) -> bool {
    vehicle.path.iter().any(|&lane| network.lane(lane).junction() == Some(junction))
}

/// The lowest of the terms the vehicles `ahead` of `id` in a junction impose on
/// it, on the lanes their entries cover, each recorded to the debug sink:
/// infinite if none does. Only on its own lanes in `stretch`, the lanes of its
/// path from the first, if any, up to the second, if any: those its entry
/// behind them was handed over for. An earlier entry of its own covers the
/// lanes before, from its own place in the queue.
pub(crate) fn junction_term<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    id: VehicleId,
    ahead: &[Entry],
    stretch: [Option<LaneId>; 2],
) -> f64 {
    let network = &state.network;
    let lanes = network.junction(junction).lanes();
    let follower = &state.vehicles[id];
    let at = |lane: Option<LaneId>| lane.and_then(|l| follower.path.iter().position(|&p| p == l));
    let (from, until) = (at(stretch[0]), at(stretch[1]));
    let mut accel = f64::INFINITY;
    let front = follower.lane_pos.s - follower.spec.rear_axle + follower.spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for (j, &mine) in follower.path.iter().enumerate() {
        let length = network.lane(mine).geometry().length();
        let within = from.is_none_or(|f| j >= f) && until.is_none_or(|u| j < u);
        if within && network.lane(mine).junction() == Some(junction) {
            for &theirs in lanes {
                let Some(conflict) = network.conflict_between(mine, theirs) else {
                    continue;
                };
                for entry in ahead {
                    let other = entry.id;
                    if other == id || !entry.covers(&state.vehicles[other], theirs) {
                        continue;
                    }
                    let Some((term, gap)) =
                        conflict_term(state, follower, mine, conflict, theirs, other, start)
                    else {
                        continue;
                    };
                    let kind = conflict.kind;
                    let source = AccelSource::Junction { id: other, kind, gap };
                    sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                    accel = accel.min(term);
                }
            }
        }
        start += length;
    }
    accel
}

/// The term a junction imposes on `follower` for `other`, committed before it,
/// where `mine`, a lane of its path whose start is `start` meters from its front,
/// conflicts with `theirs`, and the gap that term is for: `None` once `other`
/// has cleared the conflict on its lane, its rear past the end of it, or, not
/// headed onto `theirs`, while its body isn't in it.
///
/// Until then, the follower takes the gentlest of three ways of keeping out of
/// the other's way, each safe on its own:
///
/// - **Stopping short** of the conflict, as at a stopped vehicle there.
/// - **Following its progress:** a virtual vehicle at the speed of the other,
///   as far past the start of the follower's conflict as the other's rear is
///   past the end of its own, so it reaches that start just as the other clears
///   its conflict. This is virtual vehicle mapping, as in cooperative merging
///   (Uno et al., 1999). Only for one headed onto `theirs`.
/// - **Following it:** while the other moves the same general way as the
///   follower's conflict runs, and will still be as it leaves its own, its
///   nearest corner, a rear one, measured along the conflict's direction, at its
///   speed along that direction. The convex body is nowhere nearer along that
///   direction, at any angle, for a conflict that is roughly straight on the
///   follower's side. On the other's side, a lane bends one way by less than
///   half a turn, so one heading the same way now and as it leaves never swings
///   back across the follower's way in between. Without that, a vehicle turning
///   across could lead the follower into its conflict, then swing back into it.
///
/// A follower already past the start of its conflict is held back all the same:
/// it should not be there before the other has cleared, and at a merge, where
/// driving on never clears the way, dropping back is the only way out.
fn conflict_term(
    state: &SimState,
    follower: &Vehicle,
    mine: LaneId,
    conflict: &Conflict,
    theirs: LaneId,
    other: VehicleId,
    start: f64,
) -> Option<(f64, f64)> {
    let network = &state.network;
    let leader = &state.vehicles[other];
    // The other one's rear along `theirs`, if its path takes it there.
    let rear = leader.path.iter().position(|&l| l == theirs).map(|i| {
        let before: f64 =
            leader.path[..i].iter().map(|&l| network.lane(l).geometry().length()).sum();
        leader.lane_pos.s - leader.spec.rear_axle - before
    });
    let clear = match rear {
        Some(rear) => rear >= conflict.theirs.end,
        // Not headed there: only its body, lingering on the lane as it leaves
        // it, can be in the way. Beside it, overlapping it only where it runs
        // alongside its own, it is in the way only where its own lanes are.
        None if !network.left_behind(leader.path[0], theirs) => true,
        None => state.occupants[theirs.0 as usize]
            .iter()
            .find(|o| o.is_record_of(other))
            .is_none_or(|o| {
                o.s() >= conflict.theirs.end || o.s() + leader.spec.length <= conflict.theirs.start
            }),
    };
    if clear {
        return None;
    }
    let term = |gap: f64, speed: f64| {
        (
            acceleration::interaction(&follower.spec, follower.speed, gap, speed),
            gap,
        )
    };
    let gentler = |a: (f64, f64), b: (f64, f64)| if b.0 > a.0 { b } else { a };
    let to_conflict = start + conflict.mine.start;
    // Stopping short.
    let mut best = term(to_conflict, 0.0);
    // Following its progress.
    if let Some(rear) = rear {
        best = gentler(
            best,
            term(to_conflict + rear - conflict.theirs.end, leader.speed),
        );
    }
    // Following it, measured along the conflict from its start.
    let geometry = network.lane(mine).geometry();
    let at = geometry.lane_pos(conflict.mine.start, 0.0);
    let (origin, along) = (geometry.lane_to_world(at), geometry.tangent_at(at));
    let heading = DVec2::from_angle(leader.heading);
    // Leaving its conflict the same general way too: a lane bends one way, by
    // less than half a turn, so it never swings back across in between.
    let exit = network.lane(theirs).geometry();
    let leaving = exit.tangent_at(exit.lane_pos(conflict.theirs.end, 0.0));
    if heading.dot(along) > 0.0 && leaving.dot(along) > 0.0 {
        let corners = leader.corners().map(|corner| (corner - origin).dot(along));
        let nearest = corners.into_iter().fold(f64::INFINITY, f64::min);
        best = gentler(
            best,
            term(to_conflict + nearest, leader.speed * heading.dot(along)),
        );
    }
    Some(best)
}
