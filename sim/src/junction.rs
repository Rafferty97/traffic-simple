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

/// A junction's queue: the vehicles committed there, which can no longer stop
/// short of their conflicts, in the order they committed, and the lanes that
/// vehicles not yet committed may be approaching it along.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Queue {
    /// Each holds back every vehicle committed after it.
    pub committed: Vec<VehicleId>,
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
        queue.committed.retain(|&v| {
            headed(network, &vehicles[v], junction) || entered(network, occupants, junction, v)
        });
        // From the back, so a lane's vehicles are released back to front in
        // one pass. A released vehicle's way in is marked again, as the
        // scans passed it over while it was committed.
        for k in (0..queue.committed.len()).rev() {
            if may_release(network, vehicles, occupants, junction, &queue.committed, k) {
                let id = queue.committed.remove(k);
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
    /// The signals' lights, for the lanes' priority levels now.
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
fn commit_along(scan: &mut Scan, lane: LaneId, committed: &mut Vec<VehicleId>) {
    let Some(start) = conflict_start(scan.network, lane) else {
        return;
    };
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
    committed: &mut Vec<VehicleId>,
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
    committed: &mut Vec<VehicleId>,
) -> bool {
    let list = &scan.occupants[on.0 as usize];
    let end = list.partition_point(|o| o.s() < s);
    for o in list[..end].iter().rev() {
        if o.is_ghost() || committed.contains(&o.id) {
            continue;
        }
        let vehicle = &scan.vehicles[o.id];
        if vehicle.path.contains(&target) {
            if must_commit(scan.network, scan.junction, vehicle) {
                let Scan { network, vehicles, occupants, lights, junction, .. } = *scan;
                join(
                    network, vehicles, occupants, lights, junction, committed, o.id,
                );
                continue;
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

/// Puts `id` in a junction's queue of committed vehicles: at the back, behind
/// everyone committed before it, and then ahead of each there that would give
/// way to it, which can't have moved on into the conflict, with those behind it
/// on its way in, in their order (see [`gives_way`]). So a turner waiting
/// inside the junction for a gap sinks behind each vehicle it gives way to that
/// commits while it waits, and stays ahead of those it doesn't.
pub(crate) fn join(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    lights: &[MovementLight],
    junction: JunctionId,
    committed: &mut Vec<VehicleId>,
    id: VehicleId,
) {
    committed.push(id);
    // Where it is, and how many have gone behind it, just after it.
    let (mut at, mut moved) = (committed.len() - 1, 0);
    let mut i = 0;
    while i < at {
        let waiting = committed[i];
        if !gives_way(network, vehicles, lights, junction, waiting, id) {
            i += 1;
            continue;
        }
        // It and those behind it on its way in go just behind those moved
        // before, in their order, in place.
        let mut k = i;
        while k < at {
            let other = committed[k];
            if other == waiting || behind(network, vehicles, occupants, junction, waiting, other) {
                committed[k..=at + moved].rotate_left(1);
                at -= 1;
                moved += 1;
            } else {
                k += 1;
            }
        }
    }
}

/// Whether `waiting`, in a junction's queue, would give way to `other`: it is
/// held at a give-way or stop barrier in the junction it hasn't committed to,
/// so hasn't moved on into its conflicts, and a lane of its path in the
/// junction from there conflicts with one of `other`'s at a higher priority
/// level now (see [`barrier::level`]), as a turner gives way to oncoming
/// traffic.
fn gives_way(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    lights: &[MovementLight],
    junction: JunctionId,
    waiting: VehicleId,
    other: VehicleId,
) -> bool {
    let w = &vehicles[waiting];
    let Some((lane, at)) = barrier::nth(network, w, usize::from(w.committed)) else {
        return false;
    };
    if network.lane(lane).junction() != Some(junction)
        || matches!(at.kind, crate::network::BarrierKind::Signal { .. })
    {
        return false;
    }
    let inside = |l: &&LaneId| network.lane(**l).junction() == Some(junction);
    let from = w.path.iter().position(|&l| l == lane).unwrap_or(0);
    let theirs = || vehicles[other].path.iter().filter(inside);
    w.path[from..].iter().take_while(inside).any(|&mine| {
        theirs().any(|&lane| {
            network.conflict_between(mine, lane).is_some()
                && barrier::level(network, lights, mine) < barrier::level(network, lights, lane)
        })
    })
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

/// How far a vehicle's front is from the start of the first conflict on its
/// path in a junction, negative once past it, if its path has one there.
fn conflict_distance(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> Option<f64> {
    let front = vehicle.lane_pos.s - vehicle.spec.rear_axle + vehicle.spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for &lane in &vehicle.path {
        if network.lane(lane).junction() == Some(junction)
            && let Some(conflict) = conflict_start(network, lane)
        {
            return Some(start + conflict);
        }
        start += network.lane(lane).geometry().length();
    }
    None
}

/// Whether a vehicle approaching a junction must commit there: it can no
/// longer stop, braking at `SAFE_BRAKING`, before the first conflict on its
/// path in the junction, or its front is past the start of it already. One
/// with no conflict on its path never needs to.
///
/// Vehicles at the front of the ways in already slow for committed vehicles in
/// their way, so committing late costs no harder braking, and keeps the order
/// vehicles commit in close to the order they reach their conflicts.
fn must_commit(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> bool {
    conflict_distance(network, junction, vehicle)
        .is_some_and(|distance| distance <= stopping_distance(vehicle))
}

/// Whether the vehicle committed at `index` in a junction's committed list may
/// be released: it could now stop, at `SAFE_BRAKING`, a car length short of its
/// conflict, so it plainly can stop, and no vehicle that committed after it is
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
    committed: &[VehicleId],
    index: usize,
) -> bool {
    let id = committed[index];
    let vehicle = &vehicles[id];
    let Some(distance) = conflict_distance(network, junction, vehicle) else {
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
    !committed[index + 1..].iter().any(behind)
}

/// Whether `other` is behind `ahead` on its way into `junction`: on a lane
/// where `ahead` has a record, behind that record, or headed onto one. Those
/// lanes are the junction's and those on `ahead`'s path, so a vehicle behind
/// another at a fork, on the branch it didn't take, is found through its
/// record there.
pub(crate) fn behind(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    ahead: VehicleId,
    other: VehicleId,
) -> bool {
    let lanes = network.junction(junction).lanes().iter().chain(&vehicles[ahead].path);
    lanes.into_iter().any(|&lane| {
        let records = &occupants[lane.0 as usize];
        let Some(mine) = records.iter().find(|o| o.is_record_of(ahead) && !o.is_ghost()) else {
            return false;
        };
        match records.iter().find(|o| o.is_record_of(other) && !o.is_ghost()) {
            Some(theirs) => theirs.s() < mine.s(),
            None => vehicles[other].path.contains(&lane),
        }
    })
}

/// Whether a vehicle's path takes it onto any of a junction's lanes.
fn headed(network: &SimNetwork, vehicle: &Vehicle, junction: JunctionId) -> bool {
    vehicle.path.iter().any(|&lane| network.lane(lane).junction() == Some(junction))
}

/// The lowest of the terms the vehicles `ahead` of `id` in a junction impose on
/// it, each recorded to the debug sink: infinite if none does.
pub(crate) fn junction_term<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    id: VehicleId,
    ahead: &[VehicleId],
) -> f64 {
    let network = &state.network;
    let lanes = network.junction(junction).lanes();
    let follower = &state.vehicles[id];
    let mut accel = f64::INFINITY;
    let front = follower.lane_pos.s - follower.spec.rear_axle + follower.spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for &mine in &follower.path {
        let length = network.lane(mine).geometry().length();
        if network.lane(mine).junction() == Some(junction) {
            for &theirs in lanes {
                let Some(conflict) = network.conflict_between(mine, theirs) else {
                    continue;
                };
                for &other in ahead {
                    // One that would give way to it, waiting at its line, holds
                    // nothing back: joining, it goes ahead of it (see [`join`]).
                    if gives_way(network, &state.vehicles, &state.lights, junction, other, id) {
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
        // Not headed there: only its body, lingering past the lane's end or
        // beside it, can be in the way.
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
