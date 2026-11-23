//! The junction backstop (see "Conflicts, merging and junctions" in
//! `docs/DESIGN.md`): each junction's queue of the vehicles committed there,
//! which can no longer stop short of their conflicts, who arrives, commits and
//! is released, and the terms committed vehicles impose on those behind them.

use arrayvec::ArrayVec;
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
    /// The lane, by its place among the junction's lanes, from which this
    /// entry no longer covers it, where its next barrier is: `None` if it
    /// covers it right through.
    pub until: Option<u16>,
}

impl Entry {
    /// The lane it stops covering its vehicle at, if any.
    pub fn until(&self, network: &SimNetwork, junction: JunctionId) -> Option<LaneId> {
        self.until.map(|i| network.junction(junction).lanes()[usize::from(i)])
    }

    /// Whether it covers its vehicle on `lane`: anywhere but the lane it stops
    /// at and the lanes of the same connector after it, found from the
    /// junction alone.
    pub(crate) fn covers(&self, network: &SimNetwork, lane: LaneId) -> bool {
        let (Some(i), Some(junction)) = (self.until, network.lane(lane).junction()) else {
            return true;
        };
        let until = network.junction(junction).lanes()[usize::from(i)];
        lane != until && !network.leads_to(lane, until)
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
    /// The vehicles that must commit but that the junction holds out of its
    /// queue, short of their first conflicts, as they would be both blocked by
    /// a vehicle queued only part of its way and block its next stretch (see
    /// [`closes_cycle`]). Refilled every step.
    #[cfg_attr(feature = "serde", serde(default))]
    pub held: Vec<VehicleId>,
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
        queue.held.clear();
        if queue.approaching.is_empty() {
            continue;
        }
        let Queue { committed, approaching, fronts, held } = queue;
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
            held,
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
            network.lane(lane).junction() == Some(j) && network.conflict_start(lane).is_some()
        };
        if let Some(k) = (0..vehicle.path.len()).find(|&k| into(k)) {
            let start = network.conflict_start(vehicle.path[k]).unwrap_or(0.0);
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
fn handed_over(network: &SimNetwork, vehicle: &Vehicle, junction: JunctionId) -> Option<u16> {
    barrier::nth(network, vehicle, usize::from(vehicle.committed))
        .map(|(lane, _)| network.lane(lane))
        .filter(|lane| lane.junction() == Some(junction))
        .and_then(|lane| lane.junction_slot())
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
        network.lane(lane).junction() == Some(junction) && network.conflict_start(lane).is_some()
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
    /// The vehicles held out of the queue (see [`Queue::held`]).
    held: &'a mut Vec<VehicleId>,
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
    let Some(start) = scan.network.conflict_start(lane) else {
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
            if handed_over_for(scan.network, scan.junction, last, vehicle, target) {
                continue;
            }
            return true;
        }
        if let Some(k) = vehicle.path.iter().position(|&l| l == target) {
            if must_commit(scan.network, scan.junction, vehicle) {
                // Held out of the queue, short of its conflicts, so nobody
                // behind it commits before it either.
                if closes_cycle(
                    scan.network,
                    scan.vehicles,
                    scan.occupants,
                    scan.junction,
                    committed,
                    vehicle,
                ) {
                    if !scan.held.contains(&o.id) {
                        scan.held.push(o.id);
                    }
                    return true;
                }
                let until = handed_over(scan.network, vehicle, scan.junction);
                committed.push(Entry { id: o.id, until });
                continue;
            }
            // A red or amber light holds it short of the conflict, so it will
            // surely stop there: the junction's terms can't matter to it, nor to
            // those behind it.
            let at = scan.network.conflict_start(target).unwrap_or(0.0);
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

/// How long a queue [`closes_cycle`] reasons about: one longer is unheard of, and
/// holds anyone it would otherwise check, to be safe.
const QUEUE_DEPTH: usize = 256;

/// Whether queuing `vehicle` at `junction`, after `committed`, would let its
/// order hold a cycle: the fourth queue invariant (see `docs/DESIGN.md`, Queue
/// invariants).
///
/// Each vehicle queued only part of its way, T, has a next stretch still to be
/// queued, at the back. A vehicle is blocked by T if it would wait on T, its way
/// crossing the stretch T is queued for where T hasn't cleared it, or following
/// it, or would wait on a queued vehicle itself blocked by T; it blocks T if its
/// way crosses T's next stretch, which will then wait on it. One blocked by `a`
/// and blocking `b` makes `b`'s next stretch wait on `a`'s. It is queued unless
/// that would close a cycle among the next stretches: unless `b` is `a`, or `a`'s
/// next stretch already waits, through queued vehicles, on `b`'s.
fn closes_cycle(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    committed: &[Entry],
    vehicle: &Vehicle,
) -> bool {
    // The vehicles queued only part of their way, by the entry they last have.
    let mut partial = ArrayVec::<usize, 64>::new();
    for (i, e) in committed.iter().enumerate() {
        let last = !committed[i + 1..].iter().any(|later| later.id == e.id);
        if e.until.is_some() && last && partial.try_push(i).is_err() {
            // So many is unheard of: hold it, to be safe.
            return true;
        }
    }
    if partial.is_empty() {
        return false;
    }
    let inside = |l: &&LaneId| network.lane(**l).junction() == Some(junction);
    let entry_of = |p: usize| &committed[partial[p]];
    // Which of them `v` blocks, as a mask by their place in `partial`.
    let blocks = |v: &Vehicle| {
        (0..partial.len()).fold(0u64, |mask, p| {
            let e = entry_of(p);
            let t = &vehicles[e.id];
            let until = e.until(network, junction).expect("queued only part of its way");
            let Some(from) = t.path.iter().position(|&l| l == until) else {
                return mask;
            };
            let crosses = v.path.iter().filter(inside).any(|&mine| {
                t.path[from..]
                    .iter()
                    .filter(inside)
                    .any(|&theirs| network.conflict_between(mine, theirs).is_some())
            });
            if crosses { mask | 1 << p } else { mask }
        })
    };
    let waits = |v: &Vehicle, id: VehicleId, e: &Entry| {
        waits_on(network, occupants, junction, v, id, &vehicles[id], e)
    };
    // Which of them each queued vehicle is blocked by, until no more are found.
    if committed.len() > QUEUE_DEPTH {
        return true;
    }
    let mut blocked: ArrayVec<u64, QUEUE_DEPTH> = committed
        .iter()
        .map(|w| {
            let v = &vehicles[w.id];
            (0..partial.len()).fold(0u64, |mask, p| {
                let e = entry_of(p);
                if e.id != w.id && waits(v, e.id, e) {
                    mask | 1 << p
                } else {
                    mask
                }
            })
        })
        .collect();
    loop {
        let mut changed = false;
        for a in 0..committed.len() {
            let v = &vehicles[committed[a].id];
            for b in 0..committed.len() {
                let (w, u) = (&committed[a], &committed[b]);
                if w.id == u.id || blocked[b] & !blocked[a] == 0 || !waits(v, u.id, u) {
                    continue;
                }
                blocked[a] |= blocked[b];
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    // What each next stretch waits on: `b` on `a` for each queued vehicle
    // blocked by `a` that blocks `b`; then through each other.
    let mut waits_for = [0u64; 64];
    for (w, &by) in committed.iter().zip(&blocked) {
        let mask = blocks(&vehicles[w.id]);
        for (b, slot) in waits_for.iter_mut().enumerate().take(partial.len()) {
            if mask & 1 << b != 0 {
                *slot |= by;
            }
        }
    }
    for k in 0..partial.len() {
        for a in 0..partial.len() {
            if waits_for[a] & 1 << k != 0 {
                waits_for[a] |= waits_for[k];
            }
        }
    }
    // The vehicle's own: blocked by those it waits on, directly or through
    // queued vehicles, and blocking those its way crosses.
    let mut by = (0..partial.len()).fold(0u64, |mask, p| {
        let e = entry_of(p);
        if waits(vehicle, e.id, e) {
            mask | 1 << p
        } else {
            mask
        }
    });
    for (u, &mask) in committed.iter().zip(&blocked) {
        if waits(vehicle, u.id, u) {
            by |= mask;
        }
    }
    let blocking = blocks(vehicle);
    // `b` would wait on each `a` it is blocked by: a cycle if `a` is `b`, or
    // `a` waits on `b` already.
    (0..partial.len())
        .any(|a| by & 1 << a != 0 && (blocking & 1 << a != 0 || waits_for[a] & blocking != 0))
}

/// Whether `follower` would wait on `leader`, `id`, queued as `entry`: its path
/// running onto the lane `leader` is on, or onto one where `leader` has a record
/// ahead of it, as a vehicle forking off just ahead has; or crossing a lane of
/// `junction` that the entry covers where `leader` hasn't cleared it, its rear
/// short of the conflict's end.
fn waits_on(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    follower: &Vehicle,
    id: VehicleId,
    leader: &Vehicle,
    entry: &Entry,
) -> bool {
    if follower.path.contains(&leader.path[0]) {
        return true;
    }
    let own_s = follower.lane_pos.s - follower.spec.rear_axle;
    let ahead = follower.path.iter().enumerate().any(|(k, &lane)| {
        occupants[lane.0 as usize]
            .iter()
            .any(|o| o.is_record_of(id) && !o.is_ghost() && (k > 0 || o.s() > own_s))
    });
    if ahead {
        return true;
    }
    // The leader's rear, back from the start of each lane of its path.
    let mut before = 0.0;
    for &theirs in &leader.path {
        let length = network.lane(theirs).geometry().length();
        let rear = leader.lane_pos.s - leader.spec.rear_axle - before;
        before += length;
        if network.lane(theirs).junction() != Some(junction) || !entry.covers(network, theirs) {
            continue;
        }
        let crosses = follower.path.iter().any(|&mine| {
            network.lane(mine).junction() == Some(junction)
                && network.conflict_between(mine, theirs).is_some_and(|c| rear < c.theirs.end)
        });
        if crosses {
            return true;
        }
    }
    false
}

/// The term holding a vehicle the junction holds out of its queue short of
/// the first conflict on its path there, as at a stationary vehicle at its
/// start, and the gap to it; `None` once it is past that start, or has none.
pub(crate) fn hold_term(
    network: &SimNetwork,
    junction: JunctionId,
    vehicle: &Vehicle,
) -> Option<(f64, f64)> {
    let front = vehicle.lane_pos.s - vehicle.spec.rear_axle + vehicle.spec.length;
    let mut start = -front;
    for &lane in &vehicle.path {
        if network.lane(lane).junction() == Some(junction)
            && let Some(conflict) = network.conflict_start(lane)
        {
            let gap = start + conflict;
            let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, 0.0);
            return Some((term, gap));
        }
        start += network.lane(lane).geometry().length();
    }
    None
}

/// Whether `vehicle`, its last entry in a junction's queue `last`, is handed
/// over for `lane`: right through, or for a stretch of its way that `lane` comes
/// before. One handed over only part way, short of `lane`, or not headed onto it
/// at all, may wait there, in the way of others coming onto it.
fn handed_over_for(
    network: &SimNetwork,
    junction: JunctionId,
    last: &Entry,
    vehicle: &Vehicle,
    lane: LaneId,
) -> bool {
    let at = |l: LaneId| vehicle.path.iter().position(|&p| p == l);
    match (last.until(network, junction), at(lane)) {
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
            if let Some(conflict) = network.conflict_start(lane) {
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
                    if other == id || !entry.covers(network, theirs) {
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
    let heading = leader.dir;
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
