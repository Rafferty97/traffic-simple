//! The junction backstop (see "Conflicts, merging and junctions" in
//! `docs/DESIGN.md`): each junction's queue of the vehicles that can no longer
//! stop comfortably short of their first conflict there, who arrives, joins and
//! leaves it, the order it keeps, and the terms that order imposes.
//!
//! **Prototype.** Two vehicles in a queue whose ways through the junction
//! conflict are a pair. A pair is decided once either of them can no longer
//! stop comfortably short of their conflict, and the lower in the queue then
//! gives way to the higher. Until then the queue's order is only a tiebreak, and
//! neither holds the other back. A vehicle that would need clearly more braking
//! to give way to the one above it than that one would to give way to it is
//! lifted above it, with whatever it waits on, unless that would close a cycle
//! among decided pairs and vehicles following one another. So the queue is
//! always an order those agree with, and its first vehicle can always move.

use arrayvec::ArrayVec;
use slotmap::{Key, SlotMap};

use crate::barrier;
use crate::debug::{AccelSource, DebugEvent, DebugSink};
use crate::following::Occupant;
use crate::models::acceleration;
use crate::network::{Conflict, ConflictKind, JunctionId, LaneId, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// A junction's queue: the vehicles that can no longer stop comfortably short
/// of their first conflict there, in an order every decided pair agrees with,
/// and the lanes that vehicles not yet in it may be approaching it along.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct Queue {
    /// Highest first. Of each decided pair, the lower gives way to the higher;
    /// a vehicle following another is always below it.
    pub order: Vec<VehicleId>,
    /// Every lane on the way of a vehicle that has arrived, from the lane it
    /// was on to the first of the junction's lanes with a conflict on its path,
    /// that lane included, sorted and without repeats. The junction's scans
    /// for vehicles to queue start from its own lanes here and follow only
    /// lanes here (see [`commit_along`]). Vehicles only add lanes; the scans
    /// drop those they find no one on.
    pub approaching: Vec<LaneId>,
}

/// Scratch space for the junctions' updates, reused between them.
#[derive(Default)]
pub(crate) struct Scratch {
    /// Each queued vehicle's place in the queue being updated.
    places: Places,
    /// The queue's order as the update's walk over its pairs began.
    snapshot: Vec<VehicleId>,
    /// For lifting a vehicle: the search's stack, the places found below the
    /// one it goes above and above itself, which of the places between were
    /// found, and the vehicles in their new order.
    stack: Vec<usize>,
    below: Vec<usize>,
    above: Vec<usize>,
    seen: Vec<bool>,
    lifted: Vec<VehicleId>,
}

/// Brings the junctions' queues up to date after a step's motion and
/// occupancy: vehicles whose path grew or that entered a junction arrive
/// where nothing holds them back (see [`arrive`]), queued vehicles clear of a
/// junction leave its queue, those that can plainly stop comfortably are
/// released (see [`may_release`]), each lane approached is scanned for vehicles
/// to queue (see [`commit_along`]), and each queue is reordered (see
/// [`reorder`]).
pub(crate) fn update(
    state: &mut SimState,
    arrivals: &mut Vec<VehicleId>,
    scan_empty: &mut Vec<LaneId>,
    scan_kept: &mut Vec<LaneId>,
    scratch: &mut Scratch,
) {
    let SimState { network, vehicles, occupants, queues, .. } = state;
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
        queue.order.retain(|&id| {
            headed(network, &vehicles[id], junction) || entered(network, occupants, junction, id)
        });
        // From the back, so a lane's vehicles are released back to front in
        // one pass. A released vehicle's way in is marked again, as the scans
        // passed it over while it was queued.
        for k in (0..queue.order.len()).rev() {
            if may_release(network, vehicles, occupants, junction, &queue.order, k) {
                let id = queue.order.remove(k);
                mark_way(
                    network,
                    &mut queue.approaching,
                    &vehicles[id].path,
                    junction,
                    0,
                );
            }
        }
        let Queue { order, approaching } = queue;
        scratch.places.fill(order);
        if !approaching.is_empty() {
            let mut scan = Scan {
                network,
                vehicles,
                occupants,
                junction,
                approaching,
                empty: scan_empty,
                kept: scan_kept,
                places: &mut scratch.places,
            };
            scan.empty.clear();
            scan.kept.clear();
            for &lane in scan.approaching {
                if network.lane(lane).junction() == Some(junction) {
                    commit_along(&mut scan, lane, order);
                }
            }
            // A lane stays while some scan found someone on it or beyond, or
            // none reached it, so whoever is on it is behind someone found.
            approaching.retain(|lane| !scan_empty.contains(lane) || scan_kept.contains(lane));
        }
        reorder(network, vehicles, occupants, junction, order, scratch);
        scratch.places.clear(order);
    }
}

/// Makes a vehicle arrive at each junction on its path that nothing holds it
/// back from: up to and including the first junction it hasn't physically
/// entered (see [`entered`]), and short of any junction a barrier it hasn't
/// committed to holds it back from, at or before the junction's first conflict
/// on its path: it arrives there once it commits. At each, the lanes of its
/// path up to the first of the junction's lanes with a conflict, that lane
/// included, join those it is approached along, so the junction's scans find
/// it.
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
        if queues[j.0 as usize].order.contains(&id) {
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
            0,
        );
        last = Some(j);
    }
}

/// The lane of `junction` where `vehicle` will wait inside it, if it will: where
/// its next barrier it hasn't committed to is, in the junction, once it is
/// inside, on one of the junction's lanes or committed to a barrier on one. A
/// vehicle at its stop line isn't inside, though its line is on a connector
/// along which a turner waits further on.
pub(crate) fn waits_at(
    network: &SimNetwork,
    vehicle: &Vehicle,
    junction: JunctionId,
) -> Option<LaneId> {
    let k = usize::from(vehicle.committed);
    let in_it = |lane: LaneId| network.lane(lane).junction() == Some(junction);
    let (lane, _) = barrier::nth(network, vehicle, k)?;
    let inside = in_it(vehicle.path[0])
        || k > 0 && barrier::nth(network, vehicle, k - 1).is_some_and(|(l, _)| in_it(l));
    (in_it(lane) && inside).then_some(lane)
}

/// Adds the lanes of `path` up to the first of `junction`'s lanes on it with a
/// conflict from its `from`th lane on, that lane included, to `approaching`,
/// kept sorted.
fn mark_way(
    network: &SimNetwork,
    approaching: &mut Vec<LaneId>,
    path: &[LaneId],
    junction: JunctionId,
    from: usize,
) {
    let into = |&lane: &LaneId| {
        network.lane(lane).junction() == Some(junction) && network.conflict_start(lane).is_some()
    };
    let Some(k) = path[from..].iter().position(into) else {
        return;
    };
    for &lane in &path[..=from + k] {
        if let Err(i) = approaching.binary_search_by_key(&lane.0, |l| l.0) {
            approaching.insert(i, lane);
        }
    }
}

/// What a junction's scans for vehicles to queue read, and where they note
/// which lanes they found anyone on.
struct Scan<'a> {
    network: &'a SimNetwork,
    vehicles: &'a SlotMap<VehicleId, Vehicle>,
    occupants: &'a [Vec<Occupant>],
    junction: JunctionId,
    /// The lanes vehicles approach the junction along, sorted.
    approaching: &'a [LaneId],
    /// Lanes a scan found no one on, nor beyond.
    empty: &'a mut Vec<LaneId>,
    /// Lanes a scan found someone on, or beyond.
    kept: &'a mut Vec<LaneId>,
    /// Each queued vehicle's place in the queue.
    places: &'a mut Places,
}

/// Each vehicle's place in one junction's queue, by its slot, so a scan passing
/// a vehicle, or a pair's walk, finds its place at once rather than searching
/// the queue for it: scratch space, filled for a junction before its update,
/// kept up as it queues and reorders vehicles, and cleared after.
#[derive(Default)]
struct Places(Vec<Option<(VehicleId, u32)>>);

impl Places {
    fn slot(id: VehicleId) -> usize {
        // The slot's index is the key's low half.
        (id.data().as_ffi() & 0xffff_ffff) as usize
    }

    /// Notes that `id` is `k`th.
    fn set(&mut self, id: VehicleId, k: usize) {
        let slot = Self::slot(id);
        if slot >= self.0.len() {
            self.0.resize(slot + 1, None);
        }
        self.0[slot] = Some((id, k as u32));
    }

    /// The place of `id` in the queue, if it is queued.
    fn get(&self, id: VehicleId) -> Option<usize> {
        let (queued, k) = (*self.0.get(Self::slot(id))?)?;
        (queued == id).then_some(k as usize)
    }

    fn fill(&mut self, order: &[VehicleId]) {
        for (k, &id) in order.iter().enumerate() {
            self.set(id, k);
        }
    }

    fn clear(&mut self, order: &[VehicleId]) {
        for &id in order {
            self.0[Self::slot(id)] = None;
        }
    }
}

/// How many lanes a scan back from a junction's lane holds at once: that lane
/// and those it has come back through.
const SCAN_DEPTH: usize = 32;

/// Queues the vehicles approaching `lane`, one of the scan's junction's lanes
/// with a conflict, that can no longer stop comfortably short of it, nearest
/// first, at the back, noting on the way which lanes it found anyone on.
///
/// It scans back from the start of the lane's conflict, a lane being in at most
/// one junction and so having at most one conflict zone: along its occupants,
/// then the lanes leading into it, branch by branch, following only lanes
/// vehicles approach the junction along. On each branch it passes over ghosts,
/// queued vehicles and vehicles whose path has left the way to `lane` and goes
/// nowhere in the junction. At the first other one:
///
/// - one headed onto `lane` is queued if it must be (see [`must_commit`]) and
///   the scan goes on behind it; if it needn't be yet, the scan stops there,
///   since nobody behind it may be queued above it;
/// - one headed elsewhere in the junction, or whose path ends before it could
///   leave the way to `lane`, stops the scan: it may yet go in ahead of those
///   behind it, and another lane's scan, or a later one, queues it.
///
/// So vehicles are queued in their order along each lane, and one following
/// another is below it.
fn commit_along(scan: &mut Scan, lane: LaneId, order: &mut Vec<VehicleId>) {
    let Some(start) = scan.network.conflict_start(lane) else {
        return;
    };
    let mut way = ArrayVec::<LaneId, SCAN_DEPTH>::new();
    scan_back(scan, lane, lane, start, &mut way, order);
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
    order: &mut Vec<VehicleId>,
) -> bool {
    // So long a way back is unheard of: stop, as if at someone, to be safe.
    if way.try_push(on).is_err() {
        scan.kept.push(on);
        return true;
    }
    let stopped = stops_on(scan, target, on, s, way, order) || {
        let mut stopped = false;
        for &p in scan.network.lane(on).predecessors() {
            if scan.approaching.binary_search_by_key(&p.0, |l| l.0).is_ok() {
                let length = scan.network.lane(p).geometry().length();
                stopped |= scan_back(scan, target, p, length, way, order);
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
/// and queuing those that must be as it goes.
fn stops_on(
    scan: &mut Scan,
    target: LaneId,
    on: LaneId,
    s: f64,
    way: &[LaneId],
    order: &mut Vec<VehicleId>,
) -> bool {
    let list = &scan.occupants[on.0 as usize];
    let end = list.partition_point(|o| o.s() < s);
    for o in list[..end].iter().rev() {
        if o.is_ghost() || scan.places.get(o.id).is_some() {
            continue;
        }
        let vehicle = &scan.vehicles[o.id];
        if vehicle.path.contains(&target) {
            if !must_commit(scan.network, scan.junction, vehicle) {
                return true;
            }
            scan.places.set(o.id, order.len());
            order.push(o.id);
            continue;
        }
        let into = |&l: &LaneId| scan.network.lane(l).junction() == Some(scan.junction);
        let undecided = vehicle.path.last().is_some_and(|last| way.contains(last));
        if vehicle.path.iter().any(into) || undecided {
            return true;
        }
    }
    false
}

/// How much more braking a vehicle must need to give way to the one above it
/// than that one would to give way to it, for it to be lifted above it, in
/// m/s²: the hysteresis that keeps two needing about the same from swapping
/// back and forth.
const LIFT_MARGIN: f64 = 0.5;

/// Walks the pairs of `junction`'s queue, `order`, in the order it had as the
/// walk began, each vehicle's from the nearest above it up, so that it is lifted
/// above a nearer one before a further one that may lead to it, lifting each vehicle above the one above it where it needs
/// clearly more braking to give way to it than that one would to give way to
/// it, once either can no longer stop comfortably short of their conflict, and
/// where that closes no cycle (see [`lift`]). Each pair is seen once, by where
/// its two are now, so one lift may make a later pair's lift fail, and that
/// pair's is tried again at the next update.
fn reorder(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    order: &mut [VehicleId],
    scratch: &mut Scratch,
) {
    let view = View { network, vehicles, occupants, junction };
    scratch.snapshot.clear();
    scratch.snapshot.extend_from_slice(order);
    let n = scratch.snapshot.len();
    for j in 1..n {
        for i in (0..j).rev() {
            let (a, b) = (scratch.snapshot[i], scratch.snapshot[j]);
            let place = |id| scratch.places.get(id).expect("queued");
            let (upper, lower) = if place(a) < place(b) { (a, b) } else { (b, a) };
            let Some([nu, nl]) = view.needs(upper, lower) else {
                continue;
            };
            let (u, l) = (&vehicles[upper], &vehicles[lower]);
            if nu <= u.spec.comfy_decel && nl <= l.spec.comfy_decel {
                continue;
            }
            if nl > nu + LIFT_MARGIN {
                view.lift(order, scratch, lower, upper);
            }
        }
    }
}

impl View<'_> {
    /// Whether the vehicle `k`th in `order` must stay above the `m`th, below
    /// it: they are a decided pair, or the `m`th follows the `k`th.
    fn edge(self, order: &[VehicleId], k: usize, m: usize) -> bool {
        let (upper, lower) = (order[k], order[m]);
        follows(self.occupants, &self.vehicles[lower], upper) || self.decided(upper, lower)
    }

    /// Lifts `lower` above `upper` in `order`, with every vehicle between them
    /// that it must stay below, above `upper` and every vehicle between them
    /// that must stay below that, unless one of those is `lower` itself, so
    /// that the lift would close a cycle, or `lower` follows `upper`. Only the
    /// vehicles found move, among the places they had, so every other pair
    /// keeps its order: Pearce and Kelly's dynamic topological sort (2006).
    /// Returns whether it lifted it.
    fn lift(
        self,
        order: &mut [VehicleId],
        scratch: &mut Scratch,
        lower: VehicleId,
        upper: VehicleId,
    ) -> bool {
        let Scratch { places, stack, below, above, seen, lifted, .. } = scratch;
        let (top, bottom) = (
            places.get(upper).expect("queued"),
            places.get(lower).expect("queued"),
        );
        if follows(self.occupants, &self.vehicles[lower], upper) {
            return false;
        }
        // The pair's own edge is the one the lift reverses.
        let direct = |k: usize, m: usize| k == top && m == bottom;
        // Down from `upper`, no lower than `lower`: reaching it closes a cycle.
        let span = bottom - top + 1;
        seen.clear();
        seen.resize(span, false);
        stack.clear();
        below.clear();
        stack.push(top);
        seen[0] = true;
        while let Some(k) = stack.pop() {
            below.push(k);
            for m in k + 1..=bottom {
                if seen[m - top] || direct(k, m) || !self.edge(order, k, m) {
                    continue;
                }
                if m == bottom {
                    return false;
                }
                seen[m - top] = true;
                stack.push(m);
            }
        }
        // Up from `lower`, no higher than `upper`, which it can't reach, or the
        // search down would have reached `lower`.
        seen.fill(false);
        above.clear();
        stack.push(bottom);
        seen[span - 1] = true;
        while let Some(m) = stack.pop() {
            above.push(m);
            for k in top..m {
                if seen[k - top] || direct(k, m) || !self.edge(order, k, m) {
                    continue;
                }
                seen[k - top] = true;
                stack.push(k);
            }
        }
        // Those found going up first, then those found going down, each in
        // their order, into the places they had between them.
        above.sort_unstable();
        below.sort_unstable();
        lifted.clear();
        lifted.extend(above.iter().chain(below.iter()).map(|&k| order[k]));
        stack.clear();
        stack.extend(above.iter().chain(below.iter()));
        stack.sort_unstable();
        for (&k, &id) in stack.iter().zip(lifted.iter()) {
            order[k] = id;
            places.set(id, k);
        }
        true
    }
}

/// How far a vehicle is from where its way conflicts with another's, along its
/// path: its front back from the start, negative once in it, and its rear back
/// from the end, negative once clear of it; and whether a barrier it hasn't
/// committed to holds it back before the start.
#[derive(Clone, Copy, Debug)]
struct Reach {
    to_start: f64,
    to_clear: f64,
    held: bool,
}

impl Reach {
    /// The braking a vehicle needs to give way to the other, from `accel`, the
    /// term the junction would give it for the other (see [`pair_accel`]),
    /// worked out only if needed: infinite once its front is in their conflict;
    /// none while a barrier holds it back before the conflict, as it will stop
    /// there, as at a red light; and `None` once it has cleared it.
    fn need(self, accel: impl FnOnce() -> f64) -> Option<f64> {
        if self.to_clear <= 0.0 {
            return None;
        }
        if self.to_start <= 0.0 {
            return Some(f64::INFINITY);
        }
        if self.held {
            return Some(0.0);
        }
        Some((-accel()).max(0.0))
    }
}

/// How many of a junction's lanes [`zone`] reasons about for one vehicle: more
/// is unheard of, and those beyond are left out.
const WAY_DEPTH: usize = 8;

/// Each of the junction's lanes `vehicle`'s body or way is on, how far that
/// lane's start is ahead of its front, and its place on its path: the lanes of
/// its path in the junction, and any it is leaving behind, its rear still on
/// it, which aren't on its path.
type Way = ArrayVec<(LaneId, f64, Option<usize>), WAY_DEPTH>;

/// [`Way`]'s lanes for `vehicle`.
fn way(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    id: VehicleId,
    vehicle: &Vehicle,
) -> Way {
    let mut lanes = ArrayVec::new();
    let front = vehicle.lane_pos.s - vehicle.spec.rear_axle + vehicle.spec.length;
    let first = vehicle.path[0];
    if vehicle.lane_pos.s - vehicle.spec.rear_axle < 0.0 {
        for &p in network.lane(first).predecessors() {
            let on = || occupants[p.0 as usize].iter().any(|o| o.is_record_of(id));
            if network.lane(p).junction() == Some(junction) && on() {
                let start = -front - network.lane(p).geometry().length();
                let _ = lanes.try_push((p, start, None));
            }
        }
    }
    let mut start = -front;
    for (k, &lane) in vehicle.path.iter().enumerate() {
        let l = network.lane(lane);
        if l.junction() == Some(junction) {
            let _ = lanes.try_push((lane, start, Some(k)));
        }
        start += l.geometry().length();
    }
    lanes
}

/// Where two vehicles' ways through `junction` conflict, as each one's
/// [`Reach`] to it: from the first start to the last end, along each one's path,
/// of every conflict between a lane of one's way and a lane of the other's.
/// `None` if they don't conflict.
fn zone(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    [(ia, a), (ib, b)]: [(VehicleId, &Vehicle); 2],
) -> Option<[Reach; 2]> {
    let (way_a, way_b) = (
        way(network, occupants, junction, ia, a),
        way(network, occupants, junction, ib, b),
    );
    // Each one's first start, the lane it is on and where along it, and last
    // end.
    type Span = (f64, Option<(usize, f64)>, f64);
    let mut ra: Span = (f64::INFINITY, None, f64::NEG_INFINITY);
    let mut rb: Span = (f64::INFINITY, None, f64::NEG_INFINITY);
    let widen = |r: &mut Span, start: f64, k: Option<usize>, s: f64, end: f64| {
        if start < r.0 {
            (r.0, r.1) = (start, k.map(|k| (k, s)));
        }
        r.2 = r.2.max(end);
    };
    for &(la, sa, ka) in &way_a {
        for &(lb, sb, kb) in &way_b {
            if let Some(c) = network.conflict_between(la, lb) {
                let (mine, theirs) = (&c.mine, &c.theirs);
                widen(&mut ra, sa + mine.start, ka, mine.start, sa + mine.end);
                widen(
                    &mut rb,
                    sb + theirs.start,
                    kb,
                    theirs.start,
                    sb + theirs.end,
                );
            }
        }
    }
    let reach = |(start, at, end): Span, v: &Vehicle| Reach {
        to_start: start,
        to_clear: end + v.spec.length,
        held: at.is_some_and(|(k, s)| barrier::holds_before(network, v, k, s)),
    };
    (ra.0 < f64::INFINITY).then(|| [reach(ra, a), reach(rb, b)])
}

/// What the junction's pair rules read.
#[derive(Clone, Copy)]
struct View<'a> {
    network: &'a SimNetwork,
    vehicles: &'a SlotMap<VehicleId, Vehicle>,
    occupants: &'a [Vec<Occupant>],
    junction: JunctionId,
}

impl View<'_> {
    /// The braking each of two vehicles whose ways conflict needs to give way
    /// to the other (see [`Reach::need`]): `None` if their ways don't conflict
    /// or either has cleared their conflict.
    fn needs(self, a: VehicleId, b: VehicleId) -> Option<[f64; 2]> {
        let (va, vb) = (&self.vehicles[a], &self.vehicles[b]);
        let [ra, rb] = zone(
            self.network,
            self.occupants,
            self.junction,
            [(a, va), (b, vb)],
        )?;
        let na = ra.need(|| self.pair_accel(a, b))?;
        let nb = rb.need(|| self.pair_accel(b, a))?;
        Some([na, nb])
    }

    /// Whether two queued vehicles are a decided pair: their ways conflict,
    /// neither has cleared it, and either can no longer give way to the other
    /// braking comfortably.
    fn decided(self, a: VehicleId, b: VehicleId) -> bool {
        self.needs(a, b).is_some_and(|[na, nb]| {
            na > self.vehicles[a].spec.comfy_decel || nb > self.vehicles[b].spec.comfy_decel
        })
    }

    /// The lowest of the terms the junction would give `id` to keep out of the
    /// way of `other` (see [`pair_terms`]): infinite if none.
    fn pair_accel(self, id: VehicleId, other: VehicleId) -> f64 {
        let mut accel = f64::INFINITY;
        pair_terms(self, id, other, |term, _, _| accel = accel.min(term));
        accel
    }
}

/// Whether `a` and `b`, both in `junction`'s queue, are a decided pair (see
/// [`View::decided`]).
pub(crate) fn decided(state: &SimState, junction: JunctionId, a: VehicleId, b: VehicleId) -> bool {
    let SimState { network, vehicles, occupants, .. } = state;
    View { network, vehicles, occupants, junction }.decided(a, b)
}

/// Whether `other`, in `junction`'s queue, holds `id` back: `id` is in the
/// queue below it and they are a decided pair, or `id` isn't queued, so would
/// join below it, and `other` can no longer give way to it braking comfortably.
pub(crate) fn holds_back(
    state: &SimState,
    junction: JunctionId,
    other: VehicleId,
    id: VehicleId,
) -> bool {
    let SimState { network, vehicles, occupants, queues, .. } = state;
    let view = View { network, vehicles, occupants, junction };
    let order = &queues[junction.0 as usize].order;
    let place = |v: VehicleId| order.iter().position(|&q| q == v);
    let Some(theirs) = place(other) else {
        return false;
    };
    match place(id) {
        Some(mine) => theirs < mine && view.decided(other, id),
        None => view.needs(other, id).is_some_and(|[n, _]| n > vehicles[other].spec.comfy_decel),
    }
}

/// Whether `follower` follows `leader`, `id`: the leader's body is ahead of it
/// on a lane of its path, so car following holds it back. Its body is on a
/// lane it is headed along or leaving, or, at a fork, overlapping a branch
/// beside its own until it has diverged: in a junction, those are the only
/// records a vehicle has, as crossing lanes are kept apart by the junction.
fn follows(occupants: &[Vec<Occupant>], follower: &Vehicle, id: VehicleId) -> bool {
    let rear = follower.lane_pos.s - follower.spec.rear_axle;
    follower.path.iter().enumerate().any(|(k, &lane)| {
        occupants[lane.0 as usize]
            .iter()
            .any(|o| o.is_record_of(id) && (k > 0 || o.s() > rear))
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

/// How far a vehicle's front is from where it joins a junction's queue,
/// negative once past it, if its path has such a point there: the start of the
/// first of the junction's lanes on its path that is to be kept clear, or the
/// start of the first conflict, whichever comes first.
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

/// Whether a vehicle approaching a junction must join its queue: it can no
/// longer stop comfortably before the first of the junction's lanes on its path
/// to be kept clear, or before its first conflict there, or its front is past
/// that already (see [`commit_distance`]). So a vehicle inside the junction,
/// past its stop line, is always in the queue, never unseen, and one not yet in
/// it can always stop comfortably for anything in it. One with neither on its
/// path never needs to.
fn must_commit(network: &SimNetwork, junction: JunctionId, vehicle: &Vehicle) -> bool {
    commit_distance(network, junction, vehicle)
        .is_some_and(|distance| distance <= vehicle.comfortable_stop())
}

/// Whether the vehicle `index`th in a junction's queue may be released: it
/// could now stop comfortably a car length short of where it joins the queue
/// (see [`commit_distance`]), so it plainly can stop, and no vehicle queued
/// after it is behind it on its way in. The car length is the hysteresis
/// against [`must_commit`], and keeps a vehicle held at the line queued.
/// Releasing from the back of each lane keeps vehicles queued in their order
/// along it.
fn may_release(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    junction: JunctionId,
    order: &[VehicleId],
    index: usize,
) -> bool {
    let id = order[index];
    let vehicle = &vehicles[id];
    let Some(distance) = commit_distance(network, junction, vehicle) else {
        return false;
    };
    if vehicle.comfortable_stop() > distance - vehicle.spec.length {
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
    !order[index + 1..].iter().any(behind)
}

/// Whether a vehicle's path takes it onto any of a junction's lanes.
fn headed(network: &SimNetwork, vehicle: &Vehicle, junction: JunctionId) -> bool {
    vehicle.path.iter().any(|&lane| network.lane(lane).junction() == Some(junction))
}

/// The lowest of the terms `other`, above `id` in a junction's queue and a
/// decided pair with it, imposes on it, each recorded to the debug sink:
/// infinite if none does (see [`pair_terms`]).
pub(crate) fn term<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    junction: JunctionId,
    id: VehicleId,
    other: VehicleId,
) -> f64 {
    let SimState { network, vehicles, occupants, .. } = state;
    let view = View { network, vehicles, occupants, junction };
    let mut accel = f64::INFINITY;
    pair_terms(view, id, other, |term, gap, kind| {
        let source = AccelSource::Junction { id: other, kind, gap };
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        accel = accel.min(term);
    });
    accel
}

/// Calls `each` with every term the junction would give `id` to keep out of
/// the way of `other`, with the gap it is for and the conflict's kind: one for
/// every conflict between a lane of its path in the junction and a lane of the
/// junction's (see [`conflict_term`]).
fn pair_terms(
    view: View,
    id: VehicleId,
    other: VehicleId,
    mut each: impl FnMut(f64, f64, ConflictKind),
) {
    let network = view.network;
    let lanes = network.junction(view.junction).lanes();
    let follower = &view.vehicles[id];
    let front = follower.lane_pos.s - follower.spec.rear_axle + follower.spec.length;
    // From its front to the start of each lane on its path.
    let mut start = -front;
    for &mine in &follower.path {
        let length = network.lane(mine).geometry().length();
        if network.lane(mine).junction() == Some(view.junction) {
            for &theirs in lanes {
                let Some(conflict) = network.conflict_between(mine, theirs) else {
                    continue;
                };
                if let Some((term, gap)) =
                    conflict_term(view, follower, mine, conflict, theirs, other, start)
                {
                    each(term, gap, conflict.kind);
                }
            }
        }
        start += length;
    }
}

/// The term a junction imposes on `follower` for `other`, above it in its
/// queue, where `mine`, a lane of its path whose start is `start` meters from
/// its front, conflicts with `theirs`, and the gap that term is for: `None` once
/// `other` has cleared the conflict on its lane, its rear past the end of it,
/// or, not headed onto `theirs`, while its body isn't in it.
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
    view: View,
    follower: &Vehicle,
    mine: LaneId,
    conflict: &Conflict,
    theirs: LaneId,
    other: VehicleId,
    start: f64,
) -> Option<(f64, f64)> {
    let network = view.network;
    let leader = &view.vehicles[other];
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
        None => view.occupants[theirs.0 as usize]
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
