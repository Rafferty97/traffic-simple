//! Gap acceptance and priority (see `docs/models/gap-acceptance.md`): when the head on its way to a barrier
//! commits to crossing it. At a signal, by its light and the dilemma zone; at a
//! give-way or stop line, once there is a gap in the traffic it gives way to,
//! with its impatience; at an all-way stop, by the order vehicles came to rest;
//! and everywhere, once there is room beyond and nobody inside it gives way to.
//!
//! It answers once per longitudinal update, for the nearest barrier the head
//! hasn't committed to, and the core carries the answer out (see
//! [`crate::barrier`]): counting what a vehicle has committed to, passing and
//! releasing it, and holding it at the line. A gap is taken ahead only of vehicles not yet in the junction's queue,
//! and behind only those queued, so that a vehicle joins the queue where it
//! means to go; a replacement must keep to that.

use crate::barrier::{held_by_light, holds_before, into_junction, nth};
use crate::models::acceleration::{Profiles, reach_time};
use crate::models::signal::Light;
use crate::network::{Barrier, BarrierKind, Conflict, JunctionId, LaneId, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// What the head on its way to a barrier does about it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Answer {
    /// It commits to crossing: from now on, the barrier isn't there for it.
    Commit,
    /// It isn't committed yet, but nothing holds it back: a green light it can
    /// still stop for, or a keep-clear zone with room past it.
    Pass,
    /// It stops at the line, giving way to this vehicle: the first with
    /// priority it found no gap before or behind, or one already in the
    /// junction it gives way to.
    Yield(VehicleId),
    /// It stops at the line, for no vehicle in particular: a red or amber light,
    /// or no room beyond.
    Stop,
}

/// What `vehicle`, the head on its way to `barrier`, the `k`th lane of its path,
/// `gap` from its front to the line, does about it.
#[allow(clippy::too_many_arguments)]
pub(crate) fn decide(
    state: &SimState,
    through: &[f64],
    profiles: &Profiles,
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: Barrier,
    gap: f64,
) -> Answer {
    let spec = &vehicle.spec;
    let speed = vehicle.speed;
    // Whether it can stop where it would like, a line gap short, at `decel`.
    let line_gap = state.drivers.line_gap(id);
    let can_stop = |decel: f64| speed * speed / (2.0 * decel) <= gap - line_gap;
    let yields_to = std::cell::Cell::new(None);
    let room = || {
        room_beyond(state, id, vehicle, k, barrier.s)
            && match yields_inside(state, id, vehicle, k, &barrier) {
                Some(other) => {
                    yields_to.set(Some(other));
                    false
                }
                None => true,
            }
    };
    let has_gap = |b: &Barrier| match has_gap(state, through, profiles, id, vehicle, k, b) {
        Ok(()) => true,
        Err(other) => {
            yields_to.set(Some(other));
            false
        }
    };
    let commit = match barrier.kind {
        // Only on green, once it can no longer stop comfortably: never on amber
        // or red, so one that hasn't committed by then brakes, and one that
        // couldn't have stopped comfortably committed while it was green.
        BarrierKind::Signal { movement, on_red } => match state.lights[movement.0 as usize].light {
            Light::Green if can_stop(spec.comfy_decel) => match room() {
                true => return Answer::Pass,
                false => false,
            },
            Light::Green => room(),
            Light::Amber => false,
            // Turning on red, a stop line giving way to the lanes it crosses.
            Light::Red => on_red.is_some_and(|set| {
                vehicle.stopped_at.is_some()
                    && has_gap(&Barrier { yields: Some(set), ..barrier })
                    && room()
            }),
        },
        BarrierKind::GiveWay => has_gap(&barrier) && room(),
        BarrierKind::Stop => vehicle.stopped_at.is_some() && has_gap(&barrier) && room(),
        // As a green light: nothing holds it back while it can still stop
        // comfortably and there is room, and it commits once it can't.
        BarrierKind::KeepClear { length } => {
            let room = room_past(state, id, vehicle, k, barrier.s, length);
            if room && can_stop(spec.comfy_decel) {
                return Answer::Pass;
            }
            room
        }
    };
    match (commit, yields_to.get()) {
        (true, _) => Answer::Commit,
        (false, Some(other)) => Answer::Yield(other),
        (false, None) => Answer::Stop,
    }
}

/// How much later than a yielding vehicle clears a conflict a vehicle with
/// priority must be able to reach it at the soonest, in seconds.
const GAP_MARGIN: f64 = 0.5;

/// How long a driver of no assertiveness waits at rest at its line before it
/// is fully impatient, in seconds; a fully assertive one, a third as long.
/// SUMO's `timeToImpatience` is 180 s by default, which leaves ties between
/// waiting drivers locked for minutes.
const TIME_TO_IMPATIENCE: f64 = 30.0;

/// The fraction of its comfortable deceleration a fully impatient driver
/// reckons a priority vehicle will brake at to give way to it.
const IMPATIENT_DECEL: f64 = 0.5;

/// How impatient `vehicle` is, from 0 to 1: nothing at first, growing the
/// longer it has waited at rest at its line, and faster the more assertive it
/// is, as SUMO's `impatience` grows with waiting time. An impatient driver
/// reckons a priority vehicle not yet committed will give way to it, braking
/// at that fraction of [`IMPATIENT_DECEL`] of its comfortable deceleration,
/// rather than coming on as fast as it can: so it comes to cut in front of traffic, stopped or not,
/// which breaks the ties where each of two waits on traffic stuck behind the
/// other. The junction keeps it safe, and a vehicle that hasn't committed can
/// always stop for it at comfortable deceleration.
fn impatience(state: &SimState, id: VehicleId, vehicle: &Vehicle) -> f64 {
    let Some(since) = vehicle.stopped_at else {
        return 0.0;
    };
    let waited = state.time_since(since);
    let rate = (1.0 + 2.0 * state.drivers.traits(id).assertiveness) / TIME_TO_IMPATIENCE;
    (waited * rate).min(1.0)
}

/// Whether `vehicle`, at a give-way or stop `barrier` on its `k`th lane, has a
/// gap: on its lanes in that junction, up to the next barrier on its path, it
/// can clear each conflict with a lane the barrier gives way to before any
/// vehicle there could reach it, plus [`GAP_MARGIN`], each accelerating as hard
/// as it can up to its desired speed; at a merge, ahead of it by that vehicle's
/// time headway too, so it needn't brake. On a lane of equal priority it takes
/// turns with, the same, except that a vehicle not yet committed to its own
/// line gives way to this one if this one came to rest at its line first, and
/// impatience counts for nothing: so vehicles of equal priority go in the order
/// they came to rest, and two that would meet both stop and take turns. On any
/// other lane it only counts vehicles already in the junction's queue, and
/// goes behind them (see `docs/models/gap-acceptance.md`). Otherwise, the
/// vehicle that refuses it.
fn has_gap(
    state: &SimState,
    through: &[f64],
    profiles: &Profiles,
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: &Barrier,
) -> Result<(), VehicleId> {
    let network = &state.network;
    let s = barrier.s;
    let yields = network.yields(barrier);
    let ties = network.ties(barrier);
    let (path, spec) = (&vehicle.path, &vehicle.spec);
    let Some((junction, _)) = into_junction(network, path, k) else {
        return Ok(());
    };
    // The next barrier on its path, as its lane's index on the path and `s` there.
    let next = (k..path.len()).find_map(|j| {
        let from = if j == k { s } else { f64::NEG_INFINITY };
        let barriers = network.lane(path[j]).barriers();
        barriers.iter().find(|b| b.s > from).map(|b| (j, b.s))
    });
    let front = vehicle.lane_pos.s - spec.rear_axle + spec.length;
    let impatience = impatience(state, id, vehicle);
    // The start of each lane of its path, from its front.
    let mut start = -front;
    for (j, &mine) in path.iter().enumerate() {
        let lane = network.lane(mine);
        if j > k && lane.junction() != Some(junction) {
            break;
        }
        if j >= k {
            for &theirs in network.junction(junction).lanes() {
                let Some(conflict) = network.conflict_between(mine, theirs) else {
                    continue;
                };
                let before_next =
                    next.is_none_or(|(nj, ns)| j < nj || (j == nj && conflict.mine.start < ns));
                let after_this = j > k || conflict.mine.end > s;
                if !before_next || !after_this {
                    continue;
                }
                // It clears a crossing once its rear is past it.
                let desired = state.drivers.desired_speed(id, spec, profiles.lowest(mine));
                let to_clear = start + conflict.mine.end + spec.length;
                let clears = reach_time(vehicle.speed, spec.max_accel, desired, to_clear);
                let to_start = start + conflict.mine.start;
                let arrives = reach_time(vehicle.speed, spec.max_accel, desired, to_start);
                let headway = spec.time_headway;
                // Not given way to, only those in the junction's queue for it
                // count, which the junction holds it back for all the same.
                let tie = !yields.contains(&theirs) && ties.contains(&theirs);
                let queued_only = !yields.contains(&theirs) && !tie;
                let way = Way {
                    junction,
                    theirs,
                    conflict,
                    clears,
                    arrives,
                    headway,
                    queued_only,
                    tie,
                    arrived: vehicle.stopped_at,
                    impatience,
                };
                let between = -network.lane(theirs).geometry().length();
                let (from, ahead) = (conflict.theirs.end, conflict.theirs.start);
                clear(
                    state, through, profiles, id, &way, theirs, from, ahead, between,
                )?;
            }
        }
        start += lane.geometry().length();
    }
    Ok(())
}

/// A conflict a yielding vehicle checks a gap at, on the priority lane `theirs`.
struct Way<'a> {
    junction: JunctionId,
    theirs: LaneId,
    conflict: &'a Conflict,
    /// When the yielding vehicle could clear it at the soonest, in seconds.
    clears: f64,
    /// When the yielding vehicle could reach it at the soonest, in seconds.
    arrives: f64,
    /// The yielding vehicle's time headway, kept behind one it merges after.
    headway: f64,
    /// Whether the barrier doesn't give way to the lane, so only vehicles that
    /// have committed count, and those only to go behind.
    queued_only: bool,
    /// Whether the lane is of equal priority: its vehicles give way to the
    /// yielding vehicle if it came to rest at its line first.
    tie: bool,
    /// When the yielding vehicle came to rest at its line, if it has.
    arrived: Option<u64>,
    /// How impatient the yielding vehicle is (see [`impatience`]).
    impatience: f64,
}

/// Whether there is a gap for the yielding vehicle among the vehicles with
/// priority on `way`, searching back from `s` on `lane` and then into the lanes
/// leading into it, as the follower search does, where `lane`'s start is
/// `ahead` back from the conflict's start on the priority lane, and its end
/// `between` back from that lane's start: minus that lane's length on the
/// priority lane itself.
///
/// On each branch, from the nearest vehicle headed for the conflict back: if it
/// can go ahead of one, clearing the conflict before that one could reach it,
/// there is a gap, since those behind arrive later; if it can go behind one,
/// reaching the conflict no sooner than that one, at its speed now, has
/// cleared it, the next one decides; otherwise there is none. It can go ahead
/// only of one not yet in the junction's queue, and behind only one in it, so
/// that it joins the queue where it means to go. Either way the
/// margin is [`GAP_MARGIN`] plus the time headway of whichever then follows the
/// other through the conflict, as the junction has it follow, so neither need
/// brake. One at rest never clears, so nothing goes behind it.
///
/// A vehicle a red or amber light holds short of the conflict holds back the
/// rest of its branch (see [`held_by_light`]); one at a green light or a
/// give-way line may yet go, and counts.
#[allow(clippy::too_many_arguments)]
fn clear(
    state: &SimState,
    through: &[f64],
    profiles: &Profiles,
    id: VehicleId,
    way: &Way,
    lane: LaneId,
    s: f64,
    ahead: f64,
    between: f64,
) -> Result<(), VehicleId> {
    let network = &state.network;
    let at = way.conflict.theirs.start;
    let list = &state.occupants[lane.0 as usize];
    let end = list.partition_point(|o| o.s() < s);
    for o in list[..end].iter().rev() {
        let other = &state.vehicles[o.id];
        let on_it = lane == way.theirs;
        let k = other.path.iter().position(|&l| l == way.theirs);
        if o.is_ghost() || o.id == id {
            continue;
        }
        // Headed elsewhere, it is passed over, unless it waits in the lane at a
        // barrier it hasn't committed to, a turner waiting for a gap say: then
        // nobody behind it can come until it goes. On the priority lane itself,
        // its body lingering in the conflict as it leaves counts while it is
        // there; a body beside the lane, overlapping it where it runs alongside
        // its own, is in the way only where its own lanes are, as the junction
        // has it.
        let in_conflict = on_it
            && network.left_behind(other.path[0], lane)
            && o.s() + other.spec.length > way.conflict.theirs.start;
        if k.is_none() && !in_conflict {
            if waits_in_lane(network, other, o) {
                return Ok(());
            }
            continue;
        }
        // Its front, back from the conflict's start, negative once in it, and
        // its rear, back from the conflict's end, negative once clear of it.
        let distance = ahead - (o.s() + other.spec.length);
        let rear = distance + (way.conflict.theirs.end - at) + other.spec.length;
        if rear <= 0.0 {
            continue;
        }
        // One the junction's order puts after this one, by a chain of decided
        // pairs and vehicles following one another, is no matter: it can't go
        // first, and nor can anyone behind it on its way.
        if crate::junction::goes_after(state, way.junction, id, o.id) {
            return Ok(());
        }
        // One that holds this one back, or would once it joined the queue below
        // it, can't be gone ahead of, whatever their levels: only behind.
        let holds = || crate::junction::holds_back(state, way.junction, o.id, id);
        let clears = if other.speed > 0.0 {
            rear / other.speed
        } else {
            f64::INFINITY
        };
        let behind_margin = GAP_MARGIN + way.headway;
        // On a lane not given way to, only those that have committed count:
        // holding it back, or moving past every barrier before the conflict, as
        // one moving off an all-way stop line is; and those only to go behind.
        // At rest and not holding it back, it can stop, as one stuck behind a
        // vehicle released from its line, having committed behind it, is.
        let committed = || {
            holds() || other.speed > 0.0 && k.is_some_and(|k| !holds_before(network, other, k, at))
        };
        // Of equal priority and not yet committed to its own line, it gives way
        // to this one if this one came to rest at its line first: earlier, or
        // in the same step and first by vehicle, so runs repeat.
        let first = |t: u64| other.stopped_at.is_none_or(|u| (t, id) < (u, o.id));
        if way.tie
            && k.is_some_and(|k| holds_before(network, other, k, at))
            && way.arrived.is_some_and(first)
        {
            continue;
        }
        if holds() || way.queued_only && committed() {
            if way.arrives < clears + behind_margin {
                return Err(o.id);
            }
            continue;
        }
        if way.queued_only {
            continue;
        }
        // Held short of the conflict by a light, it will surely stop there.
        if k.is_some_and(|k| held_by_light(state, other, k, at)) {
            return Ok(());
        }
        let limit = profiles.lowest(way.theirs);
        let desired = state.drivers.desired_speed(o.id, &other.spec, limit);
        // As fast as it can, or, to an impatient driver, giving way to it: at
        // full impatience braking at `IMPATIENT_DECEL` of its comfortable
        // deceleration to stop its minimum gap short, as IDM, following, brakes
        // harder than the constant deceleration that would just stop it.
        let i = if way.tie { 0.0 } else { way.impatience };
        let decel = IMPATIENT_DECEL * other.spec.comfy_decel;
        let accel = (1.0 - i) * other.spec.max_accel - i * decel;
        let reaches = reach_time(
            other.speed,
            accel,
            desired,
            distance - i * other.spec.min_gap,
        );
        // Whichever follows the other through the conflict keeps its time
        // headway behind it there, as the junction makes it, so needn't brake.
        let ahead_margin = GAP_MARGIN + other.spec.time_headway;
        if distance >= 0.0 && reaches > way.clears + ahead_margin {
            return Ok(());
        }
        // Not queued yet, it can't be gone behind either: going, this one
        // would join the queue ahead of it, and the junction would have it
        // give way after all. Once it is queued, this one may follow it.
        return Err(o.id);
    }
    // Nobody on this lane: on to those leading into it that a path through
    // reaches the priority lane from.
    let between = between + network.lane(lane).geometry().length();
    let reaches = |p: &&LaneId| through[p.0 as usize] > between;
    for &p in network.lane(lane).predecessors().iter().filter(reaches) {
        let length = network.lane(p).geometry().length();
        clear(
            state,
            through,
            profiles,
            id,
            way,
            p,
            length,
            ahead + length,
            between,
        )?;
    }
    Ok(())
}

/// Whether `vehicle`, with record `o` on a lane leading to a conflict, blocks
/// that lane, waiting: at rest in the lane, with a barrier it hasn't committed
/// to on the lane it follows or the next.
fn waits_in_lane(network: &SimNetwork, vehicle: &Vehicle, o: &crate::following::Occupant) -> bool {
    let next = nth(network, vehicle, vehicle.committed as usize);
    o.lateral == crate::following::Lateral::InLane
        && vehicle.speed == 0.0
        && next.is_some_and(|(lane, _)| vehicle.path.iter().take(2).any(|&l| l == lane))
}

/// Whether `vehicle`, entering the junction its `k`th lane is in at `barrier`,
/// first gives way to a vehicle already in it part way through, as drivers give
/// way to vehicles in an intersection: one waiting to take a lane the barrier
/// gives way to, a turner's next stretch, which compilation lists for the
/// turners whose way on crosses this one's (see `docs/DESIGN.md`, Barriers from
/// control). That is one in the junction's queue about to wait at that lane, or
/// one committed to its barrier into the junction but not yet in the queue, on
/// a lane the junction is approached along, whose next barrier is at that lane.
/// Driver behaviour: the junction doesn't rely on it. Never, for a vehicle in
/// the junction's queue already: its barriers inside the junction decide.
fn yields_inside(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: &Barrier,
) -> Option<VehicleId> {
    let network = &state.network;
    let yields = network.yields(barrier);
    if yields.is_empty() {
        return None;
    }
    let (junction, _) = into_junction(network, &vehicle.path, k)?;
    let queue = &state.queues[junction.0 as usize];
    let order = &queue.order;
    if order.contains(&id) {
        return None;
    }
    // In the queue, and about to wait at such a lane.
    let queued = order.iter().copied().find(|&q| {
        crate::junction::waits_at(network, &state.vehicles[q], junction)
            .is_some_and(|at| yields.contains(&at))
    });
    // Committed to its barrier into the junction, not yet in the queue, and
    // next held at such a lane.
    let coming = || {
        queue.approaching.iter().find_map(|&lane| {
            state.occupants[lane.0 as usize].iter().find(|o| {
                let other = &state.vehicles[o.id];
                let next = nth(network, other, usize::from(other.committed));
                o.id != id
                    && !o.is_ghost()
                    && other.committed > 0
                    && !order.contains(&o.id)
                    && next.is_some_and(|(at, _)| yields.contains(&at))
            })
        })
    };
    queued.or_else(|| coming().map(|o| o.id))
}

/// Whether there is room for `vehicle`, entering a keep-clear zone `length`
/// long at `s` on its `k`th lane, beyond the zone's far end: room for itself
/// and every vehicle ahead of it short of the far end, each its length and
/// minimum gap. Taking the vehicles beyond nearest first: if there is that
/// room behind one now, there is room; if there wouldn't be even once it had
/// gone on as far as it would if it stopped comfortably, there isn't; and
/// otherwise there is room only if there is room behind the next one for this
/// one too. Along its path, as far as it knows it.
fn room_past(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    s: f64,
    length: f64,
) -> bool {
    let network = &state.network;
    let path = &vehicle.path;
    let mut needed = vehicle.spec.length + vehicle.spec.min_gap;
    // Where each lane of its path starts, back from the zone's far end.
    let to_zone: f64 = path[..k].iter().map(|&l| network.lane(l).geometry().length()).sum();
    let mut start = -(to_zone + s + length);
    for (j, &lane) in path.iter().enumerate() {
        let from = if j == 0 {
            vehicle.lane_pos.s
        } else {
            f64::NEG_INFINITY
        };
        for o in state.occupants[lane.0 as usize].iter().filter(|o| o.s() >= from) {
            if o.is_ghost() || o.id == id {
                continue;
            }
            let other = &state.vehicles[o.id];
            let beyond = start + o.s();
            if beyond >= 0.0 {
                if beyond >= needed {
                    return true;
                }
                if beyond + other.comfortable_stop() < needed {
                    return false;
                }
            }
            needed += other.spec.length + other.spec.min_gap;
        }
        start += network.lane(lane).geometry().length();
    }
    true
}

/// Whether, entering the junction its `k`th lane is in at `s` along it, `vehicle`
/// would find room on the lane after it, as the road rules require: room for
/// itself and every vehicle ahead of it on its way through the junction to the
/// same lane, each its length and minimum gap, behind the rearmost vehicle
/// there, allowing for how far that one would go on if it stopped comfortably.
/// Nor would it find room behind a vehicle ahead of it on its way through,
/// headed elsewhere, that is at rest or will wait inside the junction, as a
/// turner waiting for a gap does: it couldn't drive through. Unless it will
/// wait inside the junction itself, so may draw up behind one. Always, for a
/// barrier that isn't into a junction.
fn room_beyond(state: &SimState, id: VehicleId, vehicle: &Vehicle, k: usize, s: f64) -> bool {
    let network = &state.network;
    let path = &vehicle.path;
    let Some((junction, first)) = into_junction(network, path, k) else {
        return true;
    };
    // The first lane of its path out of the junction.
    let Some(exit) = (first..path.len()).find(|&j| network.lane(path[j]).junction().is_none())
    else {
        return true;
    };
    let exit_lane = path[exit];
    let junction = Some(junction);
    // Whether a vehicle will wait inside the junction: its next barrier it
    // hasn't committed to, after this one for this vehicle, is in it.
    let waits_inside = |v: &Vehicle, after: usize| {
        nth(network, v, after).is_some_and(|(lane, _)| network.lane(lane).junction() == junction)
    };
    let mut needed = vehicle.spec.length + vehicle.spec.min_gap;
    let may_wait = waits_inside(vehicle, usize::from(vehicle.committed) + 1);
    for (j, &lane) in path.iter().enumerate().take(exit).skip(k) {
        let from = if j == k { s } else { f64::NEG_INFINITY };
        for o in &state.occupants[lane.0 as usize] {
            let other = &state.vehicles[o.id];
            if o.is_ghost() || o.id == id || o.s() < from {
                continue;
            }
            if !other.path.contains(&exit_lane) {
                // A body beside the lane, overlapping it where it runs alongside
                // its own, as at a fork, is in the way only where its own lanes
                // are, as the junction has it: on its way, or left behind.
                let on_it = other.path.contains(&lane) || network.left_behind(other.path[0], lane);
                let blocks = on_it
                    && (other.speed == 0.0 || waits_inside(other, usize::from(other.committed)));
                if blocks && !may_wait {
                    return false;
                }
                continue;
            }
            needed += other.spec.length + other.spec.min_gap;
        }
    }
    let rearmost = state.occupants[exit_lane.0 as usize]
        .iter()
        .find(|o| !o.is_ghost() && o.id != id);
    match rearmost {
        None => true,
        Some(o) => o.s() + state.vehicles[o.id].comfortable_stop() >= needed,
    }
}
