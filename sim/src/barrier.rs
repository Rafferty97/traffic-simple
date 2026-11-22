//! Barriers: stop lines, give-way lines and signals that vehicles stop at until
//! they commit to crossing (see `docs/DESIGN.md`, Barriers and signals).
//!
//! A vehicle's commitment is a count, of the barriers ahead of its rear axle on
//! its path it has committed to, in path order. Here are the rules for
//! counting them, for deciding to commit, and for the room a junction entry
//! needs beyond.

use crate::lane_changing::comfortable_stop;
use crate::models::acceleration::{self, reach_time};
use crate::models::signal::{Light, MovementLight};
use crate::network::{Barrier, BarrierKind, Conflict, JunctionId, Lane, LaneId, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId, VehicleSpec};

/// How many of `lane`'s barriers lie after `from` and at or before `to`, along it.
pub(crate) fn between(lane: &Lane, from: f64, to: f64) -> usize {
    let barriers = lane.barriers();
    let lo = barriers.partition_point(|b| b.s <= from);
    let hi = barriers.partition_point(|b| b.s <= to);
    hi.saturating_sub(lo)
}

/// How many barriers lie on `vehicle`'s path ahead of its rear axle and before
/// `s` on its `k`th lane, or at `s` too if `inclusive`: so a barrier's rank
/// among those ahead of it, counting from 0.
pub(crate) fn rank(
    network: &SimNetwork,
    vehicle: &Vehicle,
    k: usize,
    s: f64,
    inclusive: bool,
) -> usize {
    let mut count = 0;
    for (j, &lane) in vehicle.path.iter().enumerate().take(k + 1) {
        let lane = network.lane(lane);
        let from = if j == 0 {
            vehicle.lane_pos.s
        } else {
            f64::NEG_INFINITY
        };
        let to = match (j == k, inclusive) {
            (false, _) => f64::INFINITY,
            (true, true) => s,
            // Just short of `s`: the last barrier before it.
            (true, false) => s.next_down(),
        };
        count += between(lane, from, to);
    }
    count
}

/// Whether `vehicle` has committed to the barrier at `s` on `lane`: it is ahead
/// of its rear axle on its path, among the first of its count.
pub(crate) fn committed_to(network: &SimNetwork, vehicle: &Vehicle, lane: LaneId, s: f64) -> bool {
    let ahead = |k: usize| vehicle.path[k] == lane && (k > 0 || s > vehicle.lane_pos.s);
    match (0..vehicle.path.len()).find(|&k| ahead(k)) {
        Some(k) => rank(network, vehicle, k, s, false) < vehicle.committed as usize,
        None => false,
    }
}

/// The barrier `rank`th ahead of `vehicle`'s rear axle on its path, and its
/// lane, if there is one.
pub(crate) fn nth(
    network: &SimNetwork,
    vehicle: &Vehicle,
    rank: usize,
) -> Option<(LaneId, Barrier)> {
    let mut count = 0;
    for (j, &lane) in vehicle.path.iter().enumerate() {
        let from = if j == 0 {
            vehicle.lane_pos.s
        } else {
            f64::NEG_INFINITY
        };
        for &barrier in network.lane(lane).barriers().iter().filter(|b| b.s > from) {
            if count == rank {
                return Some((lane, barrier));
            }
            count += 1;
        }
    }
    None
}

/// Whether a barrier ahead holds `vehicle` back from the point `s` on its `k`th
/// lane: one at or before it that it hasn't committed to.
pub(crate) fn holds_before(network: &SimNetwork, vehicle: &Vehicle, k: usize, s: f64) -> bool {
    rank(network, vehicle, k, s, true) > vehicle.committed as usize
}

/// Whether a light holds `vehicle` short of the point `s` on its `k`th lane: a
/// signal at or before it, that it hasn't committed to, on amber, or on red
/// unless it may turn on red. Under [`decide`], nothing commits to one then, so
/// it will surely stop there. One at a green light, or a give-way line, or a
/// red light it may turn on, it hasn't committed to may yet go.
pub(crate) fn held_by_light(state: &SimState, vehicle: &Vehicle, k: usize, s: f64) -> bool {
    held_by(&state.network, &state.lights, vehicle, k, s)
}

/// [`held_by_light`], from the network and the lights alone.
pub(crate) fn held_by(
    network: &SimNetwork,
    lights: &[MovementLight],
    vehicle: &Vehicle,
    k: usize,
    s: f64,
) -> bool {
    let mut rank = 0;
    for (j, &lane) in vehicle.path.iter().enumerate().take(k + 1) {
        let from = if j == 0 {
            vehicle.lane_pos.s
        } else {
            f64::NEG_INFINITY
        };
        let to = if j == k { s } else { f64::INFINITY };
        for barrier in network.lane(lane).barriers().iter().filter(|b| b.s > from && b.s <= to) {
            let holds = match barrier.kind {
                // Turning on red, it may go after stopping, as at a stop line.
                BarrierKind::Signal { movement, on_red } => {
                    match lights[movement.0 as usize].light {
                        Light::Green => false,
                        Light::Amber => true,
                        Light::Red => on_red.is_none(),
                    }
                }
                BarrierKind::GiveWay
                | BarrierKind::Stop
                | BarrierKind::AllWayStop
                | BarrierKind::KeepClear { .. } => false,
            };
            if rank >= vehicle.committed as usize && holds {
                return true;
            }
            rank += 1;
        }
    }
    false
}

/// The junction a barrier on `path[k]` leads into, if any, and the index on the
/// path of its first lane there: `path[k]` itself, or, for a barrier on a
/// lead-in before the junction (see [`SimNetwork::first_of_connector`]), the
/// lane after it.
fn into_junction(network: &SimNetwork, path: &[LaneId], k: usize) -> Option<(JunctionId, usize)> {
    if let Some(junction) = network.lane(path[k]).junction() {
        return Some((junction, k));
    }
    let &next = path.get(k + 1)?;
    let junction = network.lane(next).junction()?;
    (network.first_of_connector(next) == path[k]).then_some((junction, k + 1))
}

/// How much further back than its gap to lines a vehicle at rest may be and
/// still have come to rest at a line, in meters: finishing a stop leaves it a
/// few centimetres further back (see `models::acceleration::stopping`).
pub(crate) const AT_LINE: f64 = 0.5;

/// The deceleration that amber times are set for a driver to stop at, in m/s²,
/// as ITE's change interval has it.
pub const AMBER_DECEL: f64 = 3.0;

/// What the head on its way to a barrier does about it at a longitudinal update.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Decision {
    /// It commits to crossing: from now on, the barrier isn't there for it.
    Commit,
    /// It isn't committed yet, but nothing holds it back: a green light it can
    /// still stop for.
    Pass,
    /// It is held back, stopping a line gap short: this is its term.
    Hold(f64),
}

/// What `vehicle`, the head on its way to `barrier`, the `k`th lane of its path,
/// `gap` from its front to the line, does about it, and the vehicle it gives
/// way to, if it is held for one: the first with priority it found no gap
/// before or behind, or one already in the junction it gives way to.
pub(crate) fn decide(
    state: &SimState,
    through: &[f64],
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: Barrier,
    gap: f64,
) -> (Decision, Option<VehicleId>) {
    let spec = &vehicle.spec;
    let speed = vehicle.speed;
    // Whether it can stop where it would like, a line gap short, at `decel`.
    let can_stop = |decel: f64| speed * speed / (2.0 * decel) <= gap - vehicle.line_gap;
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
    let has_gap = |b: &Barrier| match has_gap(state, through, id, vehicle, k, b) {
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
                true => return (Decision::Pass, None),
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
                return (Decision::Pass, None);
            }
            room
        }
        BarrierKind::AllWayStop => {
            vehicle.stopped_at.is_some()
                && first_to_arrive(state, id, vehicle, k, barrier)
                && has_gap(&barrier)
                && room()
        }
    };
    match commit {
        true => (Decision::Commit, None),
        false => (
            Decision::Hold(hold(spec, vehicle.line_gap, speed, gap)),
            yields_to.get(),
        ),
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

/// How impatient `vehicle` is, from 0 to 1: nothing at first, growing the
/// longer it has waited at rest at its line, and faster the more assertive it
/// is, as SUMO's `impatience` grows with waiting time. An impatient driver
/// reckons a priority vehicle not yet committed will give way to it, braking
/// at that fraction of its comfortable deceleration, rather than coming on as
/// fast as it can: so it comes to cut in front of traffic, stopped or not,
/// which breaks the ties where each of two waits on traffic stuck behind the
/// other. The junction keeps it safe, and a vehicle that hasn't committed can
/// always stop for it at comfortable deceleration.
fn impatience(state: &SimState, vehicle: &Vehicle) -> f64 {
    let Some(since) = vehicle.stopped_at else {
        return 0.0;
    };
    let waited = state.steps.saturating_sub(since) as f64 * state.config.step;
    let rate = (1.0 + 2.0 * vehicle.spec.assertiveness) / TIME_TO_IMPATIENCE;
    (waited * rate).min(1.0)
}

/// Whether `vehicle`, at a give-way or stop `barrier` on its `k`th lane, has a
/// gap: on its lanes in that junction, up to the next barrier on its path, it
/// can clear each conflict with a lane the barrier gives way to before any
/// vehicle there could reach it, plus [`GAP_MARGIN`], each accelerating as hard
/// as it can up to its desired speed; at a merge, ahead of it by that vehicle's
/// time headway too, so it needn't brake. On any other lane, and on every lane
/// at an all-way stop, so that vehicles go in the order they came to rest, it
/// only counts vehicles already in the junction's queue, and goes behind them
/// (see `docs/DESIGN.md`, Gap acceptance and priority). Otherwise, the vehicle
/// that refuses it.
fn has_gap(
    state: &SimState,
    through: &[f64],
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: &Barrier,
) -> Result<(), VehicleId> {
    let network = &state.network;
    let s = barrier.s;
    let yields = network.yields(barrier);
    let all_way = barrier.kind == BarrierKind::AllWayStop;
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
    let impatience = impatience(state, vehicle);
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
                let desired = spec.desired_speed(lane.speed_limit());
                let to_clear = start + conflict.mine.end + spec.length;
                let clears = reach_time(vehicle.speed, spec.max_accel, desired, to_clear);
                let to_start = start + conflict.mine.start;
                let arrives = reach_time(vehicle.speed, spec.max_accel, desired, to_start);
                let headway = spec.time_headway;
                // Not given way to, only those in the junction's queue for it
                // count, which the junction holds it back for all the same.
                let queued_only = all_way || !yields.contains(&theirs);
                let way = Way {
                    junction,
                    theirs,
                    conflict,
                    clears,
                    arrives,
                    headway,
                    queued_only,
                    impatience,
                };
                let between = -network.lane(theirs).geometry().length();
                let (from, ahead) = (conflict.theirs.end, conflict.theirs.start);
                clear(state, through, id, &way, theirs, from, ahead, between)?;
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
        // One in the junction's queue for the conflict can't be gone ahead of:
        // going, this one joins the queue behind it, for its way from here, so
        // would stop for it in the conflict, whatever their levels. Committing
        // late, only once it can't stop safely, a vehicle that is still
        // approaching can stop, if this one couldn't clear as it meant to.
        let queued = || {
            let queue = &state.queues[way.junction.0 as usize].committed;
            queue.iter().any(|e| e.id == o.id && e.covers(&state.network, way.theirs))
        };
        // Queued only as far as where it waits, with the conflict where it will
        // stop, it won't clear it until it is queued on, so can't be gone behind.
        let stops_in_it = || {
            let queue = &state.queues[way.junction.0 as usize].committed;
            let Some(last) = queue.iter().rev().find(|e| e.id == o.id) else {
                return false;
            };
            let Some(until) = last.until(network, way.junction) else {
                return false;
            };
            if !network.lane(way.theirs).successors().contains(&until) {
                return false;
            }
            let length = network.lane(way.theirs).geometry().length();
            let rear_at_rest = length - other.line_gap - other.spec.length;
            rear_at_rest < way.conflict.theirs.end
        };
        let clears = if other.speed > 0.0 && !stops_in_it() {
            rear / other.speed
        } else {
            f64::INFINITY
        };
        let behind_margin = GAP_MARGIN + way.headway;
        // On a lane not given way to, only those that have committed count:
        // queued, or moving past every barrier before the conflict, as one
        // moving off an all-way stop line is; and those only to go behind. At
        // rest and not queued, it can stop, as one stuck behind a vehicle
        // released from its line, having committed behind it, is.
        let committed = || {
            queued()
                || other.speed > 0.0
                    && k.is_some_and(|k| !holds_before(network, other, k, at))
                    && !state.queues[way.junction.0 as usize].held.contains(&o.id)
        };
        if queued() || way.queued_only && committed() {
            if way.arrives < clears + behind_margin {
                return Err(o.id);
            }
            continue;
        }
        if way.queued_only {
            continue;
        }
        // Held short of the conflict by a light, or by the junction, it will
        // surely stop there.
        let held = k.is_some_and(|k| held_by_light(state, other, k, at))
            || state.queues[way.junction.0 as usize].held.contains(&o.id);
        if held {
            return Ok(());
        }
        let limit = network.lane(way.theirs).speed_limit();
        let desired = other.spec.desired_speed(limit);
        // As fast as it can, or, to an impatient driver, giving way to it.
        let i = way.impatience;
        let accel = (1.0 - i) * other.spec.max_accel - i * other.spec.comfy_decel;
        let reaches = reach_time(other.speed, accel, desired, distance);
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
        clear(state, through, id, way, p, length, ahead + length, between)?;
    }
    Ok(())
}

/// How far `vehicle` is short of `barrier`, given its front is `along` short of
/// its `s` along the lanes: with a line, the greater of that and how far its
/// nearest corner is short of the line. `s` is safe to stop at, the line
/// exact, so whichever lets it go further.
pub(crate) fn gap_to(vehicle: &Vehicle, barrier: &Barrier, along: f64) -> f64 {
    match barrier.line {
        Some(line) => {
            let nearest = vehicle
                .corners()
                .map(|c| line.short_of(c))
                .into_iter()
                .fold(f64::INFINITY, f64::min);
            along.max(nearest)
        }
        None => along,
    }
}

/// Whether `vehicle`, `id`, committed to `barrier` on `lane`, `gap` ahead of
/// its front, is released from it, to decide afresh: at a red light, it can
/// still stop before the line without braking harder than
/// [`acceleration::SAFE_BRAKING`], having been slowed short of it by traffic
/// ahead or a junction, as a driver who could stop does, unless it may turn on
/// red, when it is released as at a stop line; at a give-way or stop
/// line, or a keep-clear zone, it has come to rest short of the line, so the
/// gap it took, or the room it saw beyond the zone, is stale, unless its
/// junction has queued it past the line already, so that others are queued
/// behind it on that footing (see `docs/DESIGN.md`, Queue invariants). On amber,
/// a committed vehicle goes on: one caught in the dilemma zone, or a turner
/// waiting past its line for a gap, which comes as the oncoming traffic stops.
pub(crate) fn released(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    lane: LaneId,
    barrier: Barrier,
    gap: f64,
) -> bool {
    let speed = vehicle.speed;
    let queued_past = || {
        let Some(junction) = state.network.lane(lane).junction() else {
            return false;
        };
        let queue = &state.queues[junction.0 as usize].committed;
        queue
            .iter()
            .rev()
            .find(|e| e.id == id)
            .is_some_and(|last| last.covers(&state.network, lane))
    };
    gap > 0.0
        && match barrier.kind {
            // Turning on red, once it has come to rest, as at a stop line: a
            // vehicle that has committed to turn moves off slowly enough to
            // stop, so isn't released otherwise.
            BarrierKind::Signal { movement, on_red } => {
                state.lights[movement.0 as usize].light == Light::Red
                    && match on_red {
                        None => speed * speed / (2.0 * acceleration::SAFE_BRAKING) <= gap,
                        Some(_) => speed == 0.0 && !queued_past(),
                    }
            }
            BarrierKind::GiveWay
            | BarrierKind::Stop
            | BarrierKind::AllWayStop
            | BarrierKind::KeepClear { .. } => speed == 0.0 && !queued_past(),
        }
}

/// Whether `vehicle`, at rest at the all-way stop `barrier` on its `k`th lane,
/// came to rest there before the head at every other all-way stop line of its
/// junction whose lane conflicts with its own: earlier, or in the same step and
/// first in the lines' order, by lane and then along it, so runs repeat. The
/// head at a line is the nearest vehicle short of it that is heading for it.
fn first_to_arrive(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: Barrier,
) -> bool {
    let network = &state.network;
    let mine = vehicle.path[k];
    let Some(junction) = network.lane(mine).junction() else {
        return true;
    };
    let me = (vehicle.stopped_at.unwrap_or(u64::MAX), mine.0, barrier.s);
    for &theirs in network.junction(junction).lanes() {
        if theirs == mine || network.conflict_between(mine, theirs).is_none() {
            continue;
        }
        for line in network.lane(theirs).barriers() {
            if line.kind != BarrierKind::AllWayStop {
                continue;
            }
            let arrived = head_at(state, id, theirs, *line).and_then(|v| v.stopped_at);
            if arrived.is_some_and(|t| (t, theirs.0, line.s) < me) {
                return false;
            }
        }
    }
    true
}

/// The head on its way to `line` on `lane`: the nearest vehicle short of it,
/// on `lane` or a lane leading into it, whose next barrier it hasn't committed
/// to is that line.
fn head_at(state: &SimState, id: VehicleId, lane: LaneId, line: Barrier) -> Option<&Vehicle> {
    let network = &state.network;
    let heading = |o: &crate::occupancy::Occupant| {
        let v = &state.vehicles[o.id];
        let next = nth(network, v, v.committed as usize);
        (o.id != id && !o.is_ghost() && next.is_some_and(|(l, b)| l == lane && b.s == line.s))
            .then_some(v)
    };
    let on = state.occupants[lane.0 as usize].iter().rev().filter(|o| o.s() < line.s);
    let before = network
        .lane(lane)
        .predecessors()
        .iter()
        .flat_map(|&p| state.occupants[p.0 as usize].iter().rev());
    on.chain(before).find_map(heading)
}

/// Whether `vehicle`, with record `o` on a lane leading to a conflict, blocks
/// that lane, waiting: at rest in the lane, with a barrier it hasn't committed
/// to on the lane it follows or the next.
fn waits_in_lane(network: &SimNetwork, vehicle: &Vehicle, o: &crate::occupancy::Occupant) -> bool {
    let next = nth(network, vehicle, vehicle.committed as usize);
    o.lateral == crate::occupancy::Lateral::InLane
        && vehicle.speed == 0.0
        && next.is_some_and(|(lane, _)| vehicle.path.iter().take(2).any(|&l| l == lane))
}

/// Draws how far short of the next barrier it heads for `vehicle` will stop,
/// from its own generator, uniformly within its spec's spread of its mean.
pub(crate) fn draw_line_gap(vehicle: &mut Vehicle) {
    let draw = crate::models::routing::splitmix64(&mut vehicle.noise_rng);
    let unit = (draw >> 11) as f64 / (1u64 << 53) as f64;
    let spec = &vehicle.spec;
    vehicle.line_gap = (spec.line_gap + spec.line_gap_spread * (2.0 * unit - 1.0)).max(0.0);
}

/// The term holding a vehicle `gap` short of a line: a stationary leader there,
/// with its gap to lines, `line_gap`, as the minimum gap.
pub(crate) fn hold(spec: &VehicleSpec, line_gap: f64, speed: f64, gap: f64) -> f64 {
    let at_line = VehicleSpec { min_gap: line_gap, ..*spec };
    acceleration::interaction(&at_line, speed, gap, 0.0)
}

/// Whether `vehicle`, entering the junction its `k`th lane is in at `barrier`,
/// first gives way to a vehicle already in it part way through, as drivers give
/// way to vehicles in an intersection: one waiting to take a lane the barrier
/// gives way to, a turner's next stretch, which compilation lists for the
/// turners whose way on crosses this one's (see `docs/DESIGN.md`, Barriers from
/// control). That is one in the junction's queue only as far as that lane, or
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
    let committed = &queue.committed;
    if committed.iter().any(|e| e.id == id) {
        return None;
    }
    // In the queue only as far as such a lane, and no further since.
    let queued = committed.iter().enumerate().find_map(|(i, e)| {
        let until = e.until(network, junction)?;
        let last = !committed[i + 1..].iter().any(|later| later.id == e.id);
        (last && yields.contains(&until)).then_some(e.id)
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
                    && !committed.iter().any(|e| e.id == o.id)
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
                if beyond + comfortable_stop(other) < needed {
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
                let blocks =
                    other.speed == 0.0 || waits_inside(other, usize::from(other.committed));
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
        Some(o) => o.s() + comfortable_stop(&state.vehicles[o.id]) >= needed,
    }
}
