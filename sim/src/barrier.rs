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
/// signal at or before it, that it hasn't committed to, on amber or red. Under
/// [`decide`], nothing commits to one then, so it will surely stop there. One at
/// a green light, or a give-way line, it hasn't committed to may yet go.
pub(crate) fn held_by_light(state: &SimState, vehicle: &Vehicle, k: usize, s: f64) -> bool {
    let network = &state.network;
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
                BarrierKind::Signal { movement } => {
                    state.lights[movement.0 as usize].light != Light::Green
                }
                BarrierKind::GiveWay | BarrierKind::AllWayStop => false,
            };
            if rank >= vehicle.committed as usize && holds {
                return true;
            }
            rank += 1;
        }
    }
    false
}

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
/// `gap` from its front to the line, does about it.
pub(crate) fn decide(
    state: &SimState,
    through: &[f64],
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: Barrier,
    gap: f64,
) -> Decision {
    let spec = &vehicle.spec;
    let speed = vehicle.speed;
    // Whether it can stop where it would like, a line gap short, at `decel`.
    let can_stop = |decel: f64| speed * speed / (2.0 * decel) <= gap - vehicle.line_gap;
    let room = || room_beyond(state, id, vehicle, k, barrier.s);
    let commit = match barrier.kind {
        // Only on green, once it can no longer stop comfortably: never on amber
        // or red, so one that hasn't committed by then brakes, and one that
        // couldn't have stopped comfortably committed while it was green.
        BarrierKind::Signal { movement } => match state.lights[movement.0 as usize].light {
            Light::Green if can_stop(spec.comfy_decel) => match room() {
                true => return Decision::Pass,
                false => false,
            },
            Light::Green => room(),
            Light::Amber | Light::Red => false,
        },
        BarrierKind::GiveWay => has_gap(state, through, id, vehicle, k, barrier.s) && room(),
        BarrierKind::AllWayStop => {
            vehicle.stopped_at.is_some()
                && first_to_arrive(state, id, vehicle, k, barrier)
                && has_gap(state, through, id, vehicle, k, barrier.s)
                && room()
        }
    };
    match commit {
        true => Decision::Commit,
        false => Decision::Hold(hold(spec, vehicle.line_gap, speed, gap)),
    }
}

/// How much later than a yielding vehicle clears a conflict a vehicle with
/// priority must be able to reach it at the soonest, in seconds.
const GAP_MARGIN: f64 = 0.5;

/// Whether `vehicle`, at a give-way barrier at `s` on its `k`th lane, has a
/// gap: on its lanes in that junction, up to the next barrier on its path, it
/// can clear each conflict with a lane of a higher priority level, or of the
/// same level and with a vehicle committed there, before any vehicle there
/// could reach it, plus [`GAP_MARGIN`], each accelerating as hard as it can up
/// to its desired speed; at a merge, ahead of it by that vehicle's time
/// headway too, so it needn't brake (see `docs/DESIGN.md`, Gap acceptance and
/// priority).
fn has_gap(
    state: &SimState,
    through: &[f64],
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    s: f64,
) -> bool {
    let network = &state.network;
    let (path, spec) = (&vehicle.path, &vehicle.spec);
    let Some(junction) = network.lane(path[k]).junction() else {
        return true;
    };
    // The next barrier on its path, as its lane's index on the path and `s` there.
    let next = (k..path.len()).find_map(|j| {
        let from = if j == k { s } else { f64::NEG_INFINITY };
        let barriers = network.lane(path[j]).barriers();
        barriers.iter().find(|b| b.s > from).map(|b| (j, b.s))
    });
    let front = vehicle.lane_pos.s - spec.rear_axle + spec.length;
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
                let now = |lane| level(network, &state.lights, lane);
                let (own, other) = (now(mine), now(theirs));
                let before_next =
                    next.is_none_or(|(nj, ns)| j < nj || (j == nj && conflict.mine.start < ns));
                let after_this = j > k || conflict.mine.end > s;
                if other < own || !before_next || !after_this {
                    continue;
                }
                // It clears a crossing once its rear is past it.
                let desired = spec.desired_speed(lane.speed_limit());
                let to_clear = start + conflict.mine.end + spec.length;
                let clears = reach_time(vehicle.speed, spec.max_accel, desired, to_clear);
                let to_start = start + conflict.mine.start;
                let arrives = reach_time(vehicle.speed, spec.max_accel, desired, to_start);
                let headway = spec.time_headway;
                let equal = other == own;
                let way = Way { junction, theirs, conflict, clears, arrives, headway, equal };
                let between = -network.lane(theirs).geometry().length();
                let (from, ahead) = (conflict.theirs.end, conflict.theirs.start);
                if !clear(state, through, id, &way, theirs, from, ahead, between) {
                    return false;
                }
            }
        }
        start += lane.geometry().length();
    }
    true
}

/// A lane's priority level now: above every level a network gives while its
/// signal shows red, so a vehicle still in the junction from that phase,
/// clearing it, outranks the traffic now green, and every vehicle behind its
/// red line is held there anyway; otherwise the level its junction gives it.
pub(crate) fn level(network: &SimNetwork, lights: &[MovementLight], lane: LaneId) -> u8 {
    let lane = network.lane(lane);
    let red = lane.barriers().iter().any(|b| match b.kind {
        BarrierKind::Signal { movement } => lights[movement.0 as usize].light == Light::Red,
        BarrierKind::GiveWay | BarrierKind::AllWayStop => false,
    });
    if red { u8::MAX } else { lane.priority() }
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
    /// Whether the lane has the same priority level, so only vehicles committed
    /// at the junction have priority there.
    equal: bool,
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
/// cleared it, the next one decides; otherwise there is none. Either way the
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
) -> bool {
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
        // nobody behind it can come until it goes.
        if !on_it && k.is_none() {
            if waits_in_lane(network, other, o) {
                return true;
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
        // Committed: past every barrier before the conflict, or so committed
        // that the junction holds others back for it.
        // Of the same level, it counts only if it committed first: before this
        // one in the junction's queue, or, with this one not in it yet, past
        // every barrier before the conflict. So of two that wait for each
        // other, the first to commit goes, and they can't deadlock.
        let committed = || {
            let queue = &state.queues[way.junction.0 as usize].committed;
            let mine = queue.iter().position(|&v| v == id);
            match (queue.iter().position(|&v| v == o.id), mine) {
                (Some(theirs), Some(mine)) => theirs < mine,
                (Some(_), None) => true,
                (None, mine) => {
                    mine.is_none() && k.is_some_and(|k| !holds_before(network, other, k, at))
                }
            }
        };
        let held = k.is_some_and(|k| held_by_light(state, other, k, at));
        if held || (way.equal && !committed()) {
            return true;
        }
        let limit = network.lane(way.theirs).speed_limit();
        let desired = other.spec.desired_speed(limit);
        let reaches = reach_time(other.speed, other.spec.max_accel, desired, distance);
        let clears = if other.speed > 0.0 {
            rear / other.speed
        } else {
            f64::INFINITY
        };
        // Whichever follows the other through the conflict keeps its time
        // headway behind it there, as the junction makes it, so needn't brake.
        let (ahead_margin, behind_margin) = (
            GAP_MARGIN + other.spec.time_headway,
            GAP_MARGIN + way.headway,
        );
        // One in the junction's queue before it can't be gone ahead of: going,
        // it joins the queue behind it, so would stop for it in the conflict.
        // Committing late, only once it can't stop safely, a vehicle that is
        // still approaching can stop, if this one couldn't clear as it meant to.
        let queued = || {
            let queue = &state.queues[way.junction.0 as usize].committed;
            let theirs = queue.iter().position(|&v| v == o.id);
            let mine = queue.iter().position(|&v| v == id);
            theirs.is_some_and(|theirs| mine.is_none_or(|mine| theirs < mine))
        };
        if distance >= 0.0 && reaches > way.clears + ahead_margin && !queued() {
            return true;
        }
        if way.arrives < clears + behind_margin {
            return false;
        }
    }
    // Nobody on this lane: on to those leading into it that a path through
    // reaches the priority lane from.
    let between = between + network.lane(lane).geometry().length();
    let reaches = |p: &&LaneId| through[p.0 as usize] > between;
    network.lane(lane).predecessors().iter().filter(reaches).all(|&p| {
        let length = network.lane(p).geometry().length();
        clear(state, through, id, way, p, length, ahead + length, between)
    })
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

/// Whether `vehicle`, committed to `barrier`, `gap` ahead of its front, is
/// released from it, to decide afresh: at a red light, it can still stop before
/// the line without braking harder than [`acceleration::SAFE_BRAKING`], having
/// been slowed short of it by traffic ahead or a junction, as a driver who could
/// stop does; at a give-way line, it has come to rest short of the line, so the
/// gap it took is stale. On amber, a committed vehicle goes on: one caught in
/// the dilemma zone, or a turner waiting past its line for a gap, which comes
/// as the oncoming traffic stops.
pub(crate) fn released(state: &SimState, vehicle: &Vehicle, barrier: Barrier, gap: f64) -> bool {
    let speed = vehicle.speed;
    gap > 0.0
        && match barrier.kind {
            BarrierKind::Signal { movement } => {
                state.lights[movement.0 as usize].light == Light::Red
                    && speed * speed / (2.0 * acceleration::SAFE_BRAKING) <= gap
            }
            BarrierKind::GiveWay | BarrierKind::AllWayStop => speed == 0.0,
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
    if network.lane(path[k]).junction().is_none() {
        return true;
    }
    // The first lane of its path out of the junction.
    let Some(exit) = (k..path.len()).find(|&j| network.lane(path[j]).junction().is_none()) else {
        return true;
    };
    let exit_lane = path[exit];
    let junction = network.lane(path[k]).junction();
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
