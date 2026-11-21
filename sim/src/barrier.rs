//! Barriers: stop lines, give-way lines and signals that vehicles stop at until
//! they commit to crossing (see `docs/DESIGN.md`, Barriers and signals).
//!
//! A vehicle's commitment is a count, of the barriers ahead of its rear axle on
//! its path it has committed to, in path order. Here are the rules for
//! counting them, for deciding to commit, and for the room a junction entry
//! needs beyond.

use crate::lane_changing::comfortable_stop;
use crate::models::acceleration::{self, reach_time};
use crate::models::signal::Light;
use crate::network::{
    Barrier, BarrierKind, Conflict, ConflictKind, JunctionId, Lane, LaneId, SimNetwork,
};
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

/// The lane of the barrier `rank`th ahead of `vehicle`'s rear axle on its path,
/// if there is one.
pub(crate) fn nth(network: &SimNetwork, vehicle: &Vehicle, rank: usize) -> Option<LaneId> {
    let mut count = 0;
    for (j, &lane) in vehicle.path.iter().enumerate() {
        let from = if j == 0 {
            vehicle.lane_pos.s
        } else {
            f64::NEG_INFINITY
        };
        count += between(network.lane(lane), from, f64::INFINITY);
        if count > rank {
            return Some(lane);
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
                BarrierKind::GiveWay => false,
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
    let can_stop = |decel: f64| speed * speed / (2.0 * decel) <= gap - spec.line_gap;
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
    };
    match commit {
        true => Decision::Commit,
        false => Decision::Hold(hold(spec, speed, gap)),
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
                let (own, other) = (lane.priority(), network.lane(theirs).priority());
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
                let way = Way { junction, theirs, conflict, clears, equal: other == own };
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

/// A conflict a yielding vehicle checks a gap at, on the priority lane `theirs`.
struct Way<'a> {
    junction: JunctionId,
    theirs: LaneId,
    conflict: &'a Conflict,
    /// When the yielding vehicle could clear it at the soonest, in seconds.
    clears: f64,
    /// Whether the lane has the same priority level, so only vehicles committed
    /// at the junction have priority there.
    equal: bool,
}

/// Whether no vehicle with priority on `way` could reach its conflict too soon,
/// searching back from `s` on `lane` and then into the lanes leading into it,
/// as the follower search does, where `lane`'s start is `ahead` back from the
/// conflict's start on the priority lane, and its end `between` back from that
/// lane's start: minus that lane's length on the priority lane itself. On each branch only the nearest vehicle headed for the conflict
/// matters, since those behind it there arrive later. A vehicle a red or amber
/// light holds short of the conflict holds back the rest of its branch (see
/// [`held_by_light`]); one at a green light or a give-way line may yet go, and
/// counts. One in the conflict already refuses the gap.
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
        // Its front, back from the conflict's start: past it, it is in it.
        let distance = ahead - (o.s() + other.spec.length);
        if distance < 0.0 {
            return false;
        }
        let committed = || state.queues[way.junction.0 as usize].committed.contains(&o.id);
        let held = k.is_some_and(|k| held_by_light(state, other, k, at));
        if held || (way.equal && !committed()) {
            return true;
        }
        let limit = network.lane(way.theirs).speed_limit();
        let desired = other.spec.desired_speed(limit);
        let reaches = reach_time(other.speed, other.spec.max_accel, desired, distance);
        let margin = match way.conflict.kind {
            ConflictKind::Cross => GAP_MARGIN,
            ConflictKind::Merge => GAP_MARGIN + other.spec.time_headway,
        };
        return reaches > way.clears + margin;
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

/// Whether `vehicle`, committed to `barrier`, `gap` ahead of its front, is
/// released from it, to decide afresh: at a signal that is no longer green, it
/// can still stop comfortably before the line, having been slowed short of it
/// by traffic ahead or a junction, as a driver who could stop does when the
/// light changes; at a give-way line, it has come to rest short of the line, so
/// the gap it took is stale. A vehicle that committed to a green light as it
/// turned amber, unable to stop comfortably, stays committed.
pub(crate) fn released(state: &SimState, vehicle: &Vehicle, barrier: Barrier, gap: f64) -> bool {
    let speed = vehicle.speed;
    gap > 0.0
        && match barrier.kind {
            BarrierKind::Signal { movement } => {
                state.lights[movement.0 as usize].light != Light::Green
                    && speed * speed / (2.0 * vehicle.spec.comfy_decel) <= gap
            }
            BarrierKind::GiveWay => speed == 0.0,
        }
}

/// Whether `vehicle`, with record `o` on a lane leading to a conflict, blocks
/// that lane, waiting: at rest in the lane, with a barrier it hasn't committed
/// to on the lane it follows or the next.
fn waits_in_lane(network: &SimNetwork, vehicle: &Vehicle, o: &crate::occupancy::Occupant) -> bool {
    let next = nth(network, vehicle, vehicle.committed as usize);
    o.lateral == crate::occupancy::Lateral::InLane
        && vehicle.speed == 0.0
        && next.is_some_and(|lane| vehicle.path.iter().take(2).any(|&l| l == lane))
}

/// The term holding a vehicle `gap` short of a line: a stationary leader there,
/// with its line gap as the minimum gap.
pub(crate) fn hold(spec: &VehicleSpec, speed: f64, gap: f64) -> f64 {
    let at_line = VehicleSpec { min_gap: spec.line_gap, ..*spec };
    acceleration::interaction(&at_line, speed, gap, 0.0)
}

/// Whether, entering the junction its `k`th lane is in at `s` along it, `vehicle`
/// would find room on the lane after it, as the road rules require: room for
/// itself and every vehicle ahead of it on its way through the junction to the
/// same lane, each its length and minimum gap, behind the rearmost vehicle
/// there, allowing for how far that one would go on if it stopped comfortably.
/// Always, for a barrier that isn't into a junction.
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
    let mut needed = vehicle.spec.length + vehicle.spec.min_gap;
    for (j, &lane) in path.iter().enumerate().take(exit).skip(k) {
        let from = if j == k { s } else { f64::NEG_INFINITY };
        for o in &state.occupants[lane.0 as usize] {
            let other = &state.vehicles[o.id];
            if o.is_ghost() || o.id == id || o.s() < from || !other.path.contains(&exit_lane) {
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
