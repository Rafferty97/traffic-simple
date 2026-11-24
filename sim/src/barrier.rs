//! Barriers: stop lines, give-way lines and signals that vehicles stop at until
//! they commit to crossing (see `docs/DESIGN.md`, Barriers and signals).
//!
//! A vehicle's commitment is a count, of the barriers ahead of its rear axle on
//! its path it has committed to, in path order. Here are the rules for
//! counting them, for passing and release, and the term that holds a vehicle
//! at a line. When a head commits, what it gives way to and the room it needs
//! beyond, are the gap-acceptance model's (`models::gap_acceptance`), which
//! returns a [`Decision`].

use crate::models::acceleration;
use crate::models::signal::{Light, MovementLight};
use crate::network::{Barrier, BarrierKind, JunctionId, Lane, LaneId, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

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
/// unless it may turn on red. Under `models::gap_acceptance::decide`, nothing commits to one then, so
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
                BarrierKind::GiveWay | BarrierKind::Stop | BarrierKind::KeepClear { .. } => false,
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
/// path of its first lane there: `path[k]` itself, a barrier into a junction
/// being at the start of its first lane there.
pub(crate) fn into_junction(
    network: &SimNetwork,
    path: &[LaneId],
    k: usize,
) -> Option<(JunctionId, usize)> {
    network.lane(path[k]).junction().map(|junction| (junction, k))
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
            BarrierKind::GiveWay | BarrierKind::Stop | BarrierKind::KeepClear { .. } => {
                speed == 0.0 && !queued_past()
            }
        }
}
