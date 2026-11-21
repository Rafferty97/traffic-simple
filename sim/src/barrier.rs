//! Barriers: stop lines, give-way lines and signals that vehicles stop at until
//! they commit to crossing (see `docs/DESIGN.md`, Barriers and signals).
//!
//! A vehicle's commitment is a count, of the barriers ahead of its rear axle on
//! its path it has committed to, in path order. Here are the rules for
//! counting them, for deciding to commit, and for the room a junction entry
//! needs beyond.

use crate::lane_changing::comfortable_stop;
use crate::models::acceleration::{self, MAX_BRAKING};
use crate::models::signal::Light;
use crate::network::{Barrier, BarrierKind, Lane, LaneId, SimNetwork};
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

/// Whether a barrier ahead holds `vehicle` back from the point `s` on its `k`th
/// lane: one at or before it that it hasn't committed to.
pub(crate) fn holds_before(network: &SimNetwork, vehicle: &Vehicle, k: usize, s: f64) -> bool {
    rank(network, vehicle, k, s, true) > vehicle.committed as usize
}

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
    id: VehicleId,
    vehicle: &Vehicle,
    k: usize,
    barrier: Barrier,
    gap: f64,
) -> Decision {
    let spec = &vehicle.spec;
    let short = gap - spec.line_gap;
    // Past the line, or unable to stop even as hard as it can: it crosses
    // whatever the barrier says.
    let speed = vehicle.speed;
    if gap <= 0.0 || speed * speed / (2.0 * MAX_BRAKING) > short {
        return Decision::Commit;
    }
    let can_stop = |decel: f64| speed * speed / (2.0 * decel) <= short;
    let room = || room_beyond(state, id, vehicle, k, barrier.s);
    let commit = match barrier.kind {
        // On green, it commits once it can no longer stop comfortably; on amber,
        // once it can't stop safely either, so one braking for amber keeps on
        // stopping, though IDM's braking toward a line starts gently.
        BarrierKind::Signal { movement } => match state.lights[movement.0 as usize].light {
            Light::Red => false,
            Light::Green if can_stop(spec.comfy_decel) => match room() {
                true => return Decision::Pass,
                false => false,
            },
            Light::Amber if can_stop(acceleration::SAFE_BRAKING) => false,
            Light::Green | Light::Amber => room(),
        },
        // Gap acceptance comes with priority levels; until then, a give-way
        // line holds everyone.
        BarrierKind::GiveWay => false,
    };
    match commit {
        true => Decision::Commit,
        false => Decision::Hold(hold(spec, speed, gap)),
    }
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
