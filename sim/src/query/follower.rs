//! Searching back: the follower a vehicle would have at a point on a lane,
//! and where a vehicle's records are.

use crate::following::Occupant;
use crate::models::acceleration;
use crate::network::{LaneId, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// Where a vehicle's rear axle is along `lane`, a lane beside the one it follows,
/// projected onto it.
pub fn axle_beside(network: &SimNetwork, vehicle: &Vehicle, lane: LaneId) -> f64 {
    let (own, beside) = (network.lane(vehicle.path[0]), network.lane(lane));
    // Adjacent lanes start and end together, so the same fraction along it is a
    // good hint.
    let fraction = vehicle.lane_pos.s / own.geometry().length();
    let hint = beside.geometry().lane_pos(fraction * beside.geometry().length(), 0.0);
    beside.geometry().world_to_lane_near(vehicle.pos, hint).s
}

/// The index of a vehicle's record, not its ghost, in `lane`'s occupants: where a
/// search from it along its path starts (see [`Along::Path`](crate::following::Along::Path)).
pub fn record_index(state: &SimState, lane: LaneId, id: VehicleId) -> usize {
    state.occupants[lane.0 as usize]
        .iter()
        .position(|o| o.is_record_of(id))
        .expect("a vehicle has a record on the lane it follows")
}

/// The nearest vehicle behind `s` on `lane` that would end up behind a vehicle
/// there, and the gap from its front to `s`: searching back along `lane`'s
/// occupants, then the lanes leading into it.
///
/// Every vehicle on `lane` itself counts, whether settled, moving in or moving
/// out, since any of them can be in the way; on the lanes leading into it, one
/// counts if `lane` is on its path. Its leader search ends where its path does,
/// so one that doesn't know it will reach `lane` never sees what is there. That
/// bounds how far back the search goes: it goes back into a lane only if some
/// path through it reaches beyond its end as far as `lane` (see
/// [`crate::motion::path::Through`]), which also ends it on a loop.
/// Ghosts and `skip` are passed over.
///
/// The first vehicle found is the follower if `s` is within its [`acceleration::horizon`]:
/// further back, its own leader search wouldn't reach that far, and nothing
/// behind it would either, since it is in the way.
///
/// It splits the lane at `s` with [`find_leaders`](crate::following::find_leaders): a record whose rear is at or
/// ahead of `s` is a leader, and one whose rear is behind is a follower, so a
/// vehicle alongside is found by one or the other, with a gap of zero or less.
///
/// This is the backward search: intermittent decisions and insertion use it to
/// find the vehicles they would affect, while every per-frame search looks only
/// forward.
pub fn find_follower(
    state: &SimState,
    through: &[f64],
    lane: LaneId,
    s: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    // Its end is its length short of its start.
    let between = -state.network.lane(lane).geometry().length();
    behind(state, through, lane, lane, s, 0.0, between, skip)
}

/// [`find_follower`]'s search of `lane`, from `s` back, where `s` is `travelled`
/// back from the point searched from, on `target`, and the end of `lane` is
/// `between` back from the start of `target`: minus its length on `target`
/// itself. Going back round a loop, it only grows.
#[allow(clippy::too_many_arguments)]
fn behind(
    state: &SimState,
    through: &[f64],
    target: LaneId,
    lane: LaneId,
    s: f64,
    travelled: f64,
    between: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    let SimState { network, vehicles, occupants, .. } = state;
    let list = &occupants[lane.0 as usize];
    let end = list.partition_point(|o| o.s() < s);
    let counts = |o: &Occupant| {
        let through = || vehicles[o.id].path.contains(&target);
        !o.is_ghost() && o.id != skip && (lane == target || through())
    };
    if let Some(o) = list[..end].iter().rev().find(|o| counts(o)) {
        let follower = &vehicles[o.id];
        let gap = travelled + s - (o.s() + follower.spec.length);
        let reach = acceleration::horizon(&follower.spec, follower.speed);
        return (gap <= reach).then_some((o.id, gap));
    }
    // None on this lane: on to the ends of the lanes leading into it, those
    // some path through reaches `target` from. A path holding `target` reaches
    // beyond the end of a lane before it at least to `target`'s start, and
    // past it, so further than `between`.
    let travelled = travelled + s;
    // The lanes leading into it end where it starts.
    let between = between + network.lane(lane).geometry().length();
    network
        .lane(lane)
        .predecessors()
        .iter()
        .filter(|&&p| through[p.0 as usize] > between)
        .filter_map(|&p| {
            let length = network.lane(p).geometry().length();
            behind(state, through, target, p, length, travelled, between, skip)
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
}
