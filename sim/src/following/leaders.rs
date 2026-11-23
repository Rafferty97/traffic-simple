//! The leader search: what a vehicle must react to ahead of it, along its own
//! lanes, or along its old lanes while it straddles them ("Old-lane search"),
//! passing what the pass check lets it.

use std::ops::ControlFlow;

use arrayvec::ArrayVec;

use super::{Lateral, pass};
use crate::debug::PassCheck;
use crate::models::acceleration;
use crate::network::{LaneId, Side, SimNetwork};
use crate::query::axle_beside;
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// Which lanes a leader search runs along, relative to the searching vehicle's
/// path, how the vehicle relates to them, and where on the first it starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Along {
    /// Its own path, following its lane change, if any: from its record on the
    /// first lane, by index in that lane's occupants.
    Path(usize),
    /// The lanes beside its path on this side, which it is moving out of: the
    /// old-side search, while it straddles. From its record on the first, by
    /// index.
    Leaving(Side, usize),
    /// The lanes beside its path on this side, as if it were already settled on
    /// their centerlines: weighing up a lane change. From its rear, projected onto
    /// the first.
    Into(Side),
    /// Its own path, as Path, for a vehicle not yet inserted, so with no record:
    /// from where its rear is.
    Inserting,
}

impl Along {
    /// The `k`th lane searched: its own path's `k`th lane, or the lane beside it.
    /// `None` past the end of its path, or, beside it, where there is no lane
    /// there.
    pub fn lane(self, network: &SimNetwork, vehicle: &Vehicle, k: usize) -> Option<LaneId> {
        let own = *vehicle.path.get(k)?;
        match self {
            Along::Path(_) | Along::Inserting => Some(own),
            Along::Leaving(side, _) | Along::Into(side) => {
                network.lane(own).adj(side).map(|n| n.lane)
            }
        }
    }
}

/// Something ahead that a leader search found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ahead {
    /// A vehicle it can't pass.
    Vehicle(VehicleId),
    /// The ghost of a vehicle indicating a change into the lane. What it means is
    /// the caller's choice: weighing up a change, it is a leader, since a vehicle
    /// already indicating into the gap ahead goes first; otherwise the vehicle
    /// may make room for the nearest one, if no leader comes first.
    Ghost(VehicleId),
    /// The end of the lanes it knows it will take, a stop line: a stationary
    /// virtual leader, unless it leaves the network there.
    PathEnd,
    /// A barrier on its own path, the `index`th of `lane`'s, and the `rank`th
    /// ahead of its rear axle on its path. Searching its own path, every one it
    /// has committed to is found, and every one beyond them that no leader
    /// shields, so it is the head on its way to it; what it does there is the
    /// caller's choice (see [`crate::models::gap_acceptance::decide`]).
    Barrier { lane: LaneId, index: u16, rank: u16 },
}

/// What a leader search finds, in the order it meets them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Found {
    /// Something ahead to react to, with the bumper-to-bumper gap to it, and the
    /// share of its time headway the searching vehicle keeps to it, as
    /// relaxation has it (see [`acceleration::relaxation`]): each is its own
    /// term in the longitudinal model, whose minimum binds.
    Ahead(Ahead, f64, f64),
    /// A check for passing a vehicle ahead, passed or not. One that fails is
    /// followed by that vehicle, as [`Found::Ahead`].
    Pass(PassCheck),
}

/// Searches for what is ahead of a vehicle, `along` some lanes, within its
/// [`acceleration::horizon`] of its front, that it must react to, calling `f`
/// with each thing in order of distance, nearest first. `f` returns whether to go
/// on: a caller can stop as soon as it has what it needs, and the search then
/// returns what `f` broke with. See [`for_each_leader`] for a caller that wants it all.
///
/// It measures from the vehicle's rear on the first lane searched: its record's
/// `s`, if it has one there, or else where its rear is, projected onto that lane
/// if it is beside its path. Its front is its length ahead of that. On that
/// lane, only records from just after its own count, or, with none, those at or
/// ahead of its rear: [`find_follower`](crate::query::find_follower) takes those behind, so between them they
/// find every vehicle, one alongside included. On later lanes, every occupant
/// counts.
///
/// The search doesn't know which vehicle is searching. On a loop shorter than
/// its horizon, a vehicle can find its own record ahead, which is geometrically
/// true; and searching a lane it is indicating into, it finds its own ghost,
/// which the caller passes over.
///
/// Leaving the lanes searched (see [`Along::Leaving`]), the horizon is where its
/// lane change ends, if that is nearer: after that the vehicle is wholly on the
/// lanes it follows, so nothing it would only reach later can be in its way
/// there. Beside its path, the end of the lanes searched isn't a stop line: they
/// end where the lanes beside its path do.
///
/// A vehicle diverging from a lane searched passes any record there that its
/// body will be wholly out of the lane by the time it reaches: it never comes
/// back. Some records may also be passed, by the rules in `pass::passing_side`. One
/// of those is passed if, with this vehicle where it plans to be (see [`Along::pose`]) as its
/// front reaches that record's rear, driving straight ahead would leave at least
/// [`PASSING_CLEARANCE`](super::PASSING_CLEARANCE) beside the other vehicle's rear corners. That reads the
/// other vehicle's corners from the state's vehicles, as does working out what a
/// leader shields. Each such check is passed to `f`, as [`Found::Pass`].
///
/// Which records it needs: a vehicle in the lane that this one can't pass, B,
/// blocks the lane's whole width, so this one can't reach anything beyond it
/// that B can't, and whatever in the lane beyond constrains B reaches this one
/// through B. So in-lane vehicles beyond B are skipped, on B's lane and on the
/// lanes after it, as far as B's path goes along the lanes searched (see
/// [`shields`]): the first lane B doesn't cover is a high-water mark,
/// the furthest any leader has reached. Ghosts only partly block a lane, and B
/// may pass a vehicle diverging from it that this one can't, so neither is ever
/// skipped, and neither shields anything. A vehicle converging onto the lane
/// after a lane change is in it, so it shields like any other.
///
/// Past the end of its path, it doesn't know where it is going, so it must stop
/// there, unless it leaves the network: the last thing found, since nothing
/// lies beyond.
pub fn find_leaders<B>(
    state: &SimState,
    vehicle: &Vehicle,
    along: Along,
    reach: f64,
    mut f: impl FnMut(Found) -> ControlFlow<B>,
) -> ControlFlow<B> {
    let network = &state.network;
    let lane_at = |k: usize| along.lane(network, vehicle, k);
    let Some(first) = lane_at(0) else {
        return ControlFlow::Continue(());
    };
    let list = |lane: LaneId| state.occupants[lane.0 as usize].as_slice();
    let after = |rear: f64| list(first).partition_point(|o| o.s() < rear);
    let rear_axle = vehicle.spec.rear_axle;
    // Where it starts on the first lane, its rear there, and its rear axle along
    // the lanes it follows (see `Along::pose`).
    let (start, rear, axle_s) = match along {
        Along::Path(own) | Along::Leaving(_, own) => {
            (own + 1, list(first)[own].s(), vehicle.lane_pos.s)
        }
        Along::Inserting => {
            let rear = vehicle.lane_pos.s - rear_axle;
            (after(rear), rear, vehicle.lane_pos.s)
        }
        Along::Into(_) => {
            let axle_s = axle_beside(network, vehicle, first);
            (after(axle_s - rear_axle), axle_s - rear_axle, axle_s)
        }
    };
    let front = rear + vehicle.spec.length;
    // Its own record on the first lane, if it searches from one.
    let own = match along {
        Along::Path(own) | Along::Leaving(_, own) => Some(list(first)[own]),
        _ => None,
    };
    // How long it has been in the lanes searched, which its leader terms relax
    // with. Weighing up a change or inserting, it is checked unrelaxed: a
    // driver doesn't change lanes counting on the gap it will settle for after.
    let own_age = own.map(|own| own.age);
    let horizon = acceleration::horizon(&vehicle.spec, vehicle.speed);
    let horizon = horizon.max(reach);
    let horizon = match (along, vehicle.lane_change) {
        // A gap is how far the rear axle travels before the nose reaches a rear.
        // Leaving its old lane, it can only reach a vehicle there while it is
        // still in it, which it is no longer once its lane change ends.
        (Along::Leaving(..), change) => {
            change.map_or(0.0, |c| (c.end() - axle_s).max(0.0)).min(horizon)
        }
        _ => horizon,
    };
    // How it relates to the `k`th lane searched: on its own path, in the lane or
    // converging while it changes lanes; settled beside it, in the lane; leaving
    // its old lanes, as its records there say, and converging where it has none.
    let own_lateral = |k: usize, lane: LaneId| match (along, own) {
        (Along::Leaving(..), Some(own)) if k == 0 => own.lateral,
        (Along::Leaving(..), Some(own)) => list(lane)
            .iter()
            .find(|o| o.is_record_of(own.id))
            .map_or(Lateral::CONVERGING, |o| o.lateral),
        (Along::Into(_), _) => Lateral::InLane,
        _ => match vehicle.lane_change {
            Some(_) => Lateral::CONVERGING,
            None => Lateral::InLane,
        },
    };
    // Settled beside its path, it is nowhere yet, so where it is now is where it
    // would be.
    let now = match along {
        Along::Into(_) => along.pose(network, vehicle, axle_s, 0.0),
        _ => (vehicle.pos, vehicle.dir),
    };

    // Start of the `k`th lane, along the lanes searched from the start of the first.
    let mut offset = 0.0;
    // The first lane no leader in the lane shields: on the lanes before it, only
    // ghosts and vehicles diverging from them still count.
    let mut covered = 0;
    // Barriers count only along its own path: how many it has passed ahead of
    // its rear axle, and how many of those it has committed to.
    let barriers_count = matches!(along, Along::Path(_) | Along::Inserting);
    let committed = match along {
        Along::Path(_) => vehicle.committed as usize,
        _ => 0,
    };
    let mut rank = 0;
    // The leaders in the lane that shield, and the first lane each doesn't: a
    // barrier one shields is hidden unless it has committed to it. Should there
    // be more than fit, every barrier beyond them is hidden.
    let mut shielders = ArrayVec::<(VehicleId, usize), 8>::new();
    let mut overflowed = false;
    let mut k = 0;
    while let Some(lane) = lane_at(k) {
        if offset - front > horizon {
            return ControlFlow::Continue(());
        }
        let records = &list(lane)[if k == 0 { start } else { 0 }..];
        let barriers = match barriers_count {
            true => network.lane(lane).barriers(),
            false => &[],
        };
        // On the first lane, those behind its rear axle are behind it.
        let mut b = if k == 0 {
            barriers.partition_point(|x| x.s <= axle_s)
        } else {
            0
        };
        let next_s = |b: usize| barriers.get(b).map_or(f64::INFINITY, |x| x.s);
        // Where the next is: past every record where there is none.
        let mut next = next_s(b);
        // A barrier, in order along the lane: a barrier first where it is level
        // with a record.
        macro_rules! barrier {
            () => {{
                let (index, barrier) = (b, barriers[b]);
                b += 1;
                next = next_s(b);
                let gap = offset + barrier.s - front;
                if gap > horizon {
                    return ControlFlow::Continue(());
                }
                let this = rank;
                rank += 1;
                // Its own commitment hides nothing from itself: it may yet be
                // released.
                let shielded = this >= committed
                    && (overflowed
                        || shielders.iter().any(|&(leader, cover)| {
                            let leader = &state.vehicles[leader];
                            cover > k
                                && !crate::barrier::committed_to(network, leader, lane, barrier.s)
                        }));
                if !shielded {
                    let ahead = Ahead::Barrier { lane, index: index as u16, rank: this as u16 };
                    f(Found::Ahead(ahead, gap, 1.0))?;
                }
            }};
        }
        for o in records {
            while next <= o.s() {
                barrier!();
            }
            let gap = offset + o.s() - front;
            if gap > horizon {
                return ControlFlow::Continue(());
            }
            if o.is_ghost() {
                f(Found::Ahead(Ahead::Ghost(o.id), gap, 1.0))?;
                continue;
            }
            // Converging on the lane, it can't be passed in it, whatever it
            // covers beyond, so what lies beyond it there can't be reached first.
            let in_lane = matches!(o.lateral, Lateral::InLane | Lateral::Converging { .. });
            if in_lane && k < covered {
                continue;
            }
            let mine = own_lateral(k, lane);
            let check = pass::check(state, vehicle, along, axle_s, now, k, lane, gap, o, mine);
            if let Some(check) = check {
                f(Found::Pass(check))?;
                if check.passed {
                    continue;
                }
            }
            let headway = match own_age {
                Some(own) => acceleration::relaxation(own.min(o.age)),
                None => 1.0,
            };
            f(Found::Ahead(Ahead::Vehicle(o.id), gap, headway))?;
            if in_lane {
                let cover = shields(lane_at, k, &state.vehicles[o.id].path);
                if cover > covered && barriers_count && shielders.try_push((o.id, cover)).is_err() {
                    overflowed = true;
                }
                covered = covered.max(cover);
            }
        }
        while b < barriers.len() {
            barrier!();
        }
        offset += network.lane(lane).geometry().length();
        k += 1;
    }
    // Past the end of its path, with nowhere it knows to go.
    let own_path = matches!(along, Along::Path(_) | Along::Inserting);
    let ends = |&l: &LaneId| !network.lane(l).successors().is_empty();
    let gap = offset - front;
    if own_path && vehicle.path.last().is_some_and(ends) && gap <= horizon {
        f(Found::Ahead(Ahead::PathEnd, gap, 1.0))?;
    }
    ControlFlow::Continue(())
}

/// As [`find_leaders`], for a caller that wants everything it finds.
pub fn for_each_leader(
    state: &SimState,
    vehicle: &Vehicle,
    along: Along,
    reach: f64,
    mut f: impl FnMut(Found),
) {
    let ControlFlow::Continue(()) = find_leaders(state, vehicle, along, reach, |found| {
        f(found);
        ControlFlow::<!>::Continue(())
    });
}

/// How far along the lanes searched, `lane_at` each by index, a vehicle in the
/// lane on the `k`th of them shields, with `path` its own path: the index of the
/// first lane it doesn't. It shields the rest of lane `k`, and the lanes after it
/// as far as its own path goes along them: where its path parts from them, or
/// ends, what lies beyond never constrains it.
///
/// Its path starts at the lane it follows: lane `k`, or, while it lingers past
/// the end of lane `k` after advancing, usually the next. Anything else, it
/// shields lane `k` alone.
fn shields(lane_at: impl Fn(usize) -> Option<LaneId>, k: usize, path: &[LaneId]) -> usize {
    let j = match path.first() {
        Some(&first) if lane_at(k) == Some(first) => k,
        Some(&first) if lane_at(k + 1) == Some(first) => k + 1,
        _ => return k + 1,
    };
    let shared = path.iter().enumerate().take_while(|&(n, &l)| lane_at(j + n) == Some(l));
    (j + shared.count()).max(k + 1)
}
