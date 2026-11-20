//! Merges: where lanes outside junctions that were apart start to overlap, the
//! vehicles approaching are kept in order by estimated arrival, and each takes
//! a term that opens a gap behind the one before it by the merge point (see
//! Merges in `docs/DESIGN.md`).

use slotmap::SlotMap;

use crate::following::Overlaps;
use crate::models::acceleration;
use crate::network::{LaneId, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// How much earlier, in seconds, a vehicle's estimated arrival must be than
/// that of the one before it, on another lane, for it to go ahead.
const HYSTERESIS: f64 = 0.5;

/// The least speed, in m/s, an estimated arrival is worked out at, so a stopped
/// vehicle's doesn't blow up.
const MIN_ETA_SPEED: f64 = 1.0;

/// Which merge each lane belongs to, if any: derived from the network.
#[derive(Clone, Debug, Default)]
pub struct Merges {
    of_lane: Vec<Option<u32>>,
    count: usize,
}

impl Merges {
    /// How many merges there are.
    pub fn len(&self) -> usize {
        self.count
    }

    /// The merge `lane` belongs to, if any.
    pub fn of(&self, lane: LaneId) -> Option<usize> {
        self.of_lane[lane.0 as usize].map(|m| m as usize)
    }
}

/// Finds the merges: groups of lanes outside junctions that overlap one
/// another, joined pair by pair, where the two lanes of each pair have no
/// predecessor in common and no two of their predecessors overlap.
pub fn merges(network: &SimNetwork, overlaps: &Overlaps) -> Merges {
    let count = network.lanes().count();
    // Union-find over lanes.
    let mut parent: Vec<usize> = (0..count).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut joined = vec![false; count];
    let overlap = |x: LaneId, y: LaneId| overlaps[x.0 as usize].iter().any(|p| p.lane == y);
    for (a, list) in overlaps.iter().enumerate() {
        let la = network.lane(LaneId(a as u32));
        for p in list {
            let b = p.lane.0 as usize;
            if b <= a {
                continue;
            }
            let lb = network.lane(p.lane);
            if la.junction().is_some() || lb.junction().is_some() {
                continue;
            }
            let apart = la
                .predecessors()
                .iter()
                .all(|&x| lb.predecessors().iter().all(|&y| x != y && !overlap(x, y)));
            if !apart {
                continue;
            }
            let (ra, rb) = (root(&mut parent, a), root(&mut parent, b));
            parent[ra] = rb;
            joined[a] = true;
            joined[b] = true;
        }
    }
    let mut ids = vec![None; count];
    let mut of_lane = vec![None; count];
    let mut next = 0;
    for i in 0..count {
        if !joined[i] {
            continue;
        }
        let r = root(&mut parent, i);
        let id = *ids[r].get_or_insert_with(|| {
            next += 1;
            next - 1
        });
        of_lane[i] = Some(id);
    }
    Merges { of_lane, count: next as usize }
}

/// Where `vehicle` is headed into `merge`: the first of its lanes on its path,
/// and how far its rear axle is from that lane's start, the merge point. `None`
/// if none of them is on its path, or it is on one already.
fn ahead(
    network: &SimNetwork,
    merges: &Merges,
    vehicle: &Vehicle,
    merge: usize,
) -> Option<(LaneId, f64)> {
    let mut start = -vehicle.lane_pos.s;
    for (k, &lane) in vehicle.path.iter().enumerate() {
        if merges.of(lane) == Some(merge) {
            return (k > 0).then_some((lane, start));
        }
        start += network.lane(lane).geometry().length();
    }
    None
}

/// A vehicle in a merge's order, as the order is sorted and walked.
#[derive(Clone, Copy, Debug)]
pub struct Member {
    id: VehicleId,
    /// The merge's lane it is headed for.
    lane: LaneId,
    /// From its front to the merge point, in meters.
    front: f64,
    /// Its estimated arrival there, in seconds.
    eta: f64,
}

fn member(
    network: &SimNetwork,
    merges: &Merges,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    id: VehicleId,
    merge: usize,
) -> Option<Member> {
    let vehicle = vehicles.get(id)?;
    let (lane, axle) = ahead(network, merges, vehicle, merge)?;
    let front = axle - (vehicle.spec.length - vehicle.spec.rear_axle);
    let eta = front.max(0.0) / vehicle.speed.max(MIN_ETA_SPEED);
    Some(Member { id, lane, front, eta })
}

/// Puts `id` in the order of every merge one of whose lanes is on its path
/// beyond the lane it follows, if it isn't there yet, at its place by
/// estimated arrival.
pub fn arrive(
    network: &SimNetwork,
    merges: &Merges,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    orders: &mut [Vec<VehicleId>],
    id: VehicleId,
) {
    if merges.len() == 0 {
        return;
    }
    let vehicle = &vehicles[id];
    for &lane in &vehicle.path[1..] {
        let Some(merge) = merges.of(lane) else {
            continue;
        };
        let order = &orders[merge];
        if order.contains(&id) {
            continue;
        }
        let Some(new) = member(network, merges, vehicles, id, merge) else {
            continue;
        };
        let at = order
            .iter()
            .position(|&other| {
                member(network, merges, vehicles, other, merge).is_some_and(|m| m.eta > new.eta)
            })
            .unwrap_or(order.len());
        orders[merge].insert(at, id);
    }
}

/// Whether `behind` should go ahead of `before`, the one before it in a
/// merge's order.
fn goes_ahead(before: &Member, behind: &Member) -> bool {
    if before.lane == behind.lane {
        // Single file along the same lane: in the order they are in along it.
        behind.front < before.front
    } else if (behind.front <= 0.0) != (before.front <= 0.0) {
        // One already at the merge point goes first.
        behind.front <= 0.0
    } else {
        behind.eta < before.eta - HYSTERESIS
    }
}

impl Member {
    pub fn id(&self) -> VehicleId {
        self.id
    }
}

/// Works out where each vehicle in a merge's order is, into `members`, in
/// order, leaving out those no longer headed into it.
pub fn members(
    network: &SimNetwork,
    merges: &Merges,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    order: &[VehicleId],
    merge: usize,
    members: &mut Vec<Member>,
) {
    members.clear();
    members.extend(order.iter().filter_map(|&id| member(network, merges, vehicles, id, merge)));
}

/// Brings a merge's order up to date: drops those no longer headed into it,
/// works out the rest's positions into `members`, and sorts them by adjacent
/// swaps (see [`goes_ahead`]).
pub fn sort(
    network: &SimNetwork,
    merges: &Merges,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    order: &mut Vec<VehicleId>,
    merge: usize,
    members: &mut Vec<Member>,
) {
    self::members(network, merges, vehicles, order, merge, members);
    // Adjacent swaps, at most as many passes as members: the comparison isn't
    // a total order, so this stops rather than insisting on one.
    for _ in 0..members.len() {
        let mut swapped = false;
        for k in 1..members.len() {
            if goes_ahead(&members[k - 1], &members[k]) {
                members.swap(k - 1, k);
                swapped = true;
            }
        }
        if !swapped {
            break;
        }
    }
    order.clear();
    order.extend(members.iter().map(|m| m.id));
}

/// The term a merge imposes on `follower` behind `leader`, the one before it
/// in its order, and the gap between them projected to the merge point: `None`
/// once the follower's front has reached the merge point, or if the leader is
/// already far enough past it.
///
/// The leader's rear is the gap the follower wants behind it past the merge
/// point at `t`. The follower arrives no sooner by braking evenly until then,
/// or, if that would stop it first, by stopping at the merge point. The term
/// is the larger of that and IDM's term for stopping at the merge point, so it
/// never brakes harder to open a gap than to stop short.
pub fn term(
    vehicles: &SlotMap<VehicleId, Vehicle>,
    follower: &Member,
    leader: &Member,
) -> Option<(f64, f64)> {
    let (f, l) = (&vehicles[follower.id], &vehicles[leader.id]);
    let d_f = follower.front;
    if d_f <= 0.0 {
        return None;
    }
    // From the leader's rear to the merge point.
    let d_l = leader.front + l.spec.length;
    let want = f.spec.min_gap + l.speed * f.spec.time_headway;
    if d_l + want <= 0.0 {
        return None;
    }
    let (v, stop) = (f.speed, -f.speed * f.speed / (2.0 * d_f));
    let gap_term = if l.speed <= 1e-6 {
        stop
    } else {
        let t = (d_l + want) / l.speed;
        let a = 2.0 * (d_f - v * t) / (t * t);
        if v + a * t < 0.0 { stop } else { a }
    };
    let stop_term = acceleration::interaction(&f.spec, v, d_f, 0.0);
    let accel = gap_term.max(stop_term).max(-acceleration::MAX_BRAKING);
    Some((accel, d_f - d_l))
}

#[cfg(test)]
mod test {
    use super::*;

    fn at(front: f64, lane: u32, eta: f64) -> Member {
        let mut ids = SlotMap::<VehicleId, ()>::with_key();
        Member { id: ids.insert(()), lane: LaneId(lane), front, eta }
    }

    #[test]
    fn the_order_swaps_only_past_the_hysteresis_and_never_along_one_lane() {
        // Another lane: only once clearly earlier.
        assert!(!goes_ahead(&at(50.0, 0, 2.0), &at(45.0, 1, 1.6)));
        assert!(goes_ahead(&at(50.0, 0, 2.0), &at(45.0, 1, 1.4)));
        // The same lane: by position along it, whatever the estimate.
        assert!(!goes_ahead(&at(20.0, 0, 5.0), &at(30.0, 0, 1.0)));
        assert!(goes_ahead(&at(30.0, 0, 1.0), &at(20.0, 0, 5.0)));
        // Already at the merge point: first.
        assert!(goes_ahead(&at(5.0, 0, 0.2), &at(-0.5, 1, 0.0)));
        assert!(!goes_ahead(&at(-0.5, 1, 0.0), &at(5.0, 0, 0.0)));
    }
}
