//! Merges: where lanes outside junctions that were apart start to overlap, the
//! vehicles approaching are ordered by distance to it, and each takes a term
//! that opens a gap behind the one before it by the merge point (see Merges in
//! `docs/DESIGN.md`). The order is worked out afresh at each longitudinal
//! update, from the lanes leading in, so nothing about it is stored.

use arrayvec::ArrayVec;
use slotmap::SlotMap;

use crate::following::Occupant;
use crate::following::Overlaps;
use crate::models::acceleration;
use crate::network::{LaneId, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// Which merge each lane belongs to, if any, and each merge's lanes: derived
/// from the network.
#[derive(Clone, Debug, Default)]
pub struct Merges {
    of_lane: Vec<Option<u32>>,
    lanes: Vec<Vec<LaneId>>,
}

impl Merges {
    /// How many merges there are.
    pub fn len(&self) -> usize {
        self.lanes.len()
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
    let mut lanes = vec![Vec::new(); next as usize];
    for (i, m) in of_lane.iter().enumerate() {
        if let Some(m) = m {
            lanes[*m as usize].push(LaneId(i as u32));
        }
    }
    Merges { of_lane, lanes }
}

/// How far `vehicle`'s rear axle is from the merge point of `merge`, the start
/// of the first of its lanes on its path. `None` if none of them is on its
/// path, or it is on one already.
fn ahead(network: &SimNetwork, merges: &Merges, vehicle: &Vehicle, merge: usize) -> Option<f64> {
    let mut start = -vehicle.lane_pos.s;
    for (k, &lane) in vehicle.path.iter().enumerate() {
        if merges.of(lane) == Some(merge) {
            return (k > 0).then_some(start);
        }
        start += network.lane(lane).geometry().length();
    }
    None
}

/// A vehicle in a merge's order, as the order is sorted and walked.
#[derive(Clone, Copy, Debug)]
pub struct Member {
    id: VehicleId,
    /// From its front to the merge point, in meters.
    front: f64,
}

fn member(
    network: &SimNetwork,
    merges: &Merges,
    vehicle: &Vehicle,
    id: VehicleId,
    merge: usize,
) -> Option<Member> {
    let axle = ahead(network, merges, vehicle, merge)?;
    let front = axle - (vehicle.spec.length - vehicle.spec.rear_axle);
    Some(Member { id, front })
}

impl Member {
    pub fn id(&self) -> VehicleId {
        self.id
    }
}

/// The most lanes leading into a merge that [`order`] walks.
const APPROACH: usize = 32;

/// A lane leading into a merge, as [`order`] marches along it.
struct Way {
    lane: LaneId,
    /// From its end to the merge point, in meters.
    end: f64,
    /// How many of its occupants, from its end back, are yet to be taken.
    left: usize,
    /// The nearest of those that is in the order, and its rear's distance to
    /// the merge point.
    head: Option<(f64, Member)>,
}

/// Calls `f` with each vehicle approaching `merge`, nearest the merge point
/// first, by the distance from its rear: its order.
///
/// From each of the merge's lanes it walks back through the lanes leading in,
/// as far as a path reaches, [`PATH_TIME`](crate::motion::path::PATH_TIME) at the
/// speed limits. Each lane's occupants are in order already, so it marches
/// back along all of them at once, as the merge step of a merge sort does,
/// taking the nearest each time, from those following the lane whose path
/// leads into the merge. A vehicle that pulls ahead goes first at once, as on
/// the road; close to the merge point, the terms keep vehicles from being
/// level, so the order doesn't flicker there.
pub fn order(
    network: &SimNetwork,
    merges: &Merges,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &[Vec<Occupant>],
    merge: usize,
    mut f: impl FnMut(Member),
) {
    let mut ways = ArrayVec::<Way, APPROACH>::new();
    let add = |ways: &mut ArrayVec<Way, APPROACH>, lane: LaneId, end: f64| {
        let known = ways.iter().any(|w| w.lane == lane) || merges.of(lane) == Some(merge);
        if !known && !ways.is_full() {
            let left = occupants[lane.0 as usize].len();
            ways.push(Way { lane, end, left, head: None });
        }
    };
    for &lane in &merges.lanes[merge] {
        for &p in network.lane(lane).predecessors() {
            add(&mut ways, p, 0.0);
        }
    }
    // The lanes leading in, back as far as a path reaches at the speed limits.
    let mut k = 0;
    let mut times = ArrayVec::<f64, APPROACH>::new();
    times.extend(ways.iter().map(|_| 0.0));
    while k < ways.len() {
        let (lane, end, time) = (ways[k].lane, ways[k].end, times[k]);
        k += 1;
        let l = network.lane(lane);
        let (length, time) = (
            l.geometry().length(),
            time + l.geometry().length() / l.speed_limit(),
        );
        if time > crate::motion::path::PATH_TIME {
            continue;
        }
        for &p in l.predecessors() {
            let before = ways.len();
            add(&mut ways, p, end + length);
            if ways.len() > before {
                times.push(time);
            }
        }
    }
    // The next vehicle in the order on a way, from its occupants yet to be taken.
    let next = |way: &mut Way| {
        let list = &occupants[way.lane.0 as usize];
        let length = network.lane(way.lane).geometry().length();
        way.head = None;
        while way.head.is_none() && way.left > 0 {
            way.left -= 1;
            let o = &list[way.left];
            let Some(vehicle) = vehicles.get(o.id) else {
                continue;
            };
            // Once each: from the lane it follows.
            if o.is_ghost() || vehicle.path[0] != way.lane {
                continue;
            }
            if let Some(m) = member(network, merges, vehicle, o.id, merge) {
                way.head = Some((way.end + length - o.s(), m));
            }
        }
    };
    for way in &mut ways {
        next(way);
    }
    loop {
        let nearest = ways
            .iter()
            .enumerate()
            .filter_map(|(i, w)| w.head.map(|(key, m)| (key, m.id, i)))
            .min_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let Some((_, _, i)) = nearest else {
            break;
        };
        let (_, member) = ways[i].head.expect("the nearest has a head");
        f(member);
        next(&mut ways[i]);
    }
}

/// How long before the merge point, in seconds at its speed, a vehicle starts
/// making room behind the one before it in the order: further back, it may be
/// level with it or ahead. Fast, it starts early; crawling in a queue, it
/// zips in only close to the merge point.
const MAKING_ROOM: f64 = 8.0;

/// The least distance, in meters, before the merge point a vehicle starts
/// making room. Shorter, crawling queues reach the merge point side by side
/// and go through it by turns from rest: at 10 m, a third less gets through.
const MIN_MAKING_ROOM: f64 = 50.0;

/// The term a merge imposes on `follower` behind `leader`, the one before it
/// in its order, and the gap between them as if the leader were on the
/// follower's lane, as far from the merge point: `None` once the follower's
/// front has reached the merge point.
///
/// The follower follows that virtual leader (virtual vehicle mapping, Uno et
/// al., 1999) by IDM, with the headway it will keep just after merging,
/// [`acceleration::RELAXED_HEADWAY`] of its own, and with slack that shrinks
/// from [`MAKING_ROOM`] before the merge point to nothing at it: enough, at the
/// start, for it to be level with the leader. So the gap opens gradually as
/// both draw near, and in a queue the lanes creep on together, staggered, as
/// the leader moves off. The order is by distance to the merge point, so the
/// leader is never further from it, and the slack covers being level: a fast vehicle gives way to a slower
/// one just ahead in the other lane by easing off on the approach. It never
/// brakes harder than comfortably: before the merge point the lanes are apart,
/// and inside it following keeps vehicles apart however they arrive.
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
    // From the leader's rear to the merge point, and from the follower's front
    // to the leader's rear, as if on the same lane.
    let d_l = leader.front + l.spec.length;
    let gap = d_f - d_l;
    let headway = acceleration::RELAXED_HEADWAY;
    let room = (MAKING_ROOM * f.speed).max(MIN_MAKING_ROOM);
    let slack = (d_f / room).clamp(0.0, 1.0)
        * (f.spec.length + f.spec.min_gap + f.speed * f.spec.time_headway * headway);
    let follow = acceleration::relaxed(&f.spec, f.speed, gap + slack, l.speed, headway);
    // Before the merge point the lanes are apart, so this only smooths the way
    // in: inside, following keeps them apart however they arrive.
    Some((follow.max(-f.spec.comfy_decel), gap))
}
