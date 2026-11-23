//! Which lanes overlap, outside junctions, and where each pair diverges for
//! good: worked out when the simulation is built ("Preprocessing").

use glam::DVec2;

use crate::network::geometry::SAME_POINT;
use crate::network::{LaneId, Side, SimNetwork};

/// Another lane, outside junctions, whose outline overlaps a lane's, with the
/// lane's diverge point: the `s` along it after which the two never converge
/// again. `f64::INFINITY` where they merge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Overlap {
    pub lane: LaneId,
    pub diverge: f64,
}

/// Each lane's overlaps, by lane: derived from the network.
pub type Overlaps = Vec<Vec<Overlap>>;

/// Finds every pair of lanes whose outlines overlap, other than lanes one after
/// the other and neighbours, which touch only along an end or an edge, and
/// each lane's diverge point from the other. Junctions keep vehicles on their
/// lanes apart, so pairs there are left out, except for branches leaving the
/// same lane: junctions rely on vehicles at a fork finding each other.
pub fn overlaps(network: &SimNetwork) -> Overlaps {
    let count = network.lanes().count();
    let mut table = vec![Vec::new(); count];
    let lanes: Vec<LaneId> = network.lanes().map(|(id, _)| id).collect();
    let bounds: Vec<(DVec2, DVec2)> = lanes
        .iter()
        .map(|&id| {
            let outline = network.lane(id).geometry().outline();
            outline.fold((DVec2::MAX, DVec2::MIN), |(lo, hi), p| {
                (lo.min(p), hi.max(p))
            })
        })
        .collect();
    for (i, &a) in lanes.iter().enumerate() {
        for (j, &b) in lanes.iter().enumerate().skip(i + 1) {
            let ((alo, ahi), (blo, bhi)) = (bounds[i], bounds[j]);
            if alo.cmpgt(bhi).any() || blo.cmpgt(ahi).any() || related(network, a, b) {
                continue;
            }
            let (la, lb) = (network.lane(a), network.lane(b));
            let junction = la.junction().is_some() || lb.junction().is_some();
            let fork = la.predecessors().iter().any(|p| lb.predecessors().contains(p));
            if junction && !fork {
                continue;
            }
            let (ga, gb) = (network.lane(a).geometry(), network.lane(b).geometry());
            if ga.overlap(gb).is_none() {
                continue;
            }
            table[a.0 as usize].push(Overlap { lane: b, diverge: diverge(network, a, b) });
            table[b.0 as usize].push(Overlap { lane: a, diverge: diverge(network, b, a) });
        }
    }
    table
}

/// Whether two lanes are one after the other or neighbours.
fn related(network: &SimNetwork, a: LaneId, b: LaneId) -> bool {
    let (la, lb) = (network.lane(a), network.lane(b));
    let neighbour = |l: &crate::network::Lane, other: LaneId| {
        [Side::Left, Side::Right]
            .iter()
            .any(|&side| l.adj(side).is_some_and(|n| n.lane == other))
    };
    la.successors().contains(&b) || lb.successors().contains(&a) || neighbour(la, b)
}

/// The `s` along `a` after which `a`'s centerline never closes in on `b`'s
/// again: `f64::INFINITY` if they lead into the same lane, and 0 if it never
/// does, as where they fork.
fn diverge(network: &SimNetwork, a: LaneId, b: LaneId) -> f64 {
    let (la, lb) = (network.lane(a), network.lane(b));
    if la.successors().iter().any(|s| lb.successors().contains(s)) {
        return f64::INFINITY;
    }
    let (ga, gb) = (la.geometry(), lb.geometry());
    let point = |s: f64| ga.lane_to_world(ga.lane_pos(s, 0.0));
    // Both centerlines are piecewise straight, so the separation runs straight
    // between their vertices: at `a`'s own, and where `b`'s are beside it.
    let mut hint = ga.world_to_lane(gb.centerline().next().expect("a lane has vertices"));
    let beside = gb.centerline().map(|p| {
        hint = ga.world_to_lane_near(p, hint);
        hint.s
    });
    let mut stations: Vec<f64> = ga.vertex_s().chain(beside).collect();
    stations.retain(|s| (0.0..=ga.length()).contains(s));
    stations.sort_by(f64::total_cmp);
    stations.dedup();
    let mut hint = gb.world_to_lane(point(0.0));
    let (mut previous, mut last) = (None, 0.0);
    for (i, &s) in stations.iter().enumerate() {
        let q = gb.world_to_lane_near(point(s), hint);
        hint = q;
        // Beside `b`, where it is along it; elsewhere `b` has ended or not begun.
        let separation = (0.0..=gb.length()).contains(&q.s).then_some(q.t.abs());
        match (previous, separation) {
            // Rounding to the grid alone can't close it in.
            (Some(p), Some(q)) if q < p - SAME_POINT => last = s,
            (None, Some(_)) if i > 0 => last = s,
            _ => {}
        }
        previous = separation;
    }
    last
}
