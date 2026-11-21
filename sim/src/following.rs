//! Multi-lane car following (see `docs/multilane-car-following.md`): which lanes
//! overlap, and how a vehicle's record on a lane it isn't following is
//! classified as converging or diverging.

use glam::DVec2;

use crate::lane::{LanePos, SAME_POINT};
use crate::models::lateral;
use crate::network::{LaneId, Side, SimNetwork};
use crate::occupancy::{Along, Lateral, Occupant, body};
use crate::vehicle::{Vehicle, VehicleSpec};

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

/// The most samples classification takes. A course that needs more stays
/// converging until the vehicle next advances.
const TRACE_SAMPLES: usize = 512;

/// How far, in meters, a body with `spec` may come back toward a lane it is
/// leaving and still count as moving away from it: the swing of its tail as
/// it turns, at most. Turning about a point `R` from its rear axle, its outer
/// rear corner, `o` behind the axle and `w/2` out, is `√((R + w/2)² + o²)` from
/// it, further than the side of its body: at full lock, about 10 cm for a car.
fn swing(spec: &VehicleSpec) -> f64 {
    let radius = 1.0 / lateral::max_curvature(spec.wheelbase());
    let side = radius + 0.5 * spec.width;
    side.hypot(spec.rear_axle) - side
}

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

/// One sample of a vehicle's course against a lane: whether its body overlaps
/// it, its rear's `s` along it, and its corners' least and greatest `t`.
#[derive(Clone, Copy, Debug)]
struct Sample {
    overlaps: bool,
    rear: f64,
    t_min: f64,
    t_max: f64,
}

/// Classifies `vehicle`'s record `o` on `lane`: if converging, with where it
/// turns diverging.
///
/// - On the lane it follows: in the lane, or converging while it changes lanes.
/// - On the lane before, lingering past its end: likewise.
/// - Where the lane merges with the one it follows: converging, always.
/// - Otherwise, its course is traced until its lane change, if any, has ended
///   and it is past the lanes' diverge point, if they overlap: from there it is
///   confined to its lane, which never closes in on this one again. It turns
///   diverging after the last sample at which it overlaps the lane while
///   closing in on it or coming back into it.
pub fn classify(
    network: &SimNetwork,
    overlaps: &Overlaps,
    vehicle: &Vehicle,
    lane: LaneId,
    o: &mut Occupant,
) {
    o.lateral = lateral(network, overlaps, vehicle, lane, o);
}

fn lateral(
    network: &SimNetwork,
    overlaps: &Overlaps,
    vehicle: &Vehicle,
    lane: LaneId,
    o: &Occupant,
) -> Lateral {
    let own = vehicle.path[0];
    let changing = vehicle.lane_change.is_some();
    if lane == own || network.lane(lane).successors().contains(&own) {
        return if changing {
            Lateral::CONVERGING
        } else {
            Lateral::InLane
        };
    }
    let diverge = overlaps[own.0 as usize].iter().find(|p| p.lane == lane).map(|p| p.diverge);
    if diverge == Some(f64::INFINITY) {
        return Lateral::CONVERGING;
    }
    let axle = vehicle.lane_pos.s;
    let rear = axle - vehicle.spec.rear_axle;
    let change_end = vehicle.lane_change.map_or(0.0, |c| c.end() - axle);
    let past_diverge = diverge.map_or(0.0, |d| d - rear);
    let window = change_end.max(past_diverge).max(0.0);
    let known: f64 = vehicle.path.iter().map(|&l| network.lane(l).geometry().length()).sum();
    let known = known - axle;

    let geometry = network.lane(lane).geometry();
    let pose = |ahead: f64| {
        let (pos, tangent) = Along::Path(0).pose(network, vehicle, axle, ahead);
        body(&vehicle.spec, pos, tangent)
    };
    let mut hint = o.hint;
    let mut sample = |corners: [DVec2; 4]| {
        let mut s = Sample {
            overlaps: false,
            rear: f64::INFINITY,
            t_min: f64::INFINITY,
            t_max: f64::NEG_INFINITY,
        };
        for (i, &c) in corners.iter().enumerate() {
            let p = geometry.world_to_lane_near(c, hint);
            if i == 0 {
                hint = p;
            }
            let on = p.s >= 0.0 && p.s < geometry.length() && geometry.between_edges(p);
            s.overlaps |= on;
            if i < 2 {
                s.rear = s.rear.min(p.s);
            }
            s.t_min = s.t_min.min(p.t);
            s.t_max = s.t_max.max(p.t);
        }
        s
    };

    if window > known {
        // Its course runs beyond what it knows: until it advances.
        return Lateral::CONVERGING;
    }
    let Some(stations) = trace(network, vehicle, lane, window) else {
        return Lateral::CONVERGING;
    };
    let mut samples = arrayvec::ArrayVec::<Sample, TRACE_SAMPLES>::new();
    for &ahead in &stations {
        let corners = if ahead == 0.0 {
            vehicle.corners()
        } else {
            pose(ahead)
        };
        samples.push(sample(corners));
    }

    // Which side of the lane it ends up on: as far along as it knows its way.
    let centre = {
        let c = pose(known);
        geometry.world_to_lane_near(0.5 * (c[0] + c[2]), hint)
    };
    let side = match centre.t {
        t if t >= SAME_POINT => Side::Left,
        t if t <= -SAME_POINT => Side::Right,
        // Level with the centerline: away from where its change started.
        _ => match vehicle.lane_change.map(|c| c.offset) {
            Some(o) if o > 0.0 => Side::Right,
            _ => Side::Left,
        },
    };
    // How far its body is from the lane's centerline, on that side.
    let away = |s: &Sample| match side {
        Side::Left => s.t_min,
        Side::Right => -s.t_max,
    };
    // Closing in again, or coming back into the lane, after an earlier sample. A
    // body swings its tail toward the lane it leaves as it turns, a few
    // centimetres, so closing in counts only beyond `SWING` from the furthest
    // it has been: well inside the passing clearance.
    let swing = swing(&vehicle.spec);
    let mut furthest = f64::NEG_INFINITY;
    let mut turn = 0;
    for j in 0..samples.len() {
        let b = &samples[j];
        let back = j > 0 && b.overlaps && (!samples[j - 1].overlaps || away(b) < furthest - swing);
        if back {
            turn = j + 1;
            furthest = away(b);
        } else {
            furthest = furthest.max(away(b));
        }
    }
    if turn == 0 {
        Lateral::Diverging(side)
    } else {
        Lateral::Converging { turn: samples[turn.min(samples.len() - 1)].rear, side }
    }
}

/// Where, ahead of its rear axle along its path, classification samples a
/// vehicle's course against `lane`, from 0 to `window`, in order: where the
/// course or the lane bends or changes width, at their vertices, and along
/// its lane change's cubic as closely as keeps the straight lines between
/// samples within [`SAME_POINT`] of it. `None` if that needs more than
/// [`TRACE_SAMPLES`].
fn trace(
    network: &SimNetwork,
    vehicle: &Vehicle,
    lane: LaneId,
    window: f64,
) -> Option<arrayvec::ArrayVec<f64, TRACE_SAMPLES>> {
    let mut stations = arrayvec::ArrayVec::<f64, TRACE_SAMPLES>::new();
    let mut push = |ahead: f64| {
        if (0.0..=window).contains(&ahead) {
            stations.try_push(ahead).ok()
        } else {
            Some(())
        }
    };
    push(0.0)?;
    push(window)?;
    let axle = vehicle.lane_pos.s;
    // Its path's vertices, and the lane's beside them.
    let other = network.lane(lane).geometry();
    let mut before = -axle;
    for &l in &vehicle.path {
        let geometry = network.lane(l).geometry();
        for s in geometry.vertex_s() {
            push(before + s)?;
        }
        let mut hint = geometry.lane_pos(0.0, 0.0);
        for p in other.centerline() {
            hint = geometry.world_to_lane_near(p, hint);
            if (0.0..=geometry.length()).contains(&hint.s) {
                push(before + hint.s)?;
            }
        }
        before += geometry.length();
        if before > window {
            break;
        }
    }
    // Its lane change: the cubic's second derivative is linear, so at its
    // largest at an end, and straight lines `h` apart stray at most
    // `h²·|t''| / 8` from it.
    if let Some(c) = vehicle.lane_change {
        let ends = [
            6.0 * c.offset + 4.0 * c.length * c.slope,
            6.0 * c.offset + 2.0 * c.length * c.slope,
        ];
        let bend = ends.iter().fold(0.0f64, |m, e| m.max(e.abs())) / (c.length * c.length);
        if bend > 0.0 {
            let step = (8.0 * SAME_POINT / bend).sqrt();
            let n = (c.length / step).ceil() as usize;
            for i in 0..=n {
                push(c.s0 + c.length * i as f64 / n as f64 - axle)?;
            }
        }
    }
    stations.sort_unstable_by(f64::total_cmp);
    let mut last = f64::NAN;
    stations.retain(|s| std::mem::replace(&mut last, *s) != *s);
    Some(stations)
}

/// The most lanes [`around`] gives.
pub const AROUND: usize = 64;

/// The lanes a vehicle around `lane` can have records on: it, the lanes before
/// it, and every lane overlapping any of them.
pub fn around(
    network: &SimNetwork,
    overlaps: &Overlaps,
    lane: LaneId,
) -> arrayvec::ArrayVec<LaneId, AROUND> {
    let mut lanes = arrayvec::ArrayVec::<LaneId, AROUND>::new();
    lanes.push(lane);
    lanes.extend(network.lane(lane).predecessors().iter().copied());
    for i in 0..lanes.len() {
        for p in &overlaps[lanes[i].0 as usize] {
            if !lanes.contains(&p.lane) {
                lanes.push(p.lane);
            }
        }
    }
    lanes
}

/// A record for `vehicle` on `lane`, from where it is, classified.
pub fn record(
    network: &SimNetwork,
    overlaps: &Overlaps,
    lane: LaneId,
    id: crate::vehicle::VehicleId,
    vehicle: &Vehicle,
) -> Occupant {
    let geometry = network.lane(lane).geometry();
    let hint: LanePos = geometry.world_to_lane(vehicle.pos);
    let mut o = Occupant::new(network, lane, id, vehicle, hint, Lateral::CONVERGING);
    classify(network, overlaps, vehicle, lane, &mut o);
    o
}
