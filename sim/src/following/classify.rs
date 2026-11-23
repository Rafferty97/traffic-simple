//! Classifying a vehicle's record on a lane it isn't following as converging
//! or diverging, from the course it will drive.

use glam::DVec2;

use super::{Along, Lateral, Occupant, Overlaps, body};
use crate::motion::kinematics;
use crate::motion::lateral;
use crate::network::geometry::{LanePos, SAME_POINT};
use crate::network::{LaneId, Side, SimNetwork};
use crate::vehicle::{Vehicle, VehicleSpec};

/// The most samples classification takes. A course that needs more stays
/// converging until the vehicle next advances.
const TRACE_SAMPLES: usize = 512;

/// The longest step classification drives a course in, in meters: short enough
/// that the rear axle trails the front as it will.
const TRAIL_STEP: f64 = 0.5;

/// How far, in meters, a body with `spec` may come back toward a lane it is
/// leaving and still count as moving away from it: the swing of its tail as
/// it turns, at most. Turning about a point `R` from its rear axle, its outer
/// rear corner, `o` behind the axle and `w/2` out, is `√((R + w/2)² + o²)` from
/// it, further than the side of its body: at a lane change's tightest bend,
/// from rest, about 10 cm for a car.
fn swing(spec: &VehicleSpec) -> f64 {
    let radius = lateral::REST_CHANGE_RADIUS;
    let side = radius + 0.5 * spec.width;
    side.hypot(spec.rear_axle) - side
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
    // Its course as it will drive it: its front axle along its path, at its
    // lane change's offset, its rear axle trailing, in steps short enough to
    // follow, each station `ahead` how far its front axle has gone.
    let wheelbase = vehicle.spec.wheelbase();
    let change = vehicle.lane_change;
    let offset = |s: f64| change.map_or(0.0, |c| c.offset_at(s));
    let (mut rear, mut dir, mut done) = (vehicle.pos, vehicle.dir, 0.0);
    let mut drive = |ahead: f64| {
        while done < ahead {
            done = (done + TRAIL_STEP).min(ahead);
            let front = network.smooth_point_along(&vehicle.path, vehicle.front_s + done, offset);
            (rear, dir) = kinematics::trail(rear, dir, front, wheelbase);
        }
        body(&vehicle.spec, rear, dir)
    };
    let mut samples = arrayvec::ArrayVec::<Sample, TRACE_SAMPLES>::new();
    for &ahead in &stations {
        let corners = if ahead == 0.0 {
            vehicle.corners()
        } else {
            drive(ahead)
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

/// Where, ahead of its front axle along its path, classification samples a
/// vehicle's course against `lane`, from 0 to `window`, in order: where the
/// course or the lane bends or changes width, at their vertices, and along
/// its lane change's quintic as closely as keeps the straight lines between
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
    // Stations are how far its front axle goes, from where it is on its path.
    let axle = vehicle.front_s;
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
    // Its lane change: straight lines `h` apart stray at most `h²·|t''| / 8`
    // from the quintic, whose second derivative is at most about
    // `(5.8·|offset| + 3.9·|length·slope|) / length²`.
    if let Some(c) = vehicle.lane_change {
        let bend =
            (6.0 * c.offset.abs() + 4.0 * (c.length * c.slope).abs()) / (c.length * c.length);
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
