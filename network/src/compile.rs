//! Compiling a road network to the simulation's lanes, and to render geometry.

use std::fmt;

use glam::DVec2;
use traffic_sim::{
    AMBER_DECEL, Barrier, BarrierKind, Keep, Lane, LaneError, LaneId, Movement, MovementId,
    NetworkError, Phase, SAME_POINT, SignalPlan, SimNetwork, VehicleSpec,
};

use crate::reference::{self, Polyline};
use crate::render;
use crate::{
    Compiled, Control, Direction, Intersection, LaneRef, Pose, RoadId, RoadNetwork, SignalPhase,
    Turn,
};

/// The sideways acceleration a connector's speed limit allows on its tightest
/// bend, in m/s².
const TURN_ACCEL: f64 = 2.0;
/// The lowest speed limit a connector gets, however tight, in m/s.
const MIN_TURN_SPEED: f64 = 3.0;

/// Why a road network doesn't compile.
#[derive(Clone, Debug, PartialEq)]
pub enum CompileError {
    /// A road lane or connector isn't a valid lane.
    Lane { what: String, error: LaneError },
    /// The lanes don't make a valid simulation network.
    Network(NetworkError),
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CompileError::Lane { what, error } => write!(f, "{what}: {error}"),
            CompileError::Network(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for CompileError {}

/// A lane as compiled, before it goes into the simulation.
pub(crate) struct Built {
    /// Its centerline, in the direction of travel.
    pub points: Vec<DVec2>,
    /// Its half-widths at each point, left and right of the direction of travel.
    pub half_widths: Vec<[f64; 2]>,
}

impl Built {
    fn start(&self) -> DVec2 {
        self.points[0]
    }

    fn end(&self) -> DVec2 {
        *self.points.last().unwrap()
    }

    /// Unit direction of travel at the end.
    fn end_dir(&self) -> DVec2 {
        let n = self.points.len();
        (self.points[n - 1] - self.points[n - 2]).normalize()
    }

    fn start_dir(&self) -> DVec2 {
        (self.points[1] - self.points[0]).normalize()
    }
}

/// Compiles a road network: each road lane and each intersection connector
/// becomes a lane in the simulation.
///
/// - Lanes of a road running side by side the same way, keeping their distance,
///   are neighbors, and may change into each other as their `change_left` and
///   `change_right` allow. Lanes easing toward or away
///   from each other, at a lane drop or where a lane appears, are not.
/// - Where roads join, a lane connects to each lane of the next road that
///   starts where it ends: two meeting one merge, one meeting two diverge.
/// - Each intersection connection is a cubic Bézier from one lane's end to the
///   other's start, along their directions, its control points a third of the
///   way across. Its width blends from one to the other, and its speed limit
///   is lowered for its tightest bend.
/// - Each intersection's connectors are one junction. Road lanes merging into
///   the same lane are not: vehicles on them keep apart by following.
pub fn compile(net: &RoadNetwork) -> Result<Compiled, CompileError> {
    let mut lanes: Vec<Lane> = Vec::new();
    let mut built: Vec<Built> = Vec::new();
    let mut road_lanes = Vec::new();
    let mut polylines = Vec::new();

    for (r, road) in net.roads.iter().enumerate() {
        let profiles: Vec<_> = road.lanes.iter().flat_map(|l| [l.offset, l.width]).collect();
        let line = Polyline::new(road.start, &road.pieces, &profiles);
        let mut ids = Vec::new();
        for (i, lane) in road.lanes.iter().enumerate() {
            let mut points = line.offset(&lane.offset);
            let mut half_widths: Vec<[f64; 2]> =
                line.s.iter().map(|&s| [0.5 * lane.width.at(s); 2]).collect();
            if lane.direction == Direction::Backward {
                points.reverse();
                half_widths.reverse();
            }
            let what = || format!("road {r} lane {i}");
            let b = Built { points, half_widths };
            let keep = if road.speed_limit > net.keep_above {
                net.keep
            } else {
                Keep::NoPreference
            };
            let built_lane = build(&b, road.speed_limit, what)?;
            lanes.push(built_lane.with_keep(keep).with_leaving(lane.leaving));
            ids.push(LaneId(built.len() as u32));
            built.push(b);
        }
        road_lanes.push(ids);
        polylines.push(line);
    }

    let mut connectors = Vec::new();
    let mut turns = Vec::new();
    let mut links = Vec::new();
    for (k, intersection) in net.intersections.iter().enumerate() {
        let mut ids = Vec::new();
        let mut ways = Vec::new();
        for (c, &(from, to)) in intersection.connections.iter().enumerate() {
            let lane_id = |l: LaneRef| road_lanes[l.road.0][l.lane];
            let (a, b) = (
                &built[lane_id(from).0 as usize],
                &built[lane_id(to).0 as usize],
            );
            let turn = Turn::between(a.end_dir(), b.start_dir());
            ways.push(turn);
            let straight = match turn.far_side(net.drives_on.side()) {
                true => intersection.far_side_straight,
                false => 0.0,
            };
            let connector = connect(a, b, straight, intersection.connector_width);
            let limit = net.roads[from.road.0].speed_limit.min(net.roads[to.road.0].speed_limit);
            let what = || format!("intersection {k} connection {c}");
            lanes.push(build(
                &connector,
                turn_limit(&connector.points, limit),
                what,
            )?);
            let id = LaneId(built.len() as u32);
            built.push(connector);
            links.extend([(lane_id(from), id), (id, lane_id(to))]);
            ids.push(id);
        }
        connectors.push(ids);
        turns.push(ways);
    }

    let mut sim = SimNetwork::new(lanes);
    for &(a, b) in &net.joins {
        for (i, la) in net.roads[a.0].lanes.iter().enumerate() {
            for (j, lb) in net.roads[b.0].lanes.iter().enumerate() {
                let (x, y) = (road_lanes[a.0][i], road_lanes[b.0][j]);
                // Forward lanes run from `a` into `b`, backward lanes from `b` into `a`.
                let (from, to) = match (la.direction, lb.direction) {
                    (Direction::Forward, Direction::Forward) => (x, y),
                    (Direction::Backward, Direction::Backward) => (y, x),
                    _ => continue,
                };
                if built[from.0 as usize].end().distance(built[to.0 as usize].start()) < SAME_POINT
                {
                    links.push((from, to));
                }
            }
        }
    }
    for &(from, to) in &links {
        sim.connect(from, to);
    }

    for (r, road) in net.roads.iter().enumerate() {
        let line = &polylines[r];
        let (s0, s1) = (line.s[0], *line.s.last().unwrap());
        // Its lanes from left to right of the reference line.
        let mut order: Vec<usize> = (0..road.lanes.len()).collect();
        order.sort_by(|&a, &b| road.lanes[b].offset.at(s0).total_cmp(&road.lanes[a].offset.at(s0)));
        for pair in order.windows(2) {
            let (upper, lower) = (&road.lanes[pair[0]], &road.lanes[pair[1]]);
            let gap = |s: f64| upper.offset.at(s) - lower.offset.at(s);
            let present = |s: f64| upper.width.at(s) > 0.0 && lower.width.at(s) > 0.0;
            let parallel = (gap(s0) - gap(s1)).abs() < SAME_POINT && present(s0) && present(s1);
            if upper.direction != lower.direction || !parallel {
                continue;
            }
            let (u, l) = (road_lanes[r][pair[0]], road_lanes[r][pair[1]]);
            // Going forward, the upper lane is on the left of the lower one;
            // going backward, on its right.
            match upper.direction {
                Direction::Forward => sim.set_adjacent(u, l, upper.change_right, lower.change_left),
                Direction::Backward => {
                    sim.set_adjacent(l, u, lower.change_right, upper.change_left)
                }
            }
        }
    }

    for ids in &connectors {
        if !ids.is_empty() {
            sim.add_junction(ids);
        }
    }
    for (k, intersection) in net.intersections.iter().enumerate() {
        if let Control::Signals(phases) = &intersection.control {
            add_signals(
                net,
                &mut sim,
                intersection,
                &connectors[k],
                &turns[k],
                phases,
            );
        }
    }
    sim.validate().map_err(CompileError::Network)?;

    let render = render::build(net, &polylines, &turns);
    Ok(Compiled { sim, render, road_lanes, connectors, turns })
}

/// An intersection's signal plan, and a signal barrier at the start of each of
/// its connectors for its movement.
///
/// A movement is the connections from one road that go one way. Two conflict
/// if any of their connectors do, unless they are green in the same phase,
/// where one gives way to the other. Each one's amber time is ITE's, a second
/// to react plus the time to stop from its approach's limit at
/// [`AMBER_DECEL`]; its clearance, the time to cross its longest connector
/// and a car's length at that limit.
fn add_signals(
    net: &RoadNetwork,
    sim: &mut SimNetwork,
    intersection: &Intersection,
    connectors: &[LaneId],
    turns: &[Turn],
    phases: &[SignalPhase],
) {
    let mut movements: Vec<(RoadId, Turn)> = Vec::new();
    let of: Vec<usize> = intersection
        .connections
        .iter()
        .zip(turns)
        .map(|(&(from, _), &turn)| {
            let key = (from.road, turn);
            movements.iter().position(|&m| m == key).unwrap_or_else(|| {
                movements.push(key);
                movements.len() - 1
            })
        })
        .collect();
    let mask = |green: &[(RoadId, Turn)]| {
        green
            .iter()
            .filter_map(|key| movements.iter().position(|m| m == key))
            .fold(0u64, |mask, i| mask | (1 << i))
    };
    let phases: Vec<Phase> = phases
        .iter()
        .map(|p| Phase { green: mask(&p.green), duration: p.duration })
        .collect();
    let both = |i: usize, j: usize| (1u64 << i) | (1u64 << j);
    let masks: Vec<u64> = phases.iter().map(|p| p.green).collect();
    let together = |i: usize, j: usize| masks.iter().any(|&m| m & both(i, j) == both(i, j));
    let car = VehicleSpec::passenger_car().length;
    let plan = SignalPlan {
        movements: (0..movements.len())
            .map(|i| {
                let mine =
                    || connectors.iter().zip(&of).filter(move |&(_, &m)| m == i).map(|(&c, _)| c);
                let conflicts = (0..movements.len())
                    .filter(|&j| j != i && !together(i, j))
                    .filter(|&j| {
                        let theirs: Vec<LaneId> = connectors
                            .iter()
                            .zip(&of)
                            .filter(|&(_, &m)| m == j)
                            .map(|(&c, _)| c)
                            .collect();
                        mine().any(|a| theirs.iter().any(|&b| sim.conflict_between(a, b).is_some()))
                    })
                    .fold(0u64, |mask, j| mask | (1 << j));
                let speed = net.roads[movements[i].0.0].speed_limit;
                let longest = mine().map(|c| sim.lane(c).geometry().length()).fold(0.0, f64::max);
                Movement {
                    conflicts,
                    amber: 1.0 + speed / (2.0 * AMBER_DECEL),
                    clearance: (longest + car) / speed,
                }
            })
            .collect(),
        phases,
    };
    let first = sim.add_signal(plan);
    for (&connector, &m) in connectors.iter().zip(&of) {
        let movement = MovementId(first.0 + m as u32);
        sim.add_barrier(
            connector,
            Barrier { s: 0.0, kind: BarrierKind::Signal { movement } },
        );
    }
    // Every road has priority at signals: a far-side turn gives way to the
    // traffic it crosses, waiting inside the junction at its first conflict with
    // a connector of a higher level green with it.
    let drives_on = net.drives_on.side();
    let level = |turn: Turn| match turn.far_side(drives_on) {
        true => PRIORITY_ROAD + FAR_SIDE,
        false => PRIORITY_ROAD + STRAIGHT_OR_NEAR,
    };
    for (&connector, &turn) in connectors.iter().zip(turns) {
        sim.set_priority(connector, level(turn));
    }
    for (c, (&connector, &turn)) in connectors.iter().zip(turns).enumerate() {
        if !turn.far_side(drives_on) {
            continue;
        }
        let waits = connectors
            .iter()
            .zip(turns)
            .enumerate()
            .filter(|&(o, (_, &other))| level(other) > level(turn) && together(of[c], of[o]))
            .filter_map(|(_, (&other, _))| sim.conflict_between(connector, other))
            .map(|conflict| conflict.mine.start)
            .fold(f64::INFINITY, f64::min);
        if waits.is_finite() {
            sim.add_barrier(connector, Barrier { s: waits, kind: BarrierKind::GiveWay });
        }
    }
}

/// The priority levels of a junction's connectors (see `docs/DESIGN.md`,
/// Intersection control): straight on or near-side from a priority road, 3;
/// far-side from one, 2; from any other road, one less each.
const PRIORITY_ROAD: u8 = 2;
const STRAIGHT_OR_NEAR: u8 = 1;
const FAR_SIDE: u8 = 0;

/// A simulation lane from a built one.
fn build(b: &Built, speed_limit: f64, what: impl Fn() -> String) -> Result<Lane, CompileError> {
    Lane::with_half_widths(&b.points, &b.half_widths, speed_limit)
        .map_err(|error| CompileError::Lane { what: what(), error })
}

/// A connector from the end of `a` to the start of `b`: `straight` on along
/// `a`'s direction, then a cubic Bézier along their directions, its control
/// points a third of the way across, sampled by the same tolerance as roads,
/// `width` wide, or by default as wide as the
/// widest vehicle sweeps along it (see [`swept`]), plus the room one needs to
/// pass another on each side.
fn connect(a: &Built, b: &Built, straight: f64, width: Option<f64>) -> Built {
    let (p0, p3) = (a.end() + straight * a.end_dir(), b.start());
    let reach = p0.distance(p3) / 3.0;
    let (c1, c2) = (p0 + reach * a.end_dir(), p3 - reach * b.start_dir());
    let start = Pose { pos: p0, heading: a.end_dir().to_angle() };
    let (mut s, mut points) = reference::sample_bezier(start, c1, c2, p3);
    // The straight run on, one segment.
    if straight > 0.0 {
        s.iter_mut().for_each(|s| *s += straight);
        s.insert(0, 0.0);
        points.insert(0, a.end());
    }
    let half_widths = match width {
        Some(width) => vec![[0.5 * width; 2]; s.len()],
        // The widest vehicle: the only class so far.
        None => swept(&points, &s, &traffic_sim::VehicleSpec::passenger_car())
            .into_iter()
            .map(|h| h.map(|h| h + traffic_sim::PASSING_CLEARANCE))
            .collect(),
    };
    Built { points, half_widths }
}

/// How far out to each side of `points`, at arc lengths `s`, a vehicle like
/// `spec` sweeps along them, at each vertex, as road design's swept paths have
/// it: with its rear axle at each vertex in turn, heading along the path, each
/// corner is projected onto the nearest segment, the first and last running on
/// as lines, widening both its ends to reach it. Never less than half its width. On a bend the outer front corner
/// swings out, `√(f² + (R + w/2)²) − R` for a front `f` ahead of the rear axle
/// and a radius `R`, and the inner rear corner cuts in a little; a turn's mouth,
/// which bends little, stays narrow.
fn swept(points: &[DVec2], s: &[f64], spec: &traffic_sim::VehicleSpec) -> Vec<[f64; 2]> {
    let mut half = vec![[0.5 * spec.width; 2]; points.len()];
    let (front, rear, side) = (
        spec.length - spec.rear_axle,
        spec.rear_axle,
        0.5 * spec.width,
    );
    for (i, &axle) in points.iter().enumerate() {
        let segment = i.min(points.len() - 2);
        let dir = (points[segment + 1] - points[segment]).normalize();
        let corners = [front, -rear]
            .map(|along| [side, -side].map(|across| axle + along * dir + across * dir.perp()));
        for corner in corners.into_iter().flatten() {
            // The nearest segment, among those within the body's reach.
            let r = front + rear + side;
            let reach = |j: &usize| s[*j] <= s[i] + r && s[*j + 1] >= s[i] - r;
            let nearest = (0..points.len() - 1).filter(reach).map(|j| {
                let (a, b) = (points[j], points[j + 1]);
                // The ends run on as lines, as the lanes before and after do.
                let along = (corner - a).dot(b - a) / (b - a).length_squared();
                let lo = if j == 0 { f64::MIN } else { 0.0 };
                let hi = if j + 2 == points.len() { f64::MAX } else { 1.0 };
                let foot = a + along.clamp(lo, hi) * (b - a);
                (j, (corner - foot).length(), (b - a).perp_dot(corner - a))
            });
            let Some((j, distance, cross)) = nearest.min_by(|x, y| x.1.total_cmp(&y.1)) else {
                continue;
            };
            let e = if cross >= 0.0 { 0 } else { 1 };
            for k in [j, j + 1] {
                half[k][e] = half[k][e].max(distance);
            }
        }
    }
    half
}

/// The tightest bend of `points` each way, as curvatures bending left and
/// right.
fn tightest(points: &[DVec2]) -> [f64; 2] {
    points.windows(3).fold([0.0; 2], |[left, right], w| {
        let (a, b) = (w[1] - w[0], w[2] - w[1]);
        let k = a.angle_to(b) / (0.5 * (a.length() + b.length()));
        [left.max(k), right.max(-k)]
    })
}

/// `limit`, lowered so that a vehicle on the tightest bend of `points` turns
/// with at most [`TURN_ACCEL`] sideways.
fn turn_limit(points: &[DVec2], limit: f64) -> f64 {
    let curvature = tightest(points).into_iter().fold(0.0, f64::max);
    if curvature <= 0.0 {
        return limit;
    }
    limit.min((TURN_ACCEL / curvature).sqrt().max(MIN_TURN_SPEED))
}
