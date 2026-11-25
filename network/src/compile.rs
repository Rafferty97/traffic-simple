//! Compiling a road network to the simulation's lanes, and to render geometry.

use std::fmt;

use glam::DVec2;
use traffic_sim::{
    ALIGNED, AMBER_DECEL, Barrier, BarrierKind, Keep, Lane, LaneError, LaneId, Line, Movement,
    MovementId, NetworkError, Phase, RoadId as SimRoadId, SAME_POINT, SignalPlan, SimNetwork,
    VehicleSpec,
};

use crate::arms::{self, StopLine};
use crate::reference::{self, Polyline};
use crate::render;
use crate::{Compiled, Control, Direction, Intersection, LaneRef, RoadId, RoadNetwork, Sign, Turn};

/// The road, for routing, of a road's lanes running in `direction`: each of a
/// road's two ways is a road of its own.
fn road_way(road: RoadId, direction: Direction) -> SimRoadId {
    SimRoadId(2 * road.0 as u32 + u32::from(direction == Direction::Backward))
}

/// Why a road network doesn't compile.
#[derive(Clone, Debug, PartialEq)]
pub enum CompileError {
    /// A road lane or connector isn't a valid lane.
    Lane { what: String, error: LaneError },
    /// The lanes don't make a valid simulation network.
    Network(NetworkError),
    /// A road's arm at an intersection doesn't fit it.
    Arm { road: RoadId, reason: &'static str },
    /// A vehicle's body turning along a connector would sweep onto a road
    /// lane other than its own: its stop line should be further back, or its
    /// run out longer.
    Sweeps {
        intersection: usize,
        connection: usize,
        lane: LaneRef,
        reach: [f64; 2],
    },
    /// A keep-clear zone reaches beyond where its road's lanes are.
    KeepClear { road: RoadId },
    /// A far-side turn's lane before where it waits for a gap keeps meeting
    /// the traffic it gives way to, however far back it is split.
    Waiting { intersection: usize, connection: usize },
    /// A vehicle's nose, still turning past a connector's end, would overhang
    /// another connector: the run out of the road it goes into should be
    /// longer.
    Overhangs { intersection: usize, connection: usize, beyond: f64 },
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CompileError::Lane { what, error } => write!(f, "{what}: {error}"),
            CompileError::Network(error) => write!(f, "{error}"),
            CompileError::Arm { road, reason } => write!(f, "road {}: {reason}", road.0),
            CompileError::KeepClear { road } => {
                write!(
                    f,
                    "road {}: a keep-clear zone reaches beyond its lanes",
                    road.0
                )
            }
            CompileError::Sweeps { intersection, connection, lane, .. } => write!(
                f,
                "intersection {intersection} connection {connection} sweeps onto road {} lane {}",
                lane.road.0, lane.lane
            ),
            CompileError::Waiting { intersection, connection } => write!(
                f,
                "intersection {intersection} connection {connection} meets traffic it gives way to before it waits"
            ),
            CompileError::Overhangs { intersection, connection, .. } => write!(
                f,
                "intersection {intersection} connection {connection} overhangs another past its end"
            ),
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
/// - Each intersection connection is a quintic Bézier from one lane's end to
///   the other's start, along their directions, with no curvature at either
///   end. It is as wide as a vehicle sweeps along it, and its speed limit is
///   lowered for its tightest bend.
/// - Lanes coming into an intersection end at their own stop lines, so where
///   a line is skewed, lanes side by side end apart, and are neighbours only
///   as far as the first of them ends.
/// - Each intersection's connectors are one junction. Road lanes merging into
///   the same lane are not: vehicles on them keep apart by following.
pub fn compile(net: &RoadNetwork) -> Result<Compiled, CompileError> {
    let (compiled, mut swept) = compile_swept(net)?;
    match swept.is_empty() {
        true => Ok(compiled),
        false => Err(swept.swap_remove(0)),
    }
}

/// As [`compile`], with every sweep that fails it, which only fails it once
/// it has compiled otherwise.
fn compile_swept(net: &RoadNetwork) -> Result<(Compiled, Vec<CompileError>), CompileError> {
    assemble(net)
}

/// How near one another, in meters, the places a road's lanes are cut at one
/// end are merged into one vertex of its reference line.
const MERGE_CUTS: f64 = 0.05;

/// How far into a connector a far-side turn's give-way line must be, past its
/// stop line, in meters: one nearer would meet the traffic it gives way to as
/// it crossed its stop line.
const WAIT_MARGIN: f64 = 0.01;

/// [`compile_swept`]'s work: the network compiled, with every connector checked
/// for its sweep.
fn assemble(net: &RoadNetwork) -> Result<(Compiled, Vec<CompileError>), CompileError> {
    let mut lanes: Vec<Lane> = Vec::new();
    let mut built: Vec<Built> = Vec::new();
    let mut road_lanes = Vec::new();
    let mut polylines = Vec::new();
    // Each road lane's stretches that belong to an intersection: the one before
    // it, from the intersection to its start, and the one after it, from its
    // end into the intersection, in its direction of travel.
    let mut stretches: Vec<[Option<Vec<DVec2>>; 2]> = Vec::new();
    // Each keep-clear zone's barrier, on each lane across it.
    let mut zones: Vec<(LaneId, Barrier)> = Vec::new();
    let ends = arms::road_ends(net);

    for (r, road) in net.roads.iter().enumerate() {
        let profiles: Vec<_> = road.lanes.iter().flat_map(|l| [l.offset, l.width]).collect();
        let total =
            *Polyline::new(road.start, &road.pieces, &profiles, [&[], &[]]).s.last().unwrap();
        // How far back from each end each of its lanes is cut, coming in or
        // going out.
        let arm = ends[r].map(|end| end.map(|(_, arm)| arm));
        let cut_of = |lane: &crate::RoadLane, at_end: bool| {
            arm[at_end as usize].map_or(0.0, |arm| arms::cut(lane, total, &arm, at_end))
        };
        // Every distinct cut at an end, for the reference line's vertices: any
        // within MERGE_CUTS of another merged into the one furthest back, so no
        // sliver of a segment kinks the lanes when they are snapped to the
        // grid. A lane cut there ends a little short of its stop line, which
        // its barrier's line still holds vehicles at.
        let kept = |at_end: bool| {
            let mut cuts: Vec<f64> =
                road.lanes.iter().map(|l| cut_of(l, at_end)).filter(|&d| d > 0.0).collect();
            cuts.sort_by(|a, b| b.total_cmp(a));
            cuts.dedup_by(|a, b| *b - *a < MERGE_CUTS);
            cuts
        };
        if road
            .keep_clear
            .iter()
            .any(|z| !(0.0 <= z.from && z.from < z.to && z.to <= total))
        {
            return Err(CompileError::KeepClear { road: RoadId(r) });
        }
        // Vertices where its lanes are cut, and where its keep-clear zones
        // start and end, so they are square across it.
        let (start_cuts, end_cuts) = (kept(false), kept(true));
        let mut from_start = start_cuts.clone();
        from_start.extend(road.keep_clear.iter().flat_map(|z| [z.from, z.to]));
        let line = Polyline::new(
            road.start,
            &road.pieces,
            &profiles,
            [&from_start, &end_cuts],
        );
        let mut ids = Vec::new();
        for (i, lane) in road.lanes.iter().enumerate() {
            let mut points = line.offset(&lane.offset);
            let mut half_widths: Vec<[f64; 2]> =
                line.s.iter().map(|&s| [0.5 * lane.width.at(s); 2]).collect();
            // Where it is cut at each end, coming in or going out: at the
            // vertex its cut was merged into.
            let at = |at_end: bool| {
                let cut = cut_of(lane, at_end);
                let vertices = if at_end { &end_cuts } else { &start_cuts };
                vertices
                    .iter()
                    .copied()
                    .min_by(|a, b| (a - cut).abs().total_cmp(&(b - cut).abs()))
                    .filter(|_| cut > 0.0)
                    .unwrap_or(0.0)
            };
            let n = points.len();
            let lo = if at(false) > 0.0 {
                line.vertex(at(false))
            } else {
                0
            };
            let hi = if at(true) > 0.0 {
                line.vertex(total - at(true))
            } else {
                n - 1
            };
            if lo >= hi {
                return Err(CompileError::Arm {
                    road: RoadId(r),
                    reason: "its lanes are cut away",
                });
            }
            let mut before = (lo > 0).then(|| points[..=lo].to_vec());
            let mut after = (hi < n - 1).then(|| points[hi..].to_vec());
            points = points[lo..=hi].to_vec();
            // Each keep-clear zone across it, as a barrier at its start in the
            // lane's own direction, its length along the lane.
            let along = arc_lengths(&points);
            let length = *along.last().unwrap();
            for zone in road.keep_clear.iter().filter(|z| z.direction == lane.direction) {
                let at = |s: f64| {
                    let i = line.vertex(s);
                    (lo..=hi).contains(&i).then(|| along[i - lo])
                };
                let (Some(a), Some(b)) = (at(zone.from), at(zone.to)) else {
                    return Err(CompileError::KeepClear { road: RoadId(r) });
                };
                let start = match lane.direction {
                    Direction::Forward => a,
                    Direction::Backward => length - b,
                };
                let kind = BarrierKind::KeepClear { length: b - a };
                zones.push((LaneId(built.len() as u32), Barrier::new(start, kind)));
            }
            half_widths = half_widths[lo..=hi].to_vec();
            if lane.direction == Direction::Backward {
                points.reverse();
                half_widths.reverse();
                for stretch in [&mut before, &mut after].into_iter().flatten() {
                    stretch.reverse();
                }
                std::mem::swap(&mut before, &mut after);
            }
            stretches.push([before, after]);
            let what = || format!("road {r} lane {i}");
            let b = Built { points, half_widths };
            let keep = if road.speed_limit > net.keep_above {
                net.keep
            } else {
                Keep::NoPreference
            };
            let built_lane = build(
                &b,
                road.speed_limit,
                road_way(RoadId(r), lane.direction),
                what,
            )?;
            lanes.push(built_lane.with_keep(keep).with_leaving(lane.leaving));
            ids.push(LaneId(built.len() as u32));
            built.push(b);
        }
        road_lanes.push(ids);
        polylines.push(line);
    }

    // Each way through an intersection from one road into another, a road of
    // its own for routing, numbered after the roads' ways.
    let mut ways_through = std::collections::HashMap::new();
    let mut connectors = Vec::new();
    // Each connection's connector, by intersection.
    let mut pieces = Vec::new();
    // Each connection's first lane, where its stop line's barrier goes.
    let mut entries: Vec<Vec<LaneId>> = Vec::new();
    let mut turns = Vec::new();
    let mut stops = Vec::new();
    let mut bodies = Vec::new();
    let mut links = Vec::new();
    for (k, intersection) in net.intersections.iter().enumerate() {
        let mut ids = Vec::new();
        let mut parts = Vec::new();
        let mut firsts = Vec::new();
        let mut ways = Vec::new();
        let mut lines = Vec::new();
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
            // From its lane's end, along the stretch to the intersection.
            let start =
                stretches[lane_id(from).0 as usize][1].clone().unwrap_or_else(|| vec![a.end()]);
            let dir = match start.len() {
                1 => a.end_dir(),
                n => (start[n - 1] - start[n - 2]).normalize(),
            };
            // To the road it goes into, and along the stretch to its lane's
            // start, or, turning into a lane other than the nearest on its side,
            // drifting across to it over that stretch.
            let behind = &stretches[lane_id(to).0 as usize][0];
            let (end, end_dir, tail) = match behind {
                Some(stretch) if turn == Turn::Straight || nearest(net, to, turn) => (
                    stretch[0],
                    (stretch[1] - stretch[0]).normalize(),
                    &stretch[1..],
                ),
                _ => (b.start(), b.start_dir(), &[][..]),
            };
            let (connector, body) = connect(
                &start,
                dir,
                straight,
                end,
                end_dir,
                tail,
                intersection.connector_width,
            );
            let (safe, line) = stop(net, &polylines, &ends, from);
            lines.push((safe, line));
            bodies.push(body);
            let limit = net.roads[from.road.0].speed_limit.min(net.roads[to.road.0].speed_limit);
            let what = || format!("intersection {k} connection {c}");
            let way_of = |l: LaneRef| road_way(l.road, net.roads[l.road.0].lanes[l.lane].direction);
            let next = (2 * net.roads.len() + ways_through.len()) as u32;
            let through =
                *ways_through.entry((k, way_of(from), way_of(to))).or_insert(SimRoadId(next));
            lanes.push(build(&connector, limit, through, what)?);
            let id = LaneId(built.len() as u32);
            built.push(connector);
            links.extend([(lane_id(from), id), (id, lane_id(to))]);
            ids.push(id);
            // Its barrier at its stop line, at its start.
            firsts.push(id);
            parts.push(id);
        }
        connectors.push(ids);
        pieces.push(parts);
        turns.push(ways);
        stops.push(lines);
        entries.push(firsts);
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
    for &(lane, barrier) in &zones {
        sim.add_barrier(lane, barrier);
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
            let [over_u, over_l] = side_by_side(&sim, u, l);
            // Going forward, the upper lane is on the left of the lower one;
            // going backward, on its right.
            match upper.direction {
                Direction::Forward => sim.set_adjacent_over(
                    u,
                    l,
                    upper.change_right,
                    lower.change_left,
                    [over_u, over_l],
                ),
                Direction::Backward => sim.set_adjacent_over(
                    l,
                    u,
                    lower.change_right,
                    upper.change_left,
                    [over_l, over_u],
                ),
            }
        }
    }

    for ids in &pieces {
        if !ids.is_empty() {
            sim.add_junction(ids);
        }
        for &id in ids {
            sim.set_keep_clear(id, true);
        }
    }
    let mut waiting = Vec::new();
    for (k, intersection) in net.intersections.iter().enumerate() {
        if let Control::Signals(_) = &intersection.control {
            let waits = add_signals(
                net,
                &mut sim,
                intersection,
                &pieces[k],
                &entries[k],
                &turns[k],
                &stops[k],
            );
            waiting.extend(waits.into_iter().map(|(c, at)| (k, c, at)));
        }
        if let Control::Priority { .. } = &intersection.control {
            let waits = add_priority(
                net,
                &mut sim,
                intersection,
                &pieces[k],
                &entries[k],
                &turns[k],
                &stops[k],
            );
            waiting.extend(waits.into_iter().map(|(c, at)| (k, c, at)));
        }
        if intersection.control == Control::AllWayStop {
            add_all_way_stop(&mut sim, &pieces[k], &entries[k], &stops[k]);
        }
    }
    // A far-side turn's give-way line past its stop line, so that nothing it
    // gives way to meets it before it gets there.
    if let Some(&(intersection, connection, _)) =
        waiting.iter().find(|&&(_, _, at)| at <= WAIT_MARGIN)
    {
        return Err(CompileError::Waiting { intersection, connection });
    }
    sim.validate().map_err(CompileError::Network)?;
    let swept = check_sweeps(&sim, &road_lanes, &connectors, &bodies);

    let render = render::build(net, &polylines, &turns);
    Ok((
        Compiled { sim, render, road_lanes, connectors, turns },
        swept,
    ))
}

/// Where along each of two lanes of a road, side by side the same way, the two
/// run beside each other: from where the later of them starts to where the
/// first of them ends, each found along the other, where they are square across
/// both. Lanes coming into an intersection end at their own stop lines, apart
/// where the line is skewed.
fn side_by_side(sim: &SimNetwork, a: LaneId, b: LaneId) -> [std::ops::Range<f64>; 2] {
    let [ga, gb] = [a, b].map(|l| sim.lane(l).geometry());
    let [la, lb] = [ga.length(), gb.length()];
    let point = |g: &traffic_sim::LaneGeometry, s: f64| g.lane_to_world(g.lane_pos(s, 0.0));
    // Where `s` along `from` is along `onto`.
    let along =
        |from: &traffic_sim::LaneGeometry, s: f64, onto: &traffic_sim::LaneGeometry, near: f64| {
            onto.world_to_lane_near(point(from, s), onto.lane_pos(near, 0.0)).s
        };
    // The later start, and the earlier end, or theirs where they start or end
    // together.
    let (from_a, from_b) = match along(gb, 0.0, ga, 0.0) {
        s if s > ALIGNED => (s, 0.0),
        _ => match along(ga, 0.0, gb, 0.0) {
            s if s > ALIGNED => (0.0, s),
            _ => (0.0, 0.0),
        },
    };
    let (until_a, until_b) = match along(gb, lb, ga, la) {
        s if s < la - ALIGNED => (s, lb),
        _ => match along(ga, la, gb, lb) {
            s if s < lb - ALIGNED => (la, s),
            _ => (la, lb),
        },
    };
    [from_a..until_a, from_b..until_b]
}

/// What [`fit_arms`] rounds how far it moves a stop line or lengthens a run
/// out up to, in meters.
const FIT_STEP: f64 = 0.25;

/// Compiles `net`, first moving stop lines back and lengthening run outs
/// wherever a vehicle's body turning along a connector would sweep onto a road
/// lane: onto one coming in, its stop line goes back as far as the sweep
/// reaches along it; onto one going out, or past the connector's end onto
/// another, its run out grows as far, each rounded up to `FIT_STEP`. So road design sets stop lines back
/// to clear turning vehicles' swept paths. The arms it fits are left in `net`.
pub fn fit_arms(net: &mut RoadNetwork) -> Result<Compiled, CompileError> {
    loop {
        let (compiled, swept) = compile_swept(net)?;
        if swept.is_empty() {
            return Ok(compiled);
        }
        let ends = arms::road_ends(net);
        let mut fitted: Vec<(usize, crate::Arm)> = Vec::new();
        for error in swept {
            // Overhanging past its end, the road it goes into runs out further.
            let (intersection, lane, reach) = match error {
                CompileError::Overhangs { intersection, connection, beyond } => (
                    intersection,
                    net.intersections[intersection].connections[connection].1,
                    [None, Some(beyond)],
                ),
                CompileError::Sweeps { intersection, lane, reach, .. } => {
                    (intersection, lane, reach.map(Some))
                }
                _ => unreachable!(),
            };
            let Some(at_end) = [false, true]
                .into_iter()
                .find(|&e| ends[lane.road.0][e as usize].is_some_and(|(k, _)| k == intersection))
            else {
                // A road lane at another intersection, or none: nothing here moves it.
                return Err(error);
            };
            // How far back from its end a lane coming in is swept, or from its
            // start a lane going out: so far its arm must cut it back, in steps.
            let road_lane = &net.roads[lane.road.0].lanes[lane.lane];
            let coming_in = reach[0].is_some() && arms::arriving(road_lane, at_end);
            let by = reach[!coming_in as usize].expect("how far it reaches");
            let by = (by / FIT_STEP).ceil().max(1.0) * FIT_STEP;
            // The furthest any sweep needs, from the arm as it was.
            let i = match fitted.iter().position(|(k, a)| *k == intersection && a.road == lane.road)
            {
                Some(i) => i,
                None => {
                    fitted.push((intersection, net.intersections[intersection].arm(lane.road)));
                    fitted.len() - 1
                }
            };
            let was = net.intersections[intersection].arm(lane.road);
            let arm = &mut fitted[i].1;
            match coming_in {
                true => arm.stop_line = arm.stop_line.max(was.stop_line + by),
                false => arm.run_out = arm.run_out.max(was.run_out + by),
            }
            if arm.stop_line.max(arm.run_out) > 0.5 * net.roads[lane.road.0].length() {
                return Err(error);
            }
        }
        for (k, arm) in fitted {
            let x = &mut net.intersections[k];
            x.arms.retain(|a| a.road != arm.road);
            x.arms.push(arm);
        }
    }
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
///
/// A far-side turn's give-way barrier goes on its connector where it first
/// meets traffic it gives way to (see [`add_far_side_waits`], whose places it
/// returns).
fn add_signals(
    net: &RoadNetwork,
    sim: &mut SimNetwork,
    intersection: &Intersection,
    pieces: &[LaneId],
    entries: &[LaneId],
    turns: &[Turn],
    stops: &[(f64, Option<Line>)],
) -> Vec<(usize, f64)> {
    let Control::Signals(phases) = &intersection.control else {
        return Vec::new();
    };
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
                let of = &of;
                let of_movement = move |m: usize| {
                    pieces.iter().zip(of).filter(move |&(_, &o)| o == m).map(|(&p, _)| p)
                };
                let conflicts = (0..movements.len())
                    .filter(|&j| j != i && !together(i, j))
                    .filter(|&j| {
                        of_movement(i)
                            .any(|a| of_movement(j).any(|b| sim.conflict_between(a, b).is_some()))
                    })
                    .fold(0u64, |mask, j| mask | (1 << j));
                let speed = net.roads[movements[i].0.0].speed_limit;
                let length = |c: LaneId| sim.lane(c).geometry().length();
                let longest = of_movement(i).map(length).fold(0.0, f64::max);
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
    // Every road has priority at signals.
    let drives_on = net.drives_on.side();
    let levels: Vec<u8> = turns.iter().map(|&turn| level(true, turn, drives_on)).collect();
    let waiting = add_far_side_waits(sim, pieces, &levels, |c, o| together(of[c], of[o]));
    // Each connector's signal at its stop line, giving way to the turners
    // waiting inside on movements never green with it whose way on from their
    // waiting lines crosses its own; and, if it may turn on red, giving way
    // then to every lane it conflicts with.
    let on_red = |c: usize| {
        let (from, turn) = (intersection.connections[c].0.road, turns[c]);
        let near_side = turn != Turn::Straight && !turn.far_side(drives_on);
        net.turn_on_red && near_side || intersection.turn_on_red.contains(&(from, turn))
    };
    for (c, ((&entry, &m), &(s, line))) in entries.iter().zip(&of).zip(stops).enumerate() {
        let movement = MovementId(first.0 + m as u32);
        let crossed: Vec<LaneId> = pieces
            .iter()
            .enumerate()
            .filter(|&(o, _)| o != c)
            .map(|(_, &other)| other)
            .filter(|&other| sim.conflict_between(pieces[c], other).is_some())
            .collect();
        let on_red = on_red(c).then(|| sim.add_yields(&crossed));
        let kind = BarrierKind::Signal { movement, on_red };
        let waiting_turners: Vec<LaneId> = waiting
            .iter()
            .filter(|&&(o, _)| !together(of[c], of[o]))
            .filter(|&&(o, at)| {
                sim.conflict_between(pieces[o], pieces[c]).is_some_and(|x| x.mine.end > at)
            })
            .map(|&(o, _)| pieces[o])
            .collect();
        let yields = (!waiting_turners.is_empty()).then(|| sim.add_yields(&waiting_turners));
        sim.add_barrier(entry, Barrier { line, yields, ..Barrier::new(s, kind) });
    }
    waiting
}

/// Each far-side turn from a priority road, of level `PRIORITY_ROAD +
/// FAR_SIDE`, gives way to the traffic it crosses, waiting inside the junction
/// at its first conflict with a connection of a higher level that can be moving
/// with it, by `together`, and gives way to those connections. Levels are
/// compilation's alone: the simulation has only the lanes each barrier gives
/// way to. Each connection's connector is `pieces`, and its level `levels`, by
/// connection.
///
/// Returned, each connection that waits so, and how far along its connector.
fn add_far_side_waits(
    sim: &mut SimNetwork,
    pieces: &[LaneId],
    levels: &[u8],
    together: impl Fn(usize, usize) -> bool,
) -> Vec<(usize, f64)> {
    let mut waiting = Vec::new();
    for (c, &connector) in pieces.iter().enumerate() {
        if levels[c] != PRIORITY_ROAD + FAR_SIDE {
            continue;
        }
        let oncoming: Vec<LaneId> = pieces
            .iter()
            .enumerate()
            .filter(|&(o, _)| levels[o] > levels[c] && together(c, o))
            .map(|(_, &other)| other)
            .collect();
        let waits = oncoming
            .iter()
            .filter_map(|&other| sim.conflict_between(connector, other))
            .map(|conflict| conflict.mine.start)
            .fold(f64::INFINITY, f64::min);
        if waits.is_finite() {
            let yields = Some(sim.add_yields(&oncoming));
            let give_way = Barrier { yields, ..Barrier::new(waits, BarrierKind::GiveWay) };
            sim.add_barrier(connector, give_way);
            waiting.push((c, waits));
        }
    }
    waiting
}

/// An intersection controlled by priority (see [`Control::Priority`]): each
/// far-side turn from a priority road waits inside it for its oncoming traffic,
/// as at signals, every connection able to move with every other; and each
/// connection from any other road starts with its approach's sign at its stop
/// line, giving way to the lanes of every connection of a higher level whose
/// lanes conflict with its own, and taking turns with those of the same level
/// from another such line. It takes one gap from the line, a far-side turn
/// too.
///
/// Returned, as from [`add_far_side_waits`], where far-side turns wait.
fn add_priority(
    net: &RoadNetwork,
    sim: &mut SimNetwork,
    intersection: &Intersection,
    pieces: &[LaneId],
    entries: &[LaneId],
    turns: &[Turn],
    stops: &[(f64, Option<Line>)],
) -> Vec<(usize, f64)> {
    let Control::Priority { major, signs } = &intersection.control else {
        return Vec::new();
    };
    let drives_on = net.drives_on.side();
    let road = |c: usize| intersection.connections[c].0.road;
    let levels: Vec<u8> = turns
        .iter()
        .enumerate()
        .map(|(c, &turn)| level(major.contains(&road(c)), turn, drives_on))
        .collect();
    let waiting = add_far_side_waits(sim, pieces, &levels, |_, _| true);
    for (c, (&entry, &(s, line))) in entries.iter().zip(stops).enumerate() {
        if major.contains(&road(c)) {
            continue;
        }
        let sign = signs.iter().find(|&&(r, _)| r == road(c)).map_or(Sign::GiveWay, |&(_, s)| s);
        let kind = match sign {
            Sign::GiveWay => BarrierKind::GiveWay,
            Sign::Stop => BarrierKind::Stop,
        };
        let crosses = |o: usize| sim.conflict_between(pieces[c], pieces[o]).is_some();
        let priority: Vec<LaneId> = (0..pieces.len())
            .filter(|&o| levels[o] > levels[c] && crosses(o))
            .map(|o| pieces[o])
            .collect();
        let equal: Vec<LaneId> = (0..pieces.len())
            .filter(|&o| !major.contains(&road(o)) && levels[o] == levels[c] && crosses(o))
            .map(|o| pieces[o])
            .collect();
        let yields = (!priority.is_empty()).then(|| sim.add_yields(&priority));
        let ties = (!equal.is_empty()).then(|| sim.add_yields(&equal));
        sim.add_barrier(
            entry,
            Barrier { line, yields, ties, ..Barrier::new(s, kind) },
        );
    }
    waiting
}

/// An all-way stop: a stop line at the start of each of an intersection's
/// connectors, giving way to nothing of higher priority and taking turns with
/// every connector whose lanes conflict with its own, so vehicles go in the
/// order they came to rest at their lines (see
/// `docs/models/gap-acceptance.md`).
fn add_all_way_stop(
    sim: &mut SimNetwork,
    pieces: &[LaneId],
    entries: &[LaneId],
    stops: &[(f64, Option<Line>)],
) {
    for (c, (&entry, &(s, line))) in entries.iter().zip(stops).enumerate() {
        let crosses = |o: usize| sim.conflict_between(pieces[c], pieces[o]).is_some();
        let equal: Vec<LaneId> =
            (0..pieces.len()).filter(|&o| o != c && crosses(o)).map(|o| pieces[o]).collect();
        let ties = (!equal.is_empty()).then(|| sim.add_yields(&equal));
        sim.add_barrier(
            entry,
            Barrier { line, ties, ..Barrier::new(s, BarrierKind::Stop) },
        );
    }
}

/// The priority levels of a junction's connectors (see `docs/DESIGN.md`,
/// Intersection control): straight on or near-side from a priority road, 3;
/// far-side from one, 2; from any other road, one less each.
const PRIORITY_ROAD: u8 = 2;
const STRAIGHT_OR_NEAR: u8 = 1;
const FAR_SIDE: u8 = 0;

/// The level of a connection from a priority road or not, going `turn`, where
/// traffic drives on `drives_on`.
fn level(priority_road: bool, turn: Turn, drives_on: traffic_sim::Side) -> u8 {
    let road = if priority_road { PRIORITY_ROAD } else { 0 };
    road + match turn.far_side(drives_on) {
        true => FAR_SIDE,
        false => STRAIGHT_OR_NEAR,
    }
}

/// A simulation lane from a built one.
fn build(
    b: &Built,
    speed_limit: f64,
    road: SimRoadId,
    what: impl Fn() -> String,
) -> Result<Lane, CompileError> {
    Lane::with_half_widths(&b.points, &b.half_widths, speed_limit, road)
        .map_err(|error| CompileError::Lane { what: what(), error })
}

/// A connector along `start`, from its lane's end into the intersection,
/// heading `dir` there; then `straight` on; then a quintic Bézier to `end`,
/// heading `end_dir` there, with no curvature at either end,
/// sampled by the same tolerance as roads; then along `tail`. It is `width`
/// wide, or by default as wide as the widest vehicle sweeps along it (see
/// [`swept`]), plus the room one needs to pass another on each side. With it,
/// the widest vehicle's body's sweep, run on past its ends as far as the
/// vehicle overhangs them.
fn connect(
    start: &[DVec2],
    dir: DVec2,
    straight: f64,
    end: DVec2,
    end_dir: DVec2,
    tail: &[DVec2],
    width: Option<f64>,
) -> (Built, Sweep) {
    let mouth = *start.last().expect("a connector starts somewhere");
    let p0 = mouth + straight * dir;
    // A quintic, its first two control points on from its start along its
    // direction and its last two back from its end along its, so it starts and
    // ends with no curvature: as the cubic with its control points a third of
    // the way across would be, raised to a quintic, its first and last
    // control points and directions the same, but with the next ones moved onto
    // its ends' lines. Gentler than that cubic at its sharpest, and a few
    // percent longer.
    let reach = p0.distance(end) / 3.0;
    let control = vec![
        p0,
        p0 + 0.6 * reach * dir,
        p0 + 1.2 * reach * dir,
        end - 1.2 * reach * end_dir,
        end - 0.6 * reach * end_dir,
        end,
    ];
    let (_, curve) = reference::sample_bezier(control);
    let mut points = start.to_vec();
    if straight > 0.0 {
        points.push(p0);
    }
    points.extend(&curve[1..]);
    points.extend(tail);
    // Its straight stretches in short segments, so its width can narrow
    // along them as a vehicle straightens.
    let points = densify(&points, SEGMENT);
    let s = arc_lengths(&points);
    // The widest vehicle: the only class so far. Swept along the connector
    // run on past each end as far as it overhangs its rear axle there, so the
    // sweep beyond its ends, while its nose is in the road it turns into, is
    // known too.
    let spec = VehicleSpec::passenger_car();
    let n = points.len();
    let (first, last) = (points[0], points[n - 1]);
    let (back, on) = (
        (points[1] - first).normalize(),
        (last - points[n - 2]).normalize(),
    );
    let steps = |length: f64| {
        (1..=(length / OVERHANG_STEP).ceil() as usize).map(|i| i as f64 * OVERHANG_STEP)
    };
    // Its front axle on the path: from its front bumper reaching the start to
    // its rear bumper leaving the end.
    let front_overhang = spec.length - spec.rear_axle - spec.wheelbase();
    let behind: Vec<DVec2> = steps(front_overhang).rev().map(|d| first - d * back).collect();
    let ahead: Vec<DVec2> =
        steps(spec.wheelbase() + spec.rear_axle).map(|d| last + d * on).collect();
    let extended: Vec<DVec2> = behind.iter().chain(&points).chain(&ahead).copied().collect();
    let s_extended: Vec<f64> = arc_lengths(&extended);
    let body = swept(&extended, &s_extended, &spec);
    let own = &body[behind.len()..behind.len() + n];
    let half_widths = match width {
        Some(width) => vec![[0.5 * width; 2]; s.len()],
        None => own.iter().map(|h| h.map(|h| h + traffic_sim::PASSING_CLEARANCE)).collect(),
    };
    // Allowing for how its course rounds the path's corners, and the steps its
    // rear axle trails in.
    let tracked = body.iter().map(|h| h.map(|h| h + TRACKING)).collect();
    let overhang = ahead.len();
    let sweep = Sweep {
        body: Built { points: extended, half_widths: tracked },
        overhang,
    };
    (Built { points, half_widths }, sweep)
}

/// A vehicle's body's sweep along a connector, run on past its ends, the last
/// `overhang` points of it past its end.
struct Sweep {
    body: Built,
    overhang: usize,
}

/// How far a vehicle's body may stray from its sweep, its front axle on the
/// path, in meters: its front axle's course rounds the path's corners by
/// millimetres, and its rear axle trails in steps of a step's travel rather
/// than continuously.
const TRACKING: f64 = 0.05;

/// The longest segment of a connector, in meters: its width is linear along
/// each, so a vehicle's sweep narrowing as it straightens needs them short.
const SEGMENT: f64 = 1.0;

/// `points` with segments longer than `longest` split evenly.
fn densify(points: &[DVec2], longest: f64) -> Vec<DVec2> {
    let mut out = vec![points[0]];
    for w in points.windows(2) {
        let n = (w[0].distance(w[1]) / longest).ceil().max(1.0) as usize;
        out.extend((1..=n).map(|i| w[0].lerp(w[1], i as f64 / n as f64)));
    }
    out
}

/// How finely a connector is run on past its ends to sweep a vehicle's
/// overhang there, in meters.
const OVERHANG_STEP: f64 = 0.25;

/// Each point's distance along `points`.
fn arc_lengths(points: &[DVec2]) -> Vec<f64> {
    std::iter::once(0.0)
        .chain(points.windows(2).scan(0.0, |s, w| {
            *s += w[0].distance(w[1]);
            Some(*s)
        }))
        .collect()
}

/// Whether `lane` is the lane of its road nearest the side a vehicle turning
/// `turn` into it turns toward, of those going its way: the leftmost, turning
/// left.
fn nearest(net: &RoadNetwork, lane: LaneRef, turn: Turn) -> bool {
    let road = &net.roads[lane.road.0];
    let me = &road.lanes[lane.lane];
    let forward = me.direction == Direction::Forward;
    // Going out, forward lanes start at the road's start, backward ones at its end.
    let s = if forward { 0.0 } else { f64::MAX };
    // How far left of its direction of travel each is.
    let left = |l: &crate::RoadLane| {
        if forward {
            l.offset.at(s)
        } else {
            -l.offset.at(s)
        }
    };
    let same = road.lanes.iter().filter(|l| l.direction == me.direction);
    match turn {
        Turn::Left => same.map(left).all(|x| x <= left(me)),
        Turn::Right => same.map(left).all(|x| x >= left(me)),
        Turn::Straight => true,
    }
}

/// Where the stop line of `from`'s road end crosses a connector from it, along
/// the connector, where it is safe to stop: where no vehicle crossing the line
/// along the lane could reach it sooner, the crossing of its centreline set
/// back by the lane's half-width times the tangent of the line's skew, but no
/// earlier than the connector's start, where its lane ends, at the crossing;
/// with the line itself, if it is skewed, which a vehicle stops by.
fn stop(
    net: &RoadNetwork,
    polylines: &[Polyline],
    ends: &[[Option<(usize, crate::Arm)>; 2]],
    from: LaneRef,
) -> (f64, Option<Line>) {
    let road = &net.roads[from.road.0];
    let lane = &road.lanes[from.lane];
    let at_end = lane.direction == Direction::Forward;
    let Some((_, arm)) = ends[from.road.0][at_end as usize] else {
        return (0.0, None);
    };
    let line = &polylines[from.road.0];
    let total = *line.s.last().unwrap();
    let s_end = if at_end { total } else { 0.0 };
    let (offset, half) = (lane.offset.at(s_end), 0.5 * lane.width.at(s_end));
    let cut = arms::cut(lane, total, &arm, at_end);
    let along = cut - arms::back(&arm, at_end, offset) - half * arm.skew.tan().abs();
    let stop = StopLine::new(line, &arm, at_end);
    let skewed = arm.skew != 0.0;
    (
        along.max(0.0),
        skewed.then_some(Line { point: stop.point, normal: stop.normal }),
    )
}

/// Everywhere a vehicle's body, sweeping along a connector or overhanging its
/// ends, would reach a road lane other than the ones the connector joins,
/// where vehicles on that lane can't see it, or overhang another connector
/// past its end.
fn check_sweeps(
    sim: &SimNetwork,
    road_lanes: &[Vec<LaneId>],
    connectors: &[Vec<LaneId>],
    bodies: &[Sweep],
) -> Vec<CompileError> {
    let mut failures = Vec::new();
    let bounds = |points: &mut dyn Iterator<Item = DVec2>| {
        points.fold((DVec2::MAX, DVec2::MIN), |(lo, hi), p| {
            (lo.min(p), hi.max(p))
        })
    };
    let road_bounds: Vec<Vec<_>> = road_lanes
        .iter()
        .map(|ids| ids.iter().map(|&id| bounds(&mut sim.lane(id).geometry().outline())).collect())
        .collect();
    let mut body = bodies.iter();
    for (k, ids) in connectors.iter().enumerate() {
        for (c, &id) in ids.iter().enumerate() {
            let sweep = body.next().expect("a body for each connector");
            let b = &sweep.body;
            // Lanes only for their geometry, never in the network, so of no road.
            let no_road = SimRoadId(u32::MAX);
            let swept = Lane::with_half_widths(&b.points, &b.half_widths, 1.0, no_road)
                .expect("a connector's sweep is a valid lane");
            let geometry = swept.geometry();
            let (lo, hi) = bounds(&mut geometry.outline());
            let own = sim.lane(id);
            // Past its end, its nose overhangs the road it goes into, while
            // still turning: onto no lane there but its own, junction lane or
            // road lane.
            let end = b.points.len() - sweep.overhang;
            let ahead = Lane::with_half_widths(
                &b.points[end - 1..],
                &b.half_widths[end - 1..],
                1.0,
                no_road,
            )
            .expect("an overhang is a valid lane");
            // Those going into the same lane overhang it too, and end there.
            let others = ids
                .iter()
                .filter(|&&other| other != id && sim.lane(other).successors() != own.successors());
            let overhung =
                others.filter_map(|&other| ahead.geometry().overlap(sim.lane(other).geometry()));
            if let Some(beyond) = overhung.map(|r| r[0].end).reduce(f64::max) {
                failures.push(CompileError::Overhangs { intersection: k, connection: c, beyond });
            }
            for (r, lanes) in road_lanes.iter().enumerate() {
                for (i, &lane) in lanes.iter().enumerate() {
                    let (llo, lhi) = road_bounds[r][i];
                    let apart = (hi.cmplt(llo) | lhi.cmplt(lo)).any();
                    let joined =
                        own.predecessors().contains(&lane) || own.successors().contains(&lane);
                    if apart || joined {
                        continue;
                    }
                    if let Some([_, theirs]) = geometry.overlap(sim.lane(lane).geometry()) {
                        let length = sim.lane(lane).geometry().length();
                        let lane = LaneRef { road: RoadId(r), lane: i };
                        let reach = [length - theirs.start, theirs.end];
                        failures.push(CompileError::Sweeps {
                            intersection: k,
                            connection: c,
                            lane,
                            reach,
                        });
                    }
                }
            }
        }
    }
    failures
}

/// How far out to each side of `points`, at arc lengths `s`, a vehicle like
/// `spec` sweeps along them, at each vertex, as road design's swept paths have
/// it: driven as the simulation drives it, its front axle along the path and
/// its rear axle trailing, from straight along the path's first segment, at
/// each vertex its front axle reaches each corner is projected onto the nearest
/// segment, the first and last running on as lines, widening both its ends to
/// reach it. Never less than half its width. On a bend the rear axle cuts
/// inside the front's course, `R − √(R² − L²)` for a wheelbase `L` and a
/// radius `R`, and the outer front corner swings out a little; a turn's mouth,
/// which bends little, stays narrow. Then tapered, so it never widens faster
/// than [`TAPER`] along it.
fn swept(points: &[DVec2], s: &[f64], spec: &traffic_sim::VehicleSpec) -> Vec<[f64; 2]> {
    let mut half = vec![[0.5 * spec.width; 2]; points.len()];
    let (front, rear, side) = (
        spec.length - spec.rear_axle,
        spec.rear_axle,
        0.5 * spec.width,
    );
    let wheelbase = spec.wheelbase();
    let mut dir = (points[1] - points[0]).normalize();
    let mut axle = points[0] - wheelbase * dir;
    for (i, &at) in points.iter().enumerate() {
        // Its front axle on from the last vertex to this one, in short steps.
        if i > 0 {
            let from = points[i - 1];
            let n = (from.distance(at) / TRAIL_STEP).ceil().max(1.0) as usize;
            for k in 1..=n {
                let front_axle = from.lerp(at, k as f64 / n as f64);
                (axle, dir) = traffic_sim::trail(axle, dir, front_axle, wheelbase);
            }
        }
        let corners = [front, -rear]
            .map(|along| [side, -side].map(|across| axle + along * dir + across * dir.perp()));
        for corner in corners.into_iter().flatten() {
            // The nearest segment, among those within the body's reach.
            let r = front + rear + side;
            let first = s.partition_point(|&x| x < s[i] - r).saturating_sub(1);
            let last = s.partition_point(|&x| x <= s[i] + r).min(points.len() - 1);
            let nearest = (first..last).map(|j| {
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
    // Tapered, never narrowing faster than TAPER along it either way, so its
    // edges don't step out where a bend begins abruptly.
    for i in 1..points.len() {
        let taper = TAPER * (s[i] - s[i - 1]);
        half[i] = [0, 1].map(|e| half[i][e].max(half[i - 1][e] - taper));
    }
    for i in (0..points.len() - 1).rev() {
        let taper = TAPER * (s[i + 1] - s[i]);
        half[i] = [0, 1].map(|e| half[i][e].max(half[i + 1][e] - taper));
    }
    half
}

/// The longest step a vehicle's front axle takes along a connector as its sweep
/// is found, in meters: about a step's travel at a connector's speed.
const TRAIL_STEP: f64 = 0.25;

/// How fast a connector's swept width may change along it: 1 m sideways in 4 m.
const TAPER: f64 = 0.25;
