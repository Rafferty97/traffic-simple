//! Compiling a road network to the simulation's lanes, and to render geometry.

use std::fmt;

use glam::DVec2;
use traffic_sim::{
    AMBER_DECEL, Barrier, BarrierKind, Keep, Lane, LaneError, LaneId, Line, Movement, MovementId,
    NetworkError, Phase, SAME_POINT, SignalPlan, SimNetwork, VehicleSpec,
};

use crate::arms::{self, StopLine};
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

    /// It cut in two at `at` along it, inside it: at the point there, its
    /// half-widths between the vertices either side, or at the vertex before
    /// it if `at` is within [`SPLIT_SNAP`] of either, so that neither piece has
    /// a segment too short to build.
    fn split(&self, at: f64) -> [Built; 2] {
        let s = arc_lengths(&self.points);
        let i = s.partition_point(|&x| x <= at).clamp(1, s.len() - 1) - 1;
        let (a, b) = (s[i], s[i + 1]);
        let (cut, head, tail) = if at - a < SPLIT_SNAP || b - at < SPLIT_SNAP {
            let k = i.max(1);
            (k, k, k)
        } else {
            (usize::MAX, i + 1, i + 1)
        };
        let (point, half) = match cut {
            usize::MAX => {
                let t = (at - a) / (b - a);
                let [l, r] = [0, 1].map(|e| {
                    let h = &self.half_widths;
                    h[i][e] + t * (h[i + 1][e] - h[i][e])
                });
                (self.points[i].lerp(self.points[i + 1], t), [l, r])
            }
            k => (self.points[k], self.half_widths[k]),
        };
        let first = Built {
            points: self.points[..head].iter().copied().chain([point]).collect(),
            half_widths: self.half_widths[..head].iter().copied().chain([half]).collect(),
        };
        let second = Built {
            points: [point].into_iter().chain(self.points[tail..].iter().copied()).collect(),
            half_widths: [half]
                .into_iter()
                .chain(self.half_widths[tail..].iter().copied())
                .collect(),
        };
        // At a vertex, it starts the second piece and ends the first.
        let second = match cut {
            usize::MAX => second,
            _ => Built {
                points: second.points[1..].to_vec(),
                half_widths: second.half_widths[1..].to_vec(),
            },
        };
        [first, second]
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
    let (compiled, mut swept) = compile_swept(net)?;
    match swept.is_empty() {
        true => Ok(compiled),
        false => Err(swept.swap_remove(0)),
    }
}

/// As [`compile`], with every sweep that fails it, which only fails it once
/// it has compiled otherwise.
///
/// Where a connector gets a barrier inside its junction, a far-side turn's
/// give-way line, it is compiled again split there into two lanes, the second
/// starting at the barrier: so a vehicle is handed over to the junction a lane
/// at a time, each lane's conflicts its own (see `docs/DESIGN.md`, Barriers
/// inside a junction). Its sweep is checked whole, in the first pass.
fn compile_swept(net: &RoadNetwork) -> Result<(Compiled, Vec<CompileError>), CompileError> {
    let (mut compiled, swept, mut waiting) = assemble(net, &[])?;
    if !swept.is_empty() {
        return Ok((compiled, swept));
    }
    // Split where each waits, and again further back while the lane before
    // it still meets traffic it gives way to, as a sliver at its end can.
    let mut splits: Vec<Vec<Option<f64>>> =
        compiled.connectors.iter().map(|ids| vec![None; ids.len()]).collect();
    for _ in 0..SPLIT_PASSES {
        if waiting.is_empty() {
            return Ok((compiled, swept));
        }
        for (k, c, at) in waiting {
            if at <= SPLIT_MARGIN {
                return Err(CompileError::Waiting { intersection: k, connection: c });
            }
            splits[k][c] = Some(at);
        }
        (compiled, _, waiting) = assemble(net, &splits)?;
    }
    match waiting.first() {
        None => Ok((compiled, swept)),
        Some(&(intersection, connection, _)) => {
            Err(CompileError::Waiting { intersection, connection })
        }
    }
}

/// How many times [`compile_swept`] splits a connector again before giving up.
const SPLIT_PASSES: usize = 8;

/// How near a vertex of a connector a split goes to it instead, in meters.
const SPLIT_SNAP: f64 = 0.05;

/// How far into a connector its barrier must be for it to be split there, in
/// meters: one nearer its start stays where it is, at the start, in effect.
const SPLIT_MARGIN: f64 = 0.01;

/// [`compile_swept`]'s passes: each connector split where `splits` says, if it
/// says, and checked for its sweep if none is. With it, each connector that
/// should be split where it waits, further back if it is already, by
/// intersection, connection and how far along it.
#[allow(clippy::type_complexity)]
fn assemble(
    net: &RoadNetwork,
    splits: &[Vec<Option<f64>>],
) -> Result<(Compiled, Vec<CompileError>, Vec<(usize, usize, f64)>), CompileError> {
    let mut lanes: Vec<Lane> = Vec::new();
    let mut built: Vec<Built> = Vec::new();
    let mut road_lanes = Vec::new();
    let mut polylines = Vec::new();
    // Each road lane's stretches that belong to an intersection: the one before
    // it, from the intersection to its start, and the one after it, from its
    // end into the intersection, in its direction of travel.
    let mut stretches: Vec<[Option<Vec<DVec2>>; 2]> = Vec::new();
    let ends = arms::road_ends(net);

    for (r, road) in net.roads.iter().enumerate() {
        let profiles: Vec<_> = road.lanes.iter().flat_map(|l| [l.offset, l.width]).collect();
        let total =
            *Polyline::new(road.start, &road.pieces, &profiles, [&[], &[]]).s.last().unwrap();
        // How far back from each end its lanes coming in and going out are cut.
        let cut = ends[r].map(|end| end.map(|(_, arm)| arm));
        let cuts = [false, true].map(|at_end| {
            cut[at_end as usize].map_or([0.0; 2], |arm| arms::cuts(road, total, &arm, at_end))
        });
        let kept = |c: [f64; 2]| c.into_iter().filter(|&d| d > 0.0).collect::<Vec<_>>();
        let line = Polyline::new(
            road.start,
            &road.pieces,
            &profiles,
            [&kept(cuts[0]), &kept(cuts[1])],
        );
        let mut ids = Vec::new();
        for (i, lane) in road.lanes.iter().enumerate() {
            let mut points = line.offset(&lane.offset);
            let mut half_widths: Vec<[f64; 2]> =
                line.s.iter().map(|&s| [0.5 * lane.width.at(s); 2]).collect();
            // Where it is cut at each end, coming in or going out.
            let at = |at_end: bool| cuts[at_end as usize][!arms::arriving(lane, at_end) as usize];
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
            let built_lane = build(&b, road.speed_limit, what)?;
            lanes.push(built_lane.with_keep(keep).with_leaving(lane.leaving));
            ids.push(LaneId(built.len() as u32));
            built.push(b);
        }
        road_lanes.push(ids);
        polylines.push(line);
    }

    let mut connectors = Vec::new();
    // Each connection's lanes, more than one where it is split.
    let mut pieces = Vec::new();
    let mut turns = Vec::new();
    let mut stops = Vec::new();
    let mut bodies = Vec::new();
    let mut links = Vec::new();
    for (k, intersection) in net.intersections.iter().enumerate() {
        let mut ids = Vec::new();
        let mut parts = Vec::new();
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
            lines.push(stop(net, &polylines, &ends, from));
            bodies.push(body);
            let limit = net.roads[from.road.0].speed_limit.min(net.roads[to.road.0].speed_limit);
            let what = || format!("intersection {k} connection {c}");
            let limit = turn_limit(&connector.points, limit);
            let split = splits.get(k).and_then(|s| s[c]);
            let mut lanes_of = Vec::new();
            for piece in match split {
                Some(at) => connector.split(at).into(),
                None => vec![connector],
            } {
                lanes.push(build(&piece, limit, what)?);
                lanes_of.push(LaneId(built.len() as u32));
                built.push(piece);
            }
            let ends = [lane_id(from)].into_iter().chain(lanes_of.iter().copied());
            links.extend(ends.zip(lanes_of.iter().copied().chain([lane_id(to)])));
            ids.push(lanes_of[0]);
            parts.push(lanes_of);
        }
        connectors.push(ids);
        pieces.push(parts);
        turns.push(ways);
        stops.push(lines);
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

    for parts in &pieces {
        let ids: Vec<LaneId> = parts.iter().flatten().copied().collect();
        if !ids.is_empty() {
            sim.add_junction(&ids);
        }
    }
    let mut waiting = Vec::new();
    for (k, intersection) in net.intersections.iter().enumerate() {
        if let Control::Signals(phases) = &intersection.control {
            let waits = add_signals(
                net,
                &mut sim,
                intersection,
                &pieces[k],
                &turns[k],
                &stops[k],
                phases,
            );
            waiting.extend(waits.into_iter().map(|(c, at)| (k, c, at)));
        }
    }
    sim.validate().map_err(CompileError::Network)?;
    let swept = match splits.is_empty() {
        true => check_sweeps(&sim, &road_lanes, &connectors, &bodies),
        false => Vec::new(),
    };

    let render = render::build(net, &polylines, &turns);
    Ok((
        Compiled { sim, render, road_lanes, connectors, turns },
        swept,
        waiting,
    ))
}

/// What [`fit_arms`] rounds how far it moves a stop line or lengthens a run
/// out up to, in meters.
const FIT_STEP: f64 = 0.25;

/// Compiles `net`, first moving stop lines back and lengthening run outs
/// wherever a vehicle's body turning along a connector would sweep onto a road
/// lane: onto one coming in, its stop line goes back as far as the sweep
/// reaches along it; onto one going out, or past the connector's end onto
/// another, its run out grows as far, each rounded up to [`FIT_STEP`]. So road design sets stop lines back
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
/// A far-side turn's give-way barrier goes at the start of its second lane,
/// if it is split in two, or where it first meets traffic it gives way to.
/// Returned, each connection whose first lane meets such traffic, and where:
/// one to split there.
fn add_signals(
    net: &RoadNetwork,
    sim: &mut SimNetwork,
    intersection: &Intersection,
    pieces: &[Vec<LaneId>],
    turns: &[Turn],
    stops: &[(f64, Option<Line>)],
    phases: &[SignalPhase],
) -> Vec<(usize, f64)> {
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
                    pieces.iter().zip(of).filter(move |&(_, &o)| o == m).map(|(p, _)| p)
                };
                let mine = || of_movement(i).flatten().copied();
                let conflicts = (0..movements.len())
                    .filter(|&j| j != i && !together(i, j))
                    .filter(|&j| {
                        let theirs: Vec<LaneId> = of_movement(j).flatten().copied().collect();
                        mine().any(|a| theirs.iter().any(|&b| sim.conflict_between(a, b).is_some()))
                    })
                    .fold(0u64, |mask, j| mask | (1 << j));
                let speed = net.roads[movements[i].0.0].speed_limit;
                let length = |p: &Vec<LaneId>| {
                    p.iter().map(|&c| sim.lane(c).geometry().length()).sum::<f64>()
                };
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
    for ((piece, &m), &(s, line)) in pieces.iter().zip(&of).zip(stops) {
        let movement = MovementId(first.0 + m as u32);
        let kind = BarrierKind::Signal { movement };
        sim.add_barrier(piece[0], Barrier { line, ..Barrier::new(s, kind) });
    }
    // Every road has priority at signals: a far-side turn gives way to the
    // traffic it crosses, waiting inside the junction at its first conflict with
    // a connector of a higher level green with it.
    let drives_on = net.drives_on.side();
    let level = |turn: Turn| match turn.far_side(drives_on) {
        true => PRIORITY_ROAD + FAR_SIDE,
        false => PRIORITY_ROAD + STRAIGHT_OR_NEAR,
    };
    for (piece, &turn) in pieces.iter().zip(turns) {
        for &lane in piece {
            sim.set_priority(lane, level(turn));
        }
    }
    let mut waiting = Vec::new();
    for (c, (piece, &turn)) in pieces.iter().zip(turns).enumerate() {
        if !turn.far_side(drives_on) {
            continue;
        }
        let connector = piece[0];
        let waits = pieces
            .iter()
            .zip(turns)
            .enumerate()
            .filter(|&(o, (_, &other))| level(other) > level(turn) && together(of[c], of[o]))
            .flat_map(|(_, (other, _))| other)
            .filter_map(|&other| sim.conflict_between(connector, other))
            .map(|conflict| conflict.mine.start)
            .fold(f64::INFINITY, f64::min);
        match piece[..] {
            // Split already: it waits at its second lane's start.
            [_, inner] => {
                sim.add_barrier(inner, Barrier::new(0.0, BarrierKind::GiveWay));
                if waits.is_finite() {
                    waiting.push((c, waits));
                }
            }
            _ if waits.is_finite() => {
                sim.add_barrier(connector, Barrier::new(waits, BarrierKind::GiveWay));
                if waits > SPLIT_MARGIN {
                    waiting.push((c, waits));
                }
            }
            _ => {}
        }
    }
    waiting
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

/// A connector along `start`, from its lane's end into the intersection,
/// heading `dir` there; then `straight` on; then a cubic Bézier to `end`,
/// heading `end_dir` there, its control points a third of the way across,
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
    let reach = p0.distance(end) / 3.0;
    let (c1, c2) = (p0 + reach * dir, end - reach * end_dir);
    let pose = Pose { pos: p0, heading: dir.to_angle() };
    let (_, curve) = reference::sample_bezier(pose, c1, c2, end);
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
    let behind: Vec<DVec2> = steps(spec.rear_axle).rev().map(|d| first - d * back).collect();
    let ahead: Vec<DVec2> = steps(spec.length - spec.rear_axle).map(|d| last + d * on).collect();
    let extended: Vec<DVec2> = behind.iter().chain(&points).chain(&ahead).copied().collect();
    let s_extended: Vec<f64> = arc_lengths(&extended);
    let body = swept(&extended, &s_extended, &spec);
    let own = &body[behind.len()..behind.len() + n];
    let half_widths = match width {
        Some(width) => vec![[0.5 * width; 2]; s.len()],
        None => own.iter().map(|h| h.map(|h| h + traffic_sim::PASSING_CLEARANCE)).collect(),
    };
    // Allowing for steering: pure pursuit keeps a vehicle within TRACKING
    // of the path its rear axle is swept along.
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

/// How far a vehicle's body strays from its ideal sweep, its rear axle on the
/// path, as pure pursuit steers it, in meters: at most 6 cm on the crossroads
/// scenarios' connectors, at their limits and at 2 m/s, where a turn finishing
/// just into the road it goes into is still straightening.
const TRACKING: f64 = 0.1;

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

/// Where the stop line of `from`'s road end crosses a connector from it,
/// along the connector: where no vehicle crossing it along the lane could reach
/// it sooner, the crossing of its centreline set back by the lane's half-width
/// times the tangent of the line's skew; with the line itself, if it is skewed.
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
    let cut = arms::cuts(road, total, &arm, at_end)[0];
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
            let swept = Lane::with_half_widths(&b.points, &b.half_widths, 1.0)
                .expect("a connector's sweep is a valid lane");
            let geometry = swept.geometry();
            let (lo, hi) = bounds(&mut geometry.outline());
            let own = sim.lane(id);
            // Past its end, its nose overhangs the road it goes into, while
            // still turning: onto no lane there but its own, junction lane or
            // road lane.
            let end = b.points.len() - sweep.overhang;
            let ahead =
                Lane::with_half_widths(&b.points[end - 1..], &b.half_widths[end - 1..], 1.0)
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
/// it: with its rear axle at each vertex in turn, heading along the path, each
/// corner is projected onto the nearest segment, the first and last running on
/// as lines, widening both its ends to reach it. Never less than half its width. On a bend the outer front corner
/// swings out, `√(f² + (R + w/2)²) − R` for a front `f` ahead of the rear axle
/// and a radius `R`, and the inner rear corner cuts in a little; a turn's mouth,
/// which bends little, stays narrow. Then tapered, so it never widens faster
/// than [`TAPER`] along it: a cubic bends from its very start, so the envelope
/// would step out there.
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

/// How fast a connector's swept width may change along it: 1 m sideways in 4 m.
const TAPER: f64 = 0.25;

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
