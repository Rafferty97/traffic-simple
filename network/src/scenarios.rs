//! Example networks, built at road level.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use traffic_sim::{Keep, Leaving};

use crate::{
    Arm, CompileError, Control, Direction, HookGeometry, Intersection, KeepClear, LaneRef, Piece,
    Pose, Profile, Road, RoadId, RoadLane, RoadNetwork, Sign, SignalPhase, Turn,
};

const WIDTH: f64 = 3.5;

/// How far each road of a crossroads reaches from its centre, in meters: room
/// for long queues, and plenty of vehicles at once.
const ARM: f64 = 300.0;

/// How far right turns run straight on into a crossroads before turning, in
/// meters, as drivers pull in to wait for a gap.
const RIGHT_TURN_STRAIGHT: f64 = 3.0;

/// Two two-way roads crossing at right angles, each with `lanes` lanes each
/// way, 600 m long, driving on the left. Every lane coming in connects to the
/// lane at the same place across the junction, straight on; only the leftmost
/// turns left and only the rightmost turns right, into every lane of the road
/// it turns into, running 3 m straight on before it turns right. Its stop lines are set back, and its run outs as long, as
/// turning vehicles need (see [`crate::fit_arms`]).
pub fn crossroads(lanes: usize, speed_limit: f64) -> RoadNetwork {
    oblique_crossroads(lanes, speed_limit, FRAC_PI_2)
}

/// As [`crossroads`], the roads crossing at `angle`, between 0 and π: the
/// second road runs at `angle` counterclockwise from the first. Each arm ends
/// where its lanes are clear of the other road, and its stop line runs along
/// the other road, skewed from square by as much as the roads are from
/// crossing square.
pub fn oblique_crossroads(lanes: usize, speed_limit: f64, angle: f64) -> RoadNetwork {
    junction(
        [lanes; 2],
        speed_limit,
        angle,
        [true; 4],
        0.0,
        JUNCTION_ROOM,
        |_| {},
    )
}

/// A T-junction as [`crossroads`] has it, without its north arm: the first
/// road, with `lanes[0]` lanes each way, runs east to west, roads 0 and 1, and
/// the second, with `lanes[1]`, comes in from the south, road 2.
pub fn t_junction(lanes: [usize; 2], speed_limit: f64) -> RoadNetwork {
    junction(
        lanes,
        speed_limit,
        FRAC_PI_2,
        [true, false, true, true],
        0.0,
        JUNCTION_ROOM,
        |_| {},
    )
}

/// As [`oblique_crossroads`], the first road with `lanes[0]` lanes each way
/// and the second with `lanes[1]`, and only the arms `present` says, by where
/// they come in from: the first road's east end, the second's far end, the
/// first's west end and the second's near end. Roads are numbered in that
/// order, the arms left out skipped. Each road's lanes each way are `median`
/// from its centre line, leaving room between them, and `before_fitting`
/// finishes the intersection before its arms are fitted, as how it is
/// controlled shapes its connectors. Each arm ends `room` beyond where its
/// lanes clear the other road.
#[allow(clippy::too_many_arguments)]
fn junction(
    lanes: [usize; 2],
    speed_limit: f64,
    angle: f64,
    present: [bool; 4],
    median: f64,
    room: f64,
    before_fitting: impl FnOnce(&mut Intersection),
) -> RoadNetwork {
    try_junction(
        lanes,
        speed_limit,
        angle,
        present,
        median,
        room,
        before_fitting,
    )
    .expect("the junction compiles once fitted")
}

/// As [`junction`], or why it doesn't compile.
#[allow(clippy::too_many_arguments)]
fn try_junction(
    lanes: [usize; 2],
    speed_limit: f64,
    angle: f64,
    present: [bool; 4],
    median: f64,
    room: f64,
    before_fitting: impl FnOnce(&mut Intersection),
) -> Result<RoadNetwork, CompileError> {
    let mut net = RoadNetwork::default();
    // Each arm's road, the first or the second, and its lanes each way.
    let road_of = |arm: usize| arm % 2;
    let lanes_of = |arm: usize| lanes[road_of(arm)];
    // How far across each road's lanes reach from its centre line.
    let across = lanes.map(|n| median + n as f64 * WIDTH);
    // The junction's half-width along each road's arms, from its centre to
    // where the arm ends: where its outermost lane clears the other road, and
    // `room` more.
    let half = [0, 1].map(|r| across[1 - r] / angle.sin() + across[r] / angle.tan().abs() + room);
    // Each arm runs in toward the centre, from the first road's east end, the
    // second's far end, the first's west end and the second's near end, so
    // counterclockwise, its forward lanes coming in.
    let from = [0.0, angle, PI, PI + angle];
    let arms: Vec<Option<RoadId>> = (0..4)
        .map(|k| {
            let half = half[road_of(k)];
            let end = Pose {
                pos: half * Pose::new(0.0, 0.0, from[k]).dir(),
                heading: from[k] + PI,
            };
            let pieces = vec![Piece::Line { length: ARM - half }];
            let lanes = two_way_apart(lanes_of(k), median);
            let road = || Road::ending_at(end, pieces, lanes, speed_limit);
            present[k].then(|| net.add_road(road()))
        })
        .collect();
    let ways: [Option<Way>; 4] =
        std::array::from_fn(|k| arms[k].map(|road| Way::at_end(road, lanes_of(k))));
    let mut intersection = Intersection {
        far_side_straight: RIGHT_TURN_STRAIGHT,
        ..Intersection::default()
    };
    connect(&mut intersection, &ways);
    for (k, &road) in arms.iter().enumerate() {
        let Some(road) = road else { continue };
        // Its stop line along the other road: as far from square as the other
        // road is from crossing square, counterclockwise from the way in, and
        // back far enough that none of it is past the arm's end.
        // That is the angle between the roads on the first road's arms, and its
        // supplement on the second's.
        let crossing = if k % 2 == 0 { angle } else { PI - angle };
        let skew = crossing - FRAC_PI_2;
        let stop_line = (-across[road_of(k)] * skew.tan()).max(0.0);
        intersection.arms.push(Arm { stop_line, skew, ..Arm::new(road) });
    }
    before_fitting(&mut intersection);
    net.intersections.push(intersection);
    // Stop lines set back, and run outs, wherever turning vehicles sweep.
    crate::fit_arms(&mut net)?;
    Ok(net)
}

/// How a road meets a crossroads: its lanes coming in and going out, each
/// from the centre line outward.
struct Way {
    inbound: Vec<LaneRef>,
    outbound: Vec<LaneRef>,
}

impl Way {
    /// A road of `n` lanes each way, laid out as [`two_way`] has it, meeting
    /// the intersection at its end: its forward lanes come in.
    fn at_end(road: RoadId, n: usize) -> Self {
        let lane = |lane| LaneRef { road, lane };
        Way {
            inbound: (0..n).map(lane).collect(),
            outbound: (n..2 * n).map(lane).collect(),
        }
    }

    /// As [`Way::at_end`], meeting it at its start: its backward lanes come in.
    fn at_start(road: RoadId, n: usize) -> Self {
        let Way { inbound, outbound } = Way::at_end(road, n);
        Way { inbound: outbound, outbound: inbound }
    }
}

/// `n` lanes each way: forward lanes on the left of the reference line,
/// backward on its right, lane 0 of each by it, as driving on the left has it.
fn two_way(n: usize) -> Vec<RoadLane> {
    two_way_apart(n, 0.0)
}

/// As [`two_way`], each way's lanes `median` from the centre line.
fn two_way_apart(n: usize, median: f64) -> Vec<RoadLane> {
    let offset = |i: usize| median + WIDTH * (i as f64 + 0.5);
    (0..n)
        .map(|i| RoadLane::new(offset(i), WIDTH, Direction::Forward))
        .chain((0..n).map(|i| RoadLane::new(-offset(i), WIDTH, Direction::Backward)))
        .collect()
}

/// Connects the ways into a crossroads, `ways` counterclockwise, any of them
/// missing. Every lane coming in connects to the lane at the same place across
/// it, straight on; only the leftmost turns left and only the rightmost turns
/// right, into every lane of the road it turns into, or, from a road with
/// fewer lanes, into as many as it has, nearest the side it turns toward: by
/// the kerb turning near-side, by the centre line far-side.
fn connect(intersection: &mut Intersection, ways: &[Option<Way>; 4]) {
    for (k, way) in ways.iter().enumerate() {
        let Some(way) = way else { continue };
        // Ways counterclockwise from this one: right, straight on, left.
        let (right, straight, left) = (
            ways[(k + 1) % 4].as_ref(),
            ways[(k + 2) % 4].as_ref(),
            ways[(k + 3) % 4].as_ref(),
        );
        let n = way.inbound.len();
        let mut push = |from: LaneRef, to: LaneRef| intersection.connections.push((from, to));
        if let Some(straight) = straight {
            for i in 0..n.min(straight.outbound.len()) {
                push(way.inbound[i], straight.outbound[i]);
            }
        }
        if let Some(left) = left {
            let m = left.outbound.len();
            for &to in &left.outbound[m - m.min(n)..] {
                push(way.inbound[n - 1], to);
            }
        }
        if let Some(right) = right {
            for &to in &right.outbound[..right.outbound.len().min(n)] {
                push(way.inbound[0], to);
            }
        }
    }
}

/// As [`crossroads`], signalised: a fixed-time plan, the roads from the east
/// and west green for 30 s, every way, then those from the north and south. A
/// right turn shares its phase with the traffic coming the other way.
pub fn signalised_crossroads(lanes: usize, speed_limit: f64) -> RoadNetwork {
    signalised(crossroads(lanes, speed_limit))
}

/// How far a crossroads' arms end beyond where their lanes clear the other
/// road, in meters.
const JUNCTION_ROOM: f64 = 6.0;

/// As [`JUNCTION_ROOM`], for [`hook_turn_crossroads`]: room for a hook turner
/// to move over and wait, facing the way it came in, short of the road across
/// it, as Melbourne's wide streets have.
const HOOK_ROOM: f64 = 8.0;

/// Half the room down the middle of a Melbourne road left for its tram tracks,
/// in meters: two tracks, each way's lanes beside them.
const TRAM_ROOM: f64 = 5.0;

/// A Melbourne crossroads, after William and Collins Streets, made symmetric:
/// two two-way roads crossing at right angles, one lane each way, with room
/// down the middle of each for tram tracks, signalised as
/// [`signalised_crossroads`] is, every approach's right turns hook turns.
/// Each lane coming in goes straight on, turns left, and turns right by a hook
/// turn: on into the intersection, waiting on the left in front of the road
/// it will follow into the road it turns into, until that road's light goes
/// green. No trams yet: the room for them is empty.
pub fn hook_turn_crossroads(speed_limit: f64) -> RoadNetwork {
    hook_turn_crossroads_with(speed_limit, HookLayout::default())
        .expect("the hook-turn crossroads compiles once fitted")
}

/// How [`hook_turn_crossroads_with`] lays out its crossroads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HookLayout {
    /// How its hook turns are laid out.
    pub geometry: HookGeometry,
    /// Half the room down the middle of each road for tram tracks, in meters.
    pub tram_room: f64,
    /// How far each arm ends beyond where its lanes clear the other road, in
    /// meters.
    pub room: f64,
}

impl Default for HookLayout {
    fn default() -> Self {
        Self {
            geometry: HookGeometry::default(),
            tram_room: TRAM_ROOM,
            room: HOOK_ROOM,
        }
    }
}

/// As [`hook_turn_crossroads`], laid out as `layout` says; or why it doesn't
/// compile so, as when a hook turn is too tight for its connector to bend.
pub fn hook_turn_crossroads_with(
    speed_limit: f64,
    layout: HookLayout,
) -> Result<RoadNetwork, CompileError> {
    let roads = [0, 1, 2, 3].map(RoadId);
    try_junction(
        [1, 1],
        speed_limit,
        FRAC_PI_2,
        [true; 4],
        layout.tram_room,
        layout.room,
        |intersection| {
            intersection.hook_turns = roads.to_vec();
            intersection.hook_geometry = layout.geometry;
            intersection.control = signal_plan(roads, [30.0; 2]);
        },
    )
}

/// As [`signalised_crossroads`], with near-side turns allowed on red: each
/// comes to rest at its line, then goes through a gap.
pub fn signalised_crossroads_turn_on_red(lanes: usize, speed_limit: f64) -> RoadNetwork {
    let mut net = signalised_crossroads(lanes, speed_limit);
    net.turn_on_red = true;
    net
}

/// How far back from the end of the west arm of
/// [`signalised_crossroads_keep_clear`] its keep-clear zone starts and ends,
/// in meters.
pub const KEEP_CLEAR_BACK: [f64; 2] = [60.0, 45.0];

/// As [`signalised_crossroads`], with a keep-clear zone across the lanes
/// coming in from the west, 45 to 60 m back from the end of the road, as if
/// across a side road's mouth: the queue at a red light leaves it empty.
pub fn signalised_crossroads_keep_clear(lanes: usize, speed_limit: f64) -> RoadNetwork {
    let mut net = signalised_crossroads(lanes, speed_limit);
    let west = &mut net.roads[2];
    let length = west.length();
    west.keep_clear.push(KeepClear {
        from: length - KEEP_CLEAR_BACK[0],
        to: length - KEEP_CLEAR_BACK[1],
        direction: Direction::Forward,
    });
    net
}

/// As [`oblique_crossroads`], signalised as [`signalised_crossroads`] is.
pub fn signalised_oblique_crossroads(lanes: usize, speed_limit: f64, angle: f64) -> RoadNetwork {
    signalised(oblique_crossroads(lanes, speed_limit, angle))
}

/// As [`crossroads`], an all-way stop: every approach stops at its line, and
/// vehicles go in the order they came to rest there.
pub fn all_way_stop_crossroads(lanes: usize, speed_limit: f64) -> RoadNetwork {
    let mut net = crossroads(lanes, speed_limit);
    net.intersections[0].control = Control::AllWayStop;
    net
}

/// A crossroads as [`crossroads`] has it, controlled by priority: the first
/// road, east to west, with `lanes[0]` lanes each way, has it, and the
/// second, with `lanes[1]`, has `sign` on its approaches.
pub fn priority_crossroads(lanes: [usize; 2], speed_limit: f64, sign: Sign) -> RoadNetwork {
    let mut net = junction(
        lanes,
        speed_limit,
        FRAC_PI_2,
        [true; 4],
        0.0,
        JUNCTION_ROOM,
        |_| {},
    );
    prioritise(
        &mut net,
        &[RoadId(0), RoadId(2)],
        &[RoadId(1), RoadId(3)],
        sign,
    );
    net
}

/// As [`t_junction`], controlled by priority: the road through, east to west,
/// with `lanes[0]` lanes each way, has it, and the road from the south, with
/// `lanes[1]`, has `sign`.
pub fn priority_t_junction(lanes: [usize; 2], speed_limit: f64, sign: Sign) -> RoadNetwork {
    let mut net = t_junction(lanes, speed_limit);
    prioritise(&mut net, &[RoadId(0), RoadId(1)], &[RoadId(2)], sign);
    net
}

/// Gives `net`'s intersection priority roads, `major`, and `sign` on `minor`.
fn prioritise(net: &mut RoadNetwork, major: &[RoadId], minor: &[RoadId], sign: Sign) {
    net.intersections[0].control = Control::Priority {
        major: major.to_vec(),
        signs: minor.iter().map(|&r| (r, sign)).collect(),
    };
}

/// `net`, a crossroads, with a fixed-time plan: its first road green for
/// 30 s, every way, then its second.
fn signalised(mut net: RoadNetwork) -> RoadNetwork {
    net.intersections[0].control = signal_plan([0, 1, 2, 3].map(RoadId), [30.0; 2]);
    net
}

/// A crossroads' fixed-time plan, its ways' roads counterclockwise: the first
/// and third green for `durations[0]` seconds, every way, then the second and
/// fourth for `durations[1]`.
fn signal_plan(roads: [RoadId; 4], durations: [f64; 2]) -> Control {
    let every_way = |arms: [usize; 2]| {
        arms.iter()
            .flat_map(|&a| [Turn::Straight, Turn::Left, Turn::Right].map(|t| (roads[a], t)))
            .collect()
    };
    Control::Signals(vec![
        SignalPhase { green: every_way([0, 2]), duration: durations[0] },
        SignalPhase { green: every_way([1, 3]), duration: durations[1] },
    ])
}

/// The phases' durations of the `k`th signalised crossroads of a [`grid`]: each
/// 30 s give or take up to 5 s, in whole seconds, the same for the same `k`.
/// Their cycles differ, so neighbouring crossroads drift in and out of step
/// rather than changing together.
fn phase_durations(k: u64) -> [f64; 2] {
    // SplitMix64's finaliser, as a hash of `k`.
    let mut z = k.wrapping_add(1).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^= z >> 31;
    [z % 11, (z >> 32) % 11].map(|d| 25.0 + d as f64)
}

/// A grid of `columns` by `rows` crossroads, `spacing` meters apart, joined
/// by two-way roads of `lanes` lanes each way, driving on the left, with a
/// road [`GRID_ARM`] long out from each crossroads on its edge. The
/// crossroads alternate as a chessboard does: signalised as
/// [`signalised_crossroads`] is, starting from the south-west corner, but with
/// each phase lasting 25 to 35 s, varying from one to the next, so they don't
/// change in step; and controlled by priority, the east–west road having it
/// and the north–south one giving way. Each is connected as [`crossroads`] is, its stop lines set
/// back and its run outs as long as a lone crossroads of the same roads needs.
pub fn grid(
    columns: usize,
    rows: usize,
    lanes: usize,
    spacing: f64,
    speed_limit: f64,
) -> RoadNetwork {
    // From its centre to where each road meeting it ends, as a lone
    // crossroads has it.
    let half = lanes as f64 * WIDTH + 6.0;
    assert!(spacing > 2.0 * half, "the crossroads overlap");
    // Every crossroads is the same, so each fits its roads as a lone one does.
    let fitted = crossroads(lanes, speed_limit).intersections[0].arms.iter().fold(
        Arm::new(RoadId(0)),
        |a, b| Arm {
            stop_line: a.stop_line.max(b.stop_line),
            run_out: a.run_out.max(b.run_out),
            ..a
        },
    );
    let mut net = RoadNetwork::default();
    let centre = |i: usize, j: usize| Pose::new(i as f64 * spacing, j as f64 * spacing, 0.0).pos;
    let road = |start: Pose, length: f64| Road {
        start,
        pieces: vec![Piece::Line { length }],
        lanes: two_way(lanes),
        speed_limit,
        keep_clear: Vec::new(),
    };
    // The roads between neighbouring crossroads, running east from each with
    // one to its east, and north from each with one to its north: so their
    // forward lanes come in to the crossroads they end at.
    let link = spacing - 2.0 * half;
    let mut east = vec![vec![None; rows]; columns];
    let mut north = vec![vec![None; rows]; columns];
    for i in 0..columns {
        for j in 0..rows {
            let c = centre(i, j);
            if i + 1 < columns {
                let start = Pose::new(c.x + half, c.y, 0.0);
                east[i][j] = Some(net.add_road(road(start, link)));
            }
            if j + 1 < rows {
                let start = Pose::new(c.x, c.y + half, FRAC_PI_2);
                north[i][j] = Some(net.add_road(road(start, link)));
            }
        }
    }
    for j in 0..rows {
        for i in 0..columns {
            // Its ways counterclockwise, from the east: each a road between it
            // and a neighbour, or one out from the edge, ending at it.
            let mut edge = |k: usize| {
                let from = k as f64 * FRAC_PI_2;
                let end = Pose {
                    pos: centre(i, j) + half * Pose::new(0.0, 0.0, from).dir(),
                    heading: from + PI,
                };
                let pieces = vec![Piece::Line { length: GRID_ARM }];
                let road = net.add_road(Road::ending_at(end, pieces, two_way(lanes), speed_limit));
                Way::at_end(road, lanes)
            };
            let ways = [
                match i + 1 < columns {
                    true => Way::at_start(east[i][j].unwrap(), lanes),
                    false => edge(0),
                },
                match j + 1 < rows {
                    true => Way::at_start(north[i][j].unwrap(), lanes),
                    false => edge(1),
                },
                match i > 0 {
                    true => Way::at_end(east[i - 1][j].unwrap(), lanes),
                    false => edge(2),
                },
                match j > 0 {
                    true => Way::at_end(north[i][j - 1].unwrap(), lanes),
                    false => edge(3),
                },
            ];
            let roads = ways.each_ref().map(|w| w.inbound[0].road);
            let mut intersection = Intersection {
                far_side_straight: RIGHT_TURN_STRAIGHT,
                arms: roads.iter().map(|&road| Arm { road, ..fitted }).collect(),
                ..Intersection::default()
            };
            connect(&mut intersection, &ways.map(Some));
            intersection.control = match (i + j) % 2 {
                0 => signal_plan(roads, phase_durations(net.intersections.len() as u64)),
                _ => Control::Priority {
                    major: vec![roads[0], roads[2]],
                    signs: vec![(roads[1], Sign::GiveWay), (roads[3], Sign::GiveWay)],
                },
            };
            net.intersections.push(intersection);
        }
    }
    net
}

/// How far the roads out from the edge of a [`grid`] reach, in meters.
pub const GRID_ARM: f64 = 200.0;

/// A ring road of `lanes` concentric lanes, clockwise as roundabouts go when
/// driving on the left, the innermost `circumference` meters round.
pub fn ring(circumference: f64, lanes: usize, speed_limit: f64) -> RoadNetwork {
    let radius = circumference / TAU;
    let mut net = RoadNetwork::default();
    // Going clockwise, lanes outward of the reference line are on its left.
    let road_lanes = (0..lanes)
        .map(|i| RoadLane::new(WIDTH * i as f64, WIDTH, Direction::Forward))
        .collect();
    let road = net.add_road(Road {
        start: Pose::new(radius, 0.0, -FRAC_PI_2),
        pieces: vec![Piece::Arc { length: circumference, curvature: -1.0 / radius }],
        lanes: road_lanes,
        speed_limit,
        keep_clear: Vec::new(),
    });
    net.join(road, road);
    net
}

/// A one-way freeway, driving on the left, with an on-ramp and then an
/// off-ramp, both on the left. Two lanes come in from the west. The on-ramp
/// curves in from the north-west and runs on as an acceleration lane beside
/// them for 200 m, which then eases into the left lane over 80 m: a lane drop,
/// where the two merge. 420 m on, the left lane forks: on along the freeway, or
/// curving away north up the off-ramp, which is slower. Drivers keep left, as
/// in Australia above 80 km/h, and stay in the acceleration lane to the merge,
/// where they take turns: a zipper.
pub fn ramps(speed_limit: f64, ramp_limit: f64) -> RoadNetwork {
    ramps_leaving(speed_limit, ramp_limit, Leaving::Zipper)
}

/// As [`ramps`], with drivers leaving the acceleration lane as `leaving` says:
/// early, as broken lines beside it now ask in Australia, or at the merge.
pub fn ramps_leaving(speed_limit: f64, ramp_limit: f64, leaving: Leaving) -> RoadNetwork {
    let mut net = RoadNetwork {
        keep: Keep::Left,
        keep_above: 80.0 / 3.6,
        ..RoadNetwork::default()
    };
    let lane = |offset: f64| RoadLane::new(offset, WIDTH, Direction::Forward);
    // The reference line runs down the middle of the two through lanes: lane 0
    // is the left one, lane 1 the right.
    let through = || vec![lane(0.5 * WIDTH), lane(-0.5 * WIDTH)];
    let straight = |x: f64, length: f64, lanes: Vec<RoadLane>| Road {
        start: Pose::new(x, 0.0, 0.0),
        pieces: vec![Piece::Line { length }],
        lanes,
        speed_limit,
        keep_clear: Vec::new(),
    };
    let accel = 1.5 * WIDTH;

    let approach = net.add_road(straight(-400.0, 400.0, through()));
    let mut beside = through();
    // Nobody changes from the freeway into the acceleration lane.
    beside[0].change_left = false;
    beside.insert(0, RoadLane { leaving: Some(leaving), ..lane(accel) });
    let parallel = net.add_road(straight(0.0, 200.0, beside));
    let mut taper = through();
    taper.insert(
        0,
        RoadLane {
            offset: Profile::Ease { from: accel, to: 0.5 * WIDTH, start: 0.0, end: 80.0 },
            width: Profile::Constant(WIDTH),
            direction: Direction::Forward,
            change_left: true,
            change_right: true,
            leaving: None,
        },
    );
    let drop = net.add_road(straight(200.0, 80.0, taper));
    let middle = net.add_road(straight(280.0, 420.0, through()));
    let after = net.add_road(straight(700.0, 400.0, through()));

    // Coming in heading south-east, turning left onto the acceleration lane.
    let on_ramp = net.add_road(Road::ending_at(
        Pose::new(0.0, accel, 0.0),
        vec![
            Piece::Line { length: 150.0 },
            Piece::Arc { length: 70.0, curvature: 1.0 / 200.0 },
        ],
        vec![lane(0.0)],
        ramp_limit,
    ));
    // Leaving the left lane, turning left away from the freeway.
    let off_ramp = net.add_road(Road {
        start: Pose::new(700.0, 0.5 * WIDTH, 0.0),
        pieces: vec![
            Piece::Arc { length: 90.0, curvature: 1.0 / 250.0 },
            Piece::Line { length: 150.0 },
        ],
        lanes: vec![lane(0.0)],
        speed_limit: ramp_limit,
        keep_clear: Vec::new(),
    });

    for (from, to) in [
        (approach, parallel),
        (on_ramp, parallel),
        (parallel, drop),
        (drop, middle),
        (middle, after),
        (middle, off_ramp),
    ] {
        net.join(from, to);
    }
    net
}
