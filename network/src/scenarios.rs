//! Example networks, built at road level.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use traffic_sim::{Keep, Leaving};

use crate::{
    Arm, Control, Direction, Intersection, LaneRef, Piece, Pose, Profile, Road, RoadId, RoadLane,
    RoadNetwork, SignalPhase, Turn,
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
    let mut net = RoadNetwork::default();
    // How far across each road's lanes reach from its centre line.
    let across = lanes as f64 * WIDTH;
    // The junction's half-width along each arm, from its centre to where the
    // arm ends: where its outermost lane clears the other road, and 6 m more.
    let half = across / angle.sin() + across / angle.tan().abs() + 6.0;
    let arm_length = ARM - half;
    // Each arm runs in toward the centre, from the first road's east end, the
    // second's far end, the first's west end and the second's near end, so
    // counterclockwise. Its forward lanes come in, on the left of its
    // reference line, and its backward lanes go out, on the right; lane 0 of
    // each is by the centre line.
    let arm_lanes: Vec<RoadLane> = (0..lanes)
        .map(|i| RoadLane::new(WIDTH * (i as f64 + 0.5), WIDTH, Direction::Forward))
        .chain(
            (0..lanes)
                .map(|i| RoadLane::new(-WIDTH * (i as f64 + 0.5), WIDTH, Direction::Backward)),
        )
        .collect();
    let from = [0.0, angle, PI, PI + angle];
    let arms: Vec<RoadId> = from
        .iter()
        .map(|&at| {
            let towards = at + PI;
            let end = Pose { pos: half * Pose::new(0.0, 0.0, at).dir(), heading: towards };
            let pieces = vec![Piece::Line { length: arm_length }];
            net.add_road(Road::ending_at(end, pieces, arm_lanes.clone(), speed_limit))
        })
        .collect();
    let inbound = |arm: usize, i: usize| LaneRef { road: arms[arm], lane: i };
    let outbound = |arm: usize, i: usize| LaneRef { road: arms[arm], lane: lanes + i };
    let mut intersection = Intersection {
        far_side_straight: RIGHT_TURN_STRAIGHT,
        ..Intersection::default()
    };
    let outer = lanes - 1;
    for (k, &road) in arms.iter().enumerate() {
        // Arms counterclockwise from this one: right, straight on, left.
        let (right, straight, left) = ((k + 1) % 4, (k + 2) % 4, (k + 3) % 4);
        for i in 0..lanes {
            intersection.connections.push((inbound(k, i), outbound(straight, i)));
        }
        // Turning, into every lane of the road it turns into.
        for i in 0..lanes {
            intersection.connections.push((inbound(k, outer), outbound(left, i)));
            intersection.connections.push((inbound(k, 0), outbound(right, i)));
        }
        // Its stop line along the other road: as far from square as the other
        // road is from crossing square, counterclockwise from the way in, and
        // back far enough that none of it is past the arm's end.
        // That is the angle between the roads on the first road's arms, and its
        // supplement on the second's.
        let crossing = if k % 2 == 0 { angle } else { PI - angle };
        let skew = crossing - FRAC_PI_2;
        let stop_line = (-across * skew.tan()).max(0.0);
        intersection.arms.push(Arm { stop_line, skew, ..Arm::new(road) });
    }
    net.intersections.push(intersection);
    // Stop lines set back, and run outs, wherever turning vehicles sweep.
    crate::fit_arms(&mut net).expect("the crossroads compile once fitted");
    net
}

/// As [`crossroads`], signalised: a fixed-time plan, the roads from the east
/// and west green for 30 s, every way, then those from the north and south. A
/// right turn shares its phase with the traffic coming the other way.
pub fn signalised_crossroads(lanes: usize, speed_limit: f64) -> RoadNetwork {
    signalised(crossroads(lanes, speed_limit))
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

/// `net`, a crossroads, with a fixed-time plan: its first road green for
/// 30 s, every way, then its second.
fn signalised(mut net: RoadNetwork) -> RoadNetwork {
    let roads: Vec<RoadId> = (0..4).map(RoadId).collect();
    let every_way = |arms: [usize; 2]| {
        arms.iter()
            .flat_map(|&a| [Turn::Straight, Turn::Left, Turn::Right].map(|t| (roads[a], t)))
            .collect()
    };
    net.intersections[0].control = Control::Signals(vec![
        SignalPhase { green: every_way([0, 2]), duration: 30.0 },
        SignalPhase { green: every_way([1, 3]), duration: 30.0 },
    ]);
    net
}

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
