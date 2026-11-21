//! Example networks, built at road level.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use traffic_sim::{Keep, Leaving};

use crate::{
    Control, Direction, Intersection, LaneRef, Piece, Pose, Profile, Road, RoadId, RoadLane,
    RoadNetwork, SignalPhase, Turn,
};

const WIDTH: f64 = 3.5;

/// Two two-way roads crossing at right angles, each with `lanes` lanes each
/// way, 300 m long, driving on the left. Every lane coming in connects to the
/// lane at the same place across the junction, straight on; only the leftmost
/// turns left and only the rightmost turns right.
pub fn crossroads(lanes: usize, speed_limit: f64) -> RoadNetwork {
    let mut net = RoadNetwork::default();
    // The junction's half-width, from its centre to where the arms end.
    let half = lanes as f64 * WIDTH + 6.0;
    let arm_length = 150.0 - half;
    // Each arm runs in toward the centre, from east, north, west and south.
    // Its forward lanes come in, on the left of its reference line, and its
    // backward lanes go out, on the right; lane 0 of each is by the centre line.
    let arm_lanes: Vec<RoadLane> = (0..lanes)
        .map(|i| RoadLane::new(WIDTH * (i as f64 + 0.5), WIDTH, Direction::Forward))
        .chain(
            (0..lanes)
                .map(|i| RoadLane::new(-WIDTH * (i as f64 + 0.5), WIDTH, Direction::Backward)),
        )
        .collect();
    let arms: Vec<RoadId> = (0..4)
        .map(|k| {
            let towards = PI + FRAC_PI_2 * k as f64;
            let end = Pose {
                pos: -half * Pose::new(0.0, 0.0, towards).dir(),
                heading: towards,
            };
            let pieces = vec![Piece::Line { length: arm_length }];
            net.add_road(Road::ending_at(end, pieces, arm_lanes.clone(), speed_limit))
        })
        .collect();
    let inbound = |arm: usize, i: usize| LaneRef { road: arms[arm], lane: i };
    let outbound = |arm: usize, i: usize| LaneRef { road: arms[arm], lane: lanes + i };
    let mut intersection = Intersection::default();
    let outer = lanes - 1;
    for from in 0..4 {
        // Arms counterclockwise from this one: right, straight on, left.
        let (right, straight, left) = ((from + 1) % 4, (from + 2) % 4, (from + 3) % 4);
        for i in 0..lanes {
            intersection.connections.push((inbound(from, i), outbound(straight, i)));
        }
        intersection.connections.push((inbound(from, outer), outbound(left, outer)));
        intersection.connections.push((inbound(from, 0), outbound(right, 0)));
    }
    net.intersections.push(intersection);
    net
}

/// As [`crossroads`], signalised: a fixed-time plan, the roads from the east
/// and west green for 30 s, every way, then those from the north and south. A
/// right turn shares its phase with the traffic coming the other way.
pub fn signalised_crossroads(lanes: usize, speed_limit: f64) -> RoadNetwork {
    let mut net = crossroads(lanes, speed_limit);
    // The arms come in from the east, north, west and south, in that order.
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
