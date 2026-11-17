//! Example networks, built at road level.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use crate::{Direction, Intersection, LaneRef, Piece, Pose, Road, RoadId, RoadLane, RoadNetwork};

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
