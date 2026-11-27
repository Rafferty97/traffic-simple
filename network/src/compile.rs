//! Compiling a network to the simulation's lanes.

use std::fmt;

use traffic_sim::{Lane, LaneError, LaneId, NetworkError, SAME_POINT, SimNetwork, Simulation};

use crate::path::PathError;
use crate::{Network, RoadLane};

/// How far a simulated lane's centreline strays from its curve at most, in
/// meters: vehicles round each vertex (see `SimNetwork::smooth_point_along`),
/// so the tolerance sets only how closely they follow the curve.
pub(crate) const LANE_TOLERANCE: f64 = 0.01;

/// Why a network doesn't compile.
#[derive(Clone, Debug, PartialEq)]
pub enum CompileError {
    /// A road's path can't be sampled.
    Path { road: usize, error: PathError },
    /// A road's lane isn't a valid lane.
    Lane { road: usize, lane: usize, error: LaneError },
    /// The lanes don't make a valid simulation network.
    Network(NetworkError),
}

impl fmt::Display for CompileError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            CompileError::Path { road, error } => write!(f, "road {road}: {error}"),
            CompileError::Lane { road, lane, error } => {
                write!(f, "road {road}, lane {lane}: {error}")
            }
            CompileError::Network(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for CompileError {}

/// The network's lanes, for the simulation: each road's lanes in turn, forward
/// ones first, so a road's [`RoadId::lane`](crate::RoadId::lane) is its lane's
/// id here. Each way along a road is a road of its own for routing.
pub fn compile(net: &Network) -> Result<SimNetwork, CompileError> {
    let mut lanes = Vec::new();
    // Each lane's road, whether it runs in reverse, and its description.
    let mut described: Vec<(usize, bool, RoadLane)> = Vec::new();
    for (r, road) in net.roads().iter().enumerate() {
        let line = road
            .path
            .sample(LANE_TOLERANCE)
            .map_err(|error| CompileError::Path { road: r, error })?;
        for (i, (lane, reverse)) in road.lanes().enumerate() {
            let mut points = line.offset(lane.offset);
            if reverse {
                points.reverse();
            }
            let half_widths = vec![[lane.left, lane.right]; points.len()];
            let way = traffic_sim::RoadId(2 * r as u32 + u32::from(reverse));
            let built = Lane::with_half_widths(&points, &half_widths, road.speed_limit, way)
                .map_err(|error| CompileError::Lane { road: r, lane: i, error })?;
            lanes.push(built);
            described.push((r, reverse, *lane));
        }
    }
    let mut sim = SimNetwork::new(lanes);
    neighbours(&described, &mut sim);
    sim.validate().map_err(CompileError::Network)?;
    Ok(sim)
}

/// A simulation of the network, compiled, with no vehicles yet.
pub fn simulate(net: &Network) -> Result<Simulation, CompileError> {
    Ok(Simulation::new(compile(net)?))
}

/// Makes neighbours of each two lanes of a road running the same way whose
/// edges touch, vehicles free to change between them either way.
fn neighbours(described: &[(usize, bool, RoadLane)], sim: &mut SimNetwork) {
    for (a, &(road_a, reverse_a, lane_a)) in described.iter().enumerate() {
        for (b, &(road_b, reverse_b, lane_b)) in described.iter().enumerate() {
            if a == b || road_a != road_b || reverse_a != reverse_b {
                continue;
            }
            // `b` is on `a`'s left, as `a` runs, if its edge on the right
            // touches `a`'s edge on the left.
            let [a_right, a_left] = lane_a.edges(reverse_a);
            let [b_right, b_left] = lane_b.edges(reverse_b);
            let touches = match reverse_a {
                false => (b_right - a_left).abs() < SAME_POINT,
                true => (b_left - a_right).abs() < SAME_POINT,
            };
            if touches {
                sim.set_adjacent(LaneId(b as u32), LaneId(a as u32), true, true);
            }
        }
    }
}

#[cfg(test)]
mod test {
    use glam::DVec2;
    use traffic_sim::Side;

    use super::*;
    use crate::{Path, Road, kmh};

    fn lane(offset: f64) -> RoadLane {
        RoadLane { offset, left: 1.75, right: 1.75 }
    }

    /// A straight 100 m road with two lanes each way.
    fn two_by_two() -> (Network, crate::RoadId) {
        let mut net = Network::new(Side::Left);
        let road = net.add_road(Road {
            path: Path::new(DVec2::ZERO).line_to(DVec2::new(100.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(-1.75), lane(-5.25)],
            reverse: vec![lane(1.75), lane(5.25)],
        });
        (net, road)
    }

    #[test]
    fn forward_lanes_come_first_and_reverse_lanes_run_backward() {
        let (net, road) = two_by_two();
        let sim = compile(&net).unwrap();
        let start = |i| sim.lane(road.lane(i)).geometry().centerline().next().unwrap();
        assert_eq!(start(0), DVec2::new(0.0, -1.75));
        assert_eq!(start(1), DVec2::new(0.0, -5.25));
        assert_eq!(start(2), DVec2::new(100.0, 1.75));
        assert_eq!(start(3), DVec2::new(100.0, 5.25));
        assert_eq!(sim.lane(road.lane(0)).speed_limit(), kmh(60.0));
    }

    #[test]
    fn lanes_side_by_side_the_same_way_are_neighbours() {
        let (net, road) = two_by_two();
        let sim = compile(&net).unwrap();
        let neighbour = |i, side| sim.lane(road.lane(i)).adj(side).map(|n| n.lane);
        // Forward, lane 1 is to the right of lane 0, the inner one.
        assert_eq!(neighbour(0, Side::Right), Some(road.lane(1)));
        assert_eq!(neighbour(1, Side::Left), Some(road.lane(0)));
        assert_eq!(
            neighbour(0, Side::Left),
            None,
            "the opposite way is no neighbour"
        );
        // In reverse, lane 3 is to the right of lane 2, its inner one.
        assert_eq!(neighbour(2, Side::Right), Some(road.lane(3)));
        assert_eq!(neighbour(3, Side::Left), Some(road.lane(2)));
        assert_eq!(neighbour(2, Side::Left), None);
    }

    #[test]
    fn lanes_apart_are_no_neighbours() {
        let mut net = Network::new(Side::Left);
        let road = net.add_road(Road {
            path: Path::new(DVec2::ZERO).line_to(DVec2::new(100.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0), lane(-4.0)],
            reverse: vec![],
        });
        let sim = compile(&net).unwrap();
        assert_eq!(sim.lane(road.lane(0)).adj(Side::Right), None);
    }

    #[test]
    #[should_panic(expected = "has 4 lanes")]
    fn a_lane_beyond_the_road_s_panics() {
        let (_, road) = two_by_two();
        road.lane(4);
    }
}
