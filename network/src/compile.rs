//! Compiling a network to the simulation's lanes.

use std::fmt;

use traffic_sim::{End, Lane, LaneError, LaneId, NetworkError, SAME_POINT, SimNetwork, Simulation};

use crate::path::PathError;
use crate::{Network, RoadId, RoadLane};

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
    /// A join names a road the network doesn't have.
    NoSuchRoad { road: usize },
    /// A road's lane meets no lane across the join at one of its ends: it
    /// would end there, vehicles leaving the network, or start there, none
    /// coming in.
    Unjoined { road: usize, lane: usize, end: End },
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
            CompileError::NoSuchRoad { road } => {
                write!(f, "a join names road {road}, which isn't the network's")
            }
            CompileError::Unjoined { road, lane, end } => {
                let end = match end {
                    End::Start => "start",
                    End::End => "end",
                };
                write!(
                    f,
                    "road {road}, lane {lane}: its {end} meets no lane across the join there"
                )
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
    join(net, &mut sim)?;
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

/// Connects the lanes at each join wherever one ends where another starts:
/// the first road's forward lanes into the second's, and the second's reverse
/// lanes into the first's. Every lane at a join must meet one across it, from
/// one of the joins at its road's end: two roads may each join one lane of a
/// road of two.
fn join(net: &Network, sim: &mut SimNetwork) -> Result<(), CompileError> {
    // For each lane, its start and its end: whether it is at a join, and
    // whether it meets a lane across one.
    let mut ends = vec![[(false, false); 2]; sim.lanes().count()];
    for &[from, to] in net.joins() {
        for id in [from, to] {
            if net.road_id(id.index()) != Some(id) {
                return Err(CompileError::NoSuchRoad { road: id.index() });
            }
        }
        // A road's lanes one way.
        let lanes = |id: RoadId, reverse: bool| {
            let forward = net.roads()[id.index()].forward.len();
            let range = match reverse {
                false => 0..forward,
                true => forward..id.lane_count(),
            };
            range.map(move |i| id.lane(i)).collect::<Vec<_>>()
        };
        for (ending, starting) in [
            (lanes(from, false), lanes(to, false)),
            (lanes(to, true), lanes(from, true)),
        ] {
            for &a in &ending {
                ends[a.0 as usize][End::End as usize].0 = true;
            }
            for &b in &starting {
                ends[b.0 as usize][End::Start as usize].0 = true;
            }
            for &a in &ending {
                let end = sim.lane(a).geometry().centerline().last().expect("a lane has its ends");
                for &b in &starting {
                    let start =
                        sim.lane(b).geometry().centerline().next().expect("a lane has its ends");
                    if end.distance(start) >= SAME_POINT {
                        continue;
                    }
                    ends[a.0 as usize][End::End as usize].1 = true;
                    ends[b.0 as usize][End::Start as usize].1 = true;
                    // Once, if the join is made twice.
                    if !sim.lane(a).successors().contains(&b) {
                        sim.connect(a, b);
                    }
                }
            }
        }
    }
    for road in (0..net.roads().len()).filter_map(|r| net.road_id(r)) {
        for lane in 0..road.lane_count() {
            for end in [End::Start, End::End] {
                if ends[road.lane(lane).0 as usize][end as usize] == (true, false) {
                    return Err(CompileError::Unjoined { road: road.index(), lane, end });
                }
            }
        }
    }
    Ok(())
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

    /// A road of one lane forward along `path`.
    fn one_lane(path: Path) -> Road {
        Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0)],
            reverse: vec![],
        }
    }

    fn line(from: (f64, f64), to: (f64, f64)) -> Path {
        Path::new(DVec2::new(from.0, from.1)).line_to(DVec2::new(to.0, to.1))
    }

    #[test]
    fn a_join_runs_forward_lanes_on_and_reverse_lanes_back() {
        let mut net = Network::new(Side::Left);
        let road = |path| Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(-1.75), lane(-5.25)],
            reverse: vec![lane(1.75), lane(5.25)],
        };
        let a = net.add_road(road(line((0.0, 0.0), (50.0, 0.0))));
        let b = net.add_road(road(line((50.0, 0.0), (100.0, 0.0))));
        net.join(a, b);
        let sim = compile(&net).unwrap();
        let successors = |road: crate::RoadId, i| sim.lane(road.lane(i)).successors().to_vec();
        assert_eq!(successors(a, 0), [b.lane(0)]);
        assert_eq!(successors(a, 1), [b.lane(1)]);
        assert_eq!(successors(b, 2), [a.lane(2)]);
        assert_eq!(successors(b, 3), [a.lane(3)]);
        assert_eq!(successors(b, 0), [], "the network ends there");
    }

    #[test]
    fn a_road_may_join_itself_in_a_ring() {
        // A circle of radius 50, near enough, from four cubics.
        let (r, k) = (50.0, 0.5523 * 50.0);
        let p = DVec2::new;
        let path = Path::new(p(r, 0.0))
            .cubic_to(p(r, -k), p(k, -r), p(0.0, -r))
            .cubic_to(p(-k, -r), p(-r, -k), p(-r, 0.0))
            .cubic_to(p(-r, k), p(-k, r), p(0.0, r))
            .cubic_to(p(k, r), p(r, k), p(r, 0.0));
        let mut net = Network::new(Side::Left);
        let ring = net.add_road(Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0), lane(3.5)],
            reverse: vec![],
        });
        net.join(ring, ring);
        let sim = compile(&net).unwrap();
        for i in 0..2 {
            assert_eq!(sim.lane(ring.lane(i)).successors(), [ring.lane(i)]);
        }
    }

    #[test]
    fn two_lanes_meeting_one_merge_and_one_meeting_two_diverge() {
        let mut net = Network::new(Side::Left);
        let p = DVec2::new;
        let main = net.add_road(one_lane(line((0.0, 0.0), (100.0, 0.0))));
        // Curving in from the left to meet it end on.
        let ramp = net.add_road(one_lane(Path::new(p(0.0, 20.0)).cubic_to(
            p(50.0, 20.0),
            p(60.0, 0.0),
            p(100.0, 0.0),
        )));
        let on = net.add_road(one_lane(line((100.0, 0.0), (200.0, 0.0))));
        // And on, or curving away to the right.
        let ahead = net.add_road(one_lane(line((200.0, 0.0), (300.0, 0.0))));
        let away = net.add_road(one_lane(Path::new(p(200.0, 0.0)).cubic_to(
            p(240.0, 0.0),
            p(250.0, -20.0),
            p(300.0, -20.0),
        )));
        for (from, to) in [(main, on), (ramp, on), (on, ahead), (on, away)] {
            net.join(from, to);
        }
        let sim = compile(&net).unwrap();
        assert_eq!(
            sim.lane(on.lane(0)).predecessors(),
            [main.lane(0), ramp.lane(0)]
        );
        // Successors go from left to right.
        assert_eq!(
            sim.lane(on.lane(0)).successors(),
            [ahead.lane(0), away.lane(0)]
        );
    }

    #[test]
    fn a_lane_meeting_nothing_across_a_join_fails_to_compile() {
        let mut net = Network::new(Side::Left);
        let two = net.add_road(Road {
            path: line((0.0, 0.0), (50.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(-1.75), lane(-5.25)],
            reverse: vec![],
        });
        let one = net.add_road(one_lane(line((50.0, 0.0), (100.0, 0.0))));
        net.join(two, one);
        // Neither of the first road's lanes is on the second's centreline.
        assert_eq!(
            compile(&net).err(),
            Some(CompileError::Unjoined { road: 0, lane: 0, end: End::End })
        );

        // Nor may a lane start at a join from nothing.
        let mut net = Network::new(Side::Left);
        let one = net.add_road(one_lane(line((0.0, 0.0), (50.0, 0.0))));
        let two = net.add_road(Road {
            path: line((50.0, 0.0), (100.0, 0.0)),
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0), lane(-3.5)],
            reverse: vec![],
        });
        net.join(one, two);
        assert_eq!(
            compile(&net).err(),
            Some(CompileError::Unjoined { road: 1, lane: 1, end: End::Start })
        );
    }

    #[test]
    fn roads_side_by_side_may_each_join_a_lane_of_one_road() {
        let mut net = Network::new(Side::Left);
        let two_lanes = |path| Road {
            path,
            speed_limit: kmh(60.0),
            forward: vec![lane(0.0), lane(-3.5)],
            reverse: vec![],
        };
        let inner = net.add_road(one_lane(line((0.0, 0.0), (50.0, 0.0))));
        let outer = net.add_road(one_lane(line((0.0, -3.5), (50.0, -3.5))));
        let both = net.add_road(two_lanes(line((50.0, 0.0), (100.0, 0.0))));
        let inner_out = net.add_road(one_lane(line((100.0, 0.0), (150.0, 0.0))));
        let outer_out = net.add_road(one_lane(line((100.0, -3.5), (150.0, -3.5))));
        for (from, to) in [
            (inner, both),
            (outer, both),
            (both, inner_out),
            (both, outer_out),
        ] {
            net.join(from, to);
        }
        // Twice, to no effect.
        net.join(inner, both);
        let sim = compile(&net).unwrap();
        assert_eq!(sim.lane(both.lane(0)).predecessors(), [inner.lane(0)]);
        assert_eq!(sim.lane(both.lane(1)).predecessors(), [outer.lane(0)]);
        assert_eq!(sim.lane(both.lane(0)).successors(), [inner_out.lane(0)]);
        assert_eq!(sim.lane(both.lane(1)).successors(), [outer_out.lane(0)]);
    }

    #[test]
    fn a_join_naming_another_network_s_road_fails_to_compile() {
        let mut other = Network::new(Side::Left);
        other.add_road(one_lane(line((0.0, 0.0), (50.0, 0.0))));
        let stranger = other.add_road(one_lane(line((50.0, 0.0), (100.0, 0.0))));
        let mut net = Network::new(Side::Left);
        let road = net.add_road(one_lane(line((0.0, 0.0), (50.0, 0.0))));
        net.join(road, stranger);
        assert_eq!(
            compile(&net).err(),
            Some(CompileError::NoSuchRoad { road: 1 })
        );
    }

    #[test]
    #[should_panic(expected = "has 4 lanes")]
    fn a_lane_beyond_the_road_s_panics() {
        let (_, road) = two_by_two();
        road.lane(4);
    }
}
