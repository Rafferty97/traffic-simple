//! Compiling a network to the simulation's lanes.

use std::fmt;

use glam::DVec2;
use traffic_sim::{
    End, Lane, LaneError, LaneId, NetworkError, Placement, Route, SAME_POINT, SimNetwork,
    Simulation,
};

use crate::path::PathError;
use crate::{Network, RoadId, cuts};

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
    /// A join or an overlap names a road the network doesn't have.
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
                write!(
                    f,
                    "a join or overlap names road {road}, which isn't the network's"
                )
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

/// Why a vehicle can't be placed where it was asked to be.
#[derive(Clone, Debug, PartialEq)]
pub enum PlaceError {
    /// The road isn't the network's.
    NoSuchRoad { road: usize },
    /// The road has fewer lanes.
    NoSuchLane { road: usize, lane: usize, lanes: usize },
    /// The distance isn't along the road.
    OffRoad { road: usize, s: f64, length: f64 },
}

impl fmt::Display for PlaceError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match *self {
            PlaceError::NoSuchRoad { road } => write!(f, "road {road} isn't the network's"),
            PlaceError::NoSuchLane { road, lane, lanes } => {
                write!(f, "road {road} has {lanes} lanes, not {}", lane + 1)
            }
            PlaceError::OffRoad { road, s, length } => {
                write!(f, "road {road} is {length} m long, so {s} m isn't along it")
            }
        }
    }
}

impl std::error::Error for PlaceError {}

/// A network compiled for the simulation: its lanes, and which of them are
/// each road's.
#[derive(Clone, Debug)]
pub struct Compiled {
    network: Network,
    sim: SimNetwork,
    /// Each road's lanes in the simulation, by road.
    layouts: Vec<Layout>,
}

/// Where a road's lanes are in the simulation: a lane for each of its lanes
/// on each piece between its cuts, piece by piece, forward lanes first on
/// each.
#[derive(Clone, Debug)]
struct Layout {
    first: u32,
    /// Where along its reference line each piece starts, and the last ends.
    bounds: Vec<f64>,
}

impl Compiled {
    /// The network compiled.
    pub fn network(&self) -> &Network {
        &self.network
    }

    /// Its lanes, for the simulation.
    pub fn sim_network(&self) -> &SimNetwork {
        &self.sim
    }

    /// A simulation of it, with no vehicles yet.
    pub fn simulate(&self) -> Simulation {
        Simulation::new(self.sim.clone())
    }

    /// The simulation's lanes that are `road`'s `lane`th, in the order it runs
    /// along them: one for each piece the road is cut into.
    ///
    /// Panics if the road isn't the network's or has no such lane, as indexing
    /// a slice does.
    pub fn lanes(&self, road: RoadId, lane: usize) -> Vec<LaneId> {
        assert!(
            self.network.road_id(road.index()) == Some(road),
            "road {} isn't the network's",
            road.index()
        );
        assert!(
            lane < road.lane_count(),
            "road {} has {} lanes, not {}",
            road.index(),
            road.lane_count(),
            lane + 1
        );
        let pieces = self.layouts[road.index()].bounds.len() - 1;
        let mut lanes: Vec<LaneId> =
            (0..pieces).map(|k| self.lane_id(road.index(), k, lane)).collect();
        if lane >= self.network.roads()[road.index()].forward.len() {
            lanes.reverse();
        }
        lanes
    }

    /// Where a vehicle goes to be `s` meters along `road`'s reference line,
    /// from its start, on its `lane`th lane, at `speed`, for
    /// [`Simulation::insert`]: on the piece of the lane there, its rear axle
    /// square across the road from that point. Reverse lanes are measured
    /// along the reference line too, from the road's start.
    pub fn place(
        &self,
        road: RoadId,
        lane: usize,
        s: f64,
        speed: f64,
    ) -> Result<Placement, PlaceError> {
        let r = road.index();
        if self.network.road_id(r) != Some(road) {
            return Err(PlaceError::NoSuchRoad { road: r });
        }
        if lane >= road.lane_count() {
            return Err(PlaceError::NoSuchLane { road: r, lane, lanes: road.lane_count() });
        }
        let description = &self.network.roads()[r];
        let length = description.path.length();
        let off = PlaceError::OffRoad { road: r, s, length };
        let (point, dir) = description.path.pose(s).ok_or(off)?;
        let (road_lane, _) = description.lanes().nth(lane).expect("it has the lane");
        let at = point + road_lane.offset * dir.perp();
        let bounds = &self.layouts[r].bounds;
        let piece = bounds[1..].partition_point(|&b| b <= s).min(bounds.len() - 2);
        let id = self.lane_id(r, piece, lane);
        let geometry = self.sim.lane(id).geometry();
        let s = geometry.world_to_lane(at).s.clamp(0.0, geometry.length());
        Ok(Placement { lane: id, s, speed, route: Route::default() })
    }

    /// The simulation's lane for `road`'s `lane`th lane on its `piece`th
    /// piece.
    fn lane_id(&self, road: usize, piece: usize, lane: usize) -> LaneId {
        let layout = &self.layouts[road];
        let lanes = self.network.roads()[road].lane_count();
        LaneId(layout.first + (piece * lanes + lane) as u32)
    }

    /// The lane `road`'s `lane`th starts on, and the one it ends on.
    fn ends(&self, road: RoadId, lane: usize) -> [LaneId; 2] {
        let lanes = self.lanes(road, lane);
        [lanes[0], lanes[lanes.len() - 1]]
    }
}

/// A stretch of a road between cuts, sampled, with its lanes built.
pub(crate) struct Piece {
    /// Where along the road's reference line it starts.
    from: f64,
    /// How far along the road's reference line each vertex is.
    s: Vec<f64>,
    /// Its lanes, forward ones first, each with how far along it each of its
    /// vertices is.
    pub lanes: Vec<(Lane, Vec<f64>)>,
    /// Whether each lane runs in reverse.
    reverse: Vec<bool>,
}

impl Piece {
    /// How far along the road's reference line its lane `lane` is, `s` along
    /// the lane: from the vertices either side, as lanes are offset from the
    /// reference line vertex for vertex.
    pub fn reference_s(&self, lane: usize, s: f64) -> f64 {
        let along = &self.lanes[lane].1;
        let k = along.partition_point(|&x| x <= s).clamp(1, along.len() - 1) - 1;
        let f = ((s - along[k]) / (along[k + 1] - along[k])).clamp(0.0, 1.0);
        let n = self.s.len();
        let [i, j] = match self.reverse[lane] {
            false => [k, k + 1],
            true => [n - 1 - k, n - 2 - k],
        };
        self.s[i] + f * (self.s[j] - self.s[i])
    }

    /// How long its reference line is.
    pub fn length(&self) -> f64 {
        self.s[self.s.len() - 1] - self.s[0]
    }
}

/// The network's lanes, for the simulation. Each road is cut wherever one of
/// its lanes starts or stops overlapping a lane of a road related to it, and
/// becomes a lane for each of its lanes on each piece; the pieces of one lane
/// run on into each other, and are one road for routing, as each way along a
/// road is.
pub fn compile(net: &Network) -> Result<Compiled, CompileError> {
    for &id in net.joins().iter().chain(net.overlaps()).flatten() {
        if net.road_id(id.index()) != Some(id) {
            return Err(CompileError::NoSuchRoad { road: id.index() });
        }
    }
    // Each road whole, to find where to cut it, then cut there.
    let whole = (0..net.roads().len())
        .map(|r| Ok(pieces(net, r, &[])?.remove(0)))
        .collect::<Result<Vec<_>, CompileError>>()?;
    let related = cuts::related(net);
    let cuts = cuts::cuts(&related, &whole);
    let mut lanes = Vec::new();
    let mut layouts = Vec::new();
    for (r, (whole, cuts)) in whole.into_iter().zip(&cuts).enumerate() {
        let pieces = match cuts.is_empty() {
            true => vec![whole],
            false => pieces(net, r, cuts)?,
        };
        let mut bounds: Vec<f64> = pieces.iter().map(|p| p.from).collect();
        bounds.push(net.roads()[r].path.length());
        layouts.push(Layout { first: lanes.len() as u32, bounds });
        lanes.extend(pieces.into_iter().flat_map(|p| p.lanes).map(|(lane, _)| lane));
    }
    let mut compiled = Compiled { network: net.clone(), sim: SimNetwork::new(lanes), layouts };
    neighbours(&mut compiled);
    for road in (0..net.roads().len()).filter_map(|r| net.road_id(r)) {
        for lane in 0..road.lane_count() {
            for pair in compiled.lanes(road, lane).windows(2) {
                compiled.sim.connect(pair[0], pair[1]);
            }
        }
    }
    join(&mut compiled)?;
    declare_overlaps(&mut compiled, &related);
    compiled.sim.validate().map_err(CompileError::Network)?;
    Ok(compiled)
}

/// Road `r`'s pieces between `cuts`, distances along its reference line, each
/// sampled as a slice of it, so that its lanes end square across the road
/// where it is cut, as at its ends.
fn pieces(net: &Network, r: usize, cuts: &[f64]) -> Result<Vec<Piece>, CompileError> {
    let road = &net.roads()[r];
    let length = road.path.length();
    let bounds: Vec<f64> =
        std::iter::once(0.0).chain(cuts.iter().copied()).chain([length]).collect();
    let mut out = Vec::new();
    for w in bounds.windows(2) {
        let path = match cuts.is_empty() {
            true => road.path.clone(),
            false => road.path.slice(w[0], w[1]),
        };
        let line = path
            .sample(LANE_TOLERANCE)
            .map_err(|error| CompileError::Path { road: r, error })?;
        let mut lanes = Vec::new();
        let mut reverses = Vec::new();
        for (i, (lane, reverse)) in road.lanes().enumerate() {
            let mut points = line.offset(lane.offset);
            if reverse {
                points.reverse();
            }
            let half_widths = vec![[lane.left, lane.right]; points.len()];
            let way = traffic_sim::RoadId(2 * r as u32 + u32::from(reverse));
            let built = Lane::with_half_widths(&points, &half_widths, road.speed_limit, way)
                .map_err(|error| CompileError::Lane { road: r, lane: i, error })?;
            lanes.push((built, arc_lengths(&points)));
            reverses.push(reverse);
        }
        let s = line.s.iter().map(|s| w[0] + s).collect();
        out.push(Piece { from: w[0], s, lanes, reverse: reverses });
    }
    Ok(out)
}

/// How far along a polyline each of its vertices is.
fn arc_lengths(points: &[DVec2]) -> Vec<f64> {
    let mut s = vec![0.0];
    for w in points.windows(2) {
        s.push(s[s.len() - 1] + w[0].distance(w[1]));
    }
    s
}

/// Makes neighbours of each two lanes of a road running the same way whose
/// edges touch, on each of its pieces, vehicles free to change between them
/// either way.
fn neighbours(compiled: &mut Compiled) {
    for (r, road) in compiled.network.roads().iter().enumerate() {
        let lanes: Vec<_> = road.lanes().collect();
        let pieces = compiled.layouts[r].bounds.len() - 1;
        for (a, &(lane_a, reverse_a)) in lanes.iter().enumerate() {
            for (b, &(lane_b, reverse_b)) in lanes.iter().enumerate() {
                if a == b || reverse_a != reverse_b {
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
                if !touches {
                    continue;
                }
                for k in 0..pieces {
                    let (left, right) = (compiled.lane_id(r, k, b), compiled.lane_id(r, k, a));
                    compiled.sim.set_adjacent(left, right, true, true);
                }
            }
        }
    }
}

/// Connects the lanes at each join wherever one ends where another starts:
/// the first road's forward lanes into the second's, and the second's reverse
/// lanes into the first's. Every lane at a join must meet one across it, from
/// one of the joins at its road's end: two roads may each join one lane of a
/// road of two.
fn join(compiled: &mut Compiled) -> Result<(), CompileError> {
    // For each lane, its start and its end: whether it is at a join, and
    // whether it meets a lane across one.
    let mut ends = vec![[(false, false); 2]; compiled.sim.lanes().count()];
    let net = compiled.network.clone();
    for &[from, to] in net.joins() {
        // A road's lanes one way, by the lanes they start and end on.
        let lanes = |id: RoadId, reverse: bool| {
            let forward = net.roads()[id.index()].forward.len();
            let range = match reverse {
                false => 0..forward,
                true => forward..id.lane_count(),
            };
            range.map(|i| compiled.ends(id, i)).collect::<Vec<_>>()
        };
        for (ending, starting) in [
            (lanes(from, false), lanes(to, false)),
            (lanes(to, true), lanes(from, true)),
        ] {
            for &[_, a] in &ending {
                ends[a.0 as usize][End::End as usize].0 = true;
            }
            for &[b, _] in &starting {
                ends[b.0 as usize][End::Start as usize].0 = true;
            }
            for &[_, a] in &ending {
                let centerline = |id: LaneId| compiled.sim.lane(id).geometry().centerline();
                let end = centerline(a).last().expect("a lane has its ends");
                for &[b, _] in &starting {
                    let centerline = |id: LaneId| compiled.sim.lane(id).geometry().centerline();
                    let start = centerline(b).next().expect("a lane has its ends");
                    if end.distance(start) >= SAME_POINT {
                        continue;
                    }
                    ends[a.0 as usize][End::End as usize].1 = true;
                    ends[b.0 as usize][End::Start as usize].1 = true;
                    // Once, if the join is made twice.
                    if !compiled.sim.lane(a).successors().contains(&b) {
                        compiled.sim.connect(a, b);
                    }
                }
            }
        }
    }
    for road in (0..net.roads().len()).filter_map(|r| net.road_id(r)) {
        for lane in 0..road.lane_count() {
            let [first, last] = compiled.ends(road, lane);
            for (end, id) in [(End::Start, first), (End::End, last)] {
                if ends[id.0 as usize][end as usize] == (true, false) {
                    return Err(CompileError::Unjoined { road: road.index(), lane, end });
                }
            }
        }
    }
    Ok(())
}

/// Tells the simulation which lanes may overlap: every two of related roads,
/// on every piece, but those of one lane, which follow one another. Those
/// whose outlines don't overlap, and neighbours, which only touch, it passes
/// over.
fn declare_overlaps(compiled: &mut Compiled, related: &[[usize; 2]]) {
    let net = compiled.network.clone();
    for &[r1, r2] in related {
        let (a, b) = (
            net.road_id(r1).expect("a road"),
            net.road_id(r2).expect("a road"),
        );
        for i in 0..a.lane_count() {
            for j in 0..b.lane_count() {
                if r1 == r2 && i >= j {
                    continue;
                }
                for x in compiled.lanes(a, i) {
                    for y in compiled.lanes(b, j) {
                        compiled.sim.declare_overlap(x, y);
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use glam::DVec2;
    use traffic_sim::Side;

    use super::*;
    use crate::{Path, Road, RoadLane, kmh};

    /// The simulation's lane for a road's `i`th lane, which isn't cut.
    fn only(c: &Compiled, road: RoadId, i: usize) -> LaneId {
        let lanes = c.lanes(road, i);
        assert_eq!(lanes.len(), 1, "road {}, lane {i} is cut", road.index());
        lanes[0]
    }

    /// The lane a road's `i`th lane starts on.
    fn first(c: &Compiled, road: RoadId, i: usize) -> LaneId {
        c.lanes(road, i)[0]
    }

    /// The lane a road's `i`th lane ends on.
    fn last(c: &Compiled, road: RoadId, i: usize) -> LaneId {
        *c.lanes(road, i).last().unwrap()
    }

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
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        let start = |i| sim.lane(only(&c, road, i)).geometry().centerline().next().unwrap();
        assert_eq!(start(0), DVec2::new(0.0, -1.75));
        assert_eq!(start(1), DVec2::new(0.0, -5.25));
        assert_eq!(start(2), DVec2::new(100.0, 1.75));
        assert_eq!(start(3), DVec2::new(100.0, 5.25));
        assert_eq!(sim.lane(only(&c, road, 0)).speed_limit(), kmh(60.0));
    }

    #[test]
    fn lanes_side_by_side_the_same_way_are_neighbours() {
        let (net, road) = two_by_two();
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        let neighbour = |i, side| sim.lane(only(&c, road, i)).adj(side).map(|n| n.lane);
        // Forward, lane 1 is to the right of lane 0, the inner one.
        assert_eq!(neighbour(0, Side::Right), Some(only(&c, road, 1)));
        assert_eq!(neighbour(1, Side::Left), Some(only(&c, road, 0)));
        assert_eq!(
            neighbour(0, Side::Left),
            None,
            "the opposite way is no neighbour"
        );
        // In reverse, lane 3 is to the right of lane 2, its inner one.
        assert_eq!(neighbour(2, Side::Right), Some(only(&c, road, 3)));
        assert_eq!(neighbour(3, Side::Left), Some(only(&c, road, 2)));
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
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        assert_eq!(sim.lane(only(&c, road, 0)).adj(Side::Right), None);
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
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        let successors = |road: crate::RoadId, i| sim.lane(only(&c, road, i)).successors().to_vec();
        assert_eq!(successors(a, 0), [only(&c, b, 0)]);
        assert_eq!(successors(a, 1), [only(&c, b, 1)]);
        assert_eq!(successors(b, 2), [only(&c, a, 2)]);
        assert_eq!(successors(b, 3), [only(&c, a, 3)]);
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
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        for i in 0..2 {
            assert_eq!(
                sim.lane(only(&c, ring, i)).successors(),
                [only(&c, ring, i)]
            );
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
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        assert_eq!(
            sim.lane(only(&c, on, 0)).predecessors(),
            [last(&c, main, 0), last(&c, ramp, 0)]
        );
        // Successors go from left to right.
        assert_eq!(
            sim.lane(only(&c, on, 0)).successors(),
            [first(&c, ahead, 0), first(&c, away, 0)]
        );
    }

    #[test]
    fn a_placement_is_along_the_reference_line_whichever_way_its_lane_runs() {
        let (net, road) = two_by_two();
        let c = compile(&net).unwrap();
        let ahead = c.place(road, 0, 30.0, 10.0).unwrap();
        assert_eq!((ahead.lane, ahead.speed), (only(&c, road, 0), 10.0));
        assert!((ahead.s - 30.0).abs() < 1e-6, "{}", ahead.s);
        // A reverse lane runs back from the road's end.
        let back = c.place(road, 2, 30.0, 10.0).unwrap();
        assert_eq!(back.lane, only(&c, road, 2));
        assert!((back.s - 70.0).abs() < 1e-6, "{}", back.s);
        assert_eq!(
            c.place(road, 4, 30.0, 10.0).err(),
            Some(PlaceError::NoSuchLane { road: 0, lane: 4, lanes: 4 })
        );
        assert!(matches!(
            c.place(road, 0, 101.0, 10.0),
            Err(PlaceError::OffRoad { road: 0, .. })
        ));
    }

    #[test]
    fn a_placement_on_a_road_cut_in_pieces_is_on_the_piece_there() {
        let mut net = Network::new(Side::Left);
        let p = DVec2::new;
        let main = net.add_road(one_lane(line((0.0, 0.0), (100.0, 0.0))));
        let ramp = net.add_road(one_lane(Path::new(p(0.0, 20.0)).cubic_to(
            p(50.0, 20.0),
            p(60.0, 0.0),
            p(100.0, 0.0),
        )));
        let on = net.add_road(one_lane(line((100.0, 0.0), (200.0, 0.0))));
        net.join(main, on);
        net.join(ramp, on);
        let c = compile(&net).unwrap();
        let pieces = c.lanes(main, 0);
        assert_eq!(pieces.len(), 2, "cut where the ramp comes in over it");
        let cut = 100.0 - c.sim_network().lane(pieces[1]).geometry().length();
        let before = c.place(main, 0, cut - 1.0, 10.0).unwrap();
        assert_eq!(before.lane, pieces[0]);
        assert!((before.s - (cut - 1.0)).abs() < 1e-6);
        let after = c.place(main, 0, cut + 1.0, 10.0).unwrap();
        assert_eq!(after.lane, pieces[1]);
        assert!((after.s - 1.0).abs() < 1e-6, "{}", after.s);
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
        let c = compile(&net).unwrap();
        let sim = c.sim_network();
        assert_eq!(
            sim.lane(only(&c, both, 0)).predecessors(),
            [only(&c, inner, 0)]
        );
        assert_eq!(
            sim.lane(only(&c, both, 1)).predecessors(),
            [only(&c, outer, 0)]
        );
        assert_eq!(
            sim.lane(only(&c, both, 0)).successors(),
            [only(&c, inner_out, 0)]
        );
        assert_eq!(
            sim.lane(only(&c, both, 1)).successors(),
            [only(&c, outer_out, 0)]
        );
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
        let (net, road) = two_by_two();
        compile(&net).unwrap().lanes(road, 4);
    }
}
