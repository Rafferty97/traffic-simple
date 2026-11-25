//! Route tables: for a destination, how long it takes to reach from the start
//! of every lane, and how far along each lane a vehicle can go and still reach
//! it with one, two, three or four lane changes (see "Destination" in
//! `docs/models/routing.md`).

use std::cmp::{Ordering, Reverse};
use std::collections::{BinaryHeap, HashMap};

use crate::class::VehicleClass;
use crate::models::driver::Driver;
use crate::motion::lateral::change_length;
use crate::network::{LaneId, Leaving, Neighbor, RoadId, Side, SimNetwork};
use crate::vehicle::VehicleSpec;

/// The most lane changes a table counts: a way that needs more counts as
/// needing this many.
pub(crate) const MAX_CHANGES: usize = 4;

/// A destination's route table, by lane.
#[derive(Clone, Debug)]
pub(crate) struct Table {
    lanes: Vec<Entry>,
}

/// What a route table says of one lane.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Entry {
    /// The time to the destination from the lane's start at the speed limits,
    /// in seconds: infinite if there is no way.
    pub time: f32,
    /// For `k` lane changes, at `k − 1`: the furthest along the lane a
    /// vehicle's rear axle can be and still reach the destination with `k`, in
    /// meters along it, beyond its end if the changes are on lanes after it:
    /// infinite if it needs none, and negative infinity if `k` aren't enough.
    pub reach: [f32; MAX_CHANGES],
}

impl Entry {
    const NONE: Entry = Entry { time: f32::INFINITY, reach: [f32::NEG_INFINITY; MAX_CHANGES] };

    /// How many lane changes a vehicle with its rear axle at `s` along the lane
    /// still needs, if it can reach the destination at all.
    pub fn changes_at(&self, s: f64) -> Option<u32> {
        if self.reach[0] == f32::INFINITY {
            return Some(0);
        }
        (1..=MAX_CHANGES as u32).find(|&k| s <= f64::from(self.reach[k as usize - 1]))
    }
}

impl Table {
    /// What the table says of `lane`.
    pub fn get(&self, lane: LaneId) -> &Entry {
        &self.lanes[lane.0 as usize]
    }

    /// The table for a destination of `lanes` over `network`, by Dijkstra's
    /// algorithm backward from it over the lanes' times at their speed limits,
    /// each road's time counted once (see [`costs`]), so lanes of the same road
    /// are equal but for the lane changes they need.
    ///
    /// A lane change costs no time, but a way through one needs room for it,
    /// before the lanes run out beside each other and before the next change it
    /// needs, so a way that needs more changes than there is room for isn't
    /// one. Between ways equally quick, the one with fewer changes wins. A
    /// lane that ends by merging and is left early leads nowhere: it must be
    /// left by its end.
    ///
    /// Each lane's reach comes from the ways that are quickest from it, so a
    /// vehicle wants the lane changes that take it the quickest way, and where
    /// it can no longer make them, the table says it has no way from where it
    /// is, and it takes the quickest way from wherever its lanes go.
    ///
    /// Only over the lanes `class` may use: a way that takes any other isn't
    /// one.
    pub fn build(network: &SimNetwork, lanes: &[LaneId], class: VehicleClass) -> Self {
        let count = network.lanes().count();
        let mut best = vec![(f64::INFINITY, u32::MAX); count];
        let mut reach = vec![[f64::NEG_INFINITY; MAX_CHANGES]; count];
        let mut done = vec![false; count];
        let mut heap = BinaryHeap::new();
        let allowed = |lane: LaneId| network.lane(lane).allows().contains(class);
        for &lane in lanes.iter().filter(|&&lane| allowed(lane)) {
            best[lane.0 as usize] = (0.0, 0);
            heap.push(Reverse(Label { time: 0.0, changes: 0, lane }));
        }
        let cost = costs(network);
        let travel = |lane: LaneId| cost[lane.0 as usize];
        let is_destination = |lane: LaneId| lanes.contains(&lane);

        while let Some(Reverse(Label { time, changes, lane })) = heap.pop() {
            let i = lane.0 as usize;
            if done[i] || best[i] != (time, changes) {
                continue;
            }
            done[i] = true;
            reach[i] = if is_destination(lane) {
                [f64::INFINITY; MAX_CHANGES]
            } else {
                reach_from(network, lane, time, cost[i], &best, &reach, &done)
            };

            let mut relax = |from: LaneId, label: (f64, u32), heap: &mut BinaryHeap<_>| {
                let j = from.0 as usize;
                if allowed(from) && !done[j] && cmp(label, best[j]) == Ordering::Less {
                    best[j] = label;
                    let (time, changes) = label;
                    heap.push(Reverse(Label { time, changes, lane: from }));
                }
            };
            // The lanes leading into it, unless they are left early.
            for &from in network.lane(lane).predecessors() {
                if network.lane(from).leaving() != Some(Leaving::Early) {
                    relax(from, (travel(from) + time, changes), &mut heap);
                }
            }
            // The lanes beside it that may change into it, with room to.
            for side in [Side::Left, Side::Right] {
                let Some((from, n)) = changing_into(network, lane, side) else {
                    continue;
                };
                let room = latest_change(network, from, n, reach[i][MAX_CHANGES - 1]);
                if room >= n.from {
                    relax(from, (time, changes.saturating_add(1)), &mut heap);
                }
            }
        }

        let lanes = (0..count)
            .map(|i| match done[i] {
                true => Entry { time: best[i].0 as f32, reach: reach[i].map(|r| r as f32) },
                false => Entry::NONE,
            })
            .collect();
        Table { lanes }
    }
}

/// A lane's reach (see [`Entry::reach`]), from the lanes after it and beside
/// it already done, through those its quickest ways go by, `time` being its
/// own time to the destination and `travel` its cost.
fn reach_from(
    network: &SimNetwork,
    lane_id: LaneId,
    time: f64,
    travel: f64,
    best: &[(f64, u32)],
    reach: &[[f64; MAX_CHANGES]],
    done: &[bool],
) -> [f64; MAX_CHANGES] {
    let lane = network.lane(lane_id);
    let length = lane.geometry().length();
    let mut out = [f64::NEG_INFINITY; MAX_CHANGES];
    let quickest = |other: LaneId, via: f64| {
        let j = other.0 as usize;
        done[j] && close(via + best[j].0, time)
    };
    if lane.leaving() != Some(Leaving::Early) {
        for &next in lane.successors() {
            if quickest(next, travel) {
                for (r, beyond) in out.iter_mut().zip(reach[next.0 as usize]) {
                    *r = r.max(length + beyond);
                }
            }
        }
    }
    for side in [Side::Left, Side::Right] {
        let Some(n) = lane.adj(side).filter(|n| n.can_change) else {
            continue;
        };
        if !quickest(n.lane, 0.0) {
            continue;
        }
        let theirs = reach[n.lane.0 as usize];
        for k in 0..MAX_CHANGES {
            // With one fewer change from the lane beside, or as many, counting
            // more than the most as the most.
            let mut after = match k {
                0 if theirs[0] == f64::INFINITY => f64::INFINITY,
                0 => f64::NEG_INFINITY,
                _ => theirs[k - 1],
            };
            if k == MAX_CHANGES - 1 {
                after = after.max(theirs[k]);
            }
            let latest = latest_change(network, lane_id, n, after);
            if latest >= n.from {
                out[k] = out[k].max(latest);
            }
        }
    }
    for k in 1..MAX_CHANGES {
        out[k] = out[k].max(out[k - 1]);
    }
    out
}

/// The time each lane costs, by lane: its road's time, from where the road
/// starts, on each lane that starts the road, and nothing on the others, so
/// every way through a road costs the same. A road's time is its lanes' time
/// at their speed limits, shared among the lanes that start it, which is their
/// mean where each lane runs the road's length, as a road's lanes do, and the
/// mean of the runs through it where they are split, as a junction's
/// connectors waiting to turn are. A lane of a road every lane of which
/// follows another of it, as a ring's does, costs its own time.
fn costs(network: &SimNetwork) -> Vec<f64> {
    let travel = |lane: &crate::network::Lane| lane.geometry().length() / lane.speed_limit();
    // Each road's time and the lanes that start it.
    let mut roads: HashMap<RoadId, (f64, u32)> = HashMap::new();
    let starts = |lane: &crate::network::Lane| {
        lane.predecessors().iter().all(|&p| network.lane(p).road() != lane.road())
    };
    for (_, lane) in network.lanes() {
        let (time, starting) = roads.entry(lane.road()).or_default();
        *time += travel(lane);
        *starting += u32::from(starts(lane));
    }
    network
        .lanes()
        .map(|(_, lane)| match roads[&lane.road()] {
            (_, 0) => travel(lane),
            (time, starting) if starts(lane) => time / f64::from(starting),
            _ => 0.0,
        })
        .collect()
}

/// The lane beside `lane` on `side` that may change into it, with its link to
/// `lane`.
fn changing_into(network: &SimNetwork, lane: LaneId, side: Side) -> Option<(LaneId, Neighbor)> {
    let from = network.lane(lane).adj(side)?.lane;
    let n = network.lane(from).adj(side.opposite())?;
    (n.lane == lane && n.can_change).then_some((from, n))
}

/// The furthest along `lane` a vehicle's rear axle can be and still change
/// into the lane beside it, `n`, and be done before the two run out beside
/// each other, and before `after` along that lane, where it must start its
/// next change or can't go on.
///
/// A change that would run on past where the two run out beside each other
/// into the lanes after them isn't counted: the lanes after count their own.
fn latest_change(network: &SimNetwork, lane: LaneId, n: Neighbor, after: f64) -> f64 {
    let room = change_room(network, lane);
    let ours = network.lane(lane).geometry().length();
    let theirs = network.lane(n.lane).geometry().length();
    // Along the lane beside, as the rest of the core maps between them.
    let after = after * ours / theirs;
    (n.until - room).min(after - room)
}

/// How far beyond a car's rear axle a lane change out of `lane` at its speed
/// limit reaches before its whole body is in the lane beside, as
/// [`crate::motion::lateral::change_reach`] has it.
///
/// **Suggestion:** one table per vehicle class, each with its class's room.
fn change_room(network: &SimNetwork, lane: LaneId) -> f64 {
    let car = VehicleSpec::passenger_car();
    let lane = network.lane(lane);
    let at = lane.geometry().lane_pos(0.0, 0.0);
    let width =
        lane.geometry().half_width(at, Side::Left) + lane.geometry().half_width(at, Side::Right);
    let speed = Driver::default().desired_speed(&car, lane.speed_limit());
    car.wheelbase() + change_length(speed, width) + car.length
}

/// Whether two times to the destination are the same, but for rounding.
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-9 * b.abs().max(1.0)
}

/// Orders labels by time, then by lane changes.
fn cmp(a: (f64, u32), b: (f64, u32)) -> Ordering {
    a.0.total_cmp(&b.0).then(a.1.cmp(&b.1))
}

/// A lane reached in Dijkstra's search, with its time and lane changes.
#[derive(Clone, Copy, Debug)]
struct Label {
    time: f64,
    changes: u32,
    lane: LaneId,
}

impl PartialEq for Label {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for Label {}

impl PartialOrd for Label {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Label {
    fn cmp(&self, other: &Self) -> Ordering {
        cmp((self.time, self.changes), (other.time, other.changes))
            .then(self.lane.0.cmp(&other.lane.0))
    }
}

#[cfg(test)]
mod test {
    use glam::DVec2;

    use super::*;
    use crate::network::Lane;
    use crate::simulation::test::alone;

    const WIDTH: f64 = 3.5;
    const LIMIT: f64 = 50.0 / 3.6;

    fn lane(x0: f64, x1: f64, y: f64, limit: f64) -> Lane {
        lane_of(alone(), x0, x1, y, limit)
    }

    fn lane_of(road: RoadId, x0: f64, x1: f64, y: f64, limit: f64) -> Lane {
        Lane::new(&[DVec2::new(x0, y), DVec2::new(x1, y)], WIDTH, limit, road).unwrap()
    }

    /// A road of two lanes side by side, `length` long, the left (0) and the
    /// right (1), each going on into a lane of its own, 2 and 3, 100 m long;
    /// the right at `right_limit`; both of one road if `one_road`, and
    /// otherwise each standing alone.
    fn split(length: f64, right_limit: f64, one_road: bool) -> SimNetwork {
        let road = alone();
        let road = |_| if one_road { road } else { alone() };
        let lanes = vec![
            lane_of(road(0), 0.0, length, 0.0, LIMIT),
            lane_of(road(1), 0.0, length, -WIDTH, right_limit),
            lane(length, length + 100.0, 0.0, LIMIT),
            lane(length, length + 100.0, -WIDTH, LIMIT),
        ];
        let mut network = SimNetwork::new(lanes);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network.connect(LaneId(0), LaneId(2));
        network.connect(LaneId(1), LaneId(3));
        network.validate().unwrap();
        network
    }

    #[test]
    fn a_lane_leading_elsewhere_must_be_left_with_room_before_the_lanes_beside_end() {
        let network = split(300.0, LIMIT, true);
        let table = Table::build(&network, &[LaneId(2)], VehicleClass::Car);
        let (left, right) = (table.get(LaneId(0)), table.get(LaneId(1)));
        assert_eq!(left.changes_at(0.0), Some(0));
        assert_eq!(right.changes_at(0.0), Some(1));
        // Its change done before the lanes beside it end.
        let latest = 300.0 - change_room(&network, LaneId(1));
        assert!(
            (f64::from(right.reach[0]) - latest).abs() < 1e-3,
            "{right:?}"
        );
        assert_eq!(right.changes_at(latest + 1.0), None);
        // A change costs no time.
        assert_eq!(left.time, right.time);
        // Nothing leads from the lane going elsewhere.
        assert_eq!(table.get(LaneId(3)).time, f32::INFINITY);
    }

    #[test]
    fn too_short_a_road_to_change_lanes_along_is_no_way() {
        let network = split(30.0, LIMIT, true);
        let table = Table::build(&network, &[LaneId(2)], VehicleClass::Car);
        assert_eq!(table.get(LaneId(1)).changes_at(0.0), None);
        assert_eq!(table.get(LaneId(1)).time, f32::INFINITY);
    }

    #[test]
    fn lanes_of_one_road_are_equal_however_quick_each_is() {
        // Both lanes lead to the destination, the right a little slower.
        let both = [LaneId(2), LaneId(3)];
        let table = Table::build(&split(300.0, 0.95 * LIMIT, true), &both, VehicleClass::Car);
        assert_eq!(table.get(LaneId(1)).changes_at(0.0), Some(0));
        assert_eq!(table.get(LaneId(0)).time, table.get(LaneId(1)).time);
        // Without roads, the quicker lane is the only way.
        let table = Table::build(&split(300.0, 0.95 * LIMIT, false), &both, VehicleClass::Car);
        assert_eq!(table.get(LaneId(1)).changes_at(0.0), Some(1));
    }

    #[test]
    fn several_lane_changes_each_need_their_own_room() {
        // Three lanes side by side, only the left going on to the destination.
        let length = 400.0;
        let lanes = vec![
            lane(0.0, length, 0.0, LIMIT),
            lane(0.0, length, -WIDTH, LIMIT),
            lane(0.0, length, -2.0 * WIDTH, LIMIT),
            lane(length, length + 100.0, 0.0, LIMIT),
        ];
        let mut network = SimNetwork::new(lanes);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network.set_adjacent(LaneId(1), LaneId(2), true, true);
        network.connect(LaneId(0), LaneId(3));
        network.validate().unwrap();
        let table = Table::build(&network, &[LaneId(3)], VehicleClass::Car);
        let right = table.get(LaneId(2));
        let room = change_room(&network, LaneId(0));
        assert_eq!(right.changes_at(0.0), Some(2));
        assert!((f64::from(right.reach[1]) - (length - 2.0 * room)).abs() < 1e-3);
        assert_eq!(right.reach[0], f32::NEG_INFINITY);
    }

    #[test]
    fn at_a_fork_a_route_to_a_destination_takes_the_branch_toward_it() {
        // One lane forking into two, the second branch leading on to the destination.
        let lanes = vec![
            lane(0.0, 100.0, 0.0, LIMIT),
            lane(100.0, 200.0, 5.0, LIMIT),
            lane(100.0, 200.0, -5.0, LIMIT),
            lane(200.0, 300.0, -5.0, LIMIT),
        ];
        let mut network = SimNetwork::new(lanes);
        network.connect(LaneId(0), LaneId(1));
        network.connect(LaneId(0), LaneId(2));
        network.connect(LaneId(2), LaneId(3));
        let tables = vec![super::super::Routes::build(&network, &[LaneId(3)])];
        let mut route = super::super::Route::to(super::super::DestinationId(0));
        assert_eq!(
            route.choose(
                &network,
                &tables,
                VehicleClass::Car,
                network.lane(LaneId(0)).successors()
            ),
            Some(LaneId(2))
        );
        // With no way from any branch, the first.
        let tables = vec![super::super::Routes::build(&network, &[])];
        assert_eq!(
            route.choose(
                &network,
                &tables,
                VehicleClass::Car,
                network.lane(LaneId(0)).successors()
            ),
            Some(LaneId(1))
        );
    }
}
