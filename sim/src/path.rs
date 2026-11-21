//! Vehicles' paths: how far ahead each needs to know its way, and extending
//! its path by the lanes its route chooses (see "Vehicle state" and "Routing"
//! in `docs/DESIGN.md`).

use crate::models::acceleration;
use crate::models::lateral::{self, LaneChange};
use crate::models::routing::Route;
use crate::network::{LaneId, SimNetwork};
use crate::vehicle::Vehicle;

/// How far along its path, beyond its rear axle at `axle_s`, a vehicle
/// needs to know where it is going: as far as its leader search looks from its
/// front, its path tracker aims, and its lane change, `change`, runs.
pub(crate) fn reach(vehicle: &Vehicle, axle_s: f64, change: Option<LaneChange>) -> f64 {
    let nose = vehicle.spec.length - vehicle.spec.rear_axle;
    let search = acceleration::horizon(&vehicle.spec, vehicle.speed) + nose;
    // Path tracking, and planning a lane change, aim at the lookahead.
    let tracking = lateral::lookahead_distance(vehicle.speed);
    let change = change.map_or(0.0, |c| c.end() - axle_s);
    search.max(tracking).max(change)
}

/// How far ahead, in seconds at each lane's speed limit, a vehicle's path
/// always reaches: where it will be going, whatever its speed now, so a slow
/// vehicle in a queue still knows of a merge or a junction ahead. Its leader
/// search stays within its horizon, however far its path reaches.
pub(crate) const PATH_TIME: f64 = 15.0;

/// Extends `path` by the lanes `route` chooses until it reaches `needed`
/// beyond the rear axle, at `axle_s` on its first lane, and [`PATH_TIME`] at
/// the lanes' speed limits, or reaches a lane that leads nowhere. Returns how
/// far beyond the rear axle it then reaches.
///
/// A path that ends inside a junction goes on until it is out of it, so a
/// vehicle in a junction's queue knows its way through: the junction keeps
/// others clear of the lanes it will take, and one that chose its way only as
/// it went could be committed to a conflict before another learns it is in
/// the way.
pub(crate) fn extend_path(
    network: &SimNetwork,
    path: &mut Vec<LaneId>,
    route: &mut Route,
    axle_s: f64,
    needed: f64,
) -> f64 {
    // How far, and how long at the speed limits, it reaches beyond the rear axle.
    let along = |lane: LaneId, length: f64| length / network.lane(lane).speed_limit();
    let (mut covered, mut time) = (-axle_s, -along(path[0], axle_s));
    for &lane in path.iter() {
        let length = network.lane(lane).geometry().length();
        covered += length;
        time += along(lane, length);
    }
    // Bounds the lanes added to get out of a junction, should its lanes loop.
    let mut inside = 0;
    loop {
        let last = *path.last().expect("a path holds the lane it follows");
        let junction = network.lane(last).junction();
        let through = junction.is_some_and(|j| inside < network.junction(j).lanes().len());
        let enough = covered >= needed && time >= PATH_TIME;
        if enough && !through {
            break;
        }
        let Some(next) = route.choose(network.lane(last).successors()) else {
            break;
        };
        if enough {
            inside += 1;
        }
        path.push(next);
        let length = network.lane(next).geometry().length();
        covered += length;
        time += along(next, length);
    }
    covered
}

/// How far beyond each lane's end, in meters, the paths through it reach, by
/// lane: an upper bound, so a lane no path reaches some way beyond can't lead
/// anyone there. It bounds the follower search (see
/// [`crate::occupancy::find_follower`]).
///
/// Paths only grow by [`extend_path`], after which [`raise_through`] raises it;
/// a path shrinking or its vehicle leaving leaves it too high, which is safe,
/// until [`recompute_through`] makes it exact again.
pub(crate) type Through = Vec<f64>;

/// Raises `through` for `path`: each lane but the last reaches at least as far
/// beyond its end as the lanes after it on `path`.
pub(crate) fn raise_through(network: &SimNetwork, through: &mut [f64], path: &[LaneId]) {
    let mut beyond = 0.0;
    for (i, &lane) in path.iter().enumerate().rev() {
        if i + 1 < path.len() {
            let r = &mut through[lane.0 as usize];
            *r = r.max(beyond);
        }
        beyond += network.lane(lane).geometry().length();
    }
}

/// Makes `through` exact for the vehicles' paths now.
pub(crate) fn recompute_through<'a>(
    network: &SimNetwork,
    through: &mut Through,
    paths: impl Iterator<Item = &'a [LaneId]>,
) {
    through.clear();
    through.resize(network.lanes().count(), 0.0);
    for path in paths {
        raise_through(network, through, path);
    }
}

/// How far a vehicle's path reaches beyond its front, in meters.
#[cfg(test)]
pub(crate) fn reach_beyond_front(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let length: f64 = vehicle.path.iter().map(|&lane| network.lane(lane).geometry().length()).sum();
    length - vehicle.lane_pos.s - (vehicle.spec.length - vehicle.spec.rear_axle)
}
