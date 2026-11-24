//! The routing model: which lane a vehicle takes next, where the lane it is
//! extending its path from has several successors, and the lane changes its
//! route needs, which the lane-change model turns into desire. Replaced by
//! editing this module.

mod table;

pub(crate) use table::Table;

use slotmap::SecondaryMap;

use crate::network::{LaneId, Leaving, Neighbor, Side, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// Identifies a destination registered with the simulation (see
/// [`Simulation::add_destination`](crate::Simulation::add_destination)).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct DestinationId(pub u32);

/// Each destination's route table, by destination: derived from the network
/// and the destinations' lanes, rather than saved.
pub(crate) type Tables = Vec<Table>;

/// The route tables for `destinations`, each a set of lanes.
pub(crate) fn tables(network: &SimNetwork, destinations: &[Vec<LaneId>]) -> Tables {
    destinations.iter().map(|lanes| Table::build(network, lanes)).collect()
}

/// The routing model's state for every vehicle, which it owns: each one's
/// route.
#[derive(Clone, Debug, Default)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub(crate) struct Model {
    routes: SecondaryMap<VehicleId, Route>,
}

impl Model {
    /// Takes on a vehicle just inserted, with the route it was given.
    pub fn insert(&mut self, id: VehicleId, route: Route) {
        self.routes.insert(id, route);
    }

    /// Lets go of a vehicle removed.
    pub fn remove(&mut self, id: VehicleId) {
        self.routes.remove(id);
    }

    /// The vehicle's route, if there is such a vehicle.
    pub fn get(&self, id: VehicleId) -> Option<Route> {
        self.routes.get(id).copied()
    }

    /// The vehicle's route, to choose the lanes it adds to its path.
    pub fn route_mut(&mut self, id: VehicleId) -> &mut Route {
        &mut self.routes[id]
    }
}

/// How a vehicle chooses its way through the network.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Route {
    /// A successor chosen at random at each fork, by a seeded generator held
    /// here, so the same seed always takes the same way.
    Random { state: u64 },
    /// The quickest way to a destination, by its route table.
    Destination(DestinationId),
}

impl Route {
    /// Random choices from `seed`.
    pub fn random(seed: u64) -> Self {
        Route::Random { state: seed }
    }

    /// The quickest way to `destination`.
    pub fn to(destination: DestinationId) -> Self {
        Route::Destination(destination)
    }

    /// The successor to take next, if there is one. Only a fork, with more than
    /// one, draws from a random route's generator. Toward a destination, the
    /// quickest, then the one needing the fewest lane changes from its start,
    /// ties going to the first, so runs repeat; with no way from any, the first.
    pub(crate) fn choose(&mut self, tables: &Tables, successors: &[LaneId]) -> Option<LaneId> {
        match (self, successors) {
            (_, []) => None,
            (_, &[only]) => Some(only),
            (Route::Random { state }, _) => {
                let draw = splitmix64(state);
                Some(successors[(draw % successors.len() as u64) as usize])
            }
            (Route::Destination(destination), _) => {
                let table = &tables[destination.0 as usize];
                let key = |&lane: &LaneId| {
                    let entry = table.get(lane);
                    (entry.time, entry.changes_at(0.0).unwrap_or(u32::MAX))
                };
                let mut best = successors[0];
                for lane in &successors[1..] {
                    let (a, b) = (key(lane), key(&best));
                    if a.0 < b.0 || (a.0 == b.0 && a.1 < b.1) {
                        best = *lane;
                    }
                }
                Some(best)
            }
        }
    }
}

impl Default for Route {
    /// Random choices from seed 0.
    fn default() -> Self {
        Route::random(0)
    }
}

/// The lane changes a route still needs from a point on a lane: `changes` of
/// them, all done within `distance` ahead of the rear axle, or no limit if it
/// needs none.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Need {
    pub changes: u32,
    pub distance: f64,
}

impl Need {
    const NONE: Need = Need { changes: 0, distance: f64::INFINITY };
}

/// What a vehicle's route needs from where it is, `here`, and from level with
/// it on each lane beside it that it may change to, `left` and `right`: `None`
/// where it has no way, or the side isn't one it may change to.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Needs {
    pub here: Option<Need>,
    pub left: Option<Need>,
    pub right: Option<Need>,
}

impl Needs {
    /// What it needs from beside it on `side`.
    pub fn beside(&self, side: Side) -> Option<Need> {
        match side {
            Side::Left => self.left,
            Side::Right => self.right,
        }
    }
}

/// What `vehicle`'s route, `route`, needs from where it is and from beside it,
/// on the neighbours it may change to on each side, `permitted`; `None` if it
/// needs nothing of any of them.
///
/// Toward a destination, as its route table has them. On a random route, on a
/// lane that ends by merging, left early (see [`Leaving::Early`]), one change
/// toward the neighbour that goes on, by its end; nothing more, every lane
/// leading somewhere.
pub(crate) fn needs(
    network: &SimNetwork,
    tables: &Tables,
    route: Route,
    vehicle: &Vehicle,
    permitted: impl Fn(Side) -> Option<Neighbor>,
) -> Option<Needs> {
    if let Route::Destination(destination) = route {
        return Some(needs_toward(
            network,
            &tables[destination.0 as usize],
            vehicle,
            permitted,
        ));
    }
    let lane = network.lane(vehicle.path[0]);
    if lane.leaving() != Some(Leaving::Early) {
        return None;
    }
    let distance = lane.geometry().length() - vehicle.lane_pos.s;
    let here = Need { changes: 1, distance };
    // Beside it, none if the lane goes on, and as here if not.
    let beside = |side: Side| {
        let n = permitted(side)?;
        Some(match network.lane(n.lane).leaving() {
            None => Need::NONE,
            Some(_) => here,
        })
    };
    Some(Needs {
        here: Some(here),
        left: beside(Side::Left),
        right: beside(Side::Right),
    })
}

/// What a vehicle needs toward a destination, by its `table`.
fn needs_toward(
    network: &SimNetwork,
    table: &Table,
    vehicle: &Vehicle,
    permitted: impl Fn(Side) -> Option<Neighbor>,
) -> Needs {
    let lane_id = vehicle.path[0];
    let s = vehicle.lane_pos.s;
    let length = network.lane(lane_id).geometry().length();
    let at = |lane: LaneId, s: f64| {
        let entry = table.get(lane);
        let changes = entry.changes_at(s)?;
        Some(match changes {
            0 => Need::NONE,
            k => Need {
                changes: k,
                distance: f64::from(entry.reach[k as usize - 1]) - s,
            },
        })
    };
    // Level with it on the lane beside, as the rest of the core maps between them.
    let beside = |side: Side| {
        let n = permitted(side)?;
        at(
            n.lane,
            s / length * network.lane(n.lane).geometry().length(),
        )
    };
    Needs {
        here: at(lane_id, s),
        left: beside(Side::Left),
        right: beside(Side::Right),
    }
}

/// The next number from a SplitMix64 generator, advancing its state: small,
/// fast and good enough to pick a branch.
pub(crate) fn splitmix64(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

#[cfg(test)]
mod test {
    use super::*;

    const FORK: [LaneId; 3] = [LaneId(4), LaneId(5), LaneId(6)];

    #[test]
    fn the_same_seed_takes_the_same_way() {
        let (mut a, mut b) = (Route::random(7), Route::random(7));
        for _ in 0..20 {
            assert_eq!(
                a.choose(&Tables::new(), &FORK),
                b.choose(&Tables::new(), &FORK)
            );
        }
    }

    #[test]
    fn every_branch_gets_taken_and_only_forks_draw() {
        let mut taken = [0; 3];
        for seed in 0..300 {
            let mut route = Route::random(seed);
            // A lone successor is taken without drawing.
            assert_eq!(route.choose(&Tables::new(), &[LaneId(1)]), Some(LaneId(1)));
            assert_eq!(route, Route::random(seed));
            let lane = route.choose(&Tables::new(), &FORK).unwrap();
            taken[(lane.0 - 4) as usize] += 1;
        }
        assert!(taken.iter().all(|&n| n > 50), "{taken:?}");
        assert_eq!(Route::random(0).choose(&Tables::new(), &[]), None);
    }
}
