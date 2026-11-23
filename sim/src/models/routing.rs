//! The routing model: which lane a vehicle takes next, where the lane it is
//! extending its path from has several successors, and the lane changes its
//! route needs, which the lane-change model turns into desire. Replaced by
//! editing this module.

use crate::network::{LaneId, Leaving, Neighbor, Side, SimNetwork};
use crate::vehicle::Vehicle;

/// How a vehicle chooses its way through the network.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Route {
    /// A successor chosen at random at each fork, by a seeded generator held
    /// here, so the same seed always takes the same way.
    Random { state: u64 },
}

impl Route {
    /// Random choices from `seed`.
    pub fn random(seed: u64) -> Self {
        Route::Random { state: seed }
    }

    /// The successor to take next, if there is one. Only a fork, with more than
    /// one, draws from the generator.
    pub(crate) fn choose(&mut self, successors: &[LaneId]) -> Option<LaneId> {
        match (self, successors) {
            (_, []) => None,
            (_, &[only]) => Some(only),
            (Route::Random { state }, _) => {
                let draw = splitmix64(state);
                Some(successors[(draw % successors.len() as u64) as usize])
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

/// Lane changes a vehicle's route needs from where it is: `changes` of them
/// toward `side`, the first by `distance` ahead of its rear axle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Needed {
    pub side: Side,
    pub changes: u32,
    pub distance: f64,
}

/// The lane changes `vehicle`'s route needs, if any, given the neighbours it
/// may change to on each side, `permitted`. On a lane that ends by merging,
/// left early (see [`Leaving::Early`]), one toward the neighbour that goes on,
/// by its end; random routes need nothing more, every lane leading somewhere.
pub(crate) fn needed(
    network: &SimNetwork,
    vehicle: &Vehicle,
    permitted: impl Fn(Side) -> Option<Neighbor>,
) -> Option<Needed> {
    let lane = network.lane(vehicle.path[0]);
    if lane.leaving() != Some(Leaving::Early) {
        return None;
    }
    let goes_on =
        |side: Side| permitted(side).is_some_and(|n| network.lane(n.lane).leaving().is_none());
    let side = [Side::Left, Side::Right].into_iter().find(|&side| goes_on(side))?;
    let distance = lane.geometry().length() - vehicle.lane_pos.s;
    Some(Needed { side, changes: 1, distance })
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
            assert_eq!(a.choose(&FORK), b.choose(&FORK));
        }
    }

    #[test]
    fn every_branch_gets_taken_and_only_forks_draw() {
        let mut taken = [0; 3];
        for seed in 0..300 {
            let mut route = Route::random(seed);
            // A lone successor is taken without drawing.
            assert_eq!(route.choose(&[LaneId(1)]), Some(LaneId(1)));
            assert_eq!(route, Route::random(seed));
            let lane = route.choose(&FORK).unwrap();
            taken[(lane.0 - 4) as usize] += 1;
        }
        assert!(taken.iter().all(|&n| n > 50), "{taken:?}");
        assert_eq!(Route::random(0).choose(&[]), None);
    }
}
