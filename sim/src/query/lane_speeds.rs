//! How fast traffic goes ahead in each lane, from one pass over the occupant
//! lists per update: what drivers weigh up changing lanes for, and the
//! friction that slows them beside slow traffic.

use crate::following::Lateral;
use crate::network::{LaneId, Side};
use crate::simulation::SimState;
use crate::vehicle::Vehicle;

/// How much faster, in m/s, a driver goes than traffic in the lane beside it, at
/// most: 30 km/h. Drivers slow passing much slower traffic, the friction
/// effect Jang and Cassidy (2012) measured in carpool lanes beside congested
/// ones.
const FRICTION_MARGIN: f64 = 30.0 / 3.6;

/// The fewest vehicles in the stretch ahead beside it that count as traffic
/// for friction.
const FRICTION_TRAFFIC: usize = 3;

/// The fastest a vehicle will go for the traffic beside it (see
/// [`FRICTION_MARGIN`]): the average speed ahead in each lane beside its own,
/// over the next [`ANTICIPATION`] seconds at that lane's limit, plus the
/// margin. Unbounded with nobody there.
pub(crate) fn friction(state: &SimState, speeds: &[LaneSpeeds], vehicle: &Vehicle) -> f64 {
    let lane = state.network.lane(vehicle.path[0]);
    let rear = vehicle.lane_pos.s - vehicle.spec.rear_axle;
    let mut fastest = f64::INFINITY;
    for side in [Side::Left, Side::Right] {
        let Some(beside) = lane.adj(side) else {
            continue;
        };
        let Some(speeds) = speeds.get(beside.lane.0 as usize) else {
            continue;
        };
        let other = state.network.lane(beside.lane);
        let s = rear / lane.geometry().length() * other.geometry().length();
        let ahead = s + ANTICIPATION * other.speed_limit();
        // Traffic, not a vehicle or two.
        if speeds.count(s, ahead) >= FRICTION_TRAFFIC
            && let Some(v) = speeds.average(s, ahead)
        {
            fastest = fastest.min(v + FRICTION_MARGIN);
        }
    }
    fastest
}

/// How far ahead, in seconds at the lane's speed limit, a driver looks to judge
/// how fast traffic goes in each lane: LMRS's anticipation.
pub(crate) const ANTICIPATION: f64 = 8.0;

/// Each lane's vehicles in order along it, with a running sum of their speeds,
/// so the average speed over a stretch ahead is two binary searches. Only lanes
/// with neighbours are filled: elsewhere nobody changes lanes.
#[derive(Clone, Debug, Default)]
pub(crate) struct LaneSpeeds {
    /// Each vehicle's record's `s`, in order.
    s: Vec<f64>,
    /// The sum of the speeds of the vehicles before each, and of them all.
    sum: Vec<f64>,
}

impl LaneSpeeds {
    /// Refills `all`, one per lane, from the state: every vehicle in each lane,
    /// in it or converging on it, not ghosts or vehicles leaving it.
    pub(crate) fn refresh(all: &mut Vec<LaneSpeeds>, state: &SimState) {
        all.resize_with(state.occupants.len(), LaneSpeeds::default);
        for (i, (speeds, list)) in all.iter_mut().zip(&state.occupants).enumerate() {
            speeds.s.clear();
            speeds.sum.clear();
            let lane = state.network.lane(LaneId(i as u32));
            if lane.adj_left().is_none() && lane.adj_right().is_none() {
                continue;
            }
            let mut sum = 0.0;
            speeds.sum.push(sum);
            for o in list {
                if matches!(o.lateral, Lateral::Ghost | Lateral::Diverging(_)) {
                    continue;
                }
                sum += state.vehicles[o.id].speed;
                speeds.s.push(o.s());
                speeds.sum.push(sum);
            }
        }
    }

    /// How many vehicles there are from just after `from` to `to` along the lane.
    pub(crate) fn count(&self, from: f64, to: f64) -> usize {
        let lo = self.s.partition_point(|&s| s <= from);
        self.s.partition_point(|&s| s <= to) - lo
    }

    /// The average speed of the vehicles from just after `from` to `to` along the
    /// lane, if any.
    pub(crate) fn average(&self, from: f64, to: f64) -> Option<f64> {
        let lo = self.s.partition_point(|&s| s <= from);
        let hi = self.s.partition_point(|&s| s <= to);
        (hi > lo).then(|| (self.sum[hi] - self.sum[lo]) / (hi - lo) as f64)
    }
}
