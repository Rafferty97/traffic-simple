//! The simulation's side of lane changing (see "Lane membership, lane changes
//! and diverging" in `docs/DESIGN.md`): what the lane-change model,
//! `models::lane_change`, may ask of the network. Where a change may run, the
//! speeds traffic goes in each lane ahead, and a vehicle's acceleration
//! searching along other lanes; also the friction term those lane speeds feed.

use crate::lateral::change_reach;
use crate::models::acceleration;
use crate::network::{ALIGNED, LaneId, Side, SimNetwork};
use crate::occupancy::{Ahead, Along, Found, Lateral, for_each_leader};
use crate::simulation::{SimState, speed_of};
use crate::vehicle::{Vehicle, VehicleId};

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

/// Whether a change to `side` that a vehicle started now would be over, its
/// whole body in the new lane, before the lanes beside its path on that side
/// run out. They run out where there is no neighbor, where the neighbor stops
/// running beside it, as at a skewed stop line, and at a junction: where a
/// lane of its path or the one beside it is in one, whatever neighbors the
/// network gives them. So no change starts in a junction or runs into one,
/// which would put the vehicle where the junction doesn't know it is, partway
/// across its lanes. SUMO likewise allows no lane changes on a junction's
/// internal lanes.
pub(crate) fn room_to_change(network: &SimNetwork, vehicle: &Vehicle, side: Side) -> bool {
    let needed = change_reach(network, vehicle);
    let mut room = -vehicle.lane_pos.s;
    let in_junction = |lane: LaneId| network.lane(lane).junction().is_some();
    for (k, &lane) in vehicle.path.iter().enumerate() {
        if room >= needed {
            return true;
        }
        let beside = network.lane(lane).adj(side);
        if in_junction(lane) || beside.is_none_or(|n| in_junction(n.lane)) {
            return false;
        }
        let beside = beside.expect("checked");
        // Beside it only from part way along: not yet, on the lane it is on, or
        // not where the change would run onto it, on one after.
        let starts_late = match k {
            0 => vehicle.lane_pos.s < beside.from,
            _ => beside.from > ALIGNED,
        };
        if starts_late {
            return false;
        }
        let length = network.lane(lane).geometry().length();
        room += beside.until;
        if beside.until < length - ALIGNED {
            return room >= needed;
        }
        room += length - beside.until;
    }
    room >= needed
}

/// How far a vehicle goes braking to a stop comfortably from its speed now.
pub(crate) fn comfortable_stop(vehicle: &Vehicle) -> f64 {
    vehicle.speed * vehicle.speed / (2.0 * vehicle.spec.comfy_decel)
}

/// A vehicle's acceleration under the longitudinal model searching `along` some
/// lanes, against the free road there, its leaders and the end of its path.
/// Ghosts count as leaders if `ghosts_lead`, as when
/// weighing up a change into their lane: a vehicle already indicating into the
/// gap goes first. Its own ghost, `id`'s, is passed over. It keeps at most
/// `headway`, a share, of its time headway to each leader.
pub(crate) fn accel_along(
    state: &SimState,
    id: VehicleId,
    vehicle: &Vehicle,
    along: Along,
    ghosts_lead: bool,
    headway: f64,
) -> f64 {
    let lane = along.lane(&state.network, vehicle, 0).unwrap_or(vehicle.path[0]);
    let desired = vehicle.spec.desired_speed(state.network.lane(lane).speed_limit());
    let mut accel = acceleration::free_road(&vehicle.spec, vehicle.speed, desired);
    for_each_leader(state, vehicle, along, |found| match found {
        Found::Ahead(Ahead::Ghost(owner), ..) if owner == id || !ghosts_lead => {}
        Found::Ahead(ahead, gap, relaxed) => {
            let speed = speed_of(&state.vehicles, ahead);
            let share = relaxed.min(headway);
            let term = acceleration::relaxed(&vehicle.spec, vehicle.speed, gap, speed, share);
            accel = accel.min(term);
        }
        Found::Pass(_) => {}
    });
    accel
}
