//! The simulation's side of lane changing (see "Lane membership, lane changes
//! and diverging" in `docs/DESIGN.md`): it measures each driver's incentives,
//! checks gaps by searching the network, and plans the lateral path, while the
//! lane-change model, `models::lane_change`, decides. Also the lane speeds
//! those incentives and the friction term share.

use std::ops::ControlFlow;

use crate::lane::LanePos;
use crate::models::acceleration;
use crate::models::lane_change;
use crate::models::lateral::{self, LaneChange};
use crate::network::{ALIGNED, LaneId, Leaving, Side, SimNetwork};
use crate::occupancy::{
    self, Ahead, Along, Found, Lateral, find_follower, find_leaders, for_each_leader,
};
use crate::simulation::{SimState, speed_of};
use crate::vehicle::{Vehicle, VehicleId, VehicleSpec};

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
const ANTICIPATION: f64 = 8.0;

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
    fn count(&self, from: f64, to: f64) -> usize {
        let lo = self.s.partition_point(|&s| s <= from);
        self.s.partition_point(|&s| s <= to) - lo
    }

    /// The average speed of the vehicles from just after `from` to `to` along the
    /// lane, if any.
    fn average(&self, from: f64, to: f64) -> Option<f64> {
        let lo = self.s.partition_point(|&s| s <= from);
        let hi = self.s.partition_point(|&s| s <= to);
        (hi > lo).then(|| (self.sum[hi] - self.sum[lo]) / (hi - lo) as f64)
    }
}

/// What makes a driver want to change lanes to each side, measured cheaply,
/// every step (see [`lane_change::Incentives`]): `None` for a side it may not
/// change to there, or couldn't finish a change to before the lanes beside its
/// path run out (see [`room_to_change`]), so no gap is checked for one.
///
/// - Speed: how fast traffic goes in the lane beside it ahead, against its own
///   lane, over the next [`ANTICIPATION`] seconds at the limit, each no faster
///   than it would like to go there, and as fast as it would like where nobody
///   is.
/// - Keeping to the side its lane says.
/// - Its route: on a lane that ends by merging, left early (see
///   [`Leaving::Early`]), it wants to leave it more the nearer its end.
/// - Courtesy: letting by the follower it holds up most, by what that follower
///   would gain, weighed by its politeness, in units of its comfortable
///   deceleration.
pub(crate) fn incentives(
    state: &SimState,
    speeds: &[LaneSpeeds],
    id: VehicleId,
    vehicle: &Vehicle,
) -> [Option<lane_change::Incentives>; 2] {
    let SimState { network, vehicles, .. } = state;
    let lane_id = vehicle.path[0];
    let lane = network.lane(lane_id);
    // A side it may change to, and could finish a change to before the lanes
    // beside its path run out, or reach a junction: once it couldn't, it can't
    // again until they do, so a driver gives up wanting to.
    let permitted = |side: Side| {
        lane.adj(side)
            .filter(|n| n.can_change && room_to_change(network, vehicle, side))
    };
    if permitted(Side::Left).is_none() && permitted(Side::Right).is_none() {
        return [None, None];
    }
    let rear = vehicle.lane_pos.s - vehicle.spec.rear_axle;
    // How fast traffic goes ahead on a lane, from `s` along it.
    let anticipated = |lane: LaneId, s: f64| {
        let limit = network.lane(lane).speed_limit();
        let desired = vehicle.spec.desired_speed(limit);
        let ahead = speeds[lane.0 as usize].average(s, s + ANTICIPATION * limit);
        ahead.map_or(desired, |v| v.min(desired))
    };
    // From just past its own record, which the average leaves out.
    let own_s =
        state.occupants[lane_id.0 as usize][occupancy::record_index(state, lane_id, id)].s();
    let own = anticipated(lane_id, own_s);
    // Leaving a lane that ends: toward the neighbour that goes on.
    let route = match lane.leaving() {
        Some(Leaving::Early) => {
            let distance = lane.geometry().length() - vehicle.lane_pos.s;
            let desire = lane_change::route_desire(distance, vehicle.speed, 1);
            let goes_on = |side: Side| {
                permitted(side).is_some_and(|n| network.lane(n.lane).leaving().is_none())
            };
            if goes_on(Side::Left) {
                [desire, -desire]
            } else if goes_on(Side::Right) {
                [-desire, desire]
            } else {
                [0.0; 2]
            }
        }
        _ => [0.0; 2],
    };
    let courtesy = vehicle.old_follower.and_then(|held| {
        let follower = vehicles.get(held.id)?;
        let gain = (held.without - follower.accel).max(0.0);
        Some(vehicle.spec.politeness * gain / vehicle.spec.comfy_decel)
    });
    [Side::Left, Side::Right].map(|side| {
        let beside = permitted(side)?;
        let other = network.lane(beside.lane);
        let s = rear / lane.geometry().length() * other.geometry().length();
        let speed = (anticipated(beside.lane, s) - own) / lane_change::V_GAIN;
        let keep = if lane.keep().side() == Some(side) {
            lane_change::D_FREE
        } else {
            0.0
        };
        Some(lane_change::Incentives {
            route: route[lane_change::index(side)],
            speed,
            keep,
            courtesy: courtesy.unwrap_or(0.0).min(1.0),
        })
    })
}

/// Checks the gap for a change to `side`, at `desire`: what LMRS's gap
/// acceptance weighs. Its acceleration in the target lane, as if already
/// settled there, level with where it is, and its new follower's with it as a
/// leader, both with their time headways shortened by its desire (see
/// [`lane_change::headway`]); and whether it would be halfway through the
/// change before it had to stop behind its leader. A speed cap holds it back
/// in any lane, so it does after changing too.
pub(crate) fn check_gap(
    state: &SimState,
    through: &[f64],
    id: VehicleId,
    vehicle: &Vehicle,
    side: Side,
    desire: f64,
) -> lane_change::Prospect {
    let SimState { network, vehicles, .. } = state;
    let headway = lane_change::headway(desire);
    let cap = state.caps.iter().find(|&&(capped, _)| capped == id);
    let cap = cap.map_or(f64::INFINITY, |&(_, max_speed)| {
        acceleration::capped(vehicle.speed, max_speed)
    });
    let own_after = accel_along(state, id, vehicle, Along::Into(side), true, headway).min(cap);
    // The new follower, behind it in the target lane: the lane is split at its
    // rear there, as its search there splits it.
    let target = network.lane(vehicle.path[0]).adj(side).expect("asked only where permitted");
    let rear = occupancy::axle_beside(network, vehicle, target.lane) - vehicle.spec.rear_axle;
    let new_follower =
        find_follower(state, through, target.lane, rear, id).map(|(follower, gap)| {
            let f = &vehicles[follower];
            acceleration::relaxed(&f.spec, f.speed, gap, vehicle.speed, headway)
        });
    // Halfway through before it would have to stop behind its leader.
    let own = occupancy::record_index(state, vehicle.path[0], id);
    let mut stop = f64::INFINITY;
    let _ = find_leaders(state, vehicle, Along::Path(own), |found| match found {
        Found::Ahead(Ahead::Vehicle(leader), gap, _) if leader != id => {
            stop = room_to_stop(&vehicle.spec, gap, &vehicles[leader]);
            ControlFlow::Break(())
        }
        Found::Ahead(Ahead::PathEnd, gap, _) => {
            stop = gap - vehicle.spec.min_gap;
            ControlFlow::Break(())
        }
        _ => ControlFlow::Continue(()),
    });
    let room = stop >= halfway(network, vehicle);
    lane_change::Prospect { own_after, new_follower, room }
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
fn room_to_change(network: &SimNetwork, vehicle: &Vehicle, side: Side) -> bool {
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

/// How far a vehicle's rear axle must go to be halfway through a lane change
/// started now: the change starts at its front axle, a wheelbase ahead, and
/// runs its length.
pub(crate) fn halfway(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let width = lane_width(network, vehicle);
    vehicle.spec.wheelbase() + 0.5 * lateral::change_length(vehicle.speed, width)
}

/// Whether a vehicle wants to change lanes: it is indicating, or has built up
/// enough advantage for a side to keep indicating to it.
pub(crate) fn wants_change(vehicle: &Vehicle) -> bool {
    vehicle.intent.is_some() || vehicle.lc.wants_change(vehicle.spec.assertiveness)
}

/// As [`halfway`], for a change started from rest: as sharp as it can steer.
/// The room a vehicle that wants to change lanes keeps so it isn't boxed in:
/// at speed, its leader's stopping distance covers it.
pub(crate) fn halfway_from_rest(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let width = lane_width(network, vehicle);
    vehicle.spec.wheelbase() + 0.5 * lateral::min_change_length(width, 0.0)
}

/// How far a vehicle with `spec` could go before it had to stop behind
/// `leader`, `gap` ahead: to its minimum gap behind where the leader would
/// stop, braking comfortably from now.
fn room_to_stop(spec: &VehicleSpec, gap: f64, leader: &Vehicle) -> f64 {
    gap - spec.min_gap + comfortable_stop(leader)
}

/// How far a vehicle goes braking to a stop comfortably from its speed now.
pub(crate) fn comfortable_stop(vehicle: &Vehicle) -> f64 {
    vehicle.speed * vehicle.speed / (2.0 * vehicle.spec.comfy_decel)
}

/// How far beyond its rear axle a lane change a vehicle started now reaches
/// before its whole body is in the new lane: the change starts at its front
/// axle, a wheelbase ahead, and the body trails the axle.
fn change_reach(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    vehicle.spec.wheelbase()
        + lateral::change_length(vehicle.speed, lane_width(network, vehicle))
        + vehicle.spec.length
}

/// The width of the lane a vehicle follows, where it is: about how far across
/// a change into the lane beside it goes.
fn lane_width(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let geometry = network.lane(vehicle.path[0]).geometry();
    geometry.half_width(vehicle.lane_pos, Side::Left)
        + geometry.half_width(vehicle.lane_pos, Side::Right)
}

/// A vehicle's acceleration under the longitudinal model searching `along` some
/// lanes, against the free road there, its leaders and the end of its path.
/// Ghosts count as leaders if `ghosts_lead`, as when
/// weighing up a change into their lane: a vehicle already indicating into the
/// gap goes first. Its own ghost, `id`'s, is passed over. It keeps at most
/// `headway`, a share, of its time headway to each leader.
fn accel_along(
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

/// How a vehicle not already changing lanes would change into `target`, a lane
/// beside the one it is on: its rear axle's position on `target`, and the cubic
/// its path would follow onto `target`'s centerline, as [`Simulation::change_lane`]
/// carries it out.
///
/// The quintic starts where the front axle is on the vehicle's current path,
/// with that path's offset and slope relative to `target`, so the target point
/// carries on smoothly instead of jumping.
pub(crate) fn plan_change(
    network: &SimNetwork,
    vehicle: &Vehicle,
    target: LaneId,
) -> (LanePos, LaneChange) {
    let current = network.lane(vehicle.path[0]);
    let lane = network.lane(target);
    // Adjacent lanes start and end together, so the same fraction is a good hint.
    let hint = lane.geometry().lane_pos(
        vehicle.lane_pos.s / current.geometry().length() * lane.geometry().length(),
        0.0,
    );
    let lane_pos = lane.geometry().world_to_lane_near(vehicle.pos, hint);
    // From the front axle, where it is on its path's spline, and the way the
    // spline runs there, so the front axle's course doesn't kink.
    let along = |s: f64| network.smooth_point_along(&vehicle.path, s, |_| 0.0);
    let aim = along(vehicle.front_s);
    let heading = (along(vehicle.front_s + 0.01) - along(vehicle.front_s - 0.01)).normalize();
    // Where on the new lane, and how far across it, its rounded course runs
    // through the front axle: from the polyline's answer, nudged along and
    // across by what is left over, twice, as the corners' rounding is small.
    let start = lane.geometry().world_to_lane_near(aim, lane_pos);
    let on_new = |s: f64, t: f64| network.smooth_point_along(&[target], s, |_| t);
    let (mut s0, mut t0) = (start.s, start.t);
    for _ in 0..2 {
        let tangent = lane.geometry().tangent_at(lane.geometry().lane_pos(s0, t0));
        let miss = aim - on_new(s0, t0);
        s0 += miss.dot(tangent);
        t0 += miss.dot(tangent.perp());
    }
    // How its offset must change to head where the front axle heads: against the
    // new lane's rounded course there.
    let along_new = (on_new(s0 + 0.01, t0) - on_new(s0 - 0.01, t0)).normalize();
    let slope = along_new.perp_dot(heading) / along_new.dot(heading);
    (lane_pos, LaneChange::new(s0, t0, slope, vehicle.speed))
}
