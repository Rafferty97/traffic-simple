//! What the lane-change model measures for each driver, by searching the
//! network through the core's queries: its incentives, and the gap for a change.

use std::ops::ControlFlow;

use crate::lane_changing::{
    ANTICIPATION, LaneSpeeds, accel_along, comfortable_stop, room_to_change,
};
use crate::lateral::halfway;
use crate::models::acceleration;
use crate::models::lane_change;
use crate::models::routing;
use crate::network::{LaneId, Side};
use crate::occupancy::{self, Ahead, Along, Found, find_follower, find_leaders};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId, VehicleSpec};

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
/// - Its route: toward the lane changes it needs (see [`routing::needed`]),
///   the more the nearer the point by which they must be done.
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
    // Toward the side its route needs, and away from the other.
    let route = match routing::needed(network, vehicle, permitted) {
        Some(needed) => {
            let desire = lane_change::route_desire(needed.distance, vehicle.speed, needed.changes);
            let mut route = [-desire; 2];
            route[lane_change::index(needed.side)] = desire;
            route
        }
        None => [0.0; 2],
    };
    let courtesy = state.lane_changers[id].old_follower.and_then(|held| {
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

/// Whether a vehicle wants to change lanes: it is indicating, or has built up
/// enough advantage for a side to keep indicating to it.
pub(crate) fn wants_change(state: &SimState, id: VehicleId, vehicle: &Vehicle) -> bool {
    vehicle.intent.is_some() || state.lane_changers[id].lc.wants_change(vehicle.spec.assertiveness)
}

/// How far a vehicle with `spec` could go before it had to stop behind
/// `leader`, `gap` ahead: to its minimum gap behind where the leader would
/// stop, braking comfortably from now.
fn room_to_stop(spec: &VehicleSpec, gap: f64, leader: &Vehicle) -> f64 {
    gap - spec.min_gap + comfortable_stop(leader)
}
