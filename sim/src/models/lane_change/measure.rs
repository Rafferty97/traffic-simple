//! What the lane-change model measures for each driver, cheaply, from what
//! the core keeps: its incentives.

use super::update::View;
use crate::models::lane_change;
use crate::models::routing;
use crate::motion::lateral::room_to_change;
use crate::network::{LaneId, Side};
use crate::query::ANTICIPATION;
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

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
/// - Its route: toward a lane it needs fewer lane changes from, and away from
///   one it needs more from (see [`routing::needs`]), by its destination's
///   route table, if it has one, the more the nearer the point by which they
///   must be done.
/// - Courtesy: letting by the follower it holds up most, by what that follower
///   would gain, weighed by its politeness, in units of its comfortable
///   deceleration.
pub(crate) fn incentives(
    view: View,
    id: VehicleId,
    vehicle: &Vehicle,
) -> [Option<lane_change::Incentives>; 2] {
    let View { state, routes, speeds, .. } = view;
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
        state.occupants[lane_id.0 as usize][crate::query::record_index(state, lane_id, id)].s();
    let own = anticipated(lane_id, own_s);
    // Toward a lane its route needs fewer changes from, and away from one it
    // needs more from.
    let way = state.routing.get(id).unwrap_or_default();
    let needs = routing::needs(network, routes, way, vehicle, permitted);
    let route = |side: Side| {
        needs.map_or(0.0, |n| {
            lane_change::route_incentive(n.here, n.beside(side), vehicle.speed)
        })
    };
    let courtesy = vehicle.held_up.and_then(|held| {
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
            route: route(side),
            speed,
            keep,
            courtesy: courtesy.unwrap_or(0.0).min(1.0),
        })
    })
}
