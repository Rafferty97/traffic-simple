//! What a lane change would do, measured by the core at a lane-change update
//! for the drivers whose intent asks for it, so the lane-change model can judge
//! it without searching the network itself.

use std::ops::ControlFlow;

use crate::following::{Ahead, Along, Found, find_leaders, for_each_leader};
use crate::models::acceleration;
use crate::motion::lateral::{halfway, halfway_from_rest};
use crate::network::Side;
use crate::query::{axle_beside, find_follower, record_index, speed_of};
use crate::simulation::SimState;
use crate::vehicle::{LaneChangeIntent, Vehicle, VehicleId, VehicleSpec};

/// What a change to one side would do, as the core measured it for a driver
/// whose intent asked.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Prospect {
    /// The side measured.
    pub side: Side,
    /// Its nearest leader in the target lane, if one is in reach, and the gap to
    /// it: measured for every driver indicating, which synchronises with it.
    pub leader: Option<(VehicleId, f64)>,
    /// The gap, measured only when the driver means to check it.
    pub gap: Option<Gap>,
}

/// The gap for a lane change, as the core measured it, judging time headways
/// at the share the driver asked for.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Gap {
    /// Its own acceleration in the target lane, as if already settled there,
    /// level with where it is, against its leaders and any ghost ahead there.
    /// A speed cap holds it back in any lane, so it does after changing too.
    pub own_after: f64,
    /// The acceleration of the follower it would cut in front of, if one is in
    /// reach, with it as a leader.
    pub new_follower: Option<f64>,
    /// Whether it would be at least halfway through the change before it had
    /// to stop behind its leader in its own lane. Otherwise it would be left
    /// straddling both lanes, blocking them.
    pub room: bool,
}

/// The side to measure for `vehicle`, if its intent asks for a measurement,
/// and the share of time headways to judge its gap at, if it asks for that:
/// indicating, it always has its leader there measured.
pub(crate) fn asked(vehicle: &Vehicle) -> Option<(Side, Option<f64>)> {
    match vehicle.intent {
        LaneChangeIntent::Indicating { side, check, .. } => Some((side, check)),
        LaneChangeIntent::Desire { side, check } => check.map(|headway| (side, Some(headway))),
        LaneChangeIntent::None => None,
    }
}

/// Measures a change to `side` for `vehicle`: its leader in the target lane,
/// and, with `check`, the gap, its own time headway and its new follower's
/// shortened to that share. `None` if its lane has no neighbour that way.
/// `through` must be exact enough for the follower search (see
/// [`find_follower`]).
pub(crate) fn measure(
    state: &SimState,
    through: &[f64],
    id: VehicleId,
    vehicle: &Vehicle,
    side: Side,
    check: Option<f64>,
) -> Option<Prospect> {
    let target = state.network.lane(vehicle.path[0]).adj(side)?;
    let Some(headway) = check else {
        // Its leader alone: the nearest vehicle ahead in the target lane.
        let leader = find_leaders(
            state,
            vehicle,
            Along::Into(side),
            0.0,
            |found| match found {
                Found::Ahead(Ahead::Vehicle(leader), gap, _) if leader != id => {
                    ControlFlow::Break((leader, gap))
                }
                _ => ControlFlow::Continue(()),
            },
        )
        .break_value();
        return Some(Prospect { side, leader, gap: None });
    };
    let SimState { network, vehicles, .. } = state;
    let cap = state.caps.iter().find(|&&(capped, _)| capped == id);
    let cap = cap.map_or(f64::INFINITY, |&(_, max_speed)| {
        acceleration::capped(vehicle.speed, max_speed)
    });

    // In the target lane: against the free road there, its leaders and the end
    // of its path, ghosts leading, since a vehicle already indicating into the
    // gap goes first; its own ghost passed over.
    let along = Along::Into(side);
    let lane = along.lane(network, vehicle, 0).unwrap_or(vehicle.path[0]);
    let desired = state.drivers.desired_speed(id, &vehicle.spec, network.lane(lane).speed_limit());
    let mut own_after = acceleration::free_road(&vehicle.spec, vehicle.speed, desired);
    let mut leader = None;
    for_each_leader(state, vehicle, along, 0.0, |found| match found {
        Found::Ahead(Ahead::Ghost(owner), ..) if owner == id => {}
        Found::Ahead(ahead, gap, relaxed) => {
            let speed = speed_of(vehicles, ahead);
            if let (None, Ahead::Vehicle(other)) = (leader, ahead)
                && other != id
            {
                leader = Some((other, gap));
            }
            let share = relaxed.min(headway);
            let term = acceleration::relaxed(&vehicle.spec, vehicle.speed, gap, speed, share);
            own_after = own_after.min(term);
        }
        Found::Pass(..) => {}
    });
    let own_after = own_after.min(cap);

    // The new follower, behind it in the target lane: the lane is split at its
    // rear there, as its search there splits it.
    let rear = axle_beside(network, vehicle, target.lane) - vehicle.spec.rear_axle;
    let new_follower =
        find_follower(state, through, target.lane, rear, id).map(|(follower, gap)| {
            let f = &vehicles[follower];
            acceleration::relaxed(&f.spec, f.speed, gap, vehicle.speed, headway)
        });

    // Halfway through before it would have to stop behind its leader. Wanting
    // to change, it keeps room to, and looks that far.
    let own = record_index(state, vehicle.path[0], id);
    let mut stop = f64::INFINITY;
    let reach = vehicle.spec.min_gap + halfway_from_rest(network, vehicle);
    let _ = find_leaders(
        state,
        vehicle,
        Along::Path(own),
        reach,
        |found| match found {
            Found::Ahead(Ahead::Vehicle(leader), gap, _) if leader != id => {
                stop = room_to_stop(&vehicle.spec, gap, &vehicles[leader]);
                ControlFlow::Break(())
            }
            Found::Ahead(Ahead::PathEnd, gap, _) => {
                stop = gap - vehicle.spec.min_gap;
                ControlFlow::Break(())
            }
            _ => ControlFlow::Continue(()),
        },
    );
    let room = stop >= halfway(network, vehicle);
    let gap = Gap { own_after, new_follower, room };
    Some(Prospect { side, leader, gap: Some(gap) })
}

/// How far a vehicle with `spec` could go before it had to stop behind
/// `leader`, `gap` ahead: to its minimum gap behind where the leader would
/// stop, braking comfortably from now.
fn room_to_stop(spec: &VehicleSpec, gap: f64, leader: &Vehicle) -> f64 {
    gap - spec.min_gap + leader.comfortable_stop()
}
