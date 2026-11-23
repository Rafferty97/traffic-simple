//! The lane-change model's phase: what each driver decides at a lane-change
//! update, from a read-only view of the state, for the simulation to carry
//! out; and the terms the model adds to a driver's acceleration.

use std::ops::ControlFlow;

use super::measure::{check_gap, incentives};
use super::{
    Ask, D_SYNC, Decision, Outcome, State, aim, headway, index, make_room, resolve, step, unchecked,
};
use crate::debug::{AccelSource, DebugEvent, DebugSink};
use crate::following::{Ahead, Along, Found, find_leaders};
use crate::models::acceleration;
use crate::network::Side;
use crate::query::LaneSpeeds;
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// Lane changes, for every vehicle, `dt` seconds after the last: each driver's
/// desires from its incentives, cheaply, and a check of the gap for those whose
/// desire has reached their threshold. Fills `decisions` with each deciding driver's
/// state afterwards and what it decided, in order, using `asks` as scratch
/// space; the simulation applies them. `through` must be exact, for the
/// follower searches.
pub(crate) fn decide<D: DebugSink>(
    state: &SimState,
    through: &[f64],
    speeds: &[LaneSpeeds],
    sink: &mut D,
    dt: f64,
    decisions: &mut Vec<(VehicleId, State, Decision)>,
    asks: &mut Vec<(usize, Side)>,
) {
    let time = state.time();
    decisions.clear();
    asks.clear();
    for (id, vehicle) in &state.vehicles {
        // Partway through a change, or still straddling, it doesn't decide.
        if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
            continue;
        }
        let mut lc = state.lane_changers[id].lc;
        let incentives = incentives(state, speeds, id, vehicle);
        aim(&mut lc, incentives);
        // Nothing to weigh up: nowhere to change to, and nothing to fade.
        if lc.idle() && vehicle.intent.is_none() {
            continue;
        }
        let ask = step(&mut lc, vehicle.intent, &vehicle.spec, time, dt);
        // How it weighed up each side it didn't ask about, for the debug view.
        sink.record(|| DebugEvent::RecomputeDecision { vehicle: id });
        for side in [Side::Left, Side::Right] {
            let Some(incentive) = incentives[index(side)] else {
                continue;
            };
            let outcome = match ask {
                Ask::Check(asked) if asked == side => continue,
                Ask::Check(_) => Outcome::OtherSide,
                Ask::Cancel => Outcome::Cancel,
                Ask::Nothing if vehicle.intent.is_some() => Outcome::Wait,
                Ask::Nothing => Outcome::NotWorthIt,
            };
            let eval = unchecked(side, incentive, lc.desire(side), outcome);
            sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
        }
        let decision = match ask {
            Ask::Cancel => Decision::Cancel,
            Ask::Check(side) => {
                asks.push((decisions.len(), side));
                Decision::Nothing
            }
            Ask::Nothing => Decision::Nothing,
        };
        decisions.push((id, lc, decision));
    }
    // The gaps: drivers' own cooldowns spread them out.
    for &(slot, side) in &*asks {
        let (id, mut lc, _) = decisions[slot];
        let vehicle = &state.vehicles[id];
        // Measured again, for the debug view: checks are few.
        let incentive = incentives(state, speeds, id, vehicle)[index(side)].unwrap_or_default();
        let desire = lc.desire(side);
        let prospect = check_gap(state, through, id, vehicle, side, desire);
        let (decision, eval) = resolve(
            &mut lc,
            vehicle.intent,
            &vehicle.spec,
            side,
            incentive,
            prospect,
            dt,
        );
        sink.record(|| DebugEvent::LaneChangeEval { vehicle: id, eval });
        decisions[slot] = (id, lc, decision);
    }
}

/// The terms the lane-change model adds to `vehicle`'s acceleration, the lowest
/// of them, and whose ghost it makes room for, if anyone's: `ghost` is the
/// nearest ghost its leader search found before anything else ahead, with the
/// gap to it.
pub(crate) fn terms<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    ghost: Option<(VehicleId, f64)>,
) -> (f64, Option<VehicleId>) {
    let mut rest = f64::INFINITY;
    // Making room for a vehicle indicating into its lane ahead, as the lane-change
    // model chooses, by what following its ghost would take.
    // LMRS's cooperation: only for a driver whose desire is strong enough.
    let ghost = ghost.map(|(owner, gap)| {
        let o = &state.vehicles[owner];
        let lc = &state.lane_changers[owner].lc;
        let desire = o.intent.map_or(0.0, |i| lc.desire(i.side));
        let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, o.speed);
        (owner, gap, desire, term)
    });
    let yielding = make_room(
        &state.lane_changers[id].lc,
        &vehicle.spec,
        ghost.map(|(owner, _, desire, term)| (owner, desire, term)),
    );
    if let Some((owner, gap, _, term)) = ghost.filter(|_| yielding.is_some()) {
        let source = AccelSource::Yield { id: owner, gap };
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        rest = rest.min(term);
    }
    // LMRS's synchronisation: indicating with desire enough, it adjusts its
    // speed to the leader it would have in the target lane, no harder than
    // comfortably, with its headway shortened by its desire.
    if let Some(intent) = vehicle.intent {
        let desire = state.lane_changers[id].lc.desire(intent.side);
        if desire >= D_SYNC {
            let headway = headway(desire);
            let _ = find_leaders(
                state,
                vehicle,
                Along::Into(intent.side),
                0.0,
                |found| match found {
                    Found::Ahead(Ahead::Vehicle(leader), gap, _) if leader != id => {
                        let speed = state.vehicles[leader].speed;
                        let term = acceleration::relaxed(
                            &vehicle.spec,
                            vehicle.speed,
                            gap,
                            speed,
                            headway,
                        )
                        .max(-vehicle.spec.comfy_decel);
                        let source = AccelSource::Sync { id: leader, gap };
                        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
                        rest = rest.min(term);
                        ControlFlow::Break(())
                    }
                    _ => ControlFlow::Continue(()),
                },
            );
        }
    }
    (rest, yielding)
}
