//! The lane-change model's phase: what each driver decides at a lane-change
//! update, from a read-only view of the state, for the simulation to carry
//! out; and the terms the model adds to a driver's acceleration.

use super::measure::incentives;
use super::{
    Ask, D_SYNC, Decision, MIN_INDICATING, Outcome, State, aim, free, headway, index, make_room,
    resolve, smooth, step, unchecked,
};
use crate::debug::{AccelSource, DebugEvent, DebugSink};
use crate::models::acceleration;
use crate::network::Side;
use crate::query::{LaneSpeeds, Prospect};
use crate::simulation::SimState;
use crate::vehicle::{LaneChangeIntent, Vehicle, VehicleId, VehicleSpec};

/// What a driver decided at a lane-change update, for the simulation to carry
/// out.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Decided {
    pub id: VehicleId,
    /// Its state under the model afterwards.
    pub state: State,
    pub decision: Decision,
    /// The side it wants to change to, if any, not indicating: it keeps room to.
    pub wants: Option<Side>,
    /// The share of time headways to measure its gap at, at its next update,
    /// if it means to check it then: indicating, always, for its
    /// synchronisation too.
    pub look: Option<f64>,
}

/// Lane changes, for every vehicle, `dt` seconds after the last: each driver's
/// desires from its incentives, cheaply, and, for those whose desire has
/// reached their threshold, a judgement of the gap the core has just measured
/// for them, `prospects`, sorted by vehicle, as their intents asked at the last
/// update. One that hasn't asked for its gap yet asks for it. Fills `decisions` with what each
/// deciding driver decided, in order, using `asks` as scratch space; the
/// simulation applies them.
pub(crate) fn decide<D: DebugSink>(
    state: &SimState,
    prospects: &[(VehicleId, Prospect)],
    speeds: &[LaneSpeeds],
    sink: &mut D,
    dt: f64,
    decisions: &mut Vec<Decided>,
    asks: &mut Vec<(usize, Side)>,
) {
    let time = state.time();
    decisions.clear();
    asks.clear();
    for (id, vehicle) in &state.vehicles {
        // What it weighed up before is stale, whether or not it weighs anything
        // up now.
        sink.record(|| DebugEvent::RecomputeDecision { vehicle: id });
        // Partway through a change, or still straddling, it doesn't decide.
        if vehicle.lane_change.is_some() || vehicle.old_side.is_some() {
            continue;
        }
        let mut lc = state.lane_changing.state(id);
        let incentives = incentives(state, speeds, id, vehicle);
        aim(&mut lc, incentives);
        // Nothing to weigh up: nowhere to change to, and nothing to fade.
        if lc.idle() && vehicle.intent == LaneChangeIntent::None {
            continue;
        }
        let ask = step(&mut lc, vehicle.intent, &vehicle.spec, time, dt);
        // How it weighed up each side it didn't ask about, for the debug view.
        for side in [Side::Left, Side::Right] {
            let Some(incentive) = incentives[index(side)] else {
                continue;
            };
            let outcome = match ask {
                Ask::Check(asked) if asked == side => continue,
                Ask::Check(_) => Outcome::OtherSide,
                Ask::Cancel => Outcome::Cancel,
                Ask::Nothing if vehicle.intent.indicating().is_some() => Outcome::Wait,
                Ask::Nothing => Outcome::NotWorthIt,
            };
            let eval = unchecked(side, incentive, lc.desire(side), outcome);
            sink.record(|| eval_event(id, vehicle, &eval));
        }
        let decision = match ask {
            Ask::Cancel => Decision::Cancel,
            Ask::Check(side) => {
                asks.push((decisions.len(), side));
                Decision::Nothing
            }
            Ask::Nothing => Decision::Nothing,
        };
        decisions.push(decided(id, vehicle, lc, decision, time, dt));
    }
    // The gaps measured for them: drivers' own cooldowns spread them out.
    for &(slot, side) in &*asks {
        let Decided { id, state: mut lc, .. } = decisions[slot];
        let vehicle = &state.vehicles[id];
        // Measured again, for the debug view: checks are few.
        let incentive = incentives(state, speeds, id, vehicle)[index(side)].unwrap_or_default();
        let measured = prospects
            .binary_search_by_key(&id, |&(id, _)| id)
            .ok()
            .map(|i| prospects[i].1)
            .filter(|p| p.side == side)
            .and_then(|p| p.gap);
        let Some(gap) = measured else {
            // Not measured that way yet: it asks, and judges it next time.
            let eval = unchecked(side, incentive, lc.desire(side), Outcome::Measuring);
            sink.record(|| eval_event(id, vehicle, &eval));
            continue;
        };
        let (decision, eval) = resolve(
            &mut lc,
            vehicle.intent,
            &vehicle.spec,
            side,
            incentive,
            gap,
            dt,
        );
        sink.record(|| eval_event(id, vehicle, &eval));
        decisions[slot] = decided(id, vehicle, lc, decision, time, dt);
    }
}

/// What a driver decided, with what it wants afterwards, its state under the
/// model then `lc`.
fn decided(
    id: VehicleId,
    vehicle: &Vehicle,
    lc: State,
    decision: Decision,
    now: f64,
    dt: f64,
) -> Decided {
    let indicating = match (decision, vehicle.intent) {
        (Decision::Indicate(side), _) => Some((side, now)),
        (Decision::Cancel | Decision::Change(_), _) => None,
        (Decision::Nothing, LaneChangeIntent::Indicating { side, since, .. }) => {
            Some((side, since))
        }
        (Decision::Nothing, _) => None,
    };
    let wants = lc.wants(vehicle.spec.assertiveness);
    let look = match decision {
        Decision::Change(_) => None,
        _ => look(&lc, &vehicle.spec, now + dt, dt, indicating, wants),
    };
    Decided { id, state: lc, decision, wants, look }
}

/// The share of time headways to have the gap measured at for a driver's next
/// update, `dt` seconds on at time `next`, if it will check it then.
/// Indicating, once it may check again, having indicated long enough and not
/// cooling down; its leader in the target lane, which it synchronises with, is
/// measured at every update regardless. Otherwise, wanting to change to a side
/// it is permitted to, once its desire will have reached its threshold by then,
/// as its smoothing toward what its incentives now make has it, and it won't be
/// cooling down. So its gap is measured by the time it first checks, and it
/// checks at once: asking only then, and checking an update later, made more
/// drivers from either side change into the same spot, and brake hard for each
/// other. Only if its incentives rise faster than that does it ask a step late.
fn look(
    lc: &State,
    spec: &VehicleSpec,
    next: f64,
    dt: f64,
    indicating: Option<(Side, f64)>,
    wants: Option<Side>,
) -> Option<f64> {
    if let Some((side, since)) = indicating {
        let desire = lc.desire(side);
        // As `step` will judge it then, give or take rounding.
        let checks = lc.cooldown == 0 && next - since >= MIN_INDICATING - 1e-9;
        return checks.then(|| headway(desire));
    }
    let side = wants?;
    let target = lc.target[index(side)]?;
    let desire = smooth(lc.desire(side), target, dt);
    let checks = desire >= free(spec.assertiveness) && lc.cooldown == 0;
    checks.then(|| headway(lc.desire(side)))
}

/// The terms the lane-change model adds to `vehicle`'s acceleration, the lowest
/// of them, and whose ghost it makes room for, if anyone's: `ghost` is the
/// nearest ghost its leader search found before anything else ahead, with the
/// gap to it, and `prospect` what a change to the side it is indicating would
/// do, as the core measured it at the lane-change update just before.
pub(crate) fn terms<D: DebugSink>(
    state: &SimState,
    sink: &mut D,
    id: VehicleId,
    vehicle: &Vehicle,
    ghost: Option<(VehicleId, f64)>,
    prospect: Option<&Prospect>,
) -> (f64, Option<VehicleId>) {
    let mut rest = f64::INFINITY;
    // Making room for a vehicle indicating into its lane ahead, as the lane-change
    // model chooses, by what following its ghost would take.
    // LMRS's cooperation: only for a driver whose desire is strong enough.
    let ghost = ghost.map(|(owner, gap)| {
        let o = &state.vehicles[owner];
        let lc = state.lane_changing.state(owner);
        let desire = o.intent.indicating().map_or(0.0, |side| lc.desire(side));
        let term = acceleration::interaction(&vehicle.spec, vehicle.speed, gap, o.speed);
        (owner, gap, desire, term)
    });
    let yielding = make_room(
        &state.lane_changing.state(id),
        &vehicle.spec,
        ghost.map(|(owner, _, desire, term)| (owner, desire, term)),
    );
    if let Some((owner, gap, _, term)) = ghost.filter(|_| yielding.is_some()) {
        let source = source("making room for", owner, gap);
        sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
        rest = rest.min(term);
    }
    // LMRS's synchronisation: indicating with desire enough, it adjusts its
    // speed to the leader it would have in the target lane, no harder than
    // comfortably, with its headway shortened by its desire.
    if let Some(side) = vehicle.intent.indicating() {
        let desire = state.lane_changing.state(id).desire(side);
        let leader = prospect.filter(|p| p.side == side).and_then(|p| p.leader);
        if let (true, Some((leader, gap))) = (desire >= D_SYNC, leader) {
            let speed = state.vehicles[leader].speed;
            let headway = headway(desire);
            let term = acceleration::relaxed(&vehicle.spec, vehicle.speed, gap, speed, headway)
                .max(-vehicle.spec.comfy_decel);
            let source = source("synchronising with", leader, gap);
            sink.record(|| DebugEvent::AccelTerm { vehicle: id, source, accel: term });
            rest = rest.min(term);
        }
    }
    (rest, yielding)
}

/// A lane-change evaluation as a debug event.
fn eval_event(id: VehicleId, vehicle: &Vehicle, eval: &super::LaneChangeEval) -> DebugEvent {
    let fields = eval.fields(&vehicle.spec);
    DebugEvent::ModelEval { vehicle: id, model: "lane change", fields }
}

/// One of the lane-change model's acceleration terms, as the debug view names
/// it: `what` it does about vehicle `id`, `gap` ahead.
fn source(what: &'static str, id: VehicleId, gap: f64) -> AccelSource {
    AccelSource::Model {
        model: "lane change",
        term: what,
        id: Some(id),
        gap: Some(gap),
    }
}
