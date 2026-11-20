//! MOBIL (Kesting, Treiber and Helbing, 2007): whether a lane change is worth it
//! and safe, from accelerations under the longitudinal model.
//!
//! Symmetric for now: no bias toward either side.
//!
//! The decision making lives here; the rest of the simulation sees only each
//! vehicle's indicator and an opaque [`State`]. The simulation measures what a
//! change would do, by searching the network, and carries out what [`decide`]
//! returns: indicating, cancelling, or changing lanes.

use crate::models::acceleration::SAFE_BRAKING;
use crate::network::Side;
use crate::vehicle::{Intent, VehicleId, VehicleSpec};

/// A driver's lane-change state, owned by this model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct State {
    /// The advantage built up so far for changing left and right, in m/s (see
    /// [`accumulate`]).
    advantage: [f64; 2],
    /// How long it has been wanting to change lanes but blocked, in seconds,
    /// fading while it isn't (see [`frustrate`]).
    frustration: f64,
    /// The vehicle whose ghost it has chosen to make room for, if any. Kept so the
    /// choice sticks rather than flickering on and off.
    yielding: Option<VehicleId>,
}

impl State {
    /// Records whom the driver is making room for, as [`make_room`] chose.
    pub fn set_yielding(&mut self, yielding: Option<VehicleId>) {
        self.yielding = yielding;
    }

    /// Whether the driver wants to change lanes: enough advantage has built up
    /// for a side to keep indicating to it, the lower bar, so the wish doesn't
    /// flicker.
    pub fn wants_change(&self, assertiveness: f64) -> bool {
        self.advantage.iter().any(|&a| still_intends(a, assertiveness))
    }
}

#[cfg(test)]
impl State {
    pub fn advantage(&self) -> [f64; 2] {
        self.advantage
    }

    pub fn frustration(&self) -> f64 {
        self.frustration
    }

    pub fn yielding(&self) -> Option<VehicleId> {
        self.yielding
    }

    pub fn set_advantage(&mut self, advantage: [f64; 2]) {
        self.advantage = advantage;
    }

    pub fn set_frustration(&mut self, frustration: f64) {
        self.frustration = frustration;
    }
}

/// The index of a side in a driver's built-up advantage.
pub fn index(side: Side) -> usize {
    match side {
        Side::Left => 0,
        Side::Right => 1,
    }
}

/// What a lane-change decision asks the simulation to do.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Nothing,
    /// Start indicating to this side.
    Indicate(Side),
    /// Stop indicating.
    Cancel,
    /// Change lanes to this side, now.
    Change(Side),
}

/// What a change to one side would do, as the simulation measured it: the
/// accelerations MOBIL weighs.
#[derive(Clone, Copy, Debug)]
pub struct Prospect {
    /// Its own acceleration now, in its own lane.
    pub own_now: f64,
    /// Its own acceleration in the target lane.
    pub own_after: f64,
    /// The follower it would cut in front of, if one is in reach.
    pub new_follower: Option<Effect>,
    /// The follower it would leave behind, if one is in reach.
    pub old_follower: Option<Effect>,
    /// Whether it would be at least halfway through the change before it had
    /// to stop behind its leader in its own lane. Otherwise it would be left
    /// straddling both lanes, blocking them, so it doesn't change yet, though
    /// it may indicate, and makes room while it does.
    pub room: bool,
}

/// A follower's acceleration now, and if the change it is weighed for went
/// ahead, and its speed.
#[derive(Clone, Copy, Debug)]
pub struct Effect {
    pub before: f64,
    pub after: f64,
    pub speed: f64,
}

impl Effect {
    fn change(self) -> f64 {
        self.after - self.before
    }
}

/// How a vehicle weighed up changing lanes to one side, under MOBIL.
#[derive(Clone, Copy, Debug)]
pub struct LaneChangeEval {
    pub side: Side,
    /// Its own acceleration in the target lane.
    pub own_after: f64,
    /// Its own acceleration there less its acceleration now.
    pub own_gain: f64,
    /// The change for the follower it would cut in front of, if one is in reach.
    pub new_follower: Option<f64>,
    /// The change for the follower it would leave behind, if one is in reach.
    pub old_follower: Option<f64>,
    /// MOBIL's net gain, weighing the followers' changes by politeness.
    pub incentive: f64,
    /// The advantage built up for this side, after this decision.
    pub advantage: f64,
    /// The driver's frustration before this decision, in seconds blocked.
    pub frustration: f64,
    /// The hardest braking it accepts from the change, for itself and its new
    /// follower, at that frustration.
    pub accepted_braking: f64,
    /// Whether it and its new follower would brake gently enough.
    pub safe: bool,
    /// Whether it would be halfway through the change before it had to stop
    /// behind its leader (see [`Prospect::room`]).
    pub room: bool,
    pub outcome: Outcome,
}

/// What a vehicle decided about changing to one side, at a decision tick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Not indicating: not enough advantage has built up yet.
    NotWorthIt,
    /// Not indicating: enough advantage has built up, but it, or the follower it
    /// would cut in front of, would have to brake too hard.
    Unsafe,
    /// Not indicating to this side: the other has more built up, or it is
    /// already indicating the other way.
    OtherSide,
    /// Started indicating.
    Indicate,
    /// Started indicating without a safe gap, frustrated, asking the follower in
    /// the target lane to make room.
    Request,
    /// Still indicating, waiting for a gap: until the change is safe and it has
    /// indicated long enough.
    Wait,
    /// Changing lanes now.
    Change,
    /// Stopped indicating: its advantage has faded, or the side is no longer permitted.
    Cancel,
}

/// The least net gain, in m/s², worth changing lanes for.
pub const THRESHOLD: f64 = 0.1;
/// The least time a driver indicates before changing lanes, in seconds.
pub const MIN_INDICATING: f64 = 0.5;

/// The net gain of a lane change: the driver's own change in acceleration plus
/// its `politeness` times the changes for its new and old followers.
pub fn incentive(politeness: f64, own_gain: f64, new_follower: f64, old_follower: f64) -> f64 {
    own_gain + politeness * (new_follower + old_follower)
}

/// How long a driver must have been blocked, in seconds, before it accepts
/// braking as hard as [`SAFE_BRAKING`], and indicates even without a gap.
pub const FULLY_FRUSTRATED: f64 = 10.0;
/// How fast frustration fades while a driver isn't blocked: it halves every
/// this many seconds.
pub const FRUSTRATION_HALF_LIFE: f64 = 5.0;
/// The least speed, in m/s, the follower in the target lane must have for a
/// frustrated driver to indicate without a gap: in a stopped queue, there is
/// nobody to make room.
pub const MOVING: f64 = 2.0;

/// Frustration after `dt` seconds, in seconds blocked: a driver that wants to
/// change lanes but can't safely grows frustrated, and otherwise its frustration
/// fades. As the critical gap shrinks with waiting in Ahmed's (1999) model and
/// SUMO's impatience grows while a driver is held up.
pub fn frustrate(frustration: f64, blocked: bool, dt: f64) -> f64 {
    match blocked {
        true => (frustration + dt).min(FULLY_FRUSTRATED),
        false => frustration * 0.5f64.powf(dt / FRUSTRATION_HALF_LIFE),
    }
}

/// The hardest braking, positive, a driver accepts from a lane change, for
/// itself and for the follower it cuts in front of: its comfortable deceleration
/// when calm, rising to [`SAFE_BRAKING`] when fully frustrated. An assertive
/// driver accepts as much calm as it would that share frustrated: at 1, safe
/// braking from the start, as SUMO's `lcAssertive` accepts smaller gaps.
pub fn accepted_braking(comfy_decel: f64, frustration: f64, assertiveness: f64) -> f64 {
    let f = (frustration / FULLY_FRUSTRATED).max(assertiveness).clamp(0.0, 1.0);
    comfy_decel + (SAFE_BRAKING - comfy_decel).max(0.0) * f
}

/// Whether a driver is frustrated enough to indicate without a gap, asking the
/// follower in the target lane to make room.
pub fn forcing(frustration: f64) -> bool {
    frustration >= FULLY_FRUSTRATED
}

/// How much advantage must build up before a driver intends a change, in m/s:
/// the net gain above [`THRESHOLD`], summed over time. A clear gain of 1 m/s²
/// gets there in about a third of a second; a marginal one has to last.
pub const INTENT: f64 = 0.3;
/// The most advantage that builds up, so that a long wait for a gap doesn't leave
/// a store that takes long to fade.
pub const MAX_ADVANTAGE: f64 = 2.0 * INTENT;
/// How fast built-up advantage fades while changing isn't worth it: it halves
/// every this many seconds.
pub const HALF_LIFE: f64 = 1.0;

/// Built-up advantage for a change, after `dt` seconds with this net gain: the
/// gain above [`THRESHOLD`] adds up while there is some, and otherwise what has
/// built up fades. As SUMO's lane-change model builds up a speed gain before
/// changing, so a fleeting or marginal advantage doesn't make a driver change.
pub fn accumulate(advantage: f64, incentive: f64, dt: f64) -> f64 {
    match incentive > THRESHOLD {
        true => (advantage + (incentive - THRESHOLD) * dt).min(MAX_ADVANTAGE),
        false => advantage * 0.5f64.powf(dt / HALF_LIFE),
    }
}

/// How much an assertive driver lowers its bar for intending a change: at an
/// assertiveness of 1, to this share of [`INTENT`] less, so it changes for
/// smaller gains, as SUMO's `lcSpeedGain` makes a driver more eager.
pub const ASSERTIVE_BAR: f64 = 2.0 / 3.0;

/// The advantage a driver of this assertiveness must build up to intend a change.
fn bar(assertiveness: f64) -> f64 {
    INTENT * (1.0 - ASSERTIVE_BAR * assertiveness.clamp(0.0, 1.0))
}

/// Whether enough advantage has built up for a driver to intend a change.
pub fn intends(advantage: f64, assertiveness: f64) -> bool {
    advantage >= bar(assertiveness)
}

/// Whether a driver already indicating should keep going: a lower bar than
/// starting, so advantage hovering around the bar doesn't make the indicator
/// flicker.
pub fn still_intends(advantage: f64, assertiveness: f64) -> bool {
    advantage >= 0.5 * bar(assertiveness)
}

/// Whether an acceleration after a change is gentle enough to accept, braking no
/// harder than `accepted`.
pub fn safe(accel: f64, accepted: f64) -> bool {
    accel >= -accepted
}

/// A driver's lane-change decision, under MOBIL, with advantage built up over
/// time as in SUMO's lane-change model, `dt` seconds after its last. Updates its
/// `state`, and returns what it asks the simulation to do, with how it weighed up
/// each side.
///
/// `prospect` measures a change to one side, or gives `None` if changing that way
/// isn't permitted. It is called at most once per side, when the decision needs
/// it. The simulation doesn't ask a vehicle partway through a change, or still
/// straddling, to decide.
///
/// Each decision, for each permitted side, it weighs up MOBIL's net gain and
/// builds up advantage from it (see [`accumulate`]).
///
/// - Not indicating: once enough advantage has built up for a side, and the change
///   would be safe, it starts indicating to it; to the side with more, if both.
///   Fully frustrated, it also starts when the change is unsafe, asking a moving
///   follower to make room.
/// - Indicating: it changes once it has indicated long enough and the change is
///   safe, and until then keeps indicating, waiting for a gap. It cancels once
///   its advantage has faded below a lower bar, or the side is no longer
///   permitted.
///
/// A change starts its advantage and frustration afresh.
pub fn decide(
    state: &mut State,
    intent: Option<Intent>,
    spec: &VehicleSpec,
    now: f64,
    dt: f64,
    mut prospect: impl FnMut(Side) -> Option<Prospect>,
) -> (Decision, [Option<LaneChangeEval>; 2]) {
    // The braking a change may ask of it and its new follower, at its frustration.
    let accepted = accepted_braking(spec.comfy_decel, state.frustration, spec.assertiveness);
    let mut advantage = [0.0; 2];
    let mut evals = [Side::Left, Side::Right].map(|side| {
        let p = prospect(side)?;
        let own_gain = p.own_after - p.own_now;
        let new_follower = p.new_follower.map(Effect::change);
        let old_follower = p.old_follower.map(Effect::change);
        let incentive = incentive(
            spec.politeness,
            own_gain,
            new_follower.unwrap_or(0.0),
            old_follower.unwrap_or(0.0),
        );
        let built = state.advantage[index(side)];
        advantage[index(side)] = accumulate(built, incentive, dt);
        // With nobody within reach behind, nobody is cut off.
        let follower_safe = p.new_follower.is_none_or(|effect| safe(effect.after, accepted));
        // Frustrated, it may ask a moving follower to make room.
        let requestable = forcing(state.frustration)
            && p.new_follower.is_some_and(|effect| effect.speed >= MOVING);
        Some(LaneChangeEval {
            side,
            own_after: p.own_after,
            own_gain,
            new_follower,
            old_follower,
            incentive,
            advantage: advantage[index(side)],
            frustration: state.frustration,
            accepted_braking: accepted,
            safe: safe(p.own_after, accepted) && follower_safe,
            room: p.room,
            outcome: if requestable {
                Outcome::Request
            } else {
                Outcome::NotWorthIt
            },
        })
    });

    let decision = match intent {
        None => {
            for eval in evals.iter_mut().flatten() {
                let requestable = eval.outcome == Outcome::Request;
                eval.outcome = match () {
                    _ if !intends(eval.advantage, spec.assertiveness) => Outcome::NotWorthIt,
                    _ if eval.safe => Outcome::Indicate,
                    _ if requestable => Outcome::Request,
                    _ => Outcome::Unsafe,
                };
            }
            // It can only indicate one way: the side with more built up.
            let indicates =
                |e: &LaneChangeEval| matches!(e.outcome, Outcome::Indicate | Outcome::Request);
            if let [Some(left), Some(right)] = &mut evals
                && indicates(left)
                && indicates(right)
            {
                let worse = if left.advantage >= right.advantage {
                    right
                } else {
                    left
                };
                worse.outcome = Outcome::OtherSide;
            }
            let indicate = evals.iter().flatten().find(|e| indicates(e));
            indicate.map_or(Decision::Nothing, |e| Decision::Indicate(e.side))
        }
        Some(intent) => {
            let indicated = now - intent.since;
            for eval in evals.iter_mut().flatten() {
                eval.outcome = match () {
                    _ if eval.side != intent.side => Outcome::OtherSide,
                    _ if !still_intends(eval.advantage, spec.assertiveness) => Outcome::Cancel,
                    // Without room to get halfway, it would be left straddling;
                    // fully frustrated, it goes all the same, or it might never.
                    _ if eval.safe
                        && (eval.room || forcing(eval.frustration))
                        && indicated >= MIN_INDICATING =>
                    {
                        Outcome::Change
                    }
                    _ => Outcome::Wait,
                };
            }
            match evals[index(intent.side)].map(|e| e.outcome) {
                // No longer permitted that way.
                None | Some(Outcome::Cancel) => Decision::Cancel,
                Some(Outcome::Change) => Decision::Change(intent.side),
                _ => Decision::Nothing,
            }
        }
    };
    // Blocked: it wants to change lanes, but can't safely, or without being
    // left straddling.
    let blocked = evals.iter().flatten().any(|e| {
        let possible = e.safe && e.room;
        intends(e.advantage, spec.assertiveness) && !possible && e.outcome != Outcome::OtherSide
    });
    state.frustration = frustrate(state.frustration, blocked, dt);
    state.advantage = advantage;
    if let Decision::Change(_) = decision {
        // A new lane, so advantage builds up afresh, and it is no longer blocked.
        state.advantage = [0.0; 2];
        state.frustration = 0.0;
    }
    (decision, evals)
}

/// Whom a follower makes room for, if anyone: a vehicle indicating into its lane
/// ahead, whose ghost it then treats as a leader. `ghost` is the first ghost its
/// search reached before any leader, if any: whose it is, and the acceleration
/// following it would take. The simulation records the answer with
/// [`State::set_yielding`].
///
/// It keeps on making room for the one it chose, so the choice doesn't flicker,
/// unless that would take braking harder than is safe; otherwise it starts if
/// that takes no more braking than its courtesy allows. A driver with no
/// courtesy never does. As Hidas's cooperative lane changing and SUMO's
/// `lcCooperative`.
pub fn make_room(
    state: &State,
    spec: &VehicleSpec,
    ghost: Option<(VehicleId, f64)>,
) -> Option<VehicleId> {
    let (owner, accel) = ghost?;
    let chosen = state.yielding == Some(owner);
    let courteous = spec.courtesy > 0.0;
    let limit = if chosen { SAFE_BRAKING } else { spec.courtesy };
    ((chosen || courteous) && accel >= -limit).then_some(owner)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn politeness_weighs_followers() {
        assert_eq!(incentive(0.2, 0.5, -1.0, 0.0), 0.5 - 0.2);
        assert_eq!(incentive(0.0, 0.5, -1.0, 0.0), 0.5, "an impolite driver");
    }

    #[test]
    fn frustration_builds_while_blocked_and_fades_after() {
        let blocked = (0..25).fold(0.0, |f, _| frustrate(f, true, 0.2));
        assert!((blocked - 5.0).abs() < 1e-9);
        assert_eq!(
            frustrate(FULLY_FRUSTRATED, true, 0.2),
            FULLY_FRUSTRATED,
            "capped"
        );
        assert!((frustrate(4.0, false, FRUSTRATION_HALF_LIFE) - 2.0).abs() < 1e-12);
        assert!(forcing(FULLY_FRUSTRATED) && !forcing(0.99 * FULLY_FRUSTRATED));
    }

    #[test]
    fn assertiveness_lowers_the_bar_and_raises_the_braking_accepted_calm() {
        assert_eq!(accepted_braking(1.5, 0.0, 1.0), SAFE_BRAKING);
        assert!((accepted_braking(1.5, 0.0, 0.5) - (1.5 + SAFE_BRAKING) / 2.0).abs() < 1e-12);
        // Frustration still raises it, beyond what assertiveness gives.
        assert_eq!(accepted_braking(1.5, FULLY_FRUSTRATED, 0.2), SAFE_BRAKING);
        assert!(!intends(0.2, 0.0) && intends(0.2, 0.5) && intends(0.1, 1.0));
    }

    #[test]
    fn frustration_raises_the_braking_a_change_accepts() {
        assert_eq!(
            accepted_braking(1.5, 0.0, 0.0),
            1.5,
            "calm: comfortable braking"
        );
        assert_eq!(accepted_braking(1.5, FULLY_FRUSTRATED, 0.0), SAFE_BRAKING);
        assert_eq!(
            accepted_braking(1.5, 2.0 * FULLY_FRUSTRATED, 0.0),
            SAFE_BRAKING,
            "capped"
        );
        let half = accepted_braking(1.5, 0.5 * FULLY_FRUSTRATED, 0.0);
        assert!((half - (1.5 + SAFE_BRAKING) / 2.0).abs() < 1e-12);
    }

    #[test]
    fn advantage_builds_up_above_the_threshold_and_fades_below_it() {
        let dt = 0.2;
        // A clear gain intends a change within a few decisions...
        let clear = (0..2).fold(0.0, |a, _| accumulate(a, 1.0, dt));
        assert!(intends(clear, 0.0), "{clear}");
        // ...a marginal one only after several seconds of it.
        let marginal = |ticks| (0..ticks).fold(0.0, |a, _| accumulate(a, 0.15, dt));
        assert!(!intends(marginal(25), 0.0), "after 5 s");
        assert!(intends(marginal(35), 0.0), "after 7 s");
        // Below the threshold, nothing builds up.
        assert_eq!(accumulate(0.0, THRESHOLD, dt), 0.0);
        // It fades by half every half-life, and is capped.
        assert!((accumulate(0.4, 0.0, HALF_LIFE) - 0.2).abs() < 1e-12);
        assert_eq!(accumulate(MAX_ADVANTAGE, 5.0, dt), MAX_ADVANTAGE);
        // Indicating, it keeps going down to half the bar.
        assert!(still_intends(0.5 * INTENT, 0.0) && !still_intends(0.49 * INTENT, 0.0));
    }

    #[test]
    fn safety_is_bounded_braking() {
        assert!(safe(-2.0, 2.0));
        assert!(!safe(-2.01, 2.0));
    }
}
