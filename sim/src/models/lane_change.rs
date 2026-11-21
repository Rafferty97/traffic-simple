//! LMRS, the Lane-change Model with Relaxation and Synchronization (Schakel,
//! Knoop and van Arem, 2012): whether a driver wants to change lanes, from
//! desires for its route, for speed and for keeping to one side, and whether
//! the gap lets it.
//!
//! At each update, each driver's desire for each side is worked out cheaply, from
//! how fast traffic goes in each lane ahead, and smoothed over time. Only once
//! it reaches the threshold for a free lane change does the driver check the
//! gap, which takes searches of the network; a check that fails waits a few
//! updates before the next. With enough desire it indicates and synchronises
//! with the target lane, and with more, followers there make room for it.
//!
//! The decision making lives here; the rest of the simulation sees only each
//! vehicle's indicator and an opaque [`State`]. The simulation measures the
//! incentives and, when asked, the gap, by searching the network, and carries
//! out what [`resolve`] returns: indicating, cancelling, or changing lanes.

use crate::models::acceleration::SAFE_BRAKING;
use crate::network::Side;
use crate::vehicle::{Intent, VehicleId, VehicleSpec};

/// Desire at which a driver changes lanes if the gap lets it, after indicating
/// for [`MIN_INDICATING`].
pub const D_FREE: f64 = 0.365;
/// Desire at which a driver indicates even without a gap, and adjusts its
/// speed to the target lane's: synchronisation.
pub const D_SYNC: f64 = 0.577;
/// Desire at which followers in the target lane make room for the driver, and
/// at which it changes even if it would be left straddling.
pub const D_COOP: f64 = 0.788;
/// The speed difference, in m/s, that makes a full desire to change lanes for
/// speed: 69.6 km/h.
pub const V_GAIN: f64 = 69.6 / 3.6;
/// How far ahead, in meters, a lane's end makes a driver want to leave it,
/// per lane change still to make: its route desire rises linearly from zero
/// this far out to one at the end.
pub const ROUTE_DISTANCE: f64 = 295.0;
/// As [`ROUTE_DISTANCE`], in seconds at the driver's speed.
pub const ROUTE_TIME: f64 = 43.0;
/// How long, in seconds, desire takes to follow its incentives: their changes
/// are smoothed over about this long, so a fleeting gain doesn't make a driver
/// change lanes.
pub const DESIRE_TIME: f64 = 1.0;
/// The least time a driver indicates before changing lanes, in seconds.
pub const MIN_INDICATING: f64 = 0.5;
/// How many updates a driver waits after a gap it checked was refused, before
/// it checks again, at a politeness of 0: 0.2 s at 20 Hz. More polite drivers
/// wait longer, and more assertive ones less, so drivers' checks don't fall in
/// step.
pub const COOLDOWN: f64 = 4.0;
/// How much an assertive driver lowers its thresholds: at an assertiveness of
/// 1, to this share less, so it changes for smaller gains, as SUMO's
/// `lcSpeedGain` makes a driver more eager.
pub const ASSERTIVE: f64 = 0.5;

/// A driver's lane-change state, owned by this model.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct State {
    /// Desire to change left and right, from -1 to 1, smoothed over time (see
    /// [`DESIRE_TIME`]).
    #[cfg_attr(feature = "serde", serde(default))]
    desire: [f64; 2],
    /// The desire its incentives make for each side, which its desire follows:
    /// `None` where it may not change (see [`aim`]).
    #[cfg_attr(feature = "serde", serde(default))]
    target: [Option<f64>; 2],
    /// Steps until it may check a gap again, after one was refused.
    #[cfg_attr(feature = "serde", serde(default))]
    cooldown: u16,
    /// The vehicle whose ghost it has chosen to make room for, if any. Kept so the
    /// choice sticks rather than flickering on and off.
    yielding: Option<VehicleId>,
}

impl State {
    /// Records whom the driver is making room for, as [`make_room`] chose.
    pub fn set_yielding(&mut self, yielding: Option<VehicleId>) {
        self.yielding = yielding;
    }

    /// Whether there is nothing for a step to do: no side it may change to, no
    /// desire left to fade, and no cooldown. Most vehicles on a road with no
    /// lanes beside them, which the simulation then skips.
    pub fn idle(&self) -> bool {
        self.target == [None, None] && self.desire == [0.0, 0.0] && self.cooldown == 0
    }

    /// Its desire to change to `side`.
    pub fn desire(&self, side: Side) -> f64 {
        self.desire[index(side)]
    }

    /// Whether the driver wants to change lanes: its desire for a side has
    /// reached half its threshold for changing, a lower bar, so the wish doesn't
    /// flicker.
    pub fn wants_change(&self, assertiveness: f64) -> bool {
        self.desire.iter().any(|&d| d >= 0.5 * free(assertiveness))
    }
}

#[cfg(test)]
impl State {
    pub fn yielding(&self) -> Option<VehicleId> {
        self.yielding
    }

    pub fn set_desire(&mut self, desire: [f64; 2]) {
        self.desire = desire;
    }

    pub fn cooldown(&self) -> u16 {
        self.cooldown
    }
}

/// The index of a side in a driver's desires.
pub fn index(side: Side) -> usize {
    match side {
        Side::Left => 0,
        Side::Right => 1,
    }
}

/// A driver's threshold for a free lane change, lowered by its assertiveness.
pub fn free(assertiveness: f64) -> f64 {
    D_FREE * (1.0 - ASSERTIVE * assertiveness.clamp(0.0, 1.0))
}

/// What makes a driver want to change to one side, as the simulation measured
/// it, each in units of desire.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Incentives {
    /// Its route: positive toward a lane it must reach, negative away from it.
    pub route: f64,
    /// Speed: how much faster traffic goes in that lane ahead than in its own,
    /// over [`V_GAIN`].
    pub speed: f64,
    /// Keeping to one side: [`D_FREE`] toward the side the lane says to keep to.
    pub keep: f64,
    /// Courtesy: letting by the follower it holds up, its gain weighed by the
    /// driver's politeness.
    pub courtesy: f64,
}

impl Incentives {
    /// The total desire, LMRS's `d = d_r + θ · (d_s + d_b + d_c)`: the
    /// voluntary incentives count fully unless they work against the route,
    /// and not at all once the route's desire is strong enough to cooperate.
    pub fn desire(&self) -> f64 {
        let voluntary = self.speed + self.keep + self.courtesy;
        let route = self.route.abs();
        let theta = if self.route * voluntary >= 0.0 || route <= D_SYNC {
            1.0
        } else if route >= D_COOP {
            0.0
        } else {
            (D_COOP - route) / (D_COOP - D_SYNC)
        };
        (self.route + theta * voluntary).clamp(-1.0, 1.0)
    }
}

/// LMRS's route desire to leave a lane that ends `distance` ahead, at
/// `speed`, with `changes` lane changes to make before then: rising linearly
/// to one at the end, by distance or by time, whichever is the higher.
pub fn route_desire(distance: f64, speed: f64, changes: u32) -> f64 {
    let n = f64::from(changes.max(1));
    let by_distance = 1.0 - distance / (n * ROUTE_DISTANCE);
    let by_time = 1.0 - distance / speed.max(0.1) / (n * ROUTE_TIME);
    by_distance.max(by_time).clamp(0.0, 1.0)
}

/// What a step of the model asks of the simulation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ask {
    Nothing,
    /// Check the gap for a change to this side, and [`resolve`] it.
    Check(Side),
    /// Stop indicating.
    Cancel,
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

/// A driver's desire after `dt` seconds following `target`: smoothed, so it
/// moves `dt / DESIRE_TIME` of the way there.
fn smooth(desire: f64, target: f64, dt: f64) -> f64 {
    desire + (target - desire) * (dt / DESIRE_TIME).min(1.0)
}

/// Sets what a driver's desire follows from its incentives for each side,
/// `None` where it may not change.
pub fn aim(state: &mut State, incentives: [Option<Incentives>; 2]) {
    state.target = incentives.map(|i| i.map(|i| i.desire()));
}

/// One update of a driver's lane-change model, `dt` seconds after the last:
/// cheap, so it runs for every vehicle, as often as the longitudinal model. Its desires follow what its
/// incentives last made them (see [`aim`]). Updates its desires and cooldown,
/// and returns whether it wants a gap checked.
///
/// - Indicating: it cancels once its desire for that side falls below half its
///   threshold, or the side is no longer permitted; otherwise, once it has
///   indicated long enough and isn't cooling down, it asks for the gap.
/// - Not indicating: once its desire for a side reaches its threshold, the
///   stronger side if both, and it isn't cooling down, it asks for the gap.
pub fn step(
    state: &mut State,
    intent: Option<Intent>,
    spec: &VehicleSpec,
    now: f64,
    dt: f64,
) -> Ask {
    let permitted = state.target.map(|t| t.is_some());
    for i in 0..2 {
        state.desire[i] = smooth(state.desire[i], state.target[i].unwrap_or(0.0), dt);
    }
    let threshold = free(spec.assertiveness);
    let waiting = state.cooldown > 0;
    state.cooldown = state.cooldown.saturating_sub(1);
    match intent {
        Some(intent) => {
            let i = index(intent.side);
            if !permitted[i] || state.desire[i] < 0.5 * threshold {
                return Ask::Cancel;
            }
            if waiting || now - intent.since < MIN_INDICATING {
                return Ask::Nothing;
            }
            Ask::Check(intent.side)
        }
        None => {
            let [left, right] = state.desire;
            let (side, d) = if left >= right {
                (Side::Left, left)
            } else {
                (Side::Right, right)
            };
            if !permitted[index(side)] || d < threshold || waiting {
                return Ask::Nothing;
            }
            Ask::Check(side)
        }
    }
}

/// What a change to one side would do, as the simulation measured it once the
/// driver asked for the gap: with its time headway, and its new follower's,
/// shortened by its desire (see [`headway`]).
#[derive(Clone, Copy, Debug)]
pub struct Prospect {
    /// Its own acceleration in the target lane.
    pub own_after: f64,
    /// The acceleration of the follower it would cut in front of, if one is in
    /// reach, with it as a leader.
    pub new_follower: Option<f64>,
    /// Whether it would be at least halfway through the change before it had
    /// to stop behind its leader in its own lane. Otherwise it would be left
    /// straddling both lanes, blocking them, so it doesn't change yet, though
    /// it may indicate, and makes room while it does.
    pub room: bool,
}

/// The share of their time headways a driver and the follower it cuts in front
/// of accept with this desire: LMRS shortens the headway with desire, from the
/// whole of it to [`RELAXED_HEADWAY`](crate::models::acceleration::RELAXED_HEADWAY)
/// of it, `T_min / T_max`, at a desire of one.
pub fn headway(desire: f64) -> f64 {
    let relaxed = crate::models::acceleration::RELAXED_HEADWAY;
    1.0 - desire.clamp(0.0, 1.0) * (1.0 - relaxed)
}

/// The hardest braking, positive, a driver accepts from a change at this
/// desire, for itself and its new follower: LMRS's desire times its
/// comfortable deceleration, raised toward [`SAFE_BRAKING`] by its
/// assertiveness, as SUMO's `lcAssertive` accepts smaller gaps.
pub fn accepted_braking(spec: &VehicleSpec, desire: f64) -> f64 {
    let desire = desire.clamp(0.0, 1.0);
    let comfy = desire * spec.comfy_decel;
    comfy + (SAFE_BRAKING - comfy).max(0.0) * spec.assertiveness.clamp(0.0, 1.0) * desire
}

/// How a vehicle weighed up changing lanes to one side.
#[derive(Clone, Copy, Debug)]
pub struct LaneChangeEval {
    pub side: Side,
    /// What made it want to change that way.
    pub incentives: Incentives,
    /// Its desire for that side, smoothed.
    pub desire: f64,
    /// The gap, if it was checked: its own acceleration there, and its new
    /// follower's.
    pub own_after: Option<f64>,
    pub new_follower: Option<f64>,
    /// The hardest braking it accepts from the change, at its desire.
    pub accepted_braking: f64,
    /// Whether it and its new follower would brake gently enough.
    pub safe: bool,
    /// Whether it would be halfway through the change before it had to stop
    /// behind its leader (see [`Prospect::room`]).
    pub room: bool,
    pub outcome: Outcome,
}

/// What a vehicle decided about changing to one side, this step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Not enough desire.
    NotWorthIt,
    /// Enough desire, but waiting a few steps after a refused gap, or for its
    /// turn, the simulation checking only so many gaps a step.
    Waiting,
    /// Enough desire, but it, or the follower it would cut in front of, would
    /// brake too hard, or it would be left straddling.
    Unsafe,
    /// Not this side: the other has more desire, or it is indicating the other
    /// way.
    OtherSide,
    /// Started indicating.
    Indicate,
    /// Started indicating without a gap, its desire strong enough to
    /// synchronise and ask for room.
    Request,
    /// Still indicating, waiting for a gap.
    Wait,
    /// Changing lanes now.
    Change,
    /// Stopped indicating.
    Cancel,
}

/// How a driver weighed up a side it didn't check the gap for this step.
pub fn unchecked(
    side: Side,
    incentives: Incentives,
    desire: f64,
    outcome: Outcome,
) -> LaneChangeEval {
    LaneChangeEval {
        side,
        incentives,
        desire,
        own_after: None,
        new_follower: None,
        accepted_braking: 0.0,
        safe: false,
        room: false,
        outcome,
    }
}

/// Resolves a gap check for a change to `side`, measured as `prospect`, and
/// returns what to do, with how it was weighed up.
///
/// - Not indicating: if the gap is safe and leaves room, it starts indicating;
///   if not, it starts all the same once its desire reaches [`D_SYNC`], asking
///   followers to make room; otherwise it waits a few steps before checking
///   again.
/// - Indicating: it changes if the gap is safe and leaves room, or, its desire
///   reaching [`D_COOP`], without room; otherwise it waits a few steps.
///
/// A change starts its desires afresh.
pub fn resolve(
    state: &mut State,
    intent: Option<Intent>,
    spec: &VehicleSpec,
    side: Side,
    incentives: Incentives,
    prospect: Prospect,
) -> (Decision, LaneChangeEval) {
    let desire = state.desire[index(side)];
    let accepted = accepted_braking(spec, desire);
    let safe =
        prospect.own_after >= -accepted && prospect.new_follower.is_none_or(|a| a >= -accepted);
    let room = prospect.room || desire >= D_COOP;
    let (decision, outcome) = match intent {
        None if safe && room => (Decision::Indicate(side), Outcome::Indicate),
        None if desire >= D_SYNC => (Decision::Indicate(side), Outcome::Request),
        None => (Decision::Nothing, Outcome::Unsafe),
        Some(_) if safe && room => (Decision::Change(side), Outcome::Change),
        Some(_) => (Decision::Nothing, Outcome::Wait),
    };
    match decision {
        Decision::Change(_) => {
            *state = State { yielding: state.yielding, ..State::default() };
        }
        Decision::Nothing => state.cooldown = cooldown(spec),
        _ => {}
    }
    let eval = LaneChangeEval {
        side,
        incentives,
        desire,
        own_after: Some(prospect.own_after),
        new_follower: prospect.new_follower,
        accepted_braking: accepted,
        safe,
        room: prospect.room,
        outcome,
    };
    (decision, eval)
}

/// How many steps a driver waits after a refused gap: [`COOLDOWN`], longer for
/// a more polite driver and shorter for a more assertive one, so drivers don't
/// check in step.
pub fn cooldown(spec: &VehicleSpec) -> u16 {
    let scale = 1.0 + spec.politeness - 0.5 * spec.assertiveness;
    (COOLDOWN * scale).round().clamp(1.0, f64::from(u16::MAX)) as u16
}

/// Whom a follower makes room for, if anyone: a vehicle indicating into its lane
/// ahead, whose ghost it then treats as a leader, if that vehicle's desire is
/// at least [`D_COOP`], LMRS's cooperation. `ghost` is the first ghost its
/// search met before any leader, with its owner's desire and the acceleration
/// following it would take. A follower that has already chosen a ghost keeps
/// making room for it up to [`SAFE_BRAKING`]; otherwise it starts only if that
/// takes no more braking than its courtesy, so nearer followers often drive on
/// and a later one, further back or more courteous, opens the gap. This is
/// Hidas's (2002, 2005) cooperative lane changing, and SUMO's `lcCooperative`.
pub fn make_room(
    state: &State,
    spec: &VehicleSpec,
    ghost: Option<(VehicleId, f64, f64)>,
) -> Option<VehicleId> {
    let (owner, desire, accel) = ghost?;
    let chosen = state.yielding == Some(owner);
    if !chosen && desire < D_COOP {
        return None;
    }
    let courteous = spec.courtesy > 0.0;
    let limit = if chosen { SAFE_BRAKING } else { spec.courtesy };
    ((chosen || courteous) && accel >= -limit).then_some(owner)
}

#[cfg(test)]
mod test {
    use super::*;

    fn spec() -> VehicleSpec {
        VehicleSpec {
            length: 4.6,
            width: 1.9,
            max_speed: 30.0,
            speed_factor: 1.0,
            max_accel: 1.0,
            comfy_decel: 1.5,
            time_headway: 1.0,
            min_gap: 1.5,
            rear_axle: 1.0,
            front_axle: 3.7,
            courtesy: 1.0,
            politeness: 0.2,
            assertiveness: 0.0,
            accel_noise: 0.0,
            line_gap: 1.0,
        }
    }

    #[test]
    fn voluntary_desire_gives_way_to_a_strong_route_desire_against_it() {
        let toward = Incentives { route: 0.0, speed: 0.4, keep: 0.0, courtesy: 0.0 };
        assert_eq!(toward.desire(), 0.4);
        // A weak route desire the other way leaves the speed desire whole.
        let weak = Incentives { route: -0.3, ..toward };
        assert!((weak.desire() - 0.1).abs() < 1e-12);
        // A route desire strong enough to cooperate overrides it.
        let strong = Incentives { route: -0.8, ..toward };
        assert_eq!(strong.desire(), -0.8);
        // In between, it counts in part.
        let some = Incentives { route: -0.7, ..toward };
        assert!(
            some.desire() < -0.3 && some.desire() > -0.7,
            "{}",
            some.desire()
        );
    }

    #[test]
    fn route_desire_rises_to_one_at_the_lane_end() {
        assert_eq!(route_desire(ROUTE_DISTANCE, 1.0, 1), 0.0);
        assert!((route_desire(0.5 * ROUTE_DISTANCE, 1.0, 1) - 0.5).abs() < 1e-12);
        assert_eq!(route_desire(0.0, 10.0, 1), 1.0);
        // Fast, time runs out before distance does.
        assert!(route_desire(ROUTE_DISTANCE, 30.0, 1) > 0.0);
        // With two changes to make, it starts twice as far out.
        assert!((route_desire(ROUTE_DISTANCE, 1.0, 2) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn desire_is_smoothed_and_a_check_waits_for_the_threshold() {
        let mut state = State::default();
        let wants = Incentives { speed: 0.9, ..Incentives::default() };
        let dt = 1.0 / 60.0;
        // A fleeting desire: one step doesn't get there.
        aim(&mut state, [Some(wants), None]);
        let ask = step(&mut state, None, &spec(), 0.0, dt);
        assert_eq!(ask, Ask::Nothing);
        assert!(state.desire(Side::Left) > 0.0 && state.desire(Side::Left) < D_FREE);
        // Lasting, it does, and asks for the gap.
        let mut ask = Ask::Nothing;
        for _ in 0..60 {
            ask = step(&mut state, None, &spec(), 0.0, dt);
            if ask != Ask::Nothing {
                break;
            }
        }
        assert_eq!(ask, Ask::Check(Side::Left));
        // Not permitted that way: no check, however strong.
        let mut state = State { desire: [0.9, 0.0], ..State::default() };
        assert_eq!(
            {
                aim(&mut state, [None, None]);
                step(&mut state, None, &spec(), 0.0, dt)
            },
            Ask::Nothing
        );
    }

    #[test]
    fn a_refused_gap_cools_off_before_the_next_check() {
        let mut state = State { desire: [0.5, 0.0], ..State::default() };
        let refused = Prospect { own_after: -5.0, new_follower: None, room: true };
        let (decision, eval) = resolve(
            &mut state,
            None,
            &spec(),
            Side::Left,
            Incentives::default(),
            refused,
        );
        assert_eq!(
            (decision, eval.outcome),
            (Decision::Nothing, Outcome::Unsafe)
        );
        let wait = state.cooldown();
        assert_eq!(wait, cooldown(&spec()));
        let wants = Some(Incentives { speed: 0.5, ..Incentives::default() });
        aim(&mut state, [wants, None]);
        for _ in 0..wait {
            let ask = step(&mut state, None, &spec(), 0.0, 1.0 / 60.0);
            assert_eq!(ask, Ask::Nothing);
        }
        let ask = step(&mut state, None, &spec(), 0.0, 1.0 / 60.0);
        assert_eq!(ask, Ask::Check(Side::Left));
        // Politer drivers wait longer, more assertive ones less.
        let polite = VehicleSpec { politeness: 0.5, ..spec() };
        let assertive = VehicleSpec { assertiveness: 1.0, ..spec() };
        assert!(cooldown(&polite) > cooldown(&spec()));
        assert!(cooldown(&assertive) < cooldown(&spec()));
    }

    #[test]
    fn enough_desire_indicates_without_a_gap_and_more_changes_without_room() {
        let unsafe_gap = Prospect { own_after: -5.0, new_follower: None, room: true };
        let mut state = State { desire: [D_SYNC, 0.0], ..State::default() };
        let (decision, _) = resolve(
            &mut state,
            None,
            &spec(),
            Side::Left,
            Incentives::default(),
            unsafe_gap,
        );
        assert_eq!(decision, Decision::Indicate(Side::Left));
        let no_room = Prospect { own_after: 0.0, new_follower: None, room: false };
        let intent = Some(Intent { side: Side::Left, since: 0.0 });
        let mut state = State { desire: [0.5, 0.0], ..State::default() };
        let (decision, _) = resolve(
            &mut state,
            intent,
            &spec(),
            Side::Left,
            Incentives::default(),
            no_room,
        );
        assert_eq!(decision, Decision::Nothing);
        let mut state = State { desire: [D_COOP, 0.0], ..State::default() };
        let (decision, _) = resolve(
            &mut state,
            intent,
            &spec(),
            Side::Left,
            Incentives::default(),
            no_room,
        );
        assert_eq!(decision, Decision::Change(Side::Left));
    }

    #[test]
    fn desire_shortens_the_headway_and_raises_the_braking_accepted() {
        assert_eq!(headway(0.0), 1.0);
        let relaxed = crate::models::acceleration::RELAXED_HEADWAY;
        assert!((headway(1.0) - relaxed).abs() < 1e-12);
        assert!(accepted_braking(&spec(), 0.5) > accepted_braking(&spec(), 0.4));
        let assertive = VehicleSpec { assertiveness: 1.0, ..spec() };
        assert!(accepted_braking(&assertive, 0.5) > accepted_braking(&spec(), 0.5));
        assert_eq!(accepted_braking(&assertive, 1.0), SAFE_BRAKING);
    }

    #[test]
    fn followers_make_room_only_for_a_strong_desire() {
        let state = State::default();
        let mut ids = slotmap::SlotMap::<VehicleId, ()>::with_key();
        let owner = ids.insert(());
        assert_eq!(make_room(&state, &spec(), Some((owner, 0.5, -0.5))), None);
        assert_eq!(
            make_room(&state, &spec(), Some((owner, 0.9, -0.5))),
            Some(owner)
        );
        // Beyond its courtesy, it drives on.
        assert_eq!(make_room(&state, &spec(), Some((owner, 0.9, -2.0))), None);
    }
}
