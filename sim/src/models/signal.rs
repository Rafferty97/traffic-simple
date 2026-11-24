//! The signal model: fixed-time plans, each running its phases in turn (see
//! `docs/models/signals.md`). Replaced by editing this module,
//! for actuated control say.

use arrayvec::ArrayVec;

use crate::network::{SignalPlan, SimNetwork};

/// What a movement's signal shows.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Light {
    #[default]
    Red,
    Amber,
    Green,
}

/// A movement's light, and for how many steps it has shown it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct MovementLight {
    pub light: Light,
    pub steps: u32,
}

/// A controller's place in its plan.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct SignalState {
    /// The phase it is in.
    pub phase: u16,
    /// Steps since the phase began.
    pub steps: u32,
}

/// Every controller at the start, in its first phase, with every light red and
/// just turned so: each plan starts with its clearance.
pub fn initial(network: &SimNetwork) -> (Vec<SignalState>, Vec<MovementLight>) {
    let states = network.signals().map(|_| SignalState::default()).collect();
    let lights = vec![MovementLight::default(); network.movement_count()];
    (states, lights)
}

/// Advances every controller by one step of `dt` seconds. Each one's lights
/// change all at once, from their states before the step.
pub fn step(
    network: &SimNetwork,
    states: &mut [SignalState],
    lights: &mut [MovementLight],
    dt: f64,
) {
    for ((plan, first), state) in network.signals().zip(states.iter_mut()) {
        let first = first.0 as usize;
        let lights = &mut lights[first..first + plan.movements.len()];
        step_one(plan, state, lights, dt);
    }
}

fn step_one(plan: &SignalPlan, state: &mut SignalState, lights: &mut [MovementLight], dt: f64) {
    if plan.phases.is_empty() {
        return;
    }
    let lasted = |steps: u32, seconds: f64| f64::from(steps) * dt >= seconds;
    // On to the next phase once this one has lasted its duration, counted from
    // when all its movements are green.
    let phase = plan.phases[state.phase as usize];
    let in_phase = |i: usize| phase.green & (1 << i) != 0;
    let mut green = (0..lights.len()).filter(|&i| in_phase(i)).map(|i| lights[i]);
    let done = match green.clone().next() {
        Some(_) => green.all(|l| l.light == Light::Green && lasted(l.steps, phase.duration)),
        None => lasted(state.steps, phase.duration),
    };
    if done {
        state.phase = ((state.phase as usize + 1) % plan.phases.len()) as u16;
        state.steps = 0;
    } else {
        state.steps = state.steps.saturating_add(1);
    }
    let phase = plan.phases[state.phase as usize];

    // Every movement's next light, from the lights as they are.
    let next: ArrayVec<Light, 64> = (0..lights.len())
        .map(|i| {
            let (now, movement) = (lights[i], &plan.movements[i]);
            let wanted = phase.green & (1 << i) != 0;
            // Every movement it conflicts with red, for long enough to clear.
            let clear = || {
                (0..lights.len()).filter(|&j| movement.conflicts & (1 << j) != 0).all(|j| {
                    lights[j].light == Light::Red
                        && lasted(lights[j].steps, plan.movements[j].clearance)
                })
            };
            match (now.light, wanted) {
                (Light::Green, false) => Light::Amber,
                (Light::Amber, false) if lasted(now.steps, movement.amber) => Light::Red,
                (Light::Amber, true) => Light::Green,
                (Light::Red, true) if clear() => Light::Green,
                (light, _) => light,
            }
        })
        .collect();
    for (light, next) in lights.iter_mut().zip(next) {
        match light.light == next {
            true => light.steps = light.steps.saturating_add(1),
            false => *light = MovementLight { light: next, steps: 0 },
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::network::{Movement, Phase};

    const DT: f64 = 0.1;

    /// Two movements in conflict, each green in its own 10 s phase, with 3 s
    /// of amber and 2 s of clearance.
    fn plan() -> SignalPlan {
        let movement = |conflicts| Movement { conflicts, amber: 3.0, clearance: 2.0 };
        SignalPlan {
            movements: vec![movement(0b10), movement(0b01)],
            phases: vec![
                Phase { green: 0b01, duration: 10.0 },
                Phase { green: 0b10, duration: 10.0 },
            ],
        }
    }

    /// The lights at each step for `seconds`.
    fn run(seconds: f64) -> Vec<[Light; 2]> {
        let plan = plan();
        let mut state = SignalState::default();
        let mut lights = [MovementLight::default(); 2];
        (0..(seconds / DT).round() as usize)
            .map(|_| {
                step_one(&plan, &mut state, &mut lights, DT);
                [lights[0].light, lights[1].light]
            })
            .collect()
    }

    /// The step at which `pattern` first appears, in seconds.
    fn first(lights: &[[Light; 2]], pattern: [Light; 2]) -> f64 {
        lights.iter().position(|&l| l == pattern).unwrap() as f64 * DT
    }

    #[test]
    fn conflicting_movements_are_never_green_together() {
        let lights = run(120.0);
        assert!(!lights.contains(&[Light::Green, Light::Green]));
        assert!(!lights.iter().any(|&l| l.contains(&Light::Amber) && l.contains(&Light::Green)));
    }

    #[test]
    fn green_comes_after_the_others_clearance_and_lasts_its_phase() {
        use Light::*;
        let lights = run(60.0);
        // All red at first, for the other movement's clearance.
        let g0 = first(&lights, [Green, Red]);
        assert!((g0 - 2.0).abs() < 0.15, "first green at {g0}");
        // Green for its phase, then amber for its amber time, then red.
        let a0 = first(&lights, [Amber, Red]);
        assert!((a0 - g0 - 10.0).abs() < 0.15, "amber at {a0}");
        let g1 = first(&lights, [Red, Green]);
        // Red after 3 s of amber, then the other's green after 2 s of clearance.
        assert!((g1 - a0 - 5.0).abs() < 0.25, "second green at {g1}");
    }
}
