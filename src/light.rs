use crate::{LinkId, TrafficControl};
use std::cell::Cell;

/// A set of coordinated traffic lights.
#[derive(Clone, Default)]
pub struct TrafficLight {
    /// The movements.
    movements: Vec<Movement>,
    /// The links controlled by each movement.
    links: Vec<(u8, LinkId)>,
    /// The traffic light phases.
    phases: Vec<Phase>,
    /// The current traffic light phase.
    phase: usize,
}

/// A single traffic light movement.
#[derive(Clone, Debug)]
struct Movement {
    /// The current state.
    state: LightState,
    /// The next state.
    next_state: Cell<LightState>,
    /// Whether the target state is green.
    active: bool,
    /// The time since the current state was entered.
    since: f64,
    /// The duration of the amber phase in seconds.
    amber_time: f64,
    /// Conflicting movements cannot go green until this movement
    /// has been red for at least this duration, in seconds.
    red_time: f64,
    /// Bitmask used for determing conflicting movements.
    conflict: u64,
}

/// The state of a traffic light movement.
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum LightState {
    /// Red light
    Red,
    /// Amber light
    Amber,
    /// Green light
    Green,
}

/// A traffic light phase.
#[derive(Clone, Copy)]
pub struct Phase {
    /// A bit mask denoting which movements should be green in this phase.
    mask: u64,
    /// The duration of the phase once all lights have turned green.
    duration: f64,
}

impl TrafficLight {
    /// Creates a new traffic light system.
    pub fn new() -> Self {
        Default::default()
    }

    /// Adds a movement to the traffic light, which is a group of one or more links
    /// controlled by the same signal head(s) and so go and stop together.
    ///
    /// # Parameters
    /// * `conflict` - Bitmask for determining movement conflicts.
    /// * `amber_time` - The duration of the amber phase of this movement, in seconds.
    /// * `links` - An iterator which produces the set of links that belong to this movement.
    pub fn add_movement(
        &mut self,
        conflict: u64,
        amber_time: f64,
        red_time: f64,
        links: impl Iterator<Item = LinkId>,
    ) {
        let idx = self.movements.len() as u8;
        self.movements.push(Movement {
            state: LightState::Red,
            next_state: Cell::new(LightState::Red),
            active: false,
            since: 0.0,
            amber_time,
            red_time,
            conflict,
        });
        self.links.extend(links.map(|link| (idx, link)));
    }

    /// Adds a phase to the traffic light timing.
    pub fn add_phase(&mut self, mask: u64, duration: f64) {
        self.phases.push(Phase { mask, duration });
    }

    /// Gets the state of each link in the traffic light.
    pub fn state(&self) -> impl Iterator<Item = (usize, LightState)> + '_ {
        self.movements.iter().map(|m| m.state).enumerate()
    }

    /// Advances the traffic light timing by one frame.
    pub fn step(&mut self, dt: f64) {
        self.update_phase();
        self.apply_phase();

        // Compute which movements should change state
        for (idx, movement) in self.movements.iter().enumerate() {
            use LightState::*;
            let next = match (movement.active, movement.state) {
                (false, Green) => Amber,
                (false, Amber) if movement.since >= movement.amber_time => Red,
                (true, Amber) => Green,
                (true, Red) if self.can_turn_green(idx) => Green,
                (_, state) => state,
            };
            movement.next_state.set(next);
        }

        // Update each movement
        for movement in &mut self.movements {
            movement.step(dt);
        }
    }

    /// Advances to the next traffic light phase if appropriate.
    fn update_phase(&mut self) {
        if self.phases.is_empty() {
            return;
        }

        let Phase { duration, mask } = self.phases[self.phase];
        let phase_complete = self
            .movements
            .iter()
            .enumerate()
            .filter(|(index, _)| mask & (1 << index) != 0)
            .all(|(_, m)| m.state == LightState::Green && m.since >= duration);

        if phase_complete {
            self.phase = (self.phase + 1) % self.phases.len();
        }
    }

    /// Applies the current traffic light phase to each movement.
    fn apply_phase(&mut self) {
        let Phase { mask, .. } = self.phases[self.phase];
        for (idx, movement) in self.movements.iter_mut().enumerate() {
            movement.active = (mask >> idx) & 1 != 0;
        }
    }

    /// Gets the current state of each link
    pub fn get_states(&self) -> impl Iterator<Item = (LinkId, TrafficControl)> + '_ {
        self.links.iter().map(|(idx, link)| {
            let control = match self.movements[*idx as usize].state {
                LightState::Red => TrafficControl::Closed,
                LightState::Amber => TrafficControl::Closed,
                LightState::Green => TrafficControl::Open,
            };
            (*link, control)
        })
    }

    /// Checks that a movement is not blocked by any other movements.
    fn can_turn_green(&self, movement: usize) -> bool {
        let mask = 1 << movement;
        self.movements
            .iter()
            .filter(|other| other.conflict & mask != 0)
            .all(|other| other.state == LightState::Red && other.since > other.red_time)
    }
}

impl Movement {
    fn step(&mut self, dt: f64) {
        if self.next_state.get() != self.state {
            self.state = self.next_state.get();
            self.since = dt;
        } else {
            self.since += dt;
        }
    }
}
