use crate::{LinkId, TrafficControl};
use serde::{Deserialize, Serialize};
use std::cell::Cell;

/// A set of coordinated traffic lights.
#[derive(Serialize, Deserialize, Clone)]
pub struct TrafficLight {
    /// The movements.
    movements: Vec<Movement>,
    /// The links controlled by each movement.
    links: Vec<(u8, LinkId)>,
    /// The conflicts between the movements.
    conflicts: Vec<Conflict>,
}

/// A single traffic light movement.
#[derive(Serialize, Deserialize, Clone)]
struct Movement {
    /// The current state.
    state: LightState,
    /// The next state.
    next_state: Cell<LightState>,
    /// Whether the target state is green.
    active: bool,
    /// The time since the current state was entered.
    since: usize,
    /// The duration of the amber phase in frames.
    amber_time: usize,
}

/// Represents one traffic light movement's conflict with another.
#[derive(Serialize, Deserialize, Clone, Copy)]
struct Conflict {
    /// The movement which is the subject of the conflict.
    subject: usize,
    /// The movement which conflicts with the subject.
    other: usize,
    /// The number of frames that the conflicting movement must be red
    /// before the subject movement is allowed to turn green.
    wait: usize,
}

/// The state of a traffic light movement.
#[derive(PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum LightState {
    Red,
    Amber,
    Green,
}

impl TrafficLight {
    /// Adds a movement to the traffic light.
    pub fn add_movement(&mut self, amber_time: usize, links: impl Iterator<Item = LinkId>) {
        let idx = self.movements.len() as u8;
        self.movements.push(Movement {
            state: LightState::Red,
            next_state: Cell::new(LightState::Red),
            active: false,
            since: 0,
            amber_time,
        });
        self.links.extend(links.map(|link| (idx, link)));
    }

    /// Adds a conflict between two movements to the traffic light.
    pub fn add_conflit(&mut self, subject: usize, other: usize, wait: usize) {
        self.conflicts.push(Conflict {
            subject,
            other,
            wait,
        });
    }

    /// Advances the traffic light timing by one frame.
    pub fn step(&mut self) {
        for (idx, movement) in self.movements.iter().enumerate() {
            use LightState::*;
            let next = match (movement.active, movement.state) {
                (false, Green) => Amber,
                (false, Amber) if movement.since >= movement.amber_time => Red,
                (true, Red) if !self.can_turn_green(idx) => Green,
                (_, state) => state,
            };
            movement.next_state.set(next);
        }
        for movement in &mut self.movements {
            movement.step();
        }
    }

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
        self.conflicts
            .iter()
            .filter(|conflict| conflict.subject == movement)
            .all(|conflict| {
                let movement = &self.movements[conflict.other];
                movement.state == LightState::Red && movement.since >= conflict.wait
            })
    }
}

impl Movement {
    fn step(&mut self) {
        if self.next_state.get() != self.state {
            self.state = self.next_state.get();
            self.since = 1;
        } else {
            self.since += 1;
        }
    }
}
