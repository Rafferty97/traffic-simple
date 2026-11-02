use glam::DVec2;
use slotmap::{new_key_type, SlotMap};

use crate::math::{sinc, wrap_angle};

new_key_type! {
    /// Unique ID of a [Link].
    pub struct LinkId;
    /// Unique ID of a [Vehicle].
    pub struct VehicleId;
}

#[derive(Debug)]
pub struct Simulation {
    vehicles: SlotMap<VehicleId, Vehicle>,
}

#[derive(Debug)]
pub struct Vehicle {
    id: VehicleId,
    attrs: VehicleAttributes,
    /// Vehicle's rear axle position in world space, in metres.
    pos: DVec2,
    /// Vehicle's heading in world space, in radians.
    /// Zero points along the positive x-axis, and positive angles turn counterclockwise.
    dir: f64,
    /// Vehicle's velocity in metres per second.
    vel: f64,
    /// Vehicle's acceleration in metres per second squared.
    acc: f64,
    /// Vehicle's steering, expressed as the trajectory curvature at the rear axle in radians per metre.
    steer: f64,
    /// Vehicle's route, including the link it's currently on.
    links: Vec<LinkId>,
}

#[derive(Clone, Copy, Debug)]
pub struct VehicleAttributes {
    /// Half the vehicle's width in metres.
    half_wid: f64,
    /// The distance from the rear axle to the vehicle's rear in metres.
    rear_len: f64,
    /// The distance from the rear axle to the vehicle's front in metres.
    front_len: f64,
    /// The distance between the rear and front axle's in metres.
    wheel_base: f64,
}

impl Simulation {
    fn integrate(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.integrate(dt);
        }
    }
}

impl Vehicle {
    fn integrate(&mut self, dt: f64) {
        let next_vel = (self.vel + dt * self.acc).max(0.0);
        let dist = 0.5 * dt * (self.vel + next_vel);
        self.vel = next_vel;

        let theta = self.steer * dist;
        self.pos += dist * sinc(0.5 * theta) * DVec2::from_angle(self.dir + 0.5 * theta);
        self.dir = wrap_angle(self.dir + theta);
    }
}
