use anyhow::Context;
use glam::DVec2;
use itertools::Itertools;
use slotmap::{new_key_type, SlotMap};

use crate::debug::{DebugSink, ObjectSink};
use crate::geometry::{Lane, LaneCoord};
use crate::math::{sinc, wrap_angle};

pub mod parse_network;

new_key_type! {
    /// Unique ID of a [Link].
    pub struct LinkId;
    /// Unique ID of a [Vehicle].
    pub struct VehicleId;
}

#[derive(Debug)]
pub struct Simulation {
    links: SlotMap<LinkId, Link>,
    vehicles: SlotMap<VehicleId, Vehicle>,
}

#[derive(Debug)]
pub struct Link {
    id: LinkId,
    name: String,
    lane: Lane,
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
    pub fn new() -> Self {
        Self { links: SlotMap::with_key(), vehicles: SlotMap::with_key() }
    }

    pub fn insert_vehicle(&mut self) -> anyhow::Result<VehicleId> {
        let attrs =
            VehicleAttributes { half_wid: 0.89, rear_len: 0.98, front_len: 3.65, wheel_base: 2.70 };

        let (link_id, link) = self.links.iter().next().context("network is empty")?;
        let pos = link
            .lane
            .lane_to_world(LaneCoord { index: 0, s: 0.0, t: 0.0 });
        let pos2 = link
            .lane
            .lane_to_world(LaneCoord { index: 0, s: 0.1, t: 0.0 });
        let dir = (pos2 - pos).to_angle();

        let id = self.vehicles.insert_with_key(|id| Vehicle {
            id,
            attrs,
            pos,
            dir,
            vel: 0.0,
            acc: 0.0,
            steer: 0.0,
            links: vec![link_id],
        });

        Ok(id)
    }

    pub fn step(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.acc = 2.0;
        }
        self.integrate(dt);
    }

    fn integrate(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.integrate(dt);
        }
    }

    pub fn debug(&self, sink: &mut impl DebugSink) {
        for (_, link) in &self.links {
            link.debug(sink);
        }
        for (_, vehicle) in &self.vehicles {
            vehicle.debug(sink);
        }
    }
}

impl Link {
    fn debug(&self, sink: &mut impl DebugSink) {
        let points = self.lane.outline().collect_vec();
        let mut obj = sink.object(&self.name);
        obj.polyline(&points, true);
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

    fn debug(&self, sink: &mut impl DebugSink) {
        let h = DVec2::from_angle(self.dir); // forward
        let n = h.perp(); // left
        let VehicleAttributes { half_wid, rear_len, front_len, .. } = self.attrs;

        let rear = self.pos - rear_len * h;
        let front = self.pos + front_len * h;

        let points = [
            rear - half_wid * n,
            front - half_wid * n,
            front + half_wid * n,
            rear + half_wid * n,
        ];

        let mut obj = sink.object(&format!("veh:{:?}", self.id));
        obj.polyline(&points, true);
    }
}
