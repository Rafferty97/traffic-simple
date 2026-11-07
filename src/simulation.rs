use anyhow::Context;
use glam::DVec2;
use itertools::Itertools;
use slotmap::{new_key_type, SlotMap};

use crate::debug::{DebugSink, ObjectSink};
use crate::geometry::{Lane, LanePos};
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
    next_link: usize,
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
    /// The vehicle's position in lane-space.
    lane_pos: LanePos,
    debug_target: DVec2,
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
    /// The maximum steering angle.
    max_steer: f64,
}

impl Simulation {
    pub fn new() -> Self {
        Self { links: SlotMap::with_key(), vehicles: SlotMap::with_key(), next_link: 0 }
    }

    pub fn insert_vehicle(&mut self) -> anyhow::Result<VehicleId> {
        let attrs = VehicleAttributes {
            half_wid: 0.89,
            rear_len: 0.98,
            front_len: 3.65,
            wheel_base: 2.70,
            max_steer: 0.1,
        };

        let (link_id, link) = self
            .links
            .iter()
            .cycle()
            .nth(self.next_link)
            .context("network is empty")?;
        self.next_link += 1;
        let lane_pos = link.lane.lane_pos(0.0, 0.0);
        let pos = link.lane.lane_to_world(lane_pos);
        let dir = link.lane.tangent_at(lane_pos).to_angle();

        let id = self.vehicles.insert_with_key(|id| Vehicle {
            id,
            attrs,
            pos,
            dir,
            vel: 0.0,
            acc: 0.0,
            steer: 0.0,
            links: vec![link_id],
            lane_pos,
            debug_target: pos,
        });

        Ok(id)
    }

    pub fn step(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.acc = 2.0;

            let link = &self.links[vehicle.links[0]];
            let target_s = vehicle.lane_pos.s + 5.5;
            let target = link.lane.lane_to_world(link.lane.lane_pos(target_s, 0.0));
            vehicle.debug_target = target;
            vehicle.steer = pure_pursuit(&vehicle.attrs, vehicle.pos, vehicle.dir, target);
        }
        self.integrate(dt);
    }

    fn integrate(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.integrate(dt);

            let link = &self.links[vehicle.links[0]];
            vehicle.lane_pos = link.lane.world_to_lane_near(vehicle.pos, vehicle.lane_pos);
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
        obj.polyline(&points, true, Some("lane"));
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
        obj.polyline(&points, true, Some("body"));
        obj.point(self.debug_target, Some("target"));
    }
}

fn pure_pursuit(attrs: &VehicleAttributes, rear: DVec2, heading: f64, target: DVec2) -> f64 {
    let to_target = target - rear;
    let ld = to_target.length().max(1e-6);
    let fwd = DVec2::from_angle(heading);
    let alpha = fwd.perp_dot(to_target).atan2(fwd.dot(to_target));
    let curvature = 2.0 * alpha.sin() / ld; // rad/m, positive = turning left (CCW)
    curvature.clamp(-attrs.max_steer, attrs.max_steer)
}
