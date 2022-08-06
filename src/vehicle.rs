use self::acceleration::AccelerationModel;
use self::dynamics::calc_direction;
use crate::group::{LinkProjection, Obstacle};
use crate::math::{project_local, rot90, CubicFn, Point2d, Vector2d};
use crate::util::Interval;
use crate::{Link, LinkId, LinkSet, VehicleId};
use cgmath::prelude::*;
use std::cell::Cell;

mod acceleration;
mod dynamics;

/// The minimum lateral clearance for own vehicle to pass another, in m.
const LATERAL_CLEARANCE: f64 = 0.5;

/// A simulated vehicle.
#[derive(Clone, Debug)]
pub struct Vehicle {
    /// The vehicle's ID
    pub(crate) id: VehicleId,
    /// Half the vehicle's width in m.
    half_wid: f64,
    /// Half the vehicle's length in m.
    half_len: f64,
    /// Distance from vehicle's centre to centre of wheel axle.
    wheel_base: f64,
    /// The acceleration model
    acc: AccelerationModel,
    /// The longitudinal position along the current link, in m.
    pos: f64,
    /// The velocity in m/s.
    vel: f64,
    /// The number of frames that the vehicle has been stopped.
    stop_cnt: usize,
    /// The vehicle's route, including the link it's currently on.
    route: Vec<LinkId>,
    /// The number of links on the route already "entered".
    entered: usize,
    /// Whether the vehicle has queued to enter the next link.
    /// Contains a sequence number which helps determine vehicle priority.
    queued: Option<usize>,
    /// Whether the vehicle will queue into or enter the next unentered link on its route.
    /// 0 = no change, 1 = enqueue, 2 = enter
    will_queue_or_enter: Cell<u8>,
    /// Whether the vehicle can exit at the end of its route.
    can_exit: bool,
    /// The in-progress lane change, if there is one.
    lane_change: Option<LaneChange>,
    /// The world space coordinates of the centre of the vehicle.
    world_pos: Point2d,
    /// A world space vector tangent to the vehicle's heading.
    world_dir: Vector2d,
    /// The lateral extents used to compute the `rear_coords`.
    rear_lats: Interval<f64>,
    /// The two end points of a line behind the vehicle used for car following.
    rear_coords: [Point2d; 2],
}

/// The attributes of a simulated vehicle.
#[derive(Clone, Copy)]
pub struct VehicleAttributes {
    /// The vehicle width in m.
    pub width: f64,
    /// The vehicle length in m.
    pub length: f64,
    /// Distance from vehicle's centre to centre of wheel axle.
    pub wheel_base: f64,
    /// The maximum acceleration of the vehicle, in m/s^2.
    pub max_acc: f64,
    /// The comfortable deceleration of the vehicle, a negative number in m/s^2.
    pub comf_dec: f64,
}

/// Represents an in-progress lane change.
#[derive(Clone, Copy, Debug)]
pub struct LaneChange {
    /// The longitudinal position at which the lane change is complete.
    pub end_pos: f64,
    /// The vehicle's lateral offset during the lane change.
    pub offset: CubicFn,
}

/// The result of an `get_route` call.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RouteState {
    NotEntered,
    QueuedAt(usize),
    Entered,
}

pub enum ObstaclePassResult {
    Pass,
    Follow { pos: f64, vel: f64 },
}

impl Vehicle {
    /// Creates a new vehicle.
    pub(crate) fn new(id: VehicleId, attributes: &VehicleAttributes) -> Self {
        Self {
            id,
            half_wid: 0.5 * attributes.width,
            half_len: 0.5 * attributes.length,
            wheel_base: attributes.wheel_base,
            acc: AccelerationModel::new(&acceleration::ModelParams {
                max_acceleration: attributes.max_acc,
                comf_deceleration: attributes.comf_dec,
            }),
            pos: 0.0,
            vel: 0.0,
            stop_cnt: 0,
            route: vec![],
            entered: 0,
            queued: None,
            will_queue_or_enter: Cell::new(0),
            can_exit: true,
            lane_change: None,
            world_pos: Point2d::new(0.0, 0.0),
            world_dir: Vector2d::new(0.0, 0.0),
            rear_lats: Default::default(),
            rear_coords: [Point2d::new(0.0, 0.0); 2],
        }
    }

    /// Gets the vehicle's ID.
    pub fn id(&self) -> VehicleId {
        self.id
    }

    /// The vehicle's width in m.
    pub fn width(&self) -> f64 {
        2.0 * self.half_wid
    }

    /// The vehicle's length in m.
    pub fn length(&self) -> f64 {
        2.0 * self.half_len
    }

    /// The ID of the link the vehicle is currently travelling on.
    pub fn link_id(&self) -> Option<LinkId> {
        self.route.get(0).copied()
    }

    /// The longitudinal position of the centre of the vehicle in m.
    pub fn pos_mid(&self) -> f64 {
        self.pos
    }

    /// The longitudinal position of the rear of the vehicle in m.
    pub fn pos_rear(&self) -> f64 {
        self.pos - self.half_len
    }

    /// The longitudinal position of the front of the vehicle in m.
    pub fn pos_front(&self) -> f64 {
        self.pos + self.half_len
    }

    /// The coordinates in world space of the centre of the vehicle.
    pub fn position(&self) -> Point2d {
        self.world_pos
    }

    /// A unit vector in world space aligned with the vehicle's heading.
    pub fn direction(&self) -> Vector2d {
        self.world_dir
    }

    /// The two end points of a line behind the vehicle used for car following.
    pub fn rear_coords(&self) -> [Point2d; 2] {
        self.rear_coords
    }

    /// The vehicle's velocity in m/s.
    pub fn vel(&self) -> f64 {
        self.vel
    }

    /// Whether the vehicle is stopped.
    pub fn has_stopped(&self) -> bool {
        self.vel < 0.1
    }

    /// Set the desired velocity adjustment factor for the vehicle, a scalar which is
    /// multiplied with the speed limit prior to calculating the speed limit
    /// acceleration of the vehicle each frame.
    pub fn set_velocity_adjust(&mut self, factor: f64) {
        self.acc.set_velocity_adjust(factor);
    }

    /// Calculates the time it would take the vehicle to reach the given `pos`
    /// if it accelerates to the maximum speed at top acceleration.
    pub fn min_reach_time(&self, pos: f64, max_vel: f64) -> f64 {
        let dist = f64::max(pos - self.pos, 0.0);
        self.acc.min_reach_time(self.vel, dist, max_vel)
    }

    /// Gets the status of a link on the vehicle's route.
    pub(crate) fn get_route(&self, idx: usize) -> Option<(LinkId, RouteState)> {
        self.route.get(idx).map(|link_id| {
            use std::cmp::Ordering::*;
            let state = match (self.entered.cmp(&idx), self.queued) {
                (Greater, _) => RouteState::Entered,
                (Equal, Some(f)) => RouteState::QueuedAt(f),
                _ => RouteState::NotEntered,
            };
            (*link_id, state)
        })
    }

    /// Determines whether the vehicle can comfortably stop before reaching `pos`.
    pub(crate) fn can_stop(&self, pos: f64) -> bool {
        let net_dist = pos - self.pos_front();
        net_dist >= self.stopping_distance()
    }

    /// Determines the comfortable stopping distance of the vehicle.
    pub(crate) fn stopping_distance(&self) -> f64 {
        self.acc.stopping_distance(self.vel)
    }

    /// Queues into the next unentered link on the vehicle's route.
    pub(crate) fn queue_link(&self) {
        self.will_queue_or_enter.set(1);
    }

    /// Queues into the next unentered link on the vehicle's route.
    pub(crate) fn enter_link(&self) {
        self.will_queue_or_enter.set(2);
    }

    /// Applies an acceleration to the vehicle so it follows an obstacle.
    pub(crate) fn stop_at_line(&self, pos: f64) {
        let net_dist = pos - self.pos_front();
        self.acc.stop_at_line(net_dist, self.vel);
    }

    /// Applies an acceleration to the vehicle so it follows an obstacle.
    pub(crate) fn follow_vehicle(&self, pos: f64, vel: f64) {
        let net_dist = pos - self.pos_front();
        self.acc.follow_vehicle(net_dist, self.vel, vel);
    }

    /// Applies an acceleration to the vehicle so it follows an obstacle.
    pub(crate) fn follow_obstacle(&self, coords: [Point2d; 2], vel: f64) {
        let mid_dist = f64::min(
            self.direction().dot(coords[0] - self.position()),
            self.direction().dot(coords[1] - self.position()),
        );
        let net_dist = mid_dist - self.half_len;
        self.acc.follow_vehicle(net_dist, self.vel, vel);
    }

    /// Applies a current speed limit to the vehicle.
    pub(crate) fn apply_current_speed_limit(&self, speed_limit: f64) {
        self.acc.apply_current_speed_limit(self.vel, speed_limit);
    }

    /// Applies an upcoming speed limit to the vehicle.
    pub(crate) fn apply_speed_limit(&self, speed_limit: f64, pos: f64) {
        self.acc
            .apply_speed_limit(self.vel, speed_limit, pos - self.pos);
    }

    /// Applies a maximum deceleration to the vehicle, causing it to stop.
    pub(crate) fn emergency_stop(&self) {
        self.acc.emergency_stop();
    }

    /// Projects the vehicle onto another link.
    pub(crate) fn project(&self, projection: &LinkProjection) -> Obstacle {
        projection.project(
            self.rear_coords,
            self.pos - self.half_len,
            self.rear_lats,
            self.vel,
        )
    }

    /// Gets the vehicle's lateral offset from the centre line
    /// at the given longitudinal position along the current link.
    pub(crate) fn offset_at(&self, pos: f64) -> f64 {
        self.lane_change
            .filter(|lc| lc.end_pos > pos)
            .map(|lc| lc.offset.y(pos))
            .unwrap_or(0.0)
    }

    /// Resets internal model states in preparation for a new step of the simulation.
    pub(crate) fn reset(&self) {
        self.acc.reset()
    }

    /// Integrates the vehicle's velocity and position
    ///
    /// # Parameters
    /// * `dt` - T he time step in seconds
    pub(crate) fn integrate(&mut self, dt: f64, seq: &mut usize) {
        // Enter/enqueue into next link
        match self.will_queue_or_enter.get() {
            0 => {}
            1 => {
                *seq += 1;
                self.queued.get_or_insert(*seq);
            }
            2 => {
                self.entered += 1;
                self.queued = None;
            }
            _ => unreachable!(),
        }
        self.will_queue_or_enter.set(0);

        // Perform the integration
        let vel = f64::max(self.vel + dt * self.acc.acc(), 0.0);
        let pos = self.pos + 0.5 * (self.vel + vel) * dt;
        self.vel = vel;
        self.pos = pos;

        // Check for lane change completion
        self.lane_change = self.lane_change.filter(|lc| lc.end_pos > pos);

        // Update the stop count
        self.update_stop_count();
    }

    /// Updates the vehicle's stop counter.
    fn update_stop_count(&mut self) {
        if self.vel < 0.1 {
            self.stop_cnt += 1;
        } else {
            self.stop_cnt = 0;
        }
    }

    /// Checks whether the vehicle has travelled past the end of its current link,
    /// and if so, advances it to the next link on its route if there is one.
    /// Returns `true` iff the vehicle was advanced.
    ///
    /// # Parameters
    /// * `links` - The links in the network
    /// * `now` - The current frame of simulation
    pub(crate) fn advance(&mut self, links: &LinkSet) -> bool {
        if let Some(link_id) = self.route.get(0) {
            let length = links[*link_id].length();
            if length < self.pos {
                self.route.remove(0);
                self.entered = usize::max(self.entered - 1, 1);
                self.pos -= length;
                if let Some(lc) = self.lane_change.as_mut() {
                    lc.offset = lc.offset.translate_x(length);
                    lc.end_pos -= length;
                }
                return true;
            }
        }
        false
    }

    /// Sets the vehicle's position in the network.
    /// This also clears the vehicle's route.
    pub(crate) fn set_location(&mut self, link: LinkId, pos: f64, lane_change: Option<LaneChange>) {
        self.route = vec![link];
        self.entered = 1;
        self.pos = pos;
        self.lane_change = lane_change;
        self.can_exit = false;
    }

    /// Sets the vehicle's route.
    pub(crate) fn set_route(&mut self, route: &[LinkId], can_exit: bool) {
        self.route.truncate(1);
        self.route.extend(route);
        self.entered = 1;
        self.can_exit = can_exit;
    }

    /// Updates the vehicle's world coordinates
    pub(crate) fn update_coords(&mut self, links: &LinkSet) {
        let link = &links[self.route[0]];
        let curve = link.curve();

        // Sample the curve
        let sample = curve.sample_centre(self.pos);
        let (pos, offset) = match self.lane_change {
            Some(lc) => {
                let offset = lc.offset.y(self.pos);
                let pos = sample.lat_offset(offset);
                (pos, offset)
            }
            None => (sample.pos, 0.0),
        };

        // Default `world_dir` to be tangent to the link
        if self.world_dir == Vector2d::new(0.0, 0.0) {
            self.world_pos = pos - sample.tan;
            self.world_dir = sample.tan;
        }

        // `perp` is perpendicular to the link
        let perp = rot90(sample.tan);

        // Compute the vehicle's heading
        let dir = calc_direction(self.world_pos, self.world_dir, pos, self.wheel_base);
        self.world_pos = pos;
        self.world_dir = dir;

        // Compute the vehicle's lateral extent, accounting for any rotation relative to the curve
        let expand = -dir.dot(perp) * self.half_len;
        self.rear_lats = Interval {
            min: f64::min(expand + offset, 0.0) - self.half_wid,
            max: f64::max(expand + offset, 0.0) + self.half_wid,
        };

        // Compute the vehicle's rear coordinates
        let rear_mid = sample.long_offset(-self.half_len);
        self.rear_coords = self.rear_lats.as_array().map(|lat| rear_mid + lat * perp);
        // debug_line("rear coords", self.rear_coords[0], self.rear_coords[1]);
    }

    /// Sets the `active` flag for the links this vehicle has entered (including the one its currently on).
    pub(crate) fn activate_links(&self, links: &mut LinkSet) {
        for link_id in self.route.iter().take(self.entered) {
            links[*link_id].activate();
        }
    }

    /// Determines whether the vehicle can pass the given obstacle.
    pub(crate) fn can_pass(&self, mut obstacle: Obstacle, link: &Link) -> ObstaclePassResult {
        let is_clear = |obstacle: Obstacle| {
            let offset = self.offset_at(obstacle.pos - self.half_len);
            let distance = obstacle.lat.distance(offset);
            distance > self.half_wid + LATERAL_CLEARANCE
        };

        if is_clear(obstacle) {
            // let p = link.curve().sample_centre(obstacle.pos);
            // let l = obstacle.lat;
            // debug_line("pass", p.lat_offset(l.min), p.lat_offset(l.max));
            return ObstaclePassResult::Pass;
        }

        loop {
            let sample = link.curve().sample_centre(obstacle.pos);
            let ps = obstacle
                .rear_coords
                .map(|p| project_local(p, sample.pos, rot90(sample.tan), sample.tan));
            let delta_pos = f64::min(ps[0].y, ps[1].y);
            obstacle.pos += delta_pos;
            if delta_pos < 0.1 {
                obstacle.lat = Interval {
                    min: ps[0].x,
                    max: ps[1].x,
                };
                break;
            }
        }

        if is_clear(obstacle) {
            // let p = link.curve().sample_centre(obstacle.pos);
            // let l = obstacle.lat;
            // debug_line("pass adjust", p.lat_offset(l.min), p.lat_offset(l.max));
            ObstaclePassResult::Pass
        } else {
            // let p = link.curve().sample_centre(obstacle.pos);
            // let l = obstacle.lat;
            // debug_line("follow", p.lat_offset(l.min), p.lat_offset(l.max));
            ObstaclePassResult::Follow {
                pos: obstacle.pos,
                vel: obstacle.vel,
            }
        }
    }
}
