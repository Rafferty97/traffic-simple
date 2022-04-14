use self::acceleration::AccelerationModel;
use crate::link::LinkSample;
use crate::math::{project_local, rot90, CubicFn, Point2d, Vector2d};
use crate::obstacle::Obstacle;
use crate::util::Interval;
use crate::{LinkId, LinkSet, VehicleId};
use cgmath::prelude::*;
use std::cell::Cell;

mod acceleration;

/// A simulated vehicle.
#[derive(Clone)]
pub struct Vehicle {
    /// The vehicle's ID
    pub(crate) id: VehicleId,
    /// Half the vehicle's width in m.
    half_wid: f64,
    /// Half the vehicle's length in m.
    half_len: f64,
    /// The acceleration model
    acc: AccelerationModel,
    /// The longitudinal position along the current link, in m.
    pos: f64,
    /// The velocity in m/s.
    vel: f64,
    /// The number of frames that the vehicle has been stopped.
    stop_cnt: usize,
    /// The vehicle's route, including the link it's currently on.
    route: Vec<RouteElement>,
    /// Whether the vehicle can exit at the end of its route.
    can_exit: bool,
    /// The in-progress lane change, if there is one.
    lane_change: Option<LaneChange>,
    /// The world space coordinates of the centre of the vehicle.
    world_pos: Point2d,
    /// A world space vector tangent to the vehicle's heading.
    world_dir: Vector2d,
    /// The two end points of a line behind the vehicle used for car following.
    rear_coords: [Point2d; 2],
    /// Tracks the vehicle's projection onto adjacent links, for performance.
    min_segments: [Cell<u16>; 8],
}

/// The attributes of a simulated vehicle.
#[derive(Clone, Copy)]
pub struct VehicleAttributes {
    /// The vehicle width in m.
    pub width: f64,
    /// The vehicle length in m.
    pub length: f64,
    /// The maximum acceleration of the vehicle, in m/s^2.
    pub max_acc: f64,
    /// The comfortable deceleration of the vehicle, a negative number in m/s^2.
    pub comf_dec: f64,
}

/// A link along a vehicle's route.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RouteElement {
    pub link: LinkId,
    pub entered_at: Option<usize>,
}

/// Represents an in-progress lane change.
#[derive(Clone, Copy)]
pub struct LaneChange {
    /// The longitudinal position at which the lane change is complete.
    pub end_pos: f64,
    /// The vehicle's lateral offset during the lane change.
    pub offset: CubicFn,
}

impl Vehicle {
    /// Creates a new vehicle.
    pub(crate) fn new(id: VehicleId, attributes: &VehicleAttributes) -> Self {
        Self {
            id,
            half_wid: 0.5 * attributes.width,
            half_len: 0.5 * attributes.length,
            acc: AccelerationModel::new(&acceleration::ModelParams {
                max_acceleration: attributes.max_acc,
                comf_deceleration: attributes.comf_dec,
            }),
            pos: 0.0,
            vel: 0.0,
            stop_cnt: 0,
            route: vec![],
            can_exit: true,
            lane_change: None,
            world_pos: Point2d::new(0.0, 0.0),
            world_dir: Vector2d::new(0.0, 0.0),
            rear_coords: [Point2d::new(0.0, 0.0); 2],
            min_segments: Default::default(),
        }
    }

    /// Gets the vehicle's ID.
    pub fn id(&self) -> VehicleId {
        self.id
    }

    /// Half the vehicle's width in m.
    pub(crate) fn half_width(&self) -> f64 {
        self.half_wid
    }

    /// Half the vehicle's length in m.
    pub(crate) fn half_length(&self) -> f64 {
        self.half_len
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
        self.route.get(0).map(|el| el.link)
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

    /// Gets the vehicle's route.
    pub(crate) fn route(&self) -> &[RouteElement] {
        &self.route
    }

    /// Gets the vehicle's route.
    pub(crate) fn on_route(&self, idx: usize, link: LinkId) -> bool {
        if let Some(element) = self.route.get(idx) {
            element.link == link
        } else {
            false
        }
    }

    /// Determines whether the vehicle can comfortably stop before reaching `pos`.
    pub(crate) fn can_stop(&self, pos: f64) -> bool {
        self.acc.can_stop(pos - self.pos_front(), self.vel)
    }

    /// Enters the link.
    pub(crate) fn enter_link(&mut self, idx: usize, now: usize) {
        for el in self.route.iter_mut().take(idx + 1) {
            el.entered_at.get_or_insert(now);
        }
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

    /// Gets the vehicle as an obstacle on its own link.
    pub(crate) fn local_obstacle(&self) -> Obstacle {
        Obstacle {
            pos: self.pos_rear(),
            lat: self.lat_extent(),
            vel: self.vel(),
        }
    }

    /// Gets the vehicle as an obstacle on an adjacent link.
    pub(crate) fn adjacent_obstacle(
        &self,
        idx: usize,
        curve: &[(f64, Point2d, Vector2d)],
    ) -> Obstacle {
        // Find the closest segment to the obstacle
        let segment_idx = curve
            .iter()
            .skip(self.min_segments[idx].get() as usize)
            .position(|(_, pos, tan)| self.rear_coords.iter().any(|c| (c - pos).dot(*tan) <= 0.0))
            .map(|i| i + self.min_segments[idx].get() as usize)
            .unwrap_or(curve.len())
            .saturating_sub(1);

        // Record the new segment index
        self.min_segments[idx].set(segment_idx as u16);

        // Get the local coordinate system around the obstacle
        let (pos, origin, tan) = unsafe {
            // SAFETY: Use of `position` above guarantees the index is in bounds.
            *curve.get_unchecked(segment_idx)
        };

        // Project the obstacle's rear coordinates
        let proj = self
            .rear_coords
            .map(|coord| project_local(coord, origin, rot90(tan), tan));

        // Create the obstacle
        Obstacle {
            pos: pos + f64::min(proj[0].y, proj[1].y),
            lat: Interval::new(proj[0].x, proj[1].x),
            vel: self.vel,
        }
    }

    /// Gets the vehicle's current lateral offset from the centre line.
    pub(crate) fn offset(&self) -> f64 {
        self.offset_at(self.pos)
    }

    /// Gets the vehicle's lateral offset from the centre line
    /// at the given longitudinal position along the current link.
    pub(crate) fn offset_at(&self, pos: f64) -> f64 {
        self.lane_change
            .filter(|lc| lc.end_pos > pos)
            .map(|lc| lc.offset.y(pos))
            .unwrap_or(0.0)
    }

    /// Gets the current lateral extents of the vehicle
    /// for the purpose of applying a car-following model.
    pub(crate) fn lat_extent(&self) -> Interval<f64> {
        self.lat_extent_at(self.pos)
    }

    /// Gets the lateral extents of the vehicle
    /// for the purpose of applying a car-following model,
    /// at the given longitudinal position along the current link.
    pub(crate) fn lat_extent_at(&self, pos: f64) -> Interval<f64> {
        let offset = self.offset_at(pos);
        Interval::new(
            f64::min(offset, 0.0) - self.half_width(),
            f64::max(offset, 0.0) + self.half_width(),
        )
    }

    /// Resets internal model states in preparation for a new step of the simulation.
    pub(crate) fn reset(&self) {
        self.acc.reset()
    }

    /// Integrates the vehicle's velocity and position
    ///
    /// # Parameters
    /// * `dt` - The time step in seconds
    pub(crate) fn integrate(&mut self, dt: f64) {
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
    pub(crate) fn advance(&mut self, links: &LinkSet, now: usize) -> bool {
        if let Some(el) = self.route.get(0) {
            let length = links[el.link].length();
            if length < self.pos {
                self.route.remove(0);
                if let Some(el) = self.route.get_mut(0) {
                    el.entered_at.get_or_insert(now);
                }
                self.pos -= length;
                if let Some(lc) = self.lane_change.as_mut() {
                    lc.offset = lc.offset.translate_x(length);
                    lc.end_pos -= length;
                }
                self.min_segments.fill(Cell::new(0));
                return true;
            }
        }
        false
    }

    /// Sets the vehicle's position in the network.
    /// This also clears the vehicle's route.
    pub(crate) fn set_location(&mut self, link: LinkId, pos: f64, lane_change: Option<LaneChange>) {
        self.route = vec![RouteElement {
            link,
            entered_at: None,
        }];
        self.pos = pos;
        self.lane_change = lane_change;
        self.can_exit = false;
        self.min_segments.fill(Cell::new(0));
    }

    /// Sets the vehicle's route.
    pub(crate) fn set_route(&mut self, route: &[LinkId], can_exit: bool) {
        self.route.truncate(1);
        self.route.extend(route.iter().map(|link| RouteElement {
            link: *link,
            entered_at: None,
        }));
        self.can_exit = can_exit;
    }

    /// Updates the vehicle's world coordinates
    pub(crate) fn update_coords(&mut self, links: &LinkSet) {
        let curve = &links[self.route[0].link].curve();

        let (pos, dir, tan, lats) = match self.lane_change {
            Some(lc) => {
                let (offset, slope) = lc.offset.y_and_dy(self.pos);
                let LinkSample { pos, dir, tan } = curve.sample(self.pos, offset, slope);
                let wid = self.half_wid + self.half_len * dir.perp_dot(tan).abs();
                let lats = [f64::min(-offset, 0.0) - wid, f64::max(-offset, 0.0) + wid];
                (pos, dir, tan, lats)
            }
            None => {
                let LinkSample { pos, dir, tan } = curve.sample_centre(self.pos);
                let lats = [-self.half_wid, self.half_wid];
                (pos, dir, tan, lats)
            }
        };

        self.world_pos = pos;
        self.world_dir = dir;

        let rear_mid = pos - self.half_len * tan;
        let perp = rot90(tan);
        self.rear_coords = lats.map(|lat| rear_mid + lat * perp);
    }
}
