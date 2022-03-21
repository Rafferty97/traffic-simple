use std::cell::Cell;
use crate::obstacle::Obstacle;
use crate::util::Interval;
use crate::{VehicleId, LinkId, LinkSet};
use crate::math::{CubicFn, Point2d, Vector2d};

/// A simulated vehicle.
pub struct Vehicle {
    /// The vehicle's ID
    pub(crate) id: VehicleId,
    /// Half the vehicle's width in m.
    half_wid: f64,
    /// Half the vehicle's length in m.
    half_len: f64,
    /// The maximum acceleration of the vehicle, in m/s^2.
    max_acc: f64,
    /// The comfortable deceleration of the vehicle, a negative number in m/s^2.
    comf_dec: f64,
    /// The longitudinal position along the current link, in m.
    pos: f64,
    /// The velocity in m/s.
    vel: f64,
    /// The acceleration in m/s^2.
    acc: Cell<f64>,
    /// The vehicle's route, including the link it's currently on.
    route: Vec<RouteElement>,
    /// The in-progress lane change, if there is one.
    lane_change: Option<LaneChange>,
    /// The world space coordinates of the centre of the vehicle.
    world_pos: Point2d,
    /// A world space vector tangent to the vehicle's heading.
    world_tan: Vector2d
}

/// A link along a vehicle's route.
struct RouteElement {
    link: LinkId,
    entered_at: Option<usize>
}

/// Represents an in-progress lane change.
#[derive(Clone, Copy)]
struct LaneChange {
    /// The longitudinal position at which the lane change is complete.
    end_pos: f64,
    /// The vehicle's lateral offset during the lane change.
    offset: CubicFn
}

impl Vehicle {
    /// Half the vehicle's width in m.
    pub(crate) fn half_wid(&self) -> f64 {
        self.half_wid
    }
    
    /// Half the vehicle's length in m.
    pub(crate) fn half_len(&self) -> f64 {
        self.half_len
    }

    /// The ID of the link the vehicle is currently travelling on.
    pub(crate) fn link_id(&self) -> Option<LinkId> {
        self.route.get(0).map(|el| el.link)
    }

    /// The longitudinal position of the centre of the vehicle in m.
    pub(crate) fn pos_mid(&self) -> f64 {
        self.pos
    }

    /// The longitudinal position of the rear of the vehicle in m.
    pub(crate) fn pos_rear(&self) -> f64 {
        self.pos - self.half_len
    }

    /// The longitudinal position of the front of the vehicle in m.
    pub(crate) fn pos_front(&self) -> f64 {
        self.pos + self.half_len
    }

    /// The vehicle's velocity in m/s.
    pub(crate) fn vel(&self) -> f64 {
        self.vel
    }

    /// Applies an acceleration to the vehicle so it follows an obstacle.
    pub(crate) fn follow_obstacle(&self, pos: f64, vel: f64) {
        let acc = 0.0; // todo: IDM
        self.acc.set(f64::min(self.acc.get(), acc));
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

    /// Gets the lateral extents of the vehicle
    /// for the purpose of applying a car-following model.
    pub(crate) fn lat_extent(&self) -> Interval<f64> {
        let offset = self.offset();
        Interval::new(
            f64::min(offset, 0.0) - self.half_wid(),
            f64::max(offset, 0.0) + self.half_wid()
        )
    }

    /// Gets the represention of the vehicle as an `Obstacle`.
    pub(crate) fn get_obstacle(&self) -> Obstacle {
        // TODO: cache?
        Obstacle {
            pos: self.pos_rear(),
            lat: self.lat_extent(),
            coords: self.obstacle_coords(),
            vel: self.vel()
        }
    }

    /// Gets the end points of the line segment behind the vehicle
    /// which is used for the car following model.
    fn obstacle_coords(&self) -> [Point2d; 2] {
        let tan = self.world_tan;
        let perp = Vector2d::new(-tan.y, tan.x);
        let rear = self.world_pos - self.half_len * tan;
        self.lat_extent()
            .as_array()
            .map(|lat| rear + lat * perp)
    }

    pub(crate) fn reset_acceleration(&self) {
        self.acc.set(self.max_acc);
    }

    /// Integrates the vehicle's velocity and position
    /// 
    /// # Parameters
    /// * `dt` - The time step in seconds
    pub(crate) fn integrate(&mut self, dt: f64) {
        // Perform the integration
        let vel = f64::max(self.vel + dt * self.acc.get(), 0.0);
        let pos = self.pos + 0.5 * (self.vel + vel) * dt;
        self.vel = vel;
        self.pos = pos;

        // Check for lane change completion
        self.lane_change = self.lane_change.filter(|lc| lc.end_pos > pos);
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
                return true;
            }
        }
        false
    }
}