use crate::obstacle::Obstacle;
use crate::util::Interval;
use crate::{VehicleId, LinkId, LinkSet};
use crate::math::{CubicFn, Point2d, Vector2d};

use self::acceleration::AccelerationModel;

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
    /// The vehicle's route, including the link it's currently on.
    route: Vec<RouteElement>,
    /// Whether the vehicle can exit at the end of its route.
    can_exit: bool,
    /// The in-progress lane change, if there is one.
    lane_change: Option<LaneChange>,
    /// The world space coordinates of the centre of the vehicle.
    world_pos: Point2d,
    /// A world space vector tangent to the vehicle's heading.
    world_tan: Vector2d
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
    pub comf_dec: f64
}

/// A link along a vehicle's route.
#[derive(Clone, Copy, Debug)]
struct RouteElement {
    link: LinkId,
    entered_at: Option<usize>
}

/// Represents an in-progress lane change.
#[derive(Clone, Copy)]
pub struct LaneChange {
    /// The longitudinal position at which the lane change is complete.
    pub end_pos: f64,
    /// The vehicle's lateral offset during the lane change.
    pub offset: CubicFn
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
                comf_deceleration: attributes.comf_dec
            }),
            pos: 0.0,
            vel: 0.0,
            route: vec![],
            can_exit: true,
            lane_change: None,
            world_pos: Point2d::new(0.0, 0.0),
            world_tan: Vector2d::new(0.0, 0.0)
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
        self.world_tan
    }

    /// The vehicle's velocity in m/s.
    pub fn vel(&self) -> f64 {
        self.vel
    }

    /// Checks whether the vehicle's route coincides with the given links.
    pub(crate) fn on_route(&self, links: &[LinkId]) -> bool {
        println!("{:?}", &self.route);
        self.route.iter()
            .skip(1)
            .take(links.len())
            .map(|el| &el.link)
            .eq(links.iter())
    }

    /// Applies an acceleration to the vehicle so it follows an obstacle.
    pub(crate) fn follow_obstacle(&self, pos: f64, vel: f64) {
        let net_dist = pos - self.pos_front();
        self.acc.follow_vehicle(net_dist, self.vel, vel);
    }

    /// Applies a current speed limit to the vehicle.
    pub(crate) fn apply_current_speed_limit(&self, speed_limit: f64) {
        self.acc.apply_current_speed_limit(self.vel, speed_limit);
    }

    /// Applies an upcoming speed limit to the vehicle.
    pub(crate) fn apply_speed_limit(&self, speed_limit: f64, pos: f64) {
        self.acc.apply_speed_limit(self.vel, speed_limit, pos - self.pos);
    }

    /// Applies a maximum deceleration to the vehicle, causing it to stop.
    pub(crate) fn emergency_stop(&self) {
        self.acc.emergency_stop();
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
            f64::max(offset, 0.0) + self.half_width()
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
            .map(|lat| rear + (lat - self.offset()) * perp)
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

    /// Sets the vehicle's position in the network.
    /// This also clears the vehicle's route.
    pub(crate) fn set_location(&mut self, link: LinkId, pos: f64, lane_change: Option<LaneChange>) {
        self.route = vec![RouteElement { link, entered_at: None }];
        self.pos = pos;
        self.lane_change = lane_change;
        self.can_exit = false;
    }

    /// Sets the vehicle's route.
    pub(crate) fn set_route(&mut self, route: &[LinkId], can_exit: bool) {
        self.route.truncate(1);
        self.route.extend(route.iter().map(|link| RouteElement {
            link: *link,
            entered_at: None
        }));
        self.can_exit = can_exit;
    }

    /// Updates the vehicle's world coordinates
    pub(crate) fn update_coords(&mut self, links: &LinkSet) {
        let curve = &links[self.route[0].link].curve();
        (self.world_pos, self.world_tan) = match self.lane_change {
            Some(lc) => {
                let (offset, slope) = lc.offset.y_and_dy(self.pos);
                curve.sample(self.pos, offset, slope)
            },
            None => curve.sample_centre(self.pos)
        };
    }
}

#[cfg(test)]
mod test {
    use slotmap::{KeyData, Key};
    use super::*;

    /// Tests the `on_route` method of `Vehicle`.
    #[test]
    fn on_route() {
        let link1 = KeyData::from_ffi(0).into();
        let link2 = KeyData::from_ffi(1).into();
        let link3 = KeyData::from_ffi(2).into();
        let mut vehicle = Vehicle::new(VehicleId::null(), &VehicleAttributes {
            width: 2.0,
            length: 5.0,
            comf_dec: 2.0,
            max_acc: 2.0
        });
        vehicle.set_location(link3, 0.0, None);
        vehicle.set_route(&[link1, link2], true);

        assert!(vehicle.on_route(&[]));
        assert!(vehicle.on_route(&[link1]));
        assert!(vehicle.on_route(&[link1, link2]));
        assert!(!vehicle.on_route(&[link2]));
        assert!(!vehicle.on_route(&[link1, link2, link3]));
    }
}