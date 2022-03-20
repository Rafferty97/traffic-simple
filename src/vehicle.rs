use std::cell::Cell;
use crate::util::Interval;
use crate::{VehicleId, LinkId};
use crate::math::{CubicFn, Point2d, Vector2d};

/// A simulated vehicle.
pub struct Vehicle {
    pub(crate) id: VehicleId,
    half_wid: f64,
    half_len: f64,
    pos: f64,
    vel: f64,
    acc: Cell<f64>,
    route: Vec<RouteElement>,
    lane_change: Option<LaneChange>,
    world_pos: Point2d,
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

    /// Gets the end points of the line segment behind the vehicle
    /// which is used for the car following model.
    pub(crate) fn obstacle_coords(&self) -> [Point2d; 2] {
        let tan = self.world_tan;
        let perp = Vector2d::new(-tan.y, tan.x);
        let rear = self.world_pos - self.half_len * tan;
        self.lat_extent()
            .as_array()
            .map(|lat| rear + lat * perp)
    }
}