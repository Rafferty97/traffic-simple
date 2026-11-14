use arrayvec::ArrayVec;
use glam::DVec2;

use crate::lane::LanePos;
use crate::models::lateral::LaneChange;
use crate::network::{LaneId, Side};

/// How many lanes a vehicle's path holds: its current lane and the next few.
pub const PATH_LEN: usize = 4;

slotmap::new_key_type! {
    /// Identifies a vehicle in the simulation.
    pub struct VehicleId;
}

/// A vehicle's static attributes. Lengths in meters, speeds in m/s,
/// accelerations in m/s².
#[derive(Clone, Copy, Debug)]
pub struct VehicleSpec {
    pub length: f64,
    pub width: f64,
    /// The fastest the driver will go; the lane's speed limit may cap it further.
    pub max_speed: f64,
    pub max_accel: f64,
    /// Comfortable deceleration, positive.
    pub comfy_decel: f64,
    /// Desired time gap to the leader, in seconds (IDM's T).
    pub time_headway: f64,
    /// Bumper-to-bumper gap kept when stopped (IDM's s0).
    pub min_gap: f64,
    /// Distance from the rear bumper to the rear axle.
    pub rear_axle: f64,
    /// Distance from the rear bumper to the front axle.
    pub front_axle: f64,
}

impl VehicleSpec {
    /// Distance between the axles.
    pub fn wheelbase(&self) -> f64 {
        self.front_axle - self.rear_axle
    }
}

/// Where a vehicle is inserted.
#[derive(Clone, Copy, Debug)]
pub struct Placement {
    pub lane: LaneId,
    /// Position of the rear axle along the lane's centerline, as in [`Vehicle::lane_pos`].
    pub s: f64,
    pub speed: f64,
}

/// A vehicle in the simulation.
///
/// The pose (rear-axle position and heading) is the source of truth; the lane
/// position is derived from it.
#[derive(Clone, Debug)]
pub struct Vehicle {
    pub(crate) spec: VehicleSpec,
    /// World position of the midpoint of the rear axle.
    pub(crate) pos: DVec2,
    /// Heading in radians, counterclockwise from +x, in [0, 2π).
    pub(crate) heading: f64,
    pub(crate) speed: f64,
    /// Held between longitudinal updates.
    pub(crate) accel: f64,
    /// Curvature of the rear axle's path, in 1/m, positive to the left.
    pub(crate) curvature: f64,
    /// The lane the vehicle is following, then the lanes it will take next.
    pub(crate) path: ArrayVec<LaneId, PATH_LEN>,
    /// The rear axle's position on `path[0]`.
    pub(crate) lane_pos: LanePos,
    /// After a lane change, the side its old lane is on, until the vehicle has
    /// left the old lane. The old lanes are the neighbors of `path` on this side.
    pub(crate) old_side: Option<Side>,
    /// The lane change in progress, if any.
    pub(crate) lane_change: Option<LaneChange>,
}

impl Vehicle {
    pub fn spec(&self) -> &VehicleSpec {
        &self.spec
    }

    /// World position of the midpoint of the rear axle.
    pub fn pos(&self) -> DVec2 {
        self.pos
    }

    /// Heading in radians, counterclockwise from +x.
    pub fn heading(&self) -> f64 {
        self.heading
    }

    pub fn speed(&self) -> f64 {
        self.speed
    }

    pub fn accel(&self) -> f64 {
        self.accel
    }

    /// Curvature of the rear axle's path, in 1/m, positive to the left.
    pub fn curvature(&self) -> f64 {
        self.curvature
    }

    /// The lane the vehicle is following.
    pub fn lane(&self) -> LaneId {
        self.path[0]
    }

    /// The lane the vehicle is following, then the lanes it will take next.
    pub fn path(&self) -> &[LaneId] {
        &self.path
    }

    /// The rear axle's position on [`Vehicle::lane`].
    pub fn lane_pos(&self) -> LanePos {
        self.lane_pos
    }

    /// After a lane change, the side its old lane is on, until it has left the old lane.
    pub fn old_side(&self) -> Option<Side> {
        self.old_side
    }

    /// The lane change in progress, if any.
    pub fn lane_change(&self) -> Option<LaneChange> {
        self.lane_change
    }

    /// The center of the body.
    pub fn center(&self) -> DVec2 {
        let offset = 0.5 * self.spec.length - self.spec.rear_axle;
        self.pos + offset * DVec2::from_angle(self.heading)
    }

    /// Distance from `point` to the body, or zero if it is inside.
    pub fn distance_to(&self, point: DVec2) -> f64 {
        let local = DVec2::from_angle(-self.heading).rotate(point - self.center());
        let half = 0.5 * DVec2::new(self.spec.length, self.spec.width);
        (local.abs() - half).max(DVec2::ZERO).length()
    }

    /// The body's corners: rear left, rear right, front right, front left.
    pub fn corners(&self) -> [DVec2; 4] {
        let dir = DVec2::from_angle(self.heading);
        let left = 0.5 * self.spec.width * dir.perp();
        let rear = self.pos - self.spec.rear_axle * dir;
        let front = rear + self.spec.length * dir;
        [rear + left, rear - left, front - left, front + left]
    }
}
