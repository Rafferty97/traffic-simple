use glam::DVec2;

use crate::class::VehicleClass;
use crate::models::routing::Route;
use crate::motion::lateral::LaneChange;
use crate::motion::path::PathReach;
use crate::network::geometry::LanePos;
use crate::network::{LaneId, Neighbor, Side, SimNetwork};

slotmap::new_key_type! {
    /// Identifies a vehicle in the simulation.
    pub struct VehicleId;
}

impl VehicleId {
    /// Its slot's index: unique among the vehicles in the simulation at once,
    /// though a vehicle inserted later may take it over.
    pub fn index(self) -> u32 {
        use slotmap::Key;
        // The index is the low half; the slot's version, the high.
        self.data().as_ffi() as u32
    }
}

/// A vehicle's static attributes: its body, and the longitudinal model's
/// parameters for its class. The driver's own traits are apart (see
/// [`Driver`](crate::Driver)). Lengths in meters, speeds in m/s,
/// accelerations in m/s².
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VehicleSpec {
    pub length: f64,
    pub width: f64,
    /// The fastest the vehicle will go, whatever the speed limit.
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
    /// Its class, for which lanes it may use.
    #[cfg_attr(feature = "serde", serde(default))]
    pub class: VehicleClass,
    /// Whether it runs on rails: its rear axle, or bogie, kept on its track a
    /// wheelbase behind its front, rather than trailing it; and it never
    /// changes lanes.
    #[cfg_attr(feature = "serde", serde(default))]
    pub rail: bool,
}

impl VehicleSpec {
    /// A passenger car with textbook parameters: IDM's as calibrated with LMRS
    /// (Schakel, Knoop and van Arem, 2012), as OpenTrafficSim uses them, a
    /// = 1.25 m/s², b = 2.09 m/s², s0 = 3 m and T = 1.2 s.
    pub const fn passenger_car() -> Self {
        Self {
            length: 4.6,
            width: 1.9,
            max_speed: 50.0,
            max_accel: 1.25,
            comfy_decel: 2.09,
            time_headway: 1.2,
            min_gap: 3.0,
            rear_axle: 1.0,
            front_axle: 3.7,
            class: VehicleClass::Car,
            rail: false,
        }
    }

    /// A Melbourne Z3-class tram, a single body on rails: 16.64 m long and
    /// 2.67 m wide, its bogie centres 8.5 m apart, so 4.07 m in from each end;
    /// IDM's parameters gentler than a car's, as passengers stand.
    pub const fn tram() -> Self {
        Self {
            length: 16.64,
            width: 2.67,
            max_speed: 20.0,
            max_accel: 1.0,
            comfy_decel: 1.3,
            time_headway: 2.0,
            min_gap: 4.0,
            rear_axle: 4.07,
            front_axle: 12.57,
            class: VehicleClass::Tram,
            rail: true,
        }
    }

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
    /// How it chooses its way at forks.
    pub route: Route,
}

/// A vehicle in the simulation.
///
/// The pose (rear-axle position and heading) is the source of truth; the lane
/// position is derived from it.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Vehicle {
    pub(crate) spec: VehicleSpec,
    /// World position of the midpoint of the rear axle.
    pub(crate) pos: DVec2,
    /// The body's heading, as a unit vector from the rear axle toward the front.
    pub(crate) dir: DVec2,
    pub(crate) speed: f64,
    /// Held between longitudinal updates.
    pub(crate) accel: f64,
    /// Curvature of the rear axle's path, in 1/m, positive to the left.
    pub(crate) curvature: f64,
    /// How far along its path the front axle is, from the start of `path[0]`:
    /// the front axle is placed on the path there, and the rear axle trails it.
    pub(crate) front_s: f64,
    /// The lane the vehicle is following, then the lanes it will take next: as
    /// far ahead as its leader search, path tracking and lane change need to
    /// know where it is going, however short the lanes, unless it leaves the
    /// network sooner.
    pub(crate) path: Vec<LaneId>,
    /// How far `path` reaches, so it is extended only when it might need to be.
    /// Derived, so not saved: worked out afresh on the first step after a
    /// restore. Saved, its infinities wrote as nulls and didn't read back.
    #[cfg_attr(feature = "serde", serde(skip))]
    pub(crate) path_reach: PathReach,
    /// The rear axle's position on `path[0]`.
    pub(crate) lane_pos: LanePos,
    /// After a lane change, the side its old lane is on, until the vehicle has
    /// left the old lane. The old lanes are the neighbors of `path` on this side.
    pub(crate) old_side: Option<Side>,
    /// The lane change in progress, if any.
    pub(crate) lane_change: Option<LaneChange>,
    /// What it means to do about changing lanes: wanting to, or indicating, which
    /// other vehicles see. Only the simulation sets it, carrying out what the
    /// lane-change model decides.
    pub(crate) intent: LaneChangeIntent,
    /// How many of the barriers ahead of its rear axle on its path it has
    /// committed to crossing (see `docs/DESIGN.md`, Barriers and signals).
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) committed: u8,
    /// The step at which it came to rest at the all-way stop line it is heading
    /// for, as the head on its way there: when it arrived, for the order the
    /// junction's lines go in.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) stopped_at: Option<u64>,
    /// The vehicle behind that it holds up most, as of the last longitudinal
    /// update, and how that one would accelerate without it: an output of the
    /// core for the models, such as the lane-change model's courtesy.
    #[cfg_attr(feature = "serde", serde(default))]
    pub(crate) held_up: Option<HeldUp>,
}

/// A vehicle held up by the one ahead of it: that one is the term binding its
/// acceleration. The one ahead can tell what getting out of the way would do
/// for it, without searching back for it.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub(crate) struct HeldUp {
    pub id: VehicleId,
    /// Its acceleration from every other term: what it would be, if the vehicle
    /// ahead were gone and nothing took its place.
    pub without: f64,
}

/// What a vehicle means to do about changing lanes, as the lane-change model
/// last decided.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum LaneChangeIntent {
    /// Nothing: it is content where it is.
    #[default]
    None,
    /// It wants to change to `side`, but isn't indicating: it keeps room behind
    /// its leaders to change from rest (see the longitudinal model's
    /// `kept_room`). With `check`, it means to check the gap that way, so at
    /// the next lane-change update the core measures what the change would do,
    /// judging time headways at that share, for the lane-change model to weigh
    /// (see `query::Prospect`).
    Desire { side: Side, check: Option<f64> },
    /// It is indicating to `side`, since `since`, in simulated seconds, keeping
    /// room as when it wants to. It has a ghost record in the target lane's
    /// occupants until the change is cancelled or carried out. With `check`,
    /// as when it wants to, the core measures what the change would do at the
    /// next lane-change update, judging time headways at that share.
    Indicating { side: Side, since: f64, check: Option<f64> },
}

impl LaneChangeIntent {
    /// The side it is indicating to, if it is.
    pub fn indicating(self) -> Option<Side> {
        match self {
            LaneChangeIntent::Indicating { side, .. } => Some(side),
            _ => None,
        }
    }

    /// Whether it wants to change lanes, indicating or not.
    pub fn wants_change(self) -> bool {
        self != LaneChangeIntent::None
    }
}

impl Vehicle {
    /// Whether it may change lanes into `neighbor`: the network permits it, its
    /// class may use the lane, and it isn't on rails, as nothing on rails ever
    /// changes lanes.
    pub(crate) fn may_change_into(&self, network: &SimNetwork, neighbor: &Neighbor) -> bool {
        neighbor.can_change
            && !self.spec.rail
            && network.lane(neighbor.lane).allows().contains(self.spec.class)
    }
    pub fn spec(&self) -> &VehicleSpec {
        &self.spec
    }

    /// World position of the midpoint of the rear axle.
    pub fn pos(&self) -> DVec2 {
        self.pos
    }

    /// How far it goes braking to a stop comfortably from its speed now.
    pub(crate) fn comfortable_stop(&self) -> f64 {
        self.speed * self.speed / (2.0 * self.spec.comfy_decel)
    }

    /// Heading in radians, counterclockwise from +x, from 0 to 2π.
    pub fn heading(&self) -> f64 {
        self.dir.to_angle().rem_euclid(std::f64::consts::TAU)
    }

    /// The heading as a unit vector.
    pub fn dir(&self) -> DVec2 {
        self.dir
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

    /// How far along its path its front axle is, in meters along
    /// [`Vehicle::lane`], beyond its end if the front axle is on a lane after it.
    pub fn front_s(&self) -> f64 {
        self.front_s
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

    /// How many of the barriers ahead of its rear axle on its path it has
    /// committed to crossing.
    pub fn committed(&self) -> u8 {
        self.committed
    }

    /// The step at which it came to rest at the all-way stop line it is heading
    /// for, if it has.
    pub fn stopped_at(&self) -> Option<u64> {
        self.stopped_at
    }

    /// What the vehicle means to do about changing lanes.
    pub fn intent(&self) -> LaneChangeIntent {
        self.intent
    }

    /// The center of the body.
    pub fn center(&self) -> DVec2 {
        let offset = 0.5 * self.spec.length - self.spec.rear_axle;
        self.pos + offset * self.dir
    }

    /// Distance from `point` to the body, or zero if it is inside.
    pub fn distance_to(&self, point: DVec2) -> f64 {
        // Rotated by minus the heading, into the body's frame.
        let local = DVec2::new(self.dir.x, -self.dir.y).rotate(point - self.center());
        let half = 0.5 * DVec2::new(self.spec.length, self.spec.width);
        (local.abs() - half).max(DVec2::ZERO).length()
    }

    /// The body's corners: rear left, rear right, front right, front left.
    pub fn corners(&self) -> [DVec2; 4] {
        let dir = self.dir;
        let left = 0.5 * self.spec.width * dir.perp();
        let rear = self.pos - self.spec.rear_axle * dir;
        let front = rear + self.spec.length * dir;
        [rear + left, rear - left, front - left, front + left]
    }
}
