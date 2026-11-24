use glam::DVec2;

use crate::models::routing::Route;
use crate::motion::lateral::LaneChange;
use crate::motion::path::PathReach;
use crate::network::geometry::LanePos;
use crate::network::{LaneId, Side};

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

/// A vehicle's static attributes. Lengths in meters, speeds in m/s,
/// accelerations in m/s².
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct VehicleSpec {
    pub length: f64,
    pub width: f64,
    /// The fastest the vehicle will go, whatever the speed limit.
    pub max_speed: f64,
    /// How fast the driver likes to go relative to the speed limit: 1 is exactly
    /// the limit. SUMO's `speedFactor`.
    pub speed_factor: f64,
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
    /// The hardest the driver will brake to let another vehicle in ahead, in
    /// m/s²: 0 never yields to one indicating into its lane.
    pub courtesy: f64,
    /// How much the driver weighs other drivers' gains and losses against its own
    /// when changing lanes: MOBIL's politeness.
    pub politeness: f64,
    /// How readily the driver changes lanes, from 0 to 1: it intends a change
    /// for a smaller gain, and accepts harder braking, for itself and the
    /// vehicle it cuts in front of, before it grows frustrated. As SUMO's
    /// `lcAssertive` and `lcSpeedGain`. A few assertive drivers pull out into a
    /// faster lane and slow it, so a fast lane doesn't form.
    #[cfg_attr(feature = "serde", serde(default))]
    pub assertiveness: f64,
    /// How much its acceleration wanders on the open road, in m/s², the
    /// standard deviation of a slowly varying noise added to its free-road
    /// term: drivers don't hold a speed exactly, as in Treiber and Kesting's
    /// IDM with acceleration noise (2017). 0 for none. Following stays exact.
    #[cfg_attr(feature = "serde", serde(default))]
    pub accel_noise: f64,
    /// How far short of a line the driver stops, nose to line, in meters, on
    /// average: each time it heads for a barrier, it draws how far, uniformly
    /// within [`VehicleSpec::line_gap_spread`] of this, so a row of vehicles at
    /// a line doesn't stand in perfect step.
    #[cfg_attr(feature = "serde", serde(default = "default_line_gap"))]
    pub line_gap: f64,
    /// How far either side of [`VehicleSpec::line_gap`] its gap at a line may
    /// fall, in meters.
    #[cfg_attr(feature = "serde", serde(default))]
    pub line_gap_spread: f64,
}

#[cfg(feature = "serde")]
fn default_line_gap() -> f64 {
    VehicleSpec::passenger_car().line_gap
}

impl VehicleSpec {
    /// A passenger car with textbook parameters: IDM's as calibrated with LMRS
    /// (Schakel, Knoop and van Arem, 2012), as OpenTrafficSim uses them, a
    /// = 1.25 m/s², b = 2.09 m/s², s0 = 3 m and T = 1.2 s, and MOBIL's
    /// politeness of 0.2 as Treiber's simulator uses it. Its speed factor,
    /// courtesy, politeness and assertiveness are the caller's to vary.
    pub const fn passenger_car() -> Self {
        Self {
            length: 4.6,
            width: 1.9,
            max_speed: 50.0,
            speed_factor: 1.0,
            max_accel: 1.25,
            comfy_decel: 2.09,
            time_headway: 1.2,
            min_gap: 3.0,
            rear_axle: 1.0,
            front_axle: 3.7,
            courtesy: 1.0,
            politeness: 0.2,
            assertiveness: 0.0,
            accel_noise: 0.2,
            line_gap: 1.0,
            line_gap_spread: 0.5,
        }
    }

    /// The speed the driver wants on a lane with the given limit.
    pub fn desired_speed(&self, speed_limit: f64) -> f64 {
        (speed_limit * self.speed_factor).min(self.max_speed)
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
    /// Seeds the driver's own generator, for its acceleration noise and its
    /// gaps to lines, mixed with where it starts, so drivers placed alike still
    /// differ.
    pub seed: u64,
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
    #[cfg_attr(feature = "serde", serde(default))]
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
