//! A vehicle's course across its lanes: the quintic its front axle follows
//! across a lane change, how sharp one may be, and how far one reaches. Part of
//! the core: outside a lane change, vehicles follow their lanes' centrelines.

use crate::network::geometry::LanePos;
use crate::network::{ALIGNED, LaneId, Side, SimNetwork};
use crate::vehicle::Vehicle;

/// The tightest bend a lane change is planned with, as a radius, in meters, at
/// [`CHANGE_SPEED`] and above: so the shortest change across a 3.5 m lane is
/// about 12 m. Nothing else limits how sharply a vehicle turns: its front axle
/// follows its course, however tight.
pub const CHANGE_RADIUS: f64 = 7.0;
/// The tightest bend a lane change is planned with from rest, as a radius, in
/// meters: about a car's at full lock, as drivers steer hard at walking pace,
/// so the shortest change across a 3.5 m lane is about 9 m, and a car stopped
/// a few metres behind another can still pull out around it.
pub const REST_CHANGE_RADIUS: f64 = 4.0;
/// The speed by which a lane change's tightest bend has eased from
/// [`REST_CHANGE_RADIUS`] to [`CHANGE_RADIUS`], in m/s.
const CHANGE_SPEED: f64 = 2.5;

/// How long a lane change takes, in seconds. Peak lateral acceleration is about
/// `5.8 · offset / T²`: about 1.3 m/s² across a 3.5 m lane.
const LANE_CHANGE_TIME: f64 = 4.0;

/// A lane change in progress. The front axle's offset from the new lane's
/// centerline follows a quintic in `u = s − s0`, from the old path's offset and
/// slope at the start to zero offset and slope at `u = length`, with no
/// curvature of its own at either end, so steering eases in and out rather than
/// starting with a jolt, as path planners for automated vehicles do. It starts
/// where the front axle is, so its course doesn't jump.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct LaneChange {
    /// Where the change started, as `s` along the vehicle's current lane.
    pub s0: f64,
    /// Distance over which the offset returns to zero, in meters.
    pub length: f64,
    /// Lateral offset at the start, in meters, positive to the left.
    pub offset: f64,
    /// Rate of change of the offset at the start, dt/ds.
    pub slope: f64,
}

impl LaneChange {
    /// A change starting at `s0`, `offset` from the new lane's centerline with
    /// `slope`, by a vehicle at `speed`.
    pub fn new(s0: f64, offset: f64, slope: f64, speed: f64) -> Self {
        Self { s0, length: change_length(speed, offset), offset, slope }
    }

    /// The offset at `s`. With `x = (s − s0) / length`, clamped to [0, 1]:
    /// `offset · (1 − 10x³ + 15x⁴ − 6x⁵) + length · slope · (x − 6x³ + 8x⁴ − 3x⁵)`,
    /// the quintic Hermite basis for a start's value and slope.
    pub fn offset_at(&self, s: f64) -> f64 {
        let x = ((s - self.s0) / self.length).clamp(0.0, 1.0);
        let (x3, x4, x5) = (x * x * x, x * x * x * x, x * x * x * x * x);
        self.offset * (1.0 - 10.0 * x3 + 15.0 * x4 - 6.0 * x5)
            + self.length * self.slope * (x - 6.0 * x3 + 8.0 * x4 - 3.0 * x5)
    }

    /// The target offset's rate of change, dt/ds, at `s`: the derivative of
    /// [`LaneChange::offset_at`], zero outside the change.
    pub fn slope_at(&self, s: f64) -> f64 {
        let x = (s - self.s0) / self.length;
        if !(0.0..1.0).contains(&x) {
            return 0.0;
        }
        let (x2, x3, x4) = (x * x, x * x * x, x * x * x * x);
        self.offset * (-30.0 * x2 + 60.0 * x3 - 30.0 * x4) / self.length
            + self.slope * (1.0 - 18.0 * x2 + 32.0 * x3 - 15.0 * x4)
    }

    /// Where the change is complete, as `s` along the vehicle's current lane.
    pub fn end(&self) -> f64 {
        self.s0 + self.length
    }
}

/// How far a lane change across `offset`, started at `speed`, runs, in meters:
/// [`LANE_CHANGE_TIME`] at its speed, or, slower, the shortest it may be (see
/// [`min_change_length`]).
pub fn change_length(speed: f64, offset: f64) -> f64 {
    (speed * LANE_CHANGE_TIME).max(min_change_length(offset, speed))
}

/// The shortest lane change across `offset`, started at `speed`, in meters: the
/// quintic's sharpest curvature, about `5.8 · offset / length²` a fifth of the
/// way from each end, rounded up to 6, bending no tighter than the radius
/// eased from [`REST_CHANGE_RADIUS`] at rest to [`CHANGE_RADIUS`] by
/// [`CHANGE_SPEED`]. From about 9 m to about 12 m across a 3.5 m lane. The
/// slope a change starts with, usually small, is left out.
pub fn min_change_length(offset: f64, speed: f64) -> f64 {
    let ease = (speed / CHANGE_SPEED).clamp(0.0, 1.0);
    let radius = REST_CHANGE_RADIUS + ease * (CHANGE_RADIUS - REST_CHANGE_RADIUS);
    (6.0 * offset.abs() * radius).sqrt()
}

/// How far a vehicle's rear axle must go to be halfway through a lane change
/// started now: the change starts at its front axle, a wheelbase ahead, and
/// runs its length.
pub(crate) fn halfway(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let width = lane_width(network, vehicle);
    vehicle.spec.wheelbase() + 0.5 * change_length(vehicle.speed, width)
}

/// As [`halfway`], for a change started from rest: as sharp as it can steer.
/// The room a vehicle that wants to change lanes keeps so it isn't boxed in:
/// at speed, its leader's stopping distance covers it.
pub(crate) fn halfway_from_rest(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let width = lane_width(network, vehicle);
    vehicle.spec.wheelbase() + 0.5 * min_change_length(width, 0.0)
}

/// How far beyond its rear axle a lane change a vehicle started now reaches
/// before its whole body is in the new lane: the change starts at its front
/// axle, a wheelbase ahead, and the body trails the axle.
pub(crate) fn change_reach(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    vehicle.spec.wheelbase()
        + change_length(vehicle.speed, lane_width(network, vehicle))
        + vehicle.spec.length
}

/// The width of the lane a vehicle follows, where it is: about how far across
/// a change into the lane beside it goes.
pub(crate) fn lane_width(network: &SimNetwork, vehicle: &Vehicle) -> f64 {
    let geometry = network.lane(vehicle.path[0]).geometry();
    geometry.half_width(vehicle.lane_pos, Side::Left)
        + geometry.half_width(vehicle.lane_pos, Side::Right)
}

/// How a vehicle not already changing lanes would change into `target`, a lane
/// beside the one it is on: its rear axle's position on `target`, and the
/// quintic its path would follow onto `target`'s centerline, as
/// [`crate::Simulation::change_lane`] carries it out.
///
/// The quintic starts where the front axle is on the vehicle's current path,
/// with that path's offset and slope relative to `target`, so the target point
/// carries on smoothly instead of jumping.
pub(crate) fn plan_change(
    network: &SimNetwork,
    vehicle: &Vehicle,
    target: LaneId,
) -> (LanePos, LaneChange) {
    let current = network.lane(vehicle.path[0]);
    let lane = network.lane(target);
    // Adjacent lanes start and end together, so the same fraction is a good hint.
    let hint = lane.geometry().lane_pos(
        vehicle.lane_pos.s / current.geometry().length() * lane.geometry().length(),
        0.0,
    );
    let lane_pos = lane.geometry().world_to_lane_near(vehicle.pos, hint);
    // From the front axle, where it is on its path's spline, and the way the
    // spline runs there, so the front axle's course doesn't kink.
    let along = |s: f64| network.smooth_point_along(&vehicle.path, s, |_| 0.0);
    let aim = along(vehicle.front_s);
    let heading = (along(vehicle.front_s + 0.01) - along(vehicle.front_s - 0.01)).normalize();
    // Where on the new lane, and how far across it, its rounded course runs
    // through the front axle: from the polyline's answer, nudged along and
    // across by what is left over, twice, as the corners' rounding is small.
    let start = lane.geometry().world_to_lane_near(aim, lane_pos);
    let on_new = |s: f64, t: f64| network.smooth_point_along(&[target], s, |_| t);
    let (mut s0, mut t0) = (start.s, start.t);
    for _ in 0..2 {
        let tangent = lane.geometry().tangent_at(lane.geometry().lane_pos(s0, t0));
        let miss = aim - on_new(s0, t0);
        s0 += miss.dot(tangent);
        t0 += miss.dot(tangent.perp());
    }
    // How its offset must change to head where the front axle heads: against the
    // new lane's rounded course there.
    let along_new = (on_new(s0 + 0.01, t0) - on_new(s0 - 0.01, t0)).normalize();
    let slope = along_new.perp_dot(heading) / along_new.dot(heading);
    (lane_pos, LaneChange::new(s0, t0, slope, vehicle.speed))
}

/// Whether a change to `side` that a vehicle started now would be over, its
/// whole body in the new lane, before the lanes beside its path on that side
/// run out. They run out where there is no neighbor, where the neighbor stops
/// running beside it, as at a skewed stop line, and at a junction: where a
/// lane of its path or the one beside it is in one, whatever neighbors the
/// network gives them. So no change starts in a junction or runs into one,
/// which would put the vehicle where the junction doesn't know it is, partway
/// across its lanes. SUMO likewise allows no lane changes on a junction's
/// internal lanes.
pub(crate) fn room_to_change(network: &SimNetwork, vehicle: &Vehicle, side: Side) -> bool {
    let needed = change_reach(network, vehicle);
    let mut room = -vehicle.lane_pos.s;
    let in_junction = |lane: LaneId| network.lane(lane).junction().is_some();
    for (k, &lane) in vehicle.path.iter().enumerate() {
        if room >= needed {
            return true;
        }
        let beside = network.lane(lane).adj(side);
        if in_junction(lane) || beside.is_none_or(|n| in_junction(n.lane)) {
            return false;
        }
        let beside = beside.expect("checked");
        // Beside it only from part way along: not yet, on the lane it is on, or
        // not where the change would run onto it, on one after.
        let starts_late = match k {
            0 => vehicle.lane_pos.s < beside.from,
            _ => beside.from > ALIGNED,
        };
        if starts_late {
            return false;
        }
        let length = network.lane(lane).geometry().length();
        room += beside.until;
        if beside.until < length - ALIGNED {
            return room >= needed;
        }
        room += length - beside.until;
    }
    room >= needed
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn slope_is_the_derivative_of_the_offset() {
        let change = LaneChange::new(10.0, 3.5, 0.05, 14.0);
        let h = 1e-6;
        for k in 0..=20 {
            let s = 5.0 + 3.0 * k as f64;
            let diff = (change.offset_at(s + h) - change.offset_at(s - h)) / (2.0 * h);
            assert!(
                (change.slope_at(s) - diff).abs() < 1e-6,
                "at {s}: {} vs {diff}",
                change.slope_at(s)
            );
        }
    }

    #[test]
    fn lane_change_offset_meets_its_ends() {
        let change = LaneChange::new(10.0, 3.5, 0.05, 14.0);
        assert_eq!(change.length, 56.0);
        assert!((change.offset_at(10.0) - 3.5).abs() < 1e-12);
        assert!(change.offset_at(change.end()).abs() < 1e-12);
        assert_eq!(change.offset_at(change.end() + 5.0), 0.0);
        // Slopes by finite differences: the given slope at the start, flat at the end.
        let h = 1e-6;
        let start_slope = (change.offset_at(10.0 + h) - change.offset_at(10.0)) / h;
        let end_slope = (change.offset_at(change.end()) - change.offset_at(change.end() - h)) / h;
        assert!((start_slope - 0.05).abs() < 1e-5);
        assert!(end_slope.abs() < 1e-5);
    }

    #[test]
    fn slow_lane_changes_are_as_short_as_they_may_be() {
        // From rest, about 9 m; moving off, about 12 m.
        let moving = LaneChange::new(0.0, 3.5, 0.0, 2.5);
        assert!((11.5..12.5).contains(&moving.length), "{}", moving.length);
        let change = LaneChange::new(0.0, 3.5, 0.0, 0.0);
        assert_eq!(change.length, min_change_length(3.5, 0.0));
        assert!((8.5..9.5).contains(&change.length), "{}", change.length);
        // At its sharpest, the quintic bends nearly as tightly as it may, no
        // more; at its ends, not at all.
        let h = 1e-3;
        let second = |s: f64| {
            (change.offset_at(s + h) - 2.0 * change.offset_at(s) + change.offset_at(s - h))
                / (h * h)
        };
        let lock = 1.0 / REST_CHANGE_RADIUS;
        let sharpest = (1..100)
            .map(|i| second(change.length * i as f64 / 100.0).abs())
            .fold(0.0, f64::max);
        assert!(sharpest <= lock && sharpest > 0.95 * lock, "{sharpest}");
        assert!(second(2.0 * h).abs() < 1e-2, "{}", second(2.0 * h));
    }
}
