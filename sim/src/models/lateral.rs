//! A vehicle's course across its lanes: the quintic its front axle follows
//! across a lane change, and how sharp one may be.

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
