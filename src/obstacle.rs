use crate::util::Interval;

/// Represents a vehicle, stop line or other object
/// a vehicle may need to follow or stop before reaching.
#[derive(Clone, Copy)]
pub struct Obstacle {
    /// The longitudinal position of the obstacle in m.
    pub pos: f64,
    /// The lateral extents of the obstacle in m.
    pub lat: Interval<f64>,
    /// The velocity of the obstacle in m/s.
    pub vel: f64
}