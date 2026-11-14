//! The Intelligent Driver Model (Treiber, Hennecke and Helbing, 2000).

/// The acceleration exponent δ.
const DELTA: i32 = 4;

/// IDM acceleration on a free road: `a · (1 − (v / v0)^δ)`.
///
/// The interaction term, `−a · (s* / s)²`, is added once vehicles have leaders.
pub fn free_road_accel(speed: f64, desired_speed: f64, max_accel: f64) -> f64 {
    max_accel * (1.0 - (speed / desired_speed).powi(DELTA))
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn full_accel_from_rest() {
        assert_eq!(free_road_accel(0.0, 14.0, 1.5), 1.5);
    }

    #[test]
    fn none_at_desired_speed() {
        assert_eq!(free_road_accel(14.0, 14.0, 1.5), 0.0);
    }
}
