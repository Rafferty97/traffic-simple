use std::f64::consts::TAU;

/// Wraps an angle into [0, 2π).
pub fn wrap_angle(a: f64) -> f64 {
    a.rem_euclid(TAU)
}
