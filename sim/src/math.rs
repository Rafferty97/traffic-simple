use std::f64::consts::{PI, TAU};

/// `sin(x) / x`, by its Taylor series. Only valid for small `x`.
pub fn sinc(x: f64) -> f64 {
    debug_assert!(x.abs() < 0.5, "sinc argument out of range: {x}");
    let x2 = x * x;
    1.0 - x2 / 6.0 * (1.0 - x2 / 20.0 * (1.0 - x2 / 42.0))
}

/// Wraps an angle into [0, 2π).
pub fn wrap_angle(a: f64) -> f64 {
    a.rem_euclid(TAU)
}

/// Signed angle from `b` to `a`, in [-π, π).
pub fn angle_diff(a: f64, b: f64) -> f64 {
    (a - b + PI).rem_euclid(TAU) - PI
}
