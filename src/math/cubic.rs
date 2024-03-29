//! Mathematical functions.

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A cubic function, which is a polynomial of the form ax^3 + bx^2 + cx + d.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CubicFn {
    coeffs: [f64; 4],
    offset: f64,
}

impl CubicFn {
    /// Creates a cubic function which outputs the fixed value `y` for any input.
    pub const fn constant(y: f64) -> Self {
        Self {
            coeffs: [0.0, 0.0, 0.0, y],
            offset: 0.0,
        }
    }

    /// Creates a cubic function which passes through the two specified points,
    /// and which has the specified derivates at those two points.
    pub fn fit(x1: f64, y1: f64, dydx1: f64, x2: f64, y2: f64, dydx2: f64) -> Self {
        let w = x2 - x1;
        let a = 2. * y1 - 2. * y2 + w * dydx1 + w * dydx2;
        let b = -3. * y1 + 3. * y2 - 2. * w * dydx1 - w * dydx2;
        let c = w * dydx1;
        let d = y1;
        Self {
            coeffs: [a * w.powi(-3), b * w.powi(-2), c * w.powi(-1), d],
            offset: -x1,
        }
    }

    /// Returns a copy of this `CubicFn` translated to the left by `amount` units.
    pub fn translate_x(&self, amount: f64) -> CubicFn {
        Self {
            coeffs: self.coeffs,
            offset: self.offset + amount,
        }
    }

    /// Evaluates the function at `x`.
    pub fn y(&self, x: f64) -> f64 {
        self.y_and_dy(x).0
    }

    /// Evaluates the derivative of the function at `x`.
    pub fn dy(&self, x: f64) -> f64 {
        self.y_and_dy(x).1
    }

    /// Evaluates the value of the function, and the derivative of the function, at `x`.
    pub fn y_and_dy(&self, x: f64) -> (f64, f64) {
        let c = &self.coeffs;
        let x = x + self.offset;

        let y = c[0] * x * x * x + c[1] * x * x + c[2] * x + c[3];
        let dy = c[0] * 3. * x * x + c[1] * 2. * x + c[2];

        (y, dy)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use assert_approx_eq::assert_approx_eq;
    use rand::{Rng, SeedableRng};

    #[test]
    pub fn fit_horizontal() {
        let cubic = CubicFn::fit(10., 20., 0.0, 45.0, 5.0, 0.0);
        assert_approx_eq!(cubic.y(10.), 20., 0.01);
        assert_approx_eq!(cubic.dy(10.), 0., 0.01);
        assert_approx_eq!(cubic.y(45.), 5., 0.01);
        assert_approx_eq!(cubic.dy(45.), 0., 0.01);
        assert_approx_eq!(cubic.y(27.5), 12.5, 0.01);
    }

    #[test]
    pub fn fit() {
        let mut rng = rand::rngs::StdRng::from_seed(*b"Vegemite sandwhich is not fun...");
        for _i in 0..100 {
            let x1 = rng.gen_range(-100.0..100.0);
            let x2 = rng.gen_range(-100.0..100.0);
            let y1 = rng.gen_range(-100.0..100.0);
            let y2 = rng.gen_range(-100.0..100.0);
            let dydx1 = rng.gen_range(-10.0..10.0);
            let dydx2 = rng.gen_range(-10.0..10.0);
            let cubic = CubicFn::fit(x1, y1, dydx1, x2, y2, dydx2);

            assert_approx_eq!(cubic.y(x1), y1, 0.01);
            assert_approx_eq!(cubic.dy(x1), dydx1, 0.01);
            assert_approx_eq!(cubic.y(x2), y2, 0.01);
            assert_approx_eq!(cubic.dy(x2), dydx2, 0.01);
        }
    }

    #[test]
    pub fn straight_lines() {
        let mut rng = rand::rngs::StdRng::from_seed(*b"Vegemite sandwhich is not fun...");
        for _i in 0..100 {
            let x1 = rng.gen_range(-100.0..100.0);
            let x2 = rng.gen_range(-100.0..100.0);
            let y1 = rng.gen_range(-100.0..100.0);
            let y2 = rng.gen_range(-100.0..100.0);
            let dydx = (y2 - y1) / (x2 - x1);
            let cubic = CubicFn::fit(x1, y1, dydx, x2, y2, dydx);

            assert_approx_eq!(cubic.y(x1), y1, 0.01);
            assert_approx_eq!(cubic.y(0.5 * (x1 + x2)), 0.5 * (y1 + y2), 0.01);
            assert_approx_eq!(cubic.y(x2), y2, 0.01);
            assert_approx_eq!(cubic.dy(x1), dydx, 0.01);
            assert_approx_eq!(cubic.dy(0.5 * (x1 + x2)), dydx, 0.01);
            assert_approx_eq!(cubic.dy(x2), dydx, 0.01);
        }
    }
}
