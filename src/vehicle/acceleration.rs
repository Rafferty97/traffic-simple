use std::cell::Cell;

/// The minimum gap to maintain between vehicles in m.
const MIN_GAP: f64 = 2.0; // m

/// The maximum deceleration of all vehicles in ms<sup>-2</sup>.
const MAX_DECEL: f64 = -6.0; // m/s^2

/// The acceleration model of a vehicle.
#[derive(Clone, Debug)]
pub struct AccelerationModel {
    headway: f64,
    max_acc: f64,
    comf_dec: f64,
    vel_adj: f64,
    acc: Cell<f64>,
}

/// The parameters of the acceleration model.
pub struct ModelParams {
    /// The desired gap between this and the vehicle ahead in seconds.
    pub time_headway: f64,
    /// The vehicle's maximum acceleration in m/s<sup>2</sup>.
    pub max_acceleration: f64,
    /// The comfortable decelleration in m/s<sup>2</sup>.
    pub comf_deceleration: f64,
}

impl AccelerationModel {
    /// Creates a new acceleration model.
    pub fn new(params: &ModelParams) -> Self {
        AccelerationModel {
            headway: params.time_headway,
            max_acc: params.max_acceleration,
            comf_dec: params.comf_deceleration,
            vel_adj: 1.0,
            acc: Cell::new(params.max_acceleration),
        }
    }

    /// Set the desired velocity adjustment factor.
    pub fn set_velocity_adjust(&mut self, factor: f64) {
        self.vel_adj = factor;
    }

    /// Resets the acceleration model. Use at the start of an update.
    pub fn reset(&self) {
        self.acc.set(self.max_acc);
    }

    /// Gets the current acceleration of the vehicle.
    pub fn acc(&self) -> f64 {
        f64::max(self.acc.get(), MAX_DECEL)
    }

    /// Applies the maximum deceleration to the vehicle.
    pub fn emergency_stop(&self) {
        self.acc.set(MAX_DECEL);
    }

    /// Calculates the time it would take the vehicle to reach the given `pos`
    /// if it maximally accelerated.
    pub fn min_reach_time(&self, vel: f64, dist: f64, max_vel: f64) -> f64 {
        let discr = 2.0 * self.max_acc * dist + vel.powi(2);
        if discr < max_vel.powi(2) {
            (discr.sqrt() - vel) / self.max_acc
        } else {
            let t = (max_vel - vel) / self.max_acc; // 5
            let d = 0.5 * (vel + max_vel) * t;
            t + (dist - d) / max_vel
        }
    }

    /// Calculates the acceleration needed to maintain the speed limit.
    /// # Arguments
    /// * `vel` - The velocity of the simulated vehicle (m/s).
    /// * `speed_limit` - The current speed limit (m/s).
    pub fn apply_current_speed_limit(&self, vel: f64, speed_limit: f64) {
        let speed_limit = self.vel_adj * speed_limit;
        let max_acc = self.max_acc;
        let this_acc = max_acc * (1. - (vel / speed_limit).powi(4));
        self.acc.set(f64::min(self.acc.get(), this_acc));
    }

    /// Calculates the acceleration needed to comfortably decelerate to
    /// the speed limit of the next link.
    /// # Arguments
    /// * `vel` - The velocity of the simulated vehicle (m/s).
    /// * `speed_limit` - The current speed limit (m/s).
    /// * `distance` - The distance to the next link (m).
    pub fn apply_speed_limit(&self, vel: f64, speed_limit: f64, distance: f64) {
        if distance <= 0.0 {
            return self.apply_current_speed_limit(vel, speed_limit);
        }

        let speed_limit = self.vel_adj * speed_limit;
        let comf_acc = -self.comf_dec;
        let this_acc = (speed_limit.powi(2) - vel.powi(2)) / (2. * distance);
        if this_acc <= comf_acc {
            let this_acc = f64::max(2.0 * comf_acc, this_acc);
            self.acc.set(f64::min(self.acc.get(), this_acc));
        }
    }

    /// Calculates the acceleration needed to stop before a stop line.
    ///
    /// # Arguments
    /// * `net_dist` - The distance between this vehicle and the stop line.
    /// * `my_vel` - The velocity of the simulated vehicle (m/s).
    pub fn stop_at_line(&self, net_dist: f64, my_vel: f64) {
        let acc = self.idm(net_dist, my_vel, 0.0);
        self.acc.set(f64::min(self.acc.get(), acc));
    }

    /// Calculates the acceleration needed to follow the vehicle ahead.
    ///
    /// # Arguments
    /// * `net_dist` - The distance between this vehicle and the vehicle ahead in metres.
    /// * `my_vel` - The velocity of the simulated vehicle (m/s).
    /// * `their_vel` - The vehicle ahead's velocity (m/s).
    pub fn follow_vehicle(&self, net_dist: f64, my_vel: f64, their_vel: f64) {
        let acc = self.idm(net_dist, my_vel, their_vel);
        self.acc.set(f64::min(self.acc.get(), acc));
    }

    /// Calculates the comfortable break distance of the vehicle.
    pub fn stopping_distance(&self, my_vel: f64) -> f64 {
        let min_dist = 2.5; // m
        let t = my_vel / self.comf_dec;
        0.5 * my_vel * t + min_dist
    }

    /// Computes an acceleration using the intelligent driver model.
    fn idm(&self, net_dist: f64, my_vel: f64, their_vel: f64) -> f64 {
        let comf_dec = self.comf_dec; // m.s^-2
        let max_acc = self.max_acc; // m.s^-2

        if net_dist <= MIN_GAP {
            -10. * max_acc
        } else {
            let appr = my_vel - their_vel;
            let factor = 1. / (2. * (max_acc * comf_dec).sqrt());
            let ss = MIN_GAP + (my_vel * self.headway) + (my_vel * appr * factor);
            let term = ss / net_dist;
            max_acc * (1. - (term * term))
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn min_reach_time() {
        let acc = AccelerationModel::new(&ModelParams {
            time_headway: 1.5,
            max_acceleration: 2.0,
            comf_deceleration: 2.0,
        });

        assert_approx_eq!(acc.min_reach_time(0.0, 25.0, 50.0), 5.0);
        assert_approx_eq!(acc.min_reach_time(0.0, 25.0, 10.0), 5.0);
        assert_approx_eq!(acc.min_reach_time(0.0, 25.0, 9.0), 5.027777777777);

        assert_approx_eq!(acc.min_reach_time(5.0, 50.0, 50.0), 5.0);
        assert_approx_eq!(acc.min_reach_time(5.0, 50.0, 15.0), 5.0);
        assert_approx_eq!(acc.min_reach_time(5.0, 50.0, 14.0), 5.01785714285);
    }
}
