use std::cell::Cell;

/// The minimum gap to maintain between vehicles.
const MIN_GAP: f64 = 2.5; // m

/// The acceleration model of a vehicle.
#[derive(Clone)]
pub struct AccelerationModel {
    max_acc: f64,
    comf_dec: f64,
    acc: Cell<f64>
}

/// The parameters of the acceleration model.
pub struct ModelParams {
    /// The vehicle's maximum acceleration in m/s<sup>2</sup>.
    pub max_acceleration: f64,
    /// The comfortable decelleration in m/s<sup>2</sup>.
    pub comf_deceleration: f64
}

impl AccelerationModel {
    /// Creates a new acceleration model.
    pub fn new(params: &ModelParams) -> Self {
        AccelerationModel {
            max_acc: params.max_acceleration,
            comf_dec: params.comf_deceleration,
            acc: Cell::new(params.max_acceleration)
        }
    }

    /// Resets the acceleration model. Use at the start of an update.
    pub fn reset(&self) {
        self.acc.set(self.max_acc);
    }

    /// Gets the current acceleration of the vehicle.
    pub fn acc(&self) -> f64 {
        self.acc.get()
    }
    

    /// Calculates the acceleration needed to maintain the speed limit.
    /// # Arguments
    /// * `vel` - The velocity of the simulated vehicle (m/s).
    /// * `speed_limit` - The current speed limit (m/s).
    pub fn apply_current_speed_limit(&self, vel: f64, speed_limit: f64) {
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
        self.follow_vehicle(net_dist, my_vel, 0.0);
    }

    /// Calculates the acceleration needed to follow the vehicle ahead.
    /// 
    /// # Arguments
    /// * `net_dist` - The distance between this vehicle and the vehicle ahead in metres.
    /// * `my_vel` - The velocity of the simulated vehicle (m/s).
    /// * `their_vel` - The vehicle ahead's velocity (m/s).
    pub fn follow_vehicle(&self, net_dist: f64, my_vel: f64, their_vel: f64) {
        let headway = 1.5; // s
        let comf_dec = self.comf_dec; // m.s^-2
        let max_acc = self.max_acc; // m.s^-2

        let acc = if net_dist <= MIN_GAP {
            -10. * max_acc
        } else {
            let appr = my_vel - their_vel;
            let factor = 1. / (2. * (max_acc * comf_dec).sqrt());
            let ss = MIN_GAP + (my_vel * headway) + (my_vel * appr * factor);
            let term = ss / net_dist;
            max_acc * (1. - (term * term))
        };

        self.acc.set(f64::min(self.acc.get(), acc));
    }

    /// Calculates whether or not the vehicle is able to come to a stop within
    /// a distance of `net_dist` metres without excessive breaking.
    pub fn can_stop(&self, net_dist: f64, my_vel: f64) -> bool {
        let min_dist = 2.5; // m
        let t = my_vel / self.comf_dec;
        let comf_dist = 0.5 * my_vel * t + min_dist;
        net_dist >= comf_dist
    }
}