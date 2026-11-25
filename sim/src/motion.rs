//! Moving one vehicle along its course: the front axle along its path, the
//! rear axle trailing it, the quintic across a lane change, and how far ahead
//! its path reaches.

pub mod kinematics;
pub mod lateral;
pub(crate) mod path;

use crate::network::SimNetwork;
use crate::vehicle::Vehicle;

/// Moves `vehicle` on by `dt` seconds at its held acceleration: its speed, its
/// front axle along its course, a lane change's offset included, and its rear
/// axle trailing it, which sets its pose and the curvature of its path. Returns
/// where its front axle now is. Where it is on its lanes is the caller's to
/// work out.
pub(crate) fn drive(network: &SimNetwork, vehicle: &mut Vehicle, dt: f64) -> glam::DVec2 {
    let wheelbase = vehicle.spec.wheelbase();

    // Trapezoidal distance is exact for constant acceleration, unless the
    // vehicle stopped during the step.
    let speed = (vehicle.speed + vehicle.accel * dt).max(0.0);
    let dist = 0.5 * (vehicle.speed + speed) * dt;
    vehicle.speed = speed;
    // The front axle moves along its path, the rear axle trailing it.
    // Changing lanes, its course runs across the lane as well as along
    // it, so it goes less far along for the same distance travelled:
    // by its slope there, exactly on a straight lane.
    vehicle.front_s += match vehicle.lane_change {
        Some(change) => {
            let slope = change.slope_at(vehicle.front_s);
            dist / (1.0 + slope * slope).sqrt()
        }
        None => dist,
    };
    let s = vehicle.front_s;
    let change = vehicle.lane_change;
    let offset = |s: f64| change.map_or(0.0, |change| change.offset_at(s));
    let front = network.smooth_point_along(&vehicle.path, s, offset);
    // On rails, its rear bogie is on the track a wheelbase behind its front,
    // rather than trailing it: a rigid body between its bogies, as a single
    // tram's is.
    let (rear, dir) = match vehicle.spec.rail {
        true => {
            let rear = network.smooth_point_along(&vehicle.path, s - wheelbase, offset);
            (rear, (front - rear).normalize_or(vehicle.dir))
        }
        false => kinematics::trail(vehicle.pos, vehicle.dir, front, wheelbase),
    };
    // The curvature of its path, for the inspector: the turn, small, over
    // the distance moved.
    if dist > 0.0 {
        vehicle.curvature = vehicle.dir.perp_dot(dir) / dist;
    }
    vehicle.pos = rear;
    vehicle.dir = dir;
    front
}
