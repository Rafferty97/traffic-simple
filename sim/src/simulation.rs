use slotmap::SlotMap;

use crate::debug::{DebugEvent, DebugSink, NoopSink};
use crate::kinematics;
use crate::math::wrap_angle;
use crate::models::{acceleration, lateral};
use crate::network::SimNetwork;
use crate::vehicle::{Placement, Vehicle, VehicleId, VehicleSpec};

/// Longest substep in seconds. Longer frames are split.
const MAX_SUBSTEP: f64 = 1.0 / 60.0;
/// Period of the longitudinal model update in seconds.
const ACCEL_PERIOD: f64 = 1.0 / 20.0;

// The acceleration clock fires at most once per substep.
const _: () = assert!(MAX_SUBSTEP <= ACCEL_PERIOD);

/// The traffic simulation.
pub struct Simulation<D: DebugSink = NoopSink> {
    network: SimNetwork,
    vehicles: SlotMap<VehicleId, Vehicle>,
    /// Time since the last longitudinal model update, in seconds.
    accel_clock: f64,
    sink: D,
}

impl<D: DebugSink> Simulation<D> {
    pub fn new(network: SimNetwork, sink: D) -> Self {
        Self {
            network,
            vehicles: SlotMap::with_key(),
            accel_clock: 0.0,
            sink,
        }
    }

    pub fn network(&self) -> &SimNetwork {
        &self.network
    }

    pub fn sink(&self) -> &D {
        &self.sink
    }

    pub fn sink_mut(&mut self) -> &mut D {
        &mut self.sink
    }

    /// Inserts a vehicle on the centreline of a lane, facing along it.
    ///
    /// It starts with zero acceleration, which it holds until the next longitudinal model update.
    pub fn insert(&mut self, spec: VehicleSpec, at: Placement) -> VehicleId {
        let lane = self.network.lane(at.lane);
        let lane_pos = lane.lane_pos(at.s, 0.0);
        let vehicle = Vehicle {
            spec,
            pos: lane.lane_to_world(lane_pos),
            heading: wrap_angle(lane.tangent_at(lane_pos).to_angle()),
            speed: at.speed,
            accel: 0.0,
            steer: 0.0,
            lane: at.lane,
            lane_pos,
        };
        self.vehicles.insert(vehicle)
    }

    pub fn remove(&mut self, id: VehicleId) -> Option<Vehicle> {
        self.vehicles.remove(id)
    }

    pub fn get(&self, id: VehicleId) -> Option<&Vehicle> {
        self.vehicles.get(id)
    }

    pub fn vehicles(&self) -> impl Iterator<Item = (VehicleId, &Vehicle)> {
        self.vehicles.iter()
    }

    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        let n = (dt / MAX_SUBSTEP).ceil().max(1.0);
        let h = dt / n;
        for _ in 0..n as u32 {
            self.substep(h);
        }
    }

    fn substep(&mut self, dt: f64) {
        self.accel_clock += dt;
        if self.accel_clock >= ACCEL_PERIOD {
            self.accel_clock -= ACCEL_PERIOD;
            self.update_accels();
        }
        self.update_motion(dt);
        self.remove_finished();
    }

    /// Longitudinal model: sets each vehicle's held acceleration.
    fn update_accels(&mut self) {
        for vehicle in self.vehicles.values_mut() {
            let lane = self.network.lane(vehicle.lane);
            let desired_speed = vehicle.spec.max_speed.min(lane.speed_limit());
            vehicle.accel =
                acceleration::free_road_accel(vehicle.speed, desired_speed, vehicle.spec.max_accel);
        }
    }

    /// Path tracking and integration: steers, then moves each vehicle.
    fn update_motion(&mut self, dt: f64) {
        for (id, vehicle) in self.vehicles.iter_mut() {
            let lane = self.network.lane(vehicle.lane);
            let wheelbase = vehicle.spec.wheelbase();

            let ahead = vehicle.lane_pos.s + lateral::lookahead_distance(vehicle.speed);
            let target = lane.lane_to_world(lane.lane_pos(ahead, 0.0));
            vehicle.steer = lateral::steer_towards(vehicle.pos, vehicle.heading, wheelbase, target);
            self.sink.record(|| DebugEvent::Lookahead { vehicle: id, point: target });

            // Trapezoidal distance is exact for constant acceleration, unless the
            // vehicle stopped during the substep.
            let speed = (vehicle.speed + vehicle.accel * dt).max(0.0);
            let dist = 0.5 * (vehicle.speed + speed) * dt;
            vehicle.speed = speed;
            (vehicle.pos, vehicle.heading) =
                kinematics::advance(vehicle.pos, vehicle.heading, vehicle.steer, wheelbase, dist);
            vehicle.lane_pos = lane.world_to_lane_near(vehicle.pos, vehicle.lane_pos);
        }
    }

    /// Removes vehicles whose rear corners have both passed the end of their lane.
    ///
    /// Lanes have no successors yet, so the end of every lane is a dead end.
    fn remove_finished(&mut self) {
        let network = &self.network;
        self.vehicles.retain(|_, vehicle| {
            let lane = network.lane(vehicle.lane);
            let [rear_left, rear_right, ..] = vehicle.corners();
            let past_end = |p| lane.world_to_lane_near(p, vehicle.lane_pos).s >= lane.length();
            !(past_end(rear_left) && past_end(rear_right))
        });
    }
}
