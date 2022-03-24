use crate::{LinkSet, VehicleSet, LinkId, VehicleId};
use crate::link::{Link, LinkAttributes};
use crate::vehicle::{VehicleAttributes, Vehicle};

/// A traffic simulation.
#[derive(Default, Clone)]
pub struct Simulation {
    /// The links in the network.
    links: LinkSet,
    /// The vehicles being simulated.
    vehicles: VehicleSet,
    /// The set of "frozen" vehicles, which will not move.
    frozen_vehs: Vec<VehicleId>,
    /// The current frame of simulation.
    frame: usize
}

impl Simulation {
    /// Creates a new simulation.
    pub fn new() -> Self {
        Default::default()
    }

    /// Adds a link to the network.
    pub fn add_link(&mut self, attributes: &LinkAttributes) -> LinkId {
        self.links.insert_with_key(|id| Link::new(id, attributes))
    }

    /// Adds a vehicle to the simulation.
    pub fn add_vehicle(&mut self, attributes: &VehicleAttributes, link: LinkId) -> VehicleId {
        let vehicle_id = self.vehicles.insert_with_key(|id| {
            let mut vehicle = Vehicle::new(id, attributes);
            vehicle.set_route(&[link], self.frame, false);
            vehicle
        });
        self.links[link].insert_vehicle(vehicle_id);
        vehicle_id
    }

    /// Freezes or unfreezes a vehicle.
    pub fn set_vehicle_frozen(&mut self, vehicle_id: VehicleId, frozen: bool) {
        let idx = self.frozen_vehs.iter().position(|id| *id == vehicle_id);
        match (frozen, idx) {
            (true, None) => { self.frozen_vehs.push(vehicle_id); }
            (false, Some(idx)) => { self.frozen_vehs.remove(idx); },
            _ => {}
        }
    }

    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        self.apply_car_following();
        self.apply_frozen_vehicles();
        self.integrate(dt);
        self.advance_vehicles();
        self.frame += 1;
    }

    /// Gets a reference to the vehicle with the given ID.
    pub fn get_vehicle(&self, vehicle_id: VehicleId) -> &Vehicle {
        &self.vehicles[vehicle_id]
    }

    /// Applies the car following model to all vehicles.
    fn apply_car_following(&mut self) {
        for (_, link) in &self.links {
            link.apply_car_following(&self.links, &self.vehicles);
        }
    }

    /// Applies a large negative acceleration to all frozen vehicles.
    fn apply_frozen_vehicles(&mut self) {
        for vehicle_id in &self.frozen_vehs {
            self.vehicles[*vehicle_id].emergency_stop();
        }
    }

    /// Integrates the velocities and positions of all vehicles,
    /// then resets their accelerations.
    fn integrate(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.integrate(dt);
            vehicle.reset();
        }
    }

    /// Find vehicles that have advanced their link and either move them
    /// to their new link or remove them from the simulation.
    fn advance_vehicles(&mut self) {
        let mut exited = vec![];

        for (vehicle_id, vehicle) in &mut self.vehicles {
            let link_id = vehicle.link_id().unwrap();
            let advanced = vehicle.advance(&self.links, self.frame);

            if advanced {
                self.links[link_id].remove_vehicle(vehicle_id);
                if let Some(link_id) = vehicle.link_id() {
                    // Vehicle is now on a new link
                    self.links[link_id].insert_vehicle(vehicle_id);
                } else {
                    // Vehicle has exited the simulation
                    exited.push(vehicle_id);
                }
            }
        }

        for vehicle_id in exited {
            self.vehicles.remove(vehicle_id);
        }
    }
}