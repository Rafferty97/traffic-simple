use crate::{LinkSet, VehicleSet};

pub struct Simulation {
    links: LinkSet,
    vehicles: VehicleSet,
    frame: usize
}

impl Simulation {
    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        self.apply_car_following();
        self.integrate(dt);
        self.advance_vehicles();
        self.frame += 1;
    }

    /// Applies the car following model to all vehicles.
    fn apply_car_following(&mut self) {
        for (_, link) in &self.links {
            link.apply_car_following(&self.links, &self.vehicles);
        }
    }

    /// Integrates the velocities and positions of all vehicles,
    /// then resets their accelerations.
    fn integrate(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.integrate(dt);
            vehicle.reset_acceleration();
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