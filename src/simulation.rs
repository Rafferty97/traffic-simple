use crate::conflict::ConflictPoint;
use crate::link::{Link, LinkAttributes, TrafficControl};
use crate::math::{CubicFn, Point2d};
use crate::vehicle::{LaneChange, Vehicle, VehicleAttributes};
use crate::{LinkId, LinkSet, VehicleId, VehicleSet};
use lz4_flex::compress_prepend_size;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;

thread_local! {
    pub(crate) static DEBUG_LINES: RefCell<Vec<Vec<Point2d>>> = RefCell::default();
}

/// A traffic simulation.
#[derive(Default, Clone, Serialize, Deserialize)]
pub struct Simulation {
    /// The links in the network.
    links: LinkSet,
    /// The vehicles being simulated.
    vehicles: VehicleSet,
    /// The conflict points.
    conflicts: Vec<ConflictPoint>,
    /// The set of "frozen" vehicles, which will not move.
    frozen_vehs: Vec<VehicleId>,
    /// The current frame of simulation.
    frame: usize,
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

    /// Gets a link.
    pub fn get_link(&self, id: LinkId) -> &Link {
        &self.links[id]
    }

    /// Specifies that two links are adjacent to each other.
    pub fn add_link_adjacency(&mut self, a: LinkId, b: LinkId) {
        if let Some([a, b]) = self.links.get_disjoint_mut([a, b]) {
            a.add_adjacent_link(b);
            b.add_adjacent_link(a);
        }
    }

    /// Specifies that two links may converge or cross.
    pub fn add_link_convergance(&mut self, a: LinkId, b: LinkId) {
        if let [Some(a), Some(b)] = [a, b].map(|i| self.links.get(i)) {
            self.conflicts.extend(ConflictPoint::new(a, b));
        }
    }

    /// Permits lane changes from one link to another.
    pub fn permit_lanechange(&mut self, from: LinkId, to: LinkId) {
        self.links[from].permit_lanechange(to);
    }

    /// Specifies that the end of the `from` link connects to the start of the `to` link.
    pub fn add_link_connection(&mut self, from: LinkId, to: LinkId) {
        self.links[from].add_link_out(to);
        self.links[to].add_link_in(from);
    }

    /// Adds a vehicle to the simulation.
    pub fn add_vehicle(&mut self, attributes: &VehicleAttributes, link: LinkId) -> VehicleId {
        let vehicle_id = self.vehicles.insert_with_key(|id| {
            let mut vehicle = Vehicle::new(id, attributes);
            vehicle.set_location(link, 0.0, None);
            vehicle.update_coords(&self.links);
            vehicle
        });
        self.links[link].insert_vehicle(&self.vehicles, vehicle_id);
        vehicle_id
    }

    /// Freezes or unfreezes a vehicle.
    pub fn get_vehicle_frozen(&mut self, vehicle_id: VehicleId) -> bool {
        self.frozen_vehs.iter().any(|id| *id == vehicle_id)
    }

    /// Freezes or unfreezes a vehicle.
    pub fn set_vehicle_frozen(&mut self, vehicle_id: VehicleId, frozen: bool) {
        let idx = self.frozen_vehs.iter().position(|id| *id == vehicle_id);
        match (frozen, idx) {
            (true, None) => {
                self.frozen_vehs.push(vehicle_id);
            }
            (false, Some(idx)) => {
                self.frozen_vehs.remove(idx);
            }
            _ => {}
        }
    }

    /// Advances the simulation by `dt` seconds.
    pub fn step(&mut self, dt: f64) {
        self.apply_accelerations();
        self.integrate(dt);
        self.advance_vehicles();
        self.update_vehicle_coords();
        self.frame += 1;
    }

    /// Advances the simulation by `dt` seconds,
    /// but only integrates vehicles positions.
    pub fn step_fast(&mut self, dt: f64) {
        self.integrate(dt);
        self.advance_vehicles();
        self.update_vehicle_coords();
        self.frame += 1;
    }

    /// Gets the current simulation frame index.
    pub fn frame(&self) -> usize {
        self.frame
    }

    /// Returns an iterator over all the vehicles in the simulation.
    pub fn iter_vehicles(&self) -> impl Iterator<Item = &Vehicle> {
        self.vehicles.iter().map(|(_, veh)| veh)
    }

    /// Gets a reference to the vehicle with the given ID.
    pub fn get_vehicle(&self, vehicle_id: VehicleId) -> &Vehicle {
        &self.vehicles[vehicle_id]
    }

    /// Gets a reference to the link with the given ID.
    pub fn set_link_control(&mut self, link_id: LinkId, control: TrafficControl) {
        self.links[link_id].set_control(control);
    }

    /// Calculates the accelerations of the vehicles.
    fn apply_accelerations(&mut self) {
        for (_, link) in &mut self.links {
            link.deactivate();
        }
        for (_, vehicle) in &mut self.vehicles {
            vehicle.reset();
            vehicle.activate_links(&mut self.links);
        }
        self.apply_stoplines();
        self.apply_link_accelerations();
        self.apply_frozen_vehicles();
    }

    /// Applies accelerations to stop vehicles before stop lines,
    /// and allows vehicles to enter links.
    fn apply_stoplines(&mut self) {
        for (_, link) in &self.links {
            link.apply_stoplines(&self.links, &mut self.vehicles, self.frame);
        }
    }

    /// Applies the car following model, speed limits, etc. to all vehicles.
    fn apply_link_accelerations(&mut self) {
        for (_, link) in &self.links {
            link.apply_accelerations(&self.links, &self.vehicles);
        }
        for conflict in &self.conflicts {
            conflict.apply_accelerations(&self.links, &self.vehicles);
        }
    }

    /// Applies a large negative acceleration to all frozen vehicles.
    fn apply_frozen_vehicles(&mut self) {
        self.frozen_vehs.retain(|vehicle_id| {
            if let Some(vehicle) = self.vehicles.get(*vehicle_id) {
                vehicle.emergency_stop();
                true
            } else {
                false
            }
        })
    }

    /// Integrates the velocities and positions of all vehicles,
    /// then resets their accelerations.
    fn integrate(&mut self, dt: f64) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.integrate(dt);
        }
    }

    /// Find vehicles that have advanced their link and either move them
    /// to their new link or remove them from the simulation.
    fn advance_vehicles(&mut self) {
        let mut advanced = vec![];
        let mut exited = vec![];

        for (vehicle_id, vehicle) in &mut self.vehicles {
            let link_id = vehicle.link_id().unwrap();
            let did_advance = vehicle.advance(&self.links);

            if did_advance {
                self.links[link_id].remove_vehicle(vehicle_id);
                if let Some(link_id) = vehicle.link_id() {
                    // Vehicle is now on a new link
                    advanced.push((vehicle_id, link_id));
                } else {
                    // Vehicle has exited the simulation
                    exited.push(vehicle_id);
                }
            }
        }

        for (vehicle_id, link_id) in advanced {
            self.links[link_id].insert_vehicle(&self.vehicles, vehicle_id);
        }

        for vehicle_id in exited {
            self.vehicles.remove(vehicle_id);
        }
    }

    /// Updates the world coordinates of all the vehicles.
    fn update_vehicle_coords(&mut self) {
        for (_, vehicle) in &mut self.vehicles {
            vehicle.update_coords(&self.links);
        }
    }

    /// Causes a vehicle to change lanes.
    pub fn do_lane_change(&mut self, vehicle_id: VehicleId, link_id: LinkId, distance: f64) {
        let vehicle = &mut self.vehicles[vehicle_id];

        // Remove the vehicle from its current link
        if let Some(link_id) = vehicle.link_id() {
            self.links[link_id].remove_vehicle(vehicle_id);
        }

        // Project the vehicle's position onto the new link
        let link = &mut self.links[link_id];
        let (pos, offset, slope) = link
            .curve()
            .inverse_sample(vehicle.position(), vehicle.direction())
            .unwrap();

        // Set the vehicle's new position
        let end_pos = pos + distance;
        let lane_change = LaneChange {
            end_pos,
            offset: CubicFn::fit(pos, offset, slope, end_pos, 0.0, 0.0),
        };
        vehicle.set_location(link_id, pos, Some(lane_change));

        // Add the vehicle to the new link
        link.insert_vehicle(&self.vehicles, vehicle_id);
    }

    pub fn set_vehicle_route(&mut self, vehicle_id: VehicleId, route: &[LinkId]) {
        self.vehicles[vehicle_id].set_route(route, true);
    }

    pub fn debug_lines(mut f: impl FnMut(&[Point2d])) {
        DEBUG_LINES.with(|lines| {
            for line in lines.borrow().iter() {
                f(&line[..]);
            }
        });
    }

    pub fn encode(&self) -> Vec<u8> {
        let raw = bson::to_vec(self).unwrap();
        compress_prepend_size(&raw)
    }
}
