use crate::conflict::ConflictPoint;
#[cfg(feature = "debug")]
use crate::debug::take_debug_frame;
use crate::light::TrafficLight;
use crate::link::{Link, LinkAttributes, TrafficControl};
use crate::math::CubicFn;
use crate::vehicle::{LaneChange, Vehicle, VehicleAttributes};
use crate::{LinkGroup, LinkId, LinkSet, TrafficLightId, VehicleId, VehicleSet};
use rand_distr::Distribution;
use slotmap::SlotMap;
use std::rc::Rc;

/// Update vehicle accelerations at least once per this interval.
const ACCELERATION_UPDATE_SEC: f64 = 0.1; // s

/// Perform lane changing calculations at least once per this interval.
const LANE_CHANGE_UPDATE_SEC: f64 = 0.3; // s

/// A traffic simulation.
#[derive(Default)]
pub struct Simulation {
    /// The links in the network.
    links: LinkSet,
    /// The traffic lights.
    lights: SlotMap<TrafficLightId, TrafficLight>,
    /// The conflict points.
    conflicts: Vec<ConflictPoint>,
    /// The vehicles being simulated.
    vehicles: VehicleSet,
    /// The set of "frozen" vehicles, which will not move.
    frozen_vehs: Vec<VehicleId>,
    /// The current frame of simulation.
    frame: usize,
    /// The next sequence number.
    seq: usize,
    /// The time since the last accelerations update in s.
    acc_update: f64,
    /// The time since the last lane change update in s.
    lc_update: f64,
    /// Debugging information from the previously simulated frame.
    #[cfg(feature = "debug")]
    debug: serde_json::Value,
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

    /// Specifies that these vehicles on these links may
    /// interact with vehicles on other links in the group.
    ///
    /// # Parameters
    /// * `link_ids` - The links in the link group; must be ordered left-to-right.
    /// * `lc_mask` - The "lane change mask".
    pub fn add_link_group(&mut self, link_ids: &[LinkId], lc_mask: u32) {
        let links = link_ids
            .iter()
            .map(|id| &self.links[*id])
            .collect::<Vec<_>>();
        let group = Rc::new(LinkGroup::new(&links, lc_mask));
        for id in link_ids {
            self.links[*id].set_group(group.clone());
        }
        for (from, to) in group.lane_changes() {
            self.links[from].add_lane_change(to);
        }
    }

    /// Specifies that two links may converge or cross.
    pub fn add_link_convergance(&mut self, a: LinkId, b: LinkId) {
        if let Some([a, b]) = self.links.get_disjoint_mut([a, b]) {
            if let Some(conflict_point) = ConflictPoint::new(a, b) {
                for (link_id, link_conflict) in conflict_point.link_conflicts() {
                    self.links[link_id].add_conflict(link_conflict);
                }
                self.conflicts.push(conflict_point);
            }
        }
    }

    /// Specifies that the end of the `from` link connects to the start of the `to` link.
    pub fn add_link_connection(&mut self, from: LinkId, to: LinkId) {
        self.links[from].add_link_out(to);
        self.links[to].add_link_in(from);
    }

    /// Adds a traffic light to the simulation.
    pub fn add_traffic_light(&mut self, light: TrafficLight) -> TrafficLightId {
        self.lights.insert(light)
    }

    /// Adds a vehicle to the simulation.
    pub fn add_vehicle(&mut self, attributes: &VehicleAttributes, link: LinkId) -> VehicleId {
        let vehicle_id = self.vehicles.insert_with_key(|id| {
            let mut vehicle = Vehicle::new(id, attributes);
            vehicle.set_location(link, 0.0, None, &self.links);
            vehicle.update_coords(&self.links);
            vehicle
        });
        self.links[link].insert_vehicle(&self.vehicles, vehicle_id);
        vehicle_id
    }

    /// Removes a vehicle from the simulation.
    pub fn remove_vehicle(&mut self, id: VehicleId) {
        let vehicle = self.vehicles.remove(id);
        if let Some(vehicle) = vehicle {
            if let Some(link_id) = vehicle.link_id() {
                self.links[link_id].remove_vehicle(vehicle.id());
            }
        }
    }

    /// Randomly assigns a desired velocity adjustment factor to each vehicle,
    /// which is sampled from a normal distribution with a mean of 1 (no adjustment)
    /// and standard deviation of `stddev`.
    pub fn randomise_velocity_adjusts(&mut self, stddev: f64) {
        let mut rand = rand::thread_rng();
        let distr = rand_distr::Normal::new(1.0, stddev).expect("Invalid standard deviation");
        for (_, vehicle) in &mut self.vehicles {
            let factor = distr.sample(&mut rand).clamp(0.75, 1.25);
            vehicle.set_velocity_adjust(factor);
        }
    }

    /// Sets the `frozen` attribute of a vehicle. When a vehicle is frozen,
    /// it will maximally decelerate until its velocity is zero and remain stopped
    /// until it is no longer frozen.
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

    /// Gets the `frozen` attribute of a vehicle. [Read more](Self::set_vehicle_frozen).
    pub fn get_vehicle_frozen(&mut self, vehicle_id: VehicleId) -> bool {
        self.frozen_vehs.iter().any(|id| *id == vehicle_id)
    }

    /// Sets the vehicle's destination link.
    pub fn set_vehicle_destination(&mut self, vehicle_id: VehicleId, dst: LinkId) {
        self.vehicles[vehicle_id].set_destination(dst, &self.links);
    }

    /// Advances the simulation by `dt` seconds.
    ///
    /// For a realistic simulation, do not use a time step greater than around 0.2.
    pub fn step(&mut self, dt: f64) {
        self.calculate_lane_changes();
        self.apply_accelerations();
        self.update_lights(dt);
        self.integrate(dt);
        self.advance_vehicles();
        self.update_vehicle_coords();
        self.frame += 1;
        self.acc_update += dt;
        self.lc_update += dt;

        #[cfg(feature = "debug")]
        {
            self.debug = take_debug_frame();
        }
    }

    /// Gets the current simulation frame index.
    pub fn frame(&self) -> usize {
        self.frame
    }

    /// Returns an iterator over all the links in the simulation.
    pub fn iter_links(&self) -> impl Iterator<Item = &Link> {
        self.links.values()
    }

    /// Returns an iterator over all the vehicles in the simulation.
    pub fn iter_vehicles(&self) -> impl Iterator<Item = &Vehicle> {
        self.vehicles.values()
    }

    /// Returns an iterator over all the traffic lights in the simulation.
    pub fn iter_lights(&self) -> impl Iterator<Item = (TrafficLightId, &TrafficLight)> {
        self.lights.iter()
    }

    /// Gets a reference to the vehicle with the given ID.
    pub fn get_vehicle(&self, vehicle_id: VehicleId) -> &Vehicle {
        &self.vehicles[vehicle_id]
    }

    /// Gets a reference to the link with the given ID.
    pub fn get_link(&self, link_id: LinkId) -> &Link {
        &self.links[link_id]
    }

    /// Sets the [TrafficControl] at the start of the given link.
    pub fn set_link_control(&mut self, link_id: LinkId, control: TrafficControl) {
        self.links[link_id].set_control(control);
    }

    /// Gets the debugging information for the previously simulated frame as JSON array.
    #[cfg(feature = "debug")]
    pub fn debug(&mut self) -> serde_json::Value {
        self.debug.clone()
    }

    /// Updates the traffic lights.
    fn update_lights(&mut self, dt: f64) {
        for (_, light) in &mut self.lights {
            light.step(dt);
            for (link_id, control) in light.get_states() {
                self.links[link_id].set_control(control);
            }
        }
    }

    /// Performs lane changes.
    fn calculate_lane_changes(&mut self) {
        if self.lc_update < LANE_CHANGE_UPDATE_SEC {
            return;
        } else {
            self.lc_update = 0.0;
        }

        let mut lanechanges = vec![];

        for vehicle in self.vehicles.values_mut() {
            if let Some(path) = vehicle.choose_path(&self.links) {
                let path = path.clone();
                if vehicle.link_id().unwrap() != path.links[0] {
                    // Queue a lane change
                    lanechanges.push((vehicle.id(), path));
                } else {
                    // Just alter the vehicle's path
                    vehicle.set_path(&path);
                }
            }
        }

        // Perform the queued lane changes
        for (vehicle_id, path) in lanechanges {
            self.do_lane_change(vehicle_id, path.links[0], 30.0);
            self.vehicles[vehicle_id].set_path(&path);
        }
    }

    /// Calculates the accelerations of the vehicles.
    fn apply_accelerations(&mut self) {
        if self.acc_update < ACCELERATION_UPDATE_SEC {
            return;
        } else {
            self.acc_update = 0.0;
        }

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
    fn apply_stoplines(&self) {
        for (_, link) in &self.links {
            link.apply_stoplines(&self.links, &self.vehicles);
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
            vehicle.integrate(dt, &mut self.seq);
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

    /// Causes the given vehicle to change lanes from its current link to the specified link.
    /// It will smoothly transition from one to the other over the given `distance` specified in metres.
    pub fn do_lane_change(&mut self, vehicle_id: VehicleId, link_id: LinkId, distance: f64) {
        let vehicle = &mut self.vehicles[vehicle_id];

        // Remove the vehicle from its current link
        if let Some(link_id) = vehicle.link_id() {
            self.links[link_id].remove_vehicle(vehicle_id);
        }

        // Project the vehicle's position onto the new link
        let (pos, offset, slope) = self.links[link_id]
            .curve()
            .inverse_sample(vehicle.position(), vehicle.direction())
            .unwrap();

        // Set the vehicle's new position
        let end_pos = pos + distance;
        let lane_change = LaneChange {
            end_pos,
            offset: CubicFn::fit(pos, offset, slope, end_pos, 0.0, 0.0),
        };
        vehicle.set_location(link_id, pos, Some(lane_change), &self.links);

        // Add the vehicle to the new link
        self.links[link_id].insert_vehicle(&self.vehicles, vehicle_id);
    }
}
