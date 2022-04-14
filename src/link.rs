use crate::math::{rot90, ParametricCurve2d, Point2d, Vector2d};
use crate::obstacle::Obstacle;
use crate::vehicle::Vehicle;
use crate::{LinkId, LinkSet, VehicleId, VehicleSet};
pub use curve::{LinkCurve, LinkSample};

mod curve;

/// The minimum lateral clearance for own vehicle to pass another, in m.
const LATERAL_CLEARANCE: f64 = 0.5;

/// The maximum lookahead for the car following model, in s.
const MAX_LOOKAHEAD: f64 = 5.0;

/// A link represents a single lane of traffic.
#[derive(Clone)]
pub struct Link {
    /// The link ID.
    id: LinkId,
    /// The geometry of the link.
    curve: LinkCurve,
    /// An approximate geometry of the link used for projecting vehicles onto it.
    approx_curve: Vec<(f64, Point2d, Vector2d)>,
    /// The links that precede this one.
    links_in: Vec<LinkId>,
    /// The links that succeed this one.
    links_out: Vec<LinkId>,
    /// The links adjacent to this one.
    links_adj: Vec<AdjacentLink>,
    /// Speed limit in m/s.
    speed_limit: f64,
    /// The vehicles on the link.
    vehicles: Vec<VehicleId>,
    /// The traffic control at the start of the link.
    control: TrafficControl,
}

/// The attributes of a link.
pub struct LinkAttributes<'a> {
    /// A curve defining the centre line of the link.
    pub curve: &'a dyn ParametricCurve2d,
    /// The speed limit in m/s.
    pub speed_limit: f64,
}

/// A traffic control.
#[derive(Clone, Copy, Debug)]
pub enum TrafficControl {
    /// Traffic can enter freely, e.g. green light or priority road.
    Open,
    /// Traffic must yield before entering.
    Yield {
        /// The minimum distance from the stop line before a vehicle can enter.
        distance: f64,
        /// Whether a vehicle needs to come to a stop before entering.
        must_stop: bool,
    },
    /// Traffic cannot enter, e.g. red light.
    Closed,
}

/// Information about a link which overlaps another link.
#[derive(Clone)]
struct AdjacentLink {
    /// The ID of the adjacent link
    link_id: LinkId,
    /// Whether a vehicle on this link can change lanes into the adjacent link
    can_lanechange: bool,
}

impl Link {
    /// Creates a new link.
    pub(crate) fn new(id: LinkId, attribs: &LinkAttributes) -> Self {
        let curve = LinkCurve::new(&attribs.curve);
        let approx_curve = (0..)
            .map(|i| i as f64)
            .take_while(|pos| *pos < curve.length())
            .map(|pos| {
                let s = curve.sample_centre(pos);
                (pos, s.pos, s.tan)
            })
            .collect();

        Self {
            id,
            curve,
            approx_curve,
            links_in: vec![],
            links_out: vec![],
            links_adj: vec![],
            speed_limit: attribs.speed_limit,
            vehicles: vec![],
            control: TrafficControl::Open,
        }
    }

    /// Gets the link ID.
    pub fn id(&self) -> LinkId {
        self.id
    }

    /// Gets the length of the link in m.
    pub fn length(&self) -> f64 {
        self.curve.length()
    }

    /// Gets the curve representing the link's centre line.
    pub fn curve(&self) -> &LinkCurve {
        &self.curve
    }

    /// Gets the links the precede this link.
    pub fn links_in(&self) -> &[LinkId] {
        &self.links_in
    }

    /// Gets the links the succeed this link.
    pub fn links_out(&self) -> &[LinkId] {
        &self.links_out
    }

    /// Gets the links that a vehicle on this link is allowed to change into.
    pub fn valid_lanechanges(&self) -> impl Iterator<Item = LinkId> + '_ {
        self.links_adj
            .iter()
            .filter(|adj| adj.can_lanechange)
            .map(|adj| adj.link_id)
    }

    /// Adds an adjacent link.
    pub(crate) fn add_adjacent_link(&mut self, link: &Link) {
        self.links_adj.push(AdjacentLink {
            link_id: link.id,
            can_lanechange: false,
        })
    }

    /// Permits lane changes to an adjacent link.
    pub(crate) fn permit_lanechange(&mut self, link_id: LinkId) {
        let adj = self
            .links_adj
            .iter_mut()
            .find(|adj| adj.link_id == link_id)
            .expect("Cannot permit lane change to non-adjacent link.");
        adj.can_lanechange = true;
    }

    /// Adds a successor link.
    pub(crate) fn add_link_out(&mut self, link_id: LinkId) {
        self.links_out.push(link_id);
    }

    /// Adds a predecessor link.
    pub(crate) fn add_link_in(&mut self, link_id: LinkId) {
        self.links_in.push(link_id);
    }

    /// Inserts the vehicle with the given ID into the link.
    pub(crate) fn insert_vehicle(&mut self, vehicles: &VehicleSet, id: VehicleId) {
        let veh_pos = vehicles[id].pos_mid();
        let idx = self
            .vehicles
            .iter()
            .map(|id| vehicles[*id].pos_mid())
            .position(|pos| pos > veh_pos)
            .unwrap_or(self.vehicles.len());
        self.vehicles.insert(idx, id);
    }

    /// Removes the vehicle with the given ID from the link.
    pub(crate) fn remove_vehicle(&mut self, id: VehicleId) {
        if let Some(idx) = self.vehicles.iter().rposition(|v| *v == id) {
            self.vehicles.remove(idx);
        }
    }

    /// Sets the traffic control at the start of the link.
    pub(crate) fn set_control(&mut self, control: TrafficControl) {
        self.control = control;
    }

    /// Allows vehicles to enter the link.
    pub(crate) fn apply_stoplines(&self, links: &LinkSet, vehicles: &mut VehicleSet, now: usize) {
        for vehicle_id in self.vehicles.iter().rev() {
            let vehicle = &mut vehicles[*vehicle_id];
            if let Some(route) = vehicle.route().get(1) {
                if route.entered_at.is_some() {
                    continue;
                }
                match &links[route.link].control {
                    TrafficControl::Open => {
                        if !vehicle.can_stop(self.length()) {
                            vehicle.enter_link(1, now);
                        } else {
                            break;
                        }
                    }
                    TrafficControl::Yield { distance, must_stop } => {
                        let close_enough = self.length() - vehicle.pos_front() < *distance;
                        let stopped = !must_stop || vehicle.has_stopped();
                        if close_enough && stopped && Self::vehicle_can_enter(links, vehicle, 1) {
                            vehicle.enter_link(1, now);
                        } else {
                            vehicle.stop_at_line(self.length());
                            break;
                        }
                    }
                    TrafficControl::Closed => {
                        vehicle.stop_at_line(self.length());
                        break;
                    }
                }
            }
        }
    }

    /// Determines whether a vehicle can enter a link
    fn vehicle_can_enter(links: &LinkSet, vehicle: &Vehicle, idx: usize) -> bool {
        // todo
        true
    }

    /// Applies the speed limit to the vehicles on and entering this link.
    pub(crate) fn apply_speed_limit(&self, links: &LinkSet, vehicles: &VehicleSet) {
        for vehicle_id in &self.vehicles {
            vehicles[*vehicle_id].apply_current_speed_limit(self.speed_limit);
        }
        for link_id in &self.links_in {
            let link = &links[*link_id];
            if let Some(vehicle_id) = link.vehicles.last() {
                vehicles[*vehicle_id].apply_speed_limit(self.speed_limit, link.length());
            }
        }
    }

    /// Applies the car following model to all vehicles following a vehicle on this link.
    pub(crate) fn apply_car_following(&self, links: &LinkSet, vehicles: &VehicleSet) {
        // Apply accelerations to vehicles on this link
        let obstacles = self
            .vehicles
            .iter()
            .rev()
            .map(|id| vehicles[*id].local_obstacle());
        for (idx, obstacle) in obstacles.enumerate() {
            self.follow_obstacle(links, vehicles, obstacle, (0, self.id), idx + 1);
        }

        // Apply accelerations to vehicles on adjacent links
        for (link_idx, adj_link) in self.links_adj.iter().enumerate() {
            let link = &links[adj_link.link_id];
            let obstacles = self
                .vehicles
                .iter()
                .rev()
                .map(|id| vehicles[*id].adjacent_obstacle(link_idx, &link.approx_curve));
            link.follow_obstacles(links, vehicles, obstacles);
        }
    }

    pub(crate) fn iter_vehicles_rev(&self) -> impl Iterator<Item = VehicleId> + '_ {
        self.vehicles.iter().rev().copied()
    }

    pub fn debug<'a>(
        &'a self,
        links: &'a LinkSet,
        vehicles: &'a VehicleSet,
    ) -> impl Iterator<Item = [Point2d; 2]> + 'a {
        self.links_adj
            .iter()
            .enumerate()
            .flat_map(move |(link_idx, adj_link)| {
                let link = &links[adj_link.link_id];
                self.vehicles
                    .iter()
                    .rev()
                    .map(move |id| vehicles[*id].adjacent_obstacle(link_idx, &link.approx_curve))
                    .map(|o| {
                        let s = link.curve().sample_centre(o.pos);
                        o.lat.as_array().map(|l| s.pos + rot90(s.tan) * l)
                    })
            })
    }

    /// Applies an acceleration to the vehicles on this and preceeding links
    /// that need to follow the given obstacles.
    /// The obstacles must be in reverse order along the link (end to beginning).
    fn follow_obstacles(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet,
        obstacles: impl Iterator<Item = Obstacle>,
    ) {
        let mut skip = 0;

        for obstacle in obstacles {
            // Skip vehicles ahead of the obstacle
            skip = self
                .vehicles
                .iter()
                .rev()
                .skip(skip)
                .map(|id| vehicles[*id].pos_mid())
                .position(|pos| pos < obstacle.pos)
                .map(|cnt| skip + cnt)
                .unwrap_or(self.vehicles.len());

            // Follow the obstacle
            self.follow_obstacle(links, vehicles, obstacle, (0, self.id), skip);
        }
    }

    /// Applies an acceleration to the vehicles on this and preceeding links
    /// that need to follow the given obstacle.
    ///
    /// # Parameters
    /// * `links` - The links in the network
    /// * `vehicles` - The vehicles in the network
    /// * `obstacle` - The obstacle to follow
    /// * `route` - Only consider vehicles following this route
    /// * `skip` - Do not consider this number of vehicles at the end of the link
    fn follow_obstacle(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet,
        obstacle: Obstacle,
        route: (usize, LinkId),
        skip: usize,
    ) {
        let min_pos = obstacle.pos - MAX_LOOKAHEAD * self.speed_limit;

        // Look for a following vehicle on this link
        for veh_id in self.vehicles.iter().rev().skip(skip) {
            let vehicle = &vehicles[*veh_id];
            if !vehicle.on_route(route.0, route.1) {
                continue;
            }
            if Self::can_pass(vehicle, &obstacle) {
                continue;
            }
            if vehicle.pos_front() > min_pos {
                vehicle.follow_vehicle(obstacle.pos, obstacle.vel);
            }
            return;
        }

        // Limit search distance
        if min_pos >= 0.0 {
            return;
        }

        // Look for a following vehicle on preceeding links
        let route = (route.0 + 1, route.1);
        for link_id in &self.links_in {
            let link = &links[*link_id];
            let obstacle = Obstacle {
                pos: obstacle.pos + link.length(),
                ..obstacle
            };
            link.follow_obstacle(links, vehicles, obstacle, route, 0);
        }
    }

    /// Determines whether the vehicle can pass the given obstacle.
    fn can_pass(vehicle: &Vehicle, obstacle: &Obstacle) -> bool {
        let own_lat = vehicle.lat_extent_at(obstacle.pos - vehicle.half_length());
        let clearance = own_lat.clearance_with(&obstacle.lat);
        clearance >= LATERAL_CLEARANCE
    }
}
