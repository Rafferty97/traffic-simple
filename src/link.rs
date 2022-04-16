use crate::math::{ParametricCurve2d, Point2d, Vector2d};
use crate::obstacle::Obstacle;
use crate::vehicle::{OnRouteResult, Vehicle};
use crate::{LinkId, LinkSet, VehicleId, VehicleSet};
pub use curve::{LinkCurve, LinkSample};
use std::ops::ControlFlow;

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
        self.process_vehicles(
            links,
            vehicles,
            &mut |vehicle| {
                let link_id = if let Some(link_id) = vehicle.link_id(1) {
                    link_id
                } else {
                    return ControlFlow::Continue(());
                };
                if vehicle.has_entered(1) {
                    return ControlFlow::Continue(());
                }
                match &links[link_id].control {
                    TrafficControl::Open => {
                        if !vehicle.can_stop(0.0) {
                            vehicle.enter_link();
                            ControlFlow::Continue(())
                        } else {
                            ControlFlow::Break(())
                        }
                    }
                    TrafficControl::Yield {
                        distance,
                        must_stop,
                    } => {
                        let close_enough = vehicle.pos_front() > -distance;
                        let stopped = !must_stop || vehicle.has_stopped();
                        if close_enough && stopped && Self::vehicle_can_enter(links, &vehicle, 1) {
                            vehicle.enter_link();
                            ControlFlow::Continue(())
                        } else {
                            vehicle.stop_at_line(0.0);
                            ControlFlow::Break(())
                        }
                    }
                    TrafficControl::Closed => {
                        vehicle.stop_at_line(0.0);
                        ControlFlow::Break(())
                    }
                }
            },
            (0, self.id),
            self.length(),
            0,
        );
    }

    /// Determines whether a vehicle can enter a link
    fn vehicle_can_enter(links: &LinkSet, vehicle: &RelativeVehicle, idx: usize) -> bool {
        // todo
        true
    }

    /// Applies the car following model and the link's speed limit to vehicles on this link and preceeding links.
    pub(crate) fn apply_accelerations(&self, links: &LinkSet, vehicles: &VehicleSet) {
        // Apply car following and speed limit to vehicles on this link
        for ids in self.vehicles.windows(2) {
            let [follower, leader] = [ids[0], ids[1]].map(|id| &vehicles[id]);
            follower.follow_vehicle(leader.pos_rear(), leader.vel());
            follower.apply_current_speed_limit(self.speed_limit);
        }
        if let Some(id) = self.vehicles.last() {
            vehicles[*id].apply_current_speed_limit(self.speed_limit);
        }

        // Apply car following and speed limit to vehicles about to enter this link
        self.process_vehicles(
            links,
            vehicles,
            &mut |veh| {
                veh.apply_speed_limit(self.speed_limit, 0.0);
                if let Some(leader) = self.vehicles.first().map(|id| &vehicles[*id]) {
                    veh.follow_vehicle(leader.pos_rear(), leader.vel());
                }
                ControlFlow::Break(())
            },
            (0, self.id),
            0.0,
            self.vehicles.len(),
        );

        // Apply accelerations to vehicles on adjacent links
        for (link_idx, adj_link) in self.links_adj.iter().enumerate() {
            let link = &links[adj_link.link_id];
            let obstacles = self
                .vehicles
                .iter()
                .rev()
                .map(|id| vehicles[*id].as_obstacle(link_idx, &link.approx_curve));
            link.follow_obstacles(links, vehicles, obstacles);
        }
    }

    /// Applies an acceleration to the vehicles on this and preceeding links
    /// that need to follow the given obstacles.
    /// The obstacles must be in reverse order (descreasing `pos`) along the link.
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
                .position(|id| vehicles[*id].pos_mid() < obstacle.pos)
                .map(|cnt| skip + cnt)
                .unwrap_or(self.vehicles.len());

            // Follow the obstacle
            self.process_vehicles(
                links,
                vehicles,
                &mut |veh| {
                    if veh.can_pass(&obstacle) {
                        return ControlFlow::Continue(());
                    }
                    veh.follow_vehicle(0.0, obstacle.vel);
                    ControlFlow::Break(())
                },
                (0, self.id),
                obstacle.pos,
                skip,
            );
        }
    }

    /// Applies the passed function on each vehicle on the link and preceeding links
    /// in reverse order, in a depth-first fashion.
    /// If the inner function returns `false`, processing on that link stops.
    ///
    /// # Parameters
    /// * `links` - The links in the network
    /// * `vehicles` - The vehicles in the network
    /// * `func` - The function to apply to each vehicle
    /// * `route` - Only process vehicles on this route
    /// * `pos` - The `pos` value that each vehicle's own `pos` will be relative to
    /// * `skip` - Skips this number of vehicles at the end of the link
    pub(crate) fn process_vehicles<'a, F: FnMut(RelativeVehicle<'a>) -> ControlFlow<()>>(
        &self,
        links: &LinkSet,
        vehicles: &'a VehicleSet,
        func: &mut F,
        route: (usize, LinkId),
        pos: f64,
        skip: usize,
    ) {
        let min_pos = pos - MAX_LOOKAHEAD * self.speed_limit;

        for veh_id in self.vehicles.iter().rev().skip(skip) {
            let vehicle = &vehicles[*veh_id];
            if vehicle.pos_front() < min_pos {
                return;
            }
            match vehicle.on_route(route.0, route.1) {
                OnRouteResult::Entered => {}
                OnRouteResult::NotEntered => return,
                OnRouteResult::NotOnRoute => continue,
            }
            let result = (*func)(RelativeVehicle {
                vehicle,
                route_idx: route.0,
                pos,
            });
            if result.is_break() {
                return;
            }
        }

        if min_pos >= 0.0 {
            return;
        }

        let route = (route.0 + 1, route.1);
        for link_id in &self.links_in {
            let link = &links[*link_id];
            let pos = pos + link.length();
            let skip = 0;
            link.process_vehicles(links, vehicles, func, route, pos, skip);
        }
    }
}

/// A reference to a vehicle with reference to another object on the network.
pub(crate) struct RelativeVehicle<'a> {
    vehicle: &'a Vehicle,
    route_idx: usize,
    pos: f64,
}

impl<'a> RelativeVehicle<'a> {
    pub fn pos_front(&self) -> f64 {
        self.vehicle.pos_front() - self.pos
    }

    pub fn pos_mid(&self) -> f64 {
        self.vehicle.pos_mid() - self.pos
    }

    pub fn pos_rear(&self) -> f64 {
        self.vehicle.pos_rear() - self.pos
    }

    /// The minimum stopping position of the front of the vehicle.
    pub fn pos_stop(&self) -> f64 {
        self.pos_front() + self.vehicle.stopping_distance()
    }

    pub fn can_stop(&self, pos: f64) -> bool {
        self.vehicle.can_stop(self.pos + pos)
    }

    pub fn vel(&self) -> f64 {
        self.vehicle.vel()
    }

    pub fn has_stopped(&self) -> bool {
        self.vehicle.has_stopped()
    }

    pub fn rear_coords(&self) -> [Point2d; 2] {
        self.vehicle.rear_coords()
    }

    pub fn apply_speed_limit(&self, speed_limit: f64, pos: f64) {
        self.vehicle.apply_speed_limit(speed_limit, self.pos + pos);
    }

    pub fn follow_vehicle(&self, pos: f64, vel: f64) {
        self.vehicle.follow_vehicle(self.pos + pos, vel);
    }

    pub fn stop_at_line(&self, pos: f64) {
        self.vehicle.stop_at_line(self.pos + pos);
    }

    pub fn follow_obstacle(&self, coords: [Point2d; 2], vel: f64) {
        self.vehicle.follow_obstacle(coords, vel);
    }

    pub fn enter_link(&self) {
        self.vehicle.enter_link();
    }

    /// Determines whether the vehicle can pass the given obstacle.
    pub fn can_pass(&self, obstacle: &Obstacle) -> bool {
        let vehicle = self.vehicle;
        let own_lat = vehicle.lat_extent_at(obstacle.pos - vehicle.half_length());
        let clearance = own_lat.clearance_with(&obstacle.lat);
        clearance >= LATERAL_CLEARANCE
    }

    pub fn on_route(&self, link_id: LinkId) -> OnRouteResult {
        self.vehicle.on_route(self.route_idx, link_id)
    }

    pub fn link_id(&self, idx: usize) -> Option<LinkId> {
        self.vehicle.route().get(self.route_idx + idx).copied()
    }

    pub fn has_entered(&self, idx: usize) -> bool {
        self.vehicle.has_entered(self.route_idx + idx)
    }
}
