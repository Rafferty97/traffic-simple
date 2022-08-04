use crate::conflict::LinkConflict;
use crate::group::Obstacle;
use crate::math::{ParametricCurve2d, Point2d};
use crate::util::rotated_range;
use crate::vehicle::{ObstaclePassResult, RouteState, Vehicle};
use crate::{LinkGroup, LinkId, LinkSet, VehicleId, VehicleSet};
pub use curve::{LinkCurve, LinkSample};
use serde::{Deserialize, Serialize};
use smallvec::{smallvec, SmallVec};
use std::cell::Cell;
use std::cmp::Ordering;
use std::ops::ControlFlow;
use std::rc::Rc;

mod curve;

/// The maximum lookahead for the car following model, in s.
const MAX_LOOKAHEAD: f64 = 5.0;

/// The buffer between reach times for gap acceptance, in s.
const GAP_BUFFER: f64 = 0.5;

/// A link represents a single lane of traffic.
#[derive(Clone, Serialize, Deserialize)]
pub struct Link {
    /// The link ID.
    id: LinkId,
    /// The link group.
    #[serde(skip)]
    group: Option<Rc<LinkGroup>>,
    /// The geometry of the link.
    curve: LinkCurve,
    /// The links that precede this one.
    links_in: SmallVec<[LinkId; 4]>,
    /// The links that succeed this one.
    links_out: SmallVec<[LinkId; 4]>,
    /// The links that are adjacent to this one.
    links_adjacent: SmallVec<[LinkId; 2]>,
    /// The links that conflict with this one.
    conflicts: Vec<LinkConflict>,
    /// The index of the last conflict with an insufficient gap; an optimisation.
    last_conflict: Cell<usize>,
    /// Speed limit in m/s.
    speed_limit: f64,
    /// The vehicles on the link.
    vehicles: Vec<VehicleId>,
    /// The traffic control at the start of the link.
    control: TrafficControl,
    /// A flag indicating whether any vehicles are on/entering this link.
    active: bool,
}

/// The attributes of a link.
pub struct LinkAttributes<'a> {
    /// A curve defining the centre line of the link.
    pub curve: &'a dyn ParametricCurve2d,
    /// The speed limit in m/s.
    pub speed_limit: f64,
}

/// A traffic control.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum TrafficControl {
    /// Traffic can enter freely, e.g. green light or priority road.
    Open,
    /// Traffic must yield before entering.
    Yield {
        /// The minimum distance from the stop line before a vehicle can enter.
        distance: f64,
        /// Whether a vehicle needs to come to a stop before entering.
        must_stop: bool,
        /// The relative priority of the movement.
        priority: i8,
    },
    /// Traffic cannot enter, e.g. red light.
    Closed,
}

impl Link {
    /// Creates a new link.
    pub(crate) fn new(id: LinkId, attribs: &LinkAttributes) -> Self {
        let curve = LinkCurve::new(&attribs.curve);
        Self {
            id,
            curve,
            group: None,
            links_in: smallvec![],
            links_out: smallvec![],
            links_adjacent: smallvec![],
            conflicts: vec![],
            last_conflict: Cell::new(0),
            speed_limit: attribs.speed_limit,
            vehicles: vec![],
            control: TrafficControl::Open,
            active: false,
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

    /// Gets the links the precede this link, which vehicles may enter from.
    pub fn links_in(&self) -> &[LinkId] {
        &self.links_in
    }

    /// Gets the links the succeed this link, which vehicles may exit to.
    pub fn links_out(&self) -> &[LinkId] {
        &self.links_out
    }

    /// Gets the links that a vehicle on this link may change lanes to.
    pub fn links_adjacent(&self) -> &[LinkId] {
        &self.links_adjacent
    }

    /// Adds a successor link.
    pub(crate) fn add_link_out(&mut self, link_id: LinkId) {
        self.links_out.push(link_id);
    }

    /// Adds a predecessor link.
    pub(crate) fn add_link_in(&mut self, link_id: LinkId) {
        self.links_in.push(link_id);
    }

    /// Adds an adjacent link.
    pub(crate) fn add_lane_change(&mut self, link_id: LinkId) {
        self.links_adjacent.push(link_id);
    }

    /// Sets the link group.
    pub(crate) fn set_group(&mut self, group: Rc<LinkGroup>) {
        self.group = Some(group);
    }

    /// Adds a conflicting link.
    pub(crate) fn add_conflict(&mut self, link_conflict: LinkConflict) {
        self.conflicts.push(link_conflict);
    }

    /// Checks that a vehicle could be inserted into the link without collision.
    #[allow(unused)]
    pub(crate) fn can_insert_vehicle(&self, vehicles: &VehicleSet, pos: f64, len: f64) -> bool {
        for id in &self.vehicles {
            let other = &vehicles[*id];
            let gap = other.pos_mid() - pos;
            let needed = 0.5 * (len + other.length());
            if gap.abs() < needed {
                return false;
            }
            if gap > 0.0 {
                break;
            }
        }
        true
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
    pub(crate) fn apply_stoplines(&self, links: &LinkSet, vehicles: &VehicleSet) {
        self.process_vehicles(
            links,
            vehicles,
            &mut |vehicle| match vehicle.get_route(1) {
                Some((_, RouteState::Entered)) => ControlFlow::Continue(()),
                Some((link_id, state)) => match links[link_id].control {
                    TrafficControl::Open => {
                        // Enter the link if it can, otherwise stop processing
                        if !vehicle.can_stop(-5.0) {
                            vehicle.enter_link();
                            ControlFlow::Continue(())
                        } else {
                            ControlFlow::Break(())
                        }
                    }
                    TrafficControl::Yield {
                        distance,
                        must_stop,
                        priority,
                    } => match state {
                        RouteState::NotEntered => {
                            // Enqueue the vehicle if it can, and stop before the line
                            let close_enough = vehicle.pos_front() > -distance;
                            let stopped = !must_stop || vehicle.has_stopped();
                            if close_enough && stopped {
                                vehicle.queue_link();
                            }
                            vehicle.stop_at_line(0.0);
                            ControlFlow::Break(())
                        }
                        RouteState::QueuedAt(frame) => {
                            // Enter the link if it can, otherwise stop before the line
                            if links[link_id].can_enter(
                                links,
                                vehicles,
                                &vehicle.offset(self.length()),
                                frame,
                                priority,
                            ) {
                                vehicle.enter_link();
                                ControlFlow::Continue(())
                            } else {
                                vehicle.stop_at_line(0.0);
                                ControlFlow::Break(())
                            }
                        }
                        RouteState::Entered => unreachable!(),
                    },
                    TrafficControl::Closed => {
                        // Stop before the line
                        vehicle.stop_at_line(0.0);
                        ControlFlow::Break(())
                    }
                },
                None => ControlFlow::Continue(()),
            },
            (0, self.id),
            self.length(),
            0,
        );
    }

    /// Determines whether a vehicle can enter this link or if it must give way.
    fn can_enter(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet,
        vehicle: &RelativeVehicle,
        queued_at: usize,
        priority: i8,
    ) -> bool {
        for idx in rotated_range(self.conflicts.len(), self.last_conflict.get()) {
            let conflict = &self.conflicts[idx];
            let link = &links[conflict.link_id];
            let queued_at = match link.control {
                TrafficControl::Open => None,
                TrafficControl::Yield { priority: p2, .. } => match priority.cmp(&p2) {
                    Ordering::Less => None,
                    Ordering::Equal => Some(queued_at),
                    Ordering::Greater => Some(0),
                },
                TrafficControl::Closed => Some(0),
            };
            let max_pos = conflict.own_max_pos;
            let has_gap = link.check_gap(
                links,
                vehicles,
                [conflict.min_pos, conflict.max_pos],
                vehicle.min_reach_time(max_pos, self.speed_limit) + GAP_BUFFER,
                queued_at,
                (0, conflict.link_id),
                0.0,
            );
            if !has_gap {
                self.last_conflict.set(idx);
                return false;
            }
        }
        true
    }

    /// Checks whether there is sufficient gap on this link.
    #[allow(clippy::too_many_arguments)]
    fn check_gap(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet,
        pos: [f64; 2],
        time: f64,
        queued_at: Option<usize>,
        route: (usize, LinkId),
        max_vel: f64,
    ) -> bool {
        let max_vel = f64::max(max_vel, self.speed_limit);

        let vehs_on_link = self
            .vehicles
            .iter()
            .map(|id| &vehicles[*id])
            .rev()
            .skip_while(|veh| veh.pos_rear() > pos[1]);

        for vehicle in vehs_on_link {
            // Check vehicle's route and priority
            match (vehicle.get_route(route.0), queued_at) {
                // On another route
                (None, _) => continue,
                (Some((link_id, _)), _) if link_id != route.1 => continue,
                // Vehicle has lower priority and hasn't entered the intersection yet
                (Some((_, RouteState::NotEntered)), Some(_)) => return true,
                (Some((_, RouteState::QueuedAt(a))), Some(b)) if a > b => return true,
                // Otherwise, check if there is a sufficient gap
                _ => return vehicle.min_reach_time(pos[0], max_vel) > time,
            }
        }

        if pos[0] / max_vel > time {
            true
        } else {
            let route = (route.0 + 1, route.1);
            self.links_in.iter().all(|link_id| {
                let link = &links[*link_id];
                let pos = pos.map(|p| p + link.length());
                link.check_gap(links, vehicles, pos, time, queued_at, route, max_vel)
            })
        }
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
        if let Some(group) = self.group.as_deref() {
            for proj in group.projections(self.id) {
                let obstacles = self
                    .vehicles
                    .iter()
                    .rev()
                    .map(|id| vehicles[*id].project(proj));
                links[proj.link_id()].follow_obstacles(links, vehicles, obstacles);
            }
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
        use ObstaclePassResult::*;

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
                &mut |veh| match veh.can_pass(obstacle, self) {
                    Pass => ControlFlow::Continue(()),
                    Follow { pos, vel } => {
                        veh.follow_vehicle(pos - obstacle.pos, vel);
                        ControlFlow::Break(())
                    }
                },
                (0, self.id),
                obstacle.pos,
                skip,
            );
        }
    }

    /// Resets the link's `active` flag
    pub(crate) fn deactivate(&mut self) {
        self.active = false;
    }

    /// Sets the link's `active` flag
    pub(crate) fn activate(&mut self) {
        self.active = true;
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
        if !self.active {
            return;
        }

        let min_pos = pos - MAX_LOOKAHEAD * self.speed_limit;

        for veh_id in self.vehicles.iter().rev().skip(skip) {
            let vehicle = &vehicles[*veh_id];
            if vehicle.pos_front() < min_pos {
                return;
            }
            match vehicle.get_route(route.0) {
                None => continue,
                Some((link_id, _)) if link_id != route.1 => continue,
                Some((_, RouteState::Entered)) => {}
                Some((_, _)) => {
                    if pos > self.length() {
                        return;
                    }
                }
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
    pub fn offset(&self, offset: f64) -> RelativeVehicle {
        Self {
            vehicle: self.vehicle,
            pos: self.pos + offset,
            route_idx: self.route_idx + 1,
        }
    }

    pub fn pos_front(&self) -> f64 {
        self.vehicle.pos_front() - self.pos
    }

    pub fn pos_rear(&self) -> f64 {
        self.vehicle.pos_rear() - self.pos
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

    pub fn min_reach_time(&self, pos: f64, max_vel: f64) -> f64 {
        self.vehicle.min_reach_time(self.pos + pos, max_vel)
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

    pub fn queue_link(&self) {
        self.vehicle.queue_link();
    }

    pub fn get_route(&self, idx: usize) -> Option<(LinkId, RouteState)> {
        self.vehicle.get_route(self.route_idx + idx)
    }

    pub fn link_id(&self, idx: usize) -> Option<LinkId> {
        self.get_route(idx).map(|(link_id, _)| link_id)
    }

    /// Determines whether the vehicle can pass the given obstacle.
    pub fn can_pass(&self, obstacle: Obstacle, link: &Link) -> ObstaclePassResult {
        self.vehicle.can_pass(obstacle, link)
    }
}
