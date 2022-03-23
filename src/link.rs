use slotmap::Key;
use curve::LinkCurve;
use crate::obstacle::Obstacle;
use crate::vehicle::Vehicle;
use crate::{LinkId, VehicleId, LinkSet, VehicleSet};
use crate::util::Interval;
use crate::math::{LookupTable, project_local, Vector2d};

mod curve;

/// The minimum lateral clearance for own vehicle to pass another, in m.
const LATERAL_CLEARANCE: f64 = 0.5;

#[derive(Clone)]
pub struct Link {
    id: LinkId,
    curve: LinkCurve,
    links_in: Vec<LinkId>,
    links_out: Vec<LinkId>,
    links_adj: Vec<AdjacentLink>,
    vehicles: Vec<VehicleId>
}

/// Information about a link which overlaps another link
#[derive(Clone)]
struct AdjacentLink {
    /// The ID of the adjacent link
    link: LinkId,
    /// An approximate mapping from longitudinal positions
    /// in the original link to positions in the adjacent link
    pos_map: LookupTable
}

impl Link {
    /// Gets the length of the link in m.
    pub fn length(&self) -> f64 {
        self.curve.length()
    }

    /// Inserts the vehicle with the given ID into the link.
    pub fn insert_vehicle(&mut self, id: VehicleId) {
        self.vehicles.insert(0, id);
    }

    /// Removes the vehicle with the given ID from the link.
    pub fn remove_vehicle(&mut self, id: VehicleId) {
        if let Some(idx) = self.vehicles.iter().rposition(|v| *v == id) {
            self.vehicles.remove(idx);
        }
    }

    /// Applies the car following model to all vehicles following a vehicle on this link.
    pub fn apply_car_following(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet
    ) {
        // Apply accelerations to vehicles on this link
        for (idx, obstacle) in self.vehicle_obstacles(vehicles).enumerate() {
            self.follow_obstacle(links, vehicles, obstacle, &[], idx + 1);
        }

        // Apply accelerations to vehicles on adjacent links
        for adj_link in &self.links_adj {
            let link = &links[adj_link.link];
            let obstacles = self.vehicle_obstacles(vehicles)
                .map(|o| Self::project_obstacle(&o, link, adj_link));
            link.follow_obstacles(links, vehicles, obstacles);
        }
    }

    /// Gets the vehicles on this link as obstacles, in reverse order.
    fn vehicle_obstacles<'a>(&'a self, vehicles: &'a VehicleSet) -> impl Iterator<Item=Obstacle> + 'a {
        self.vehicles.iter()
            .rev()
            .map(|id| vehicles[*id].get_obstacle())
    }

    /// Applies an acceleration to the vehicles on this and preceeding links
    /// that need to follow the given obstacles.
    /// The obstacles must be in reverse order along the link (end to beginning).
    fn follow_obstacles(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet,
        obstacles: impl Iterator<Item=Obstacle>
    ) {
        let mut skip = 0;

        for obstacle in obstacles {
            // Skip vehicles ahead of the obstacle
            skip = self.vehicles.iter().rev()
                .skip(skip)
                .map(|id| vehicles[*id].pos_mid())
                .position(|pos| pos < obstacle.pos)
                .map(|cnt| skip + cnt)
                .unwrap_or(self.vehicles.len());

            // Follow the obstacle
            self.follow_obstacle(links, vehicles, obstacle, &[], skip);
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
        route: &[LinkId],
        skip: usize
    ) {
        // TODO: Limit search distance

        // Look for a following vehicle on this link
        for veh_id in self.vehicles.iter().rev().skip(skip) {
            let vehicle = &vehicles[*veh_id];
            if !Self::can_pass(vehicle, &obstacle) {
                vehicle.follow_obstacle(obstacle.pos, obstacle.vel);
                return;
            }
        }

        // Look for a following vehicle on preceeding links
        let mut ext_route = [LinkId::null(); 8];
        ext_route[0] = self.id;
        ext_route[1..(route.len() + 1)].copy_from_slice(route);

        for link_id in &self.links_in {
            let link = &links[*link_id];
            let obstacle = Obstacle {
                pos: obstacle.pos + link.length(),
                ..obstacle
            };
            link.follow_obstacle(links, vehicles, obstacle, &ext_route, 0);
        }
    }

    /// Determines whether the vehicle can pass the given obstacle.
    fn can_pass(vehicle: &Vehicle, obstacle: &Obstacle) -> bool {
        let own_lat = vehicle.lat_extent();
        let clearance = own_lat.clearance_with(&obstacle.lat);
        clearance >= LATERAL_CLEARANCE
    }

    /// Projects an obstacle onto the given link.
    fn project_obstacle(
        obstacle: &Obstacle,
        link: &Link,
        mapping: &AdjacentLink
    ) -> Obstacle {
        // Find an approximate mapping of the `pos` onto the link
        let dst_pos = mapping.pos_map.sample(obstacle.pos);

        // Get the local coordinate system around this point
        let (origin, y_axis) = link.curve.sample_centre(dst_pos);
        let x_axis = Vector2d::new(-y_axis.y, y_axis.x);

        // Project the obstacle's rear coordinates
        // - x: The lateral offset
        // - y: The longitudinal offset
        let proj = obstacle.coords.map(|coord| {
            project_local(coord, origin, x_axis, y_axis)
        });

        Obstacle {
            pos: dst_pos + f64::min(proj[0].y, proj[1].y),
            lat: Interval::new(proj[0].x, proj[0].y),
            ..*obstacle
        }
    }
}