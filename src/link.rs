use curve::LinkCurve;
use smallvec::SmallVec;
use crate::obstacle::Obstacle;
use crate::vehicle::Vehicle;
use crate::{LinkId, VehicleId, LinkSet, VehicleSet};
use crate::util::Interval;
use crate::math::{LookupTable, project_local, Vector2d, ParametricCurve2d, Point2d, project_point_onto_curve};

mod curve;

/// The minimum lateral clearance for own vehicle to pass another, in m.
const LATERAL_CLEARANCE: f64 = 0.5;

/// The maximum lookahead for the car following model, in s.
const MAX_LOOKAHEAD: f64 = 10.0;

/// The quantization of the adjacent link LUT, in m.
const LUT_SPACING: f64 = 4.0;

/// A link represents a single lane of traffic.
#[derive(Clone)]
pub struct Link {
    /// The link ID.
    id: LinkId,
    /// The geometry of the link.
    curve: LinkCurve,
    /// The links that precede this one.
    links_in: Vec<LinkId>,
    /// The links that succeed this one.
    links_out: Vec<LinkId>,
    /// The links adjacent to this one.
    links_adj: Vec<AdjacentLink>,
    /// Speed limit in m/s.
    speed_limit: f64,
    /// The vehicles on the link.
    vehicles: Vec<VehicleId>
}

/// The attributes of a link.
pub struct LinkAttributes<'a> {
    /// A curve defining the centre line of the link.
    pub curve: &'a dyn ParametricCurve2d,
    /// The speed limit in m/s.
    pub speed_limit: f64
}

/// Information about a link which overlaps another link.
#[derive(Clone)]
struct AdjacentLink {
    /// The ID of the adjacent link
    link_id: LinkId,
    /// An approximate mapping from longitudinal positions
    /// in the original link to positions in the adjacent link
    pos_map: LookupTable<Option<f64>>
}

impl Link {
    /// Creates a new link.
    pub(crate) fn new(id: LinkId, attribs: &LinkAttributes) -> Self {
        Self {
            id,
            curve: LinkCurve::new(&attribs.curve),
            links_in: vec![],
            links_out: vec![],
            links_adj: vec![],
            speed_limit: attribs.speed_limit,
            vehicles: vec![]
        }
    }

    /// Gets the length of the link in m.
    pub fn length(&self) -> f64 {
        self.curve.length()
    }

    /// Gets the curve representing the link's centre line.
    pub fn curve(&self) -> &LinkCurve {
        &self.curve
    }

    /// Adds an adjacent link.
    pub(crate) fn add_adjacent_link(&mut self, link: &Link) {
        self.links_adj.push(AdjacentLink {
            link_id: link.id,
            pos_map: LookupTable::from_samples(self.curve.bounds(), LUT_SPACING, |pos| {
                project_point_onto_curve(&link.curve, self.curve.sample_centre(pos).0, 0.01, None)
            })
        })
    }

    /// Adds a successor link.
    pub(crate) fn add_successor_link(&mut self, link_id: LinkId) {
        self.links_out.push(link_id);
    }

    /// Adds a predecessor link.
    pub(crate) fn add_predecessor_link(&mut self, link_id: LinkId) {
        self.links_in.push(link_id);
    }

    /// Inserts the vehicle with the given ID into the link.
    pub(crate) fn insert_vehicle(&mut self, vehicles: &VehicleSet, id: VehicleId) {
        let veh_pos = vehicles[id].pos_mid();
        let idx = self.vehicles.iter()
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

    /// Applies the car following model to all vehicles following a vehicle on this link.
    pub(crate) fn apply_car_following(
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
            let link = &links[adj_link.link_id];
            let obstacles = self.vehicle_obstacles(vehicles)
                .flat_map(|o| Self::project_obstacle(&o, link, adj_link));
            link.follow_obstacles(links, vehicles, obstacles);
        }
    }

    // debug
    pub(crate) fn projected_lines<'a>(&'a self, links: &'a LinkSet, vehicles: &'a VehicleSet) -> impl Iterator<Item=[Point2d; 2]> + '_ {
        self.links_adj.iter().flat_map(|adj_link| {
            let link = &links[adj_link.link_id];
            let a = self.vehicle_obstacles(vehicles)
                .map(|o| o.coords);
            let b = self.vehicle_obstacles(vehicles)
                .flat_map(|o| Self::project_obstacle(&o, link, adj_link))
                .map(|o| o.lat.as_array().map(|l| link.curve.sample(o.pos, l, 0.0).0));
            a.chain(b)
        })
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
        let min_pos = obstacle.pos - MAX_LOOKAHEAD * self.speed_limit;

        // Look for a following vehicle on this link
        for veh_id in self.vehicles.iter().rev().skip(skip) {
            let vehicle = &vehicles[*veh_id];
            if !vehicle.on_route(route) {
                continue;
            }
            if Self::can_pass(vehicle, &obstacle) {
                continue;
            }
            if vehicle.pos_front() > min_pos {
                vehicle.follow_obstacle(obstacle.pos, obstacle.vel);
            }
            return;
        }

        // Limit search distance
        if min_pos >= 0.0 {
            return;
        }

        // Look for a following vehicle on preceeding links
        let route = std::iter::once(self.id)
            .chain(route.iter().copied())
            .collect::<SmallVec<[_; 8]>>();

        for link_id in &self.links_in {
            let link = &links[*link_id];
            let obstacle = Obstacle {
                pos: obstacle.pos + link.length(),
                ..obstacle
            };
            link.follow_obstacle(links, vehicles, obstacle, &route, 0);
        }
    }

    /// Determines whether the vehicle can pass the given obstacle.
    fn can_pass(vehicle: &Vehicle, obstacle: &Obstacle) -> bool {
        let own_lat = vehicle.lat_extent_at(obstacle.pos - vehicle.half_length());
        let clearance = own_lat.clearance_with(&obstacle.lat);
        clearance >= LATERAL_CLEARANCE
    }

    /// Projects an obstacle onto the given link.
    fn project_obstacle(
        obstacle: &Obstacle,
        link: &Link,
        mapping: &AdjacentLink
    ) -> Option<Obstacle> {
        // Find an approximate mapping of the `pos` onto the link, then refine it.
        let dst_pos = (*mapping.pos_map.sample(obstacle.pos))?;
        let dst_pos = project_point_onto_curve(
            &link.curve, 
            obstacle.coords[0],
            0.1,
            Some(dst_pos)
        )?;

        // Get the local coordinate system around this point
        let (origin, y_axis) = link.curve.sample_centre(dst_pos);
        let x_axis = Vector2d::new(-y_axis.y, y_axis.x);

        // Project the obstacle's rear coordinates
        // - x: The lateral offset
        // - y: The longitudinal offset
        let proj = obstacle.coords.map(|coord| {
            project_local(coord, origin, x_axis, y_axis)
        });

        Some(Obstacle {
            pos: dst_pos + f64::min(proj[0].y, proj[1].y),
            lat: Interval::new(proj[0].x, proj[1].x),
            ..*obstacle
        })
    }
}