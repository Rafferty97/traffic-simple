use slotmap::Key;
use smallvec::SmallVec;
use crate::{LinkId, VehicleId, LinkSet, VehicleSet};
use crate::util::Interval;
use crate::math::Point2d;

pub struct Link {
    id: LinkId,
    vehicles: Vec<VehicleId>,
    links_in: SmallVec<[LinkId; 4]>,
    links_out: SmallVec<[LinkId; 4]>,
    links_adj: SmallVec<[LinkId; 8]>,
}

#[derive(Clone, Copy)]
struct Obstacle {
    /// The longitudinal position of the obstacle in m.
    pub pos: f64,
    /// The lateral extents of the obstacle in m.
    pub lat: Interval<f64>,
    /// The world space coordinates of the obstacle,
    /// represented as the two ends of a line segment.
    pub coords: [Point2d; 2],
    /// The velocity of the obstacle in m/s.
    pub vel: f64
}

impl Link {
    pub fn length(&self) -> f64 {
        0.0 // TODO
    }

    pub fn apply_car_following(
        &self,
        links: &LinkSet,
        vehicles: &VehicleSet
    ) {
        // Apply accelerations to vehicles on this link
        let obstacles = self.vehicles_as_obstacles(vehicles);
        self.follow_obstacles(links, vehicles, obstacles);

        // Apply accelerations to vehicles on adjacent links
        for link_id in &self.links_adj {
            let obstacles = self.vehicles_as_obstacles(vehicles); // TODO: Project
            self.follow_obstacles(links, vehicles, obstacles);
        }
    }

    /// Returns an iterator over the vehicles on the link as obstacles,
    /// ordered from the end of the link to the beginning.
    fn vehicles_as_obstacles<'a>(&'a self, vehicles: &'a VehicleSet) -> impl Iterator<Item=Obstacle> + 'a {
        self.vehicles.iter().rev().map(|id| {
            let veh = &vehicles[*id];
            Obstacle {
                pos: veh.pos_rear(),
                lat: veh.lat_extent(),
                coords: veh.obstacle_coords(),
                vel: veh.vel()
            }
        })
    }

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
            self.follow_obstacle(links, obstacle, &[], skip);
        }
    }

    /// Applies an acceleration to the vehicles on this and preceeding links
    /// that need to follow the given obstacle.
    /// 
    /// # Parameters
    /// * `links` - The other links in the network
    /// * `obstacle` - The obstacle to follow
    /// * `route` - Only consider vehicles following this route
    /// * `skip` - Do not consider this number of vehicles at the end of the link
    fn follow_obstacle(&self, links: &LinkSet, obstacle: Obstacle, route: &[LinkId], skip: usize) {
        // TODO: Limit search distance

        for vehicle in self.vehicles.iter().rev().skip(skip) {
            // TODO: Check lat, and continue
            // TODO: Apply acceleration and return
        }

        let mut ext_route = [LinkId::null(); 8];
        ext_route[0] = self.id;
        ext_route[1..(route.len() + 1)].copy_from_slice(route);

        for link_id in &self.links_in {
            let link = &links[*link_id];
            let obstacle = Obstacle {
                pos: obstacle.pos + link.length(),
                ..obstacle
            };
            link.follow_obstacle(links, obstacle, &ext_route, 0);
        }
    }

    fn project_obstacle(obstacle: &Obstacle, src: &Link, dst: &Link) -> Obstacle {
        // TODO
        // - Use LUT to translate `pos` from `src` to `dst`
        // - Sample the `dst` link at the new `pos`
        // - Calculate `pos` and `lat` offsets, and apply to obstacle

        unimplemented!()
    }
}