use crate::{LinkId, LinkSet};
use arrayvec::ArrayVec;
use slotmap::{Key, SparseSecondaryMap};

const PLAN_DIST: f64 = 200.0;

/// The path finding model of a vehicle.
/// This can be conceptualised as the vehicle's GPS navigation unit.
#[derive(Clone, Debug, Default)]
pub(crate) struct PathfindingModel {
    /// The link the vehicle is currently on.
    src: LinkId,
    /// The destination link the vehicle would like to reach.
    dst: LinkId,
    /// A matrix used to determine the desirability of each link.
    matrix: SparseSecondaryMap<LinkId, LaneDists>,
    /// The current set of available paths.
    paths: Vec<Path>,
}

/// A series of links that advances the vehicle towards its goal link.
#[derive(Clone, Debug)]
pub(crate) struct Path {
    pub links: ArrayVec<LinkId, 8>,
    pub dists: LaneDists,
    pub can_exit: bool,
}

impl PathfindingModel {
    /// Updates the link the vehicle is currently on.
    pub fn set_origin(&mut self, link_id: LinkId, links: &LinkSet) {
        self.src = link_id;
        if !self.matrix.contains_key(link_id) {
            self.reroute(links);
        }
        self.make_paths(links);
    }

    /// Updates the link's destination link.
    pub fn set_destination(&mut self, link_id: LinkId, links: &LinkSet) {
        self.dst = link_id;
        self.reroute(links);
        self.make_paths(links);
    }

    /// Gets the paths the vehicle could traverse either from its current link
    /// or an adjacent link it's allowed to lane change to.
    pub fn paths(&self) -> &[Path] {
        &self.paths
    }

    /// Gets the vehicle's current destination link.
    pub fn destination(&self) -> LinkId {
        self.dst
    }

    /// Finds the best route to the vehicle's destination, if one exists.
    fn reroute(&mut self, links: &LinkSet) {
        if self.src.is_null() || self.dst.is_null() {
            self.matrix = Default::default();
            return;
        }

        let dsts = links[self.dst].reachable_from_lanes();
        let result = pathfinding::directed::dijkstra::dijkstra(
            &self.src,
            |id| successors(*id, links),
            |id| dsts.contains(id),
        );
        if let Some((route, _)) = result {
            self.calc_lane_matrix(&route, links);
        }
    }

    /// Calculates a lane matrix from a path.
    fn calc_lane_matrix(&mut self, route: &[LinkId], links: &LinkSet) {
        let mut matrix = SparseSecondaryMap::new();
        matrix.insert(self.dst, LaneDists::exit());

        for link_id in route.iter().rev() {
            let group = links[*link_id].group();
            let group_links = group
                .map(|g| g.link_ids())
                .unwrap_or_else(|| core::slice::from_ref(link_id));

            // Propagate backwards
            for link_id in group_links {
                let link = &links[*link_id];
                let dists = link
                    .links_out()
                    .iter()
                    .flat_map(|id| matrix.get(*id))
                    .copied()
                    .reduce(|a, b| a.combine(&b))
                    .map(|d| d.offset(link.length()));
                if let Some(dists) = dists {
                    matrix.insert(*link_id, dists);
                }
            }

            // Propagate across
            if let Some(group) = group {
                for (from, to) in group.lane_changes() {
                    let to_dists = matrix.get(to);
                    if let Some(to_dists) = to_dists.map(|d| d.shift()) {
                        let from_dists = matrix
                            .entry(from)
                            .unwrap()
                            .or_insert(LaneDists::dead(links[from].length()));
                        *from_dists = from_dists.combine(&to_dists);
                    }
                }
            }
        }

        self.matrix = matrix;
    }

    fn make_paths(&mut self, links: &LinkSet) {
        self.paths.clear();
        let mut path = ArrayVec::new();

        path.push(self.src);
        self.make_paths_inner(links, &mut path, 0.0);

        for link_id in links[self.src].lane_changes() {
            path[0] = *link_id;
            self.make_paths_inner(links, &mut path, 0.0);
        }
    }

    fn make_paths_inner(
        &mut self,
        links: &LinkSet,
        path: &mut ArrayVec<LinkId, 8>,
        dist: f64,
    ) -> bool {
        let link_id = *path.last_mut().unwrap();

        let dists = if let Some(dists) = self.matrix.get(link_id) {
            dists.offset(dist)
        } else {
            return false;
        };

        let mut found = false;

        if !path.is_full() && dist < PLAN_DIST {
            let link = &links[link_id];
            let dist = dist + link.length();
            let last_idx = path.len();
            path.push(LinkId::null());
            for succ in links[link_id].links_out() {
                path[last_idx] = *succ;
                found |= self.make_paths_inner(links, path, dist);
            }
            path.pop();
        }

        if !found {
            let can_exit = link_id == self.dst;
            self.paths.push(Path {
                links: path.clone(),
                dists,
                can_exit,
            });
        }

        true
    }
}

fn successors(link_id: LinkId, links: &LinkSet) -> impl Iterator<Item = (LinkId, usize)> + '_ {
    let lanes = links[link_id].reachable_lanes();
    lanes.iter().flat_map(move |link_id| {
        let link = &links[*link_id];
        link.links_out().iter().map(move |id| (*id, link.cost()))
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(crate) struct LaneDists([u32; 4]);

impl LaneDists {
    pub fn dead(dist: f64) -> Self {
        Self([Self::make_dist(dist); 4])
    }

    pub fn exit() -> Self {
        Self([u32::MAX; 4])
    }

    /// Add an amount to all distances.
    pub fn offset(&self, distance: f64) -> Self {
        Self(self.0.map(|x| x.saturating_add(Self::make_dist(distance))))
    }

    /// Offsets all distances by one lane.
    pub fn shift(&self) -> Self {
        Self([0, self.0[0], self.0[1], self.0[2]])
    }

    /// Combines two `LaneDist`s.
    pub fn combine(&self, other: &Self) -> Self {
        Self([0, 1, 2, 3].map(|i| u32::max(self.0[i], other.0[i])))
    }

    /// Computes the cost of being on this link at the given `pos`.
    pub fn cost(&self, pos: f64) -> f64 {
        self.0
            .iter()
            .map(|dist| (0.1 * *dist as f64) - pos)
            .enumerate()
            .map(|(idx, dist)| (idx + 1) as f64 / dist)
            .reduce(|a, b| a.max(b))
            .unwrap()
    }

    fn make_dist(dist: f64) -> u32 {
        (10.0 * dist) as u32
    }
}
