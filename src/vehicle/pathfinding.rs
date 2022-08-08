use crate::{LinkId, LinkSet};
use itertools::unfold;
use slotmap::{Key, SparseSecondaryMap};

/// The path finding model of a vehicle.
/// This can be conceptualised as the vehicle's GPS navigation unit.
#[derive(Clone, Debug, Default)]
pub struct PathfindingModel {
    /// The link the vehicle is currently on.
    src: LinkId,
    /// The destination link the vehicle would like to reach.
    dst: LinkId,
    /// A matrix used to determine the desirability of each link.
    matrix: SparseSecondaryMap<LinkId, LaneDists>,
}

/// Inputs into the [`PathfindingModel::evaluate_link`] method.
pub struct EvaluateLinkInput<'a> {
    /// The link to evaluate.
    pub link_id: LinkId,
    /// The vehicle's position along the link.
    pub pos: f64,
    /// The links in the network.
    pub links: &'a LinkSet,
}

impl PathfindingModel {
    /// Updates the link the vehicle is currently on.
    pub fn set_origin(&mut self, link_id: LinkId, links: &LinkSet) {
        self.src = link_id;
        if !self.matrix.contains_key(link_id) {
            self.reroute(links);
        }
    }

    /// Updates the link's destination link.
    pub fn set_destination(&mut self, link_id: LinkId, links: &LinkSet) {
        self.dst = link_id;
        self.reroute(links);
    }

    /// Evaluates the desirability of being on a given link.
    pub fn evaluate_link(&self, input: EvaluateLinkInput) -> f64 {
        if let Some(dists) = self.matrix.get(input.link_id) {
            // Cost of changing lanes now
            let lc_now_cost = if input.link_id == self.src {
                0.0
            } else {
                0.002
            };
            // Cost of changing lanes later
            let lc_later_cost = dists.get_cost(input.pos);
            lc_now_cost + lc_later_cost
        } else {
            f64::INFINITY
        }
    }

    /// Finds the best route to the vehicle's destination, if one exists.
    pub fn reroute(&mut self, links: &LinkSet) {
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

    /// Finds a path (series of links) for the vehicle to follow after it succeeds it current link
    /// which allows it to travel as far as possible towards its goal link without needing to change lanes.
    pub fn calc_path(&self, src: LinkId, links: &LinkSet) -> (Vec<LinkId>, bool) {
        if self.dst.is_null() {
            return (vec![], false);
        }

        let path = unfold(src, |link_id| {
            *link_id = links[*link_id]
                .links_out()
                .iter()
                .flat_map(|id| self.matrix.get(*id).map(|d| (*id, d.0[0])))
                .max_by(|a, b| a.1.cmp(&b.1))
                .map(|(id, _)| id)?;
            Some(*link_id)
        });
        let path = path.take(16).collect::<Vec<_>>();
        let can_exit = path.last().copied() == Some(self.dst);

        (path, can_exit)
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
}

fn successors(link_id: LinkId, links: &LinkSet) -> impl Iterator<Item = (LinkId, usize)> + '_ {
    let lanes = links[link_id].reachable_lanes();
    lanes.iter().flat_map(move |link_id| {
        let link = &links[*link_id];
        let cost = (10. * link.length() / link.speed_limit()) as _;
        link.links_out().iter().map(move |id| (*id, cost))
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

    pub fn offset(&self, dist: f64) -> Self {
        Self(self.0.map(|x| x.saturating_add(Self::make_dist(dist))))
    }

    pub fn shift(&self) -> Self {
        Self([0, self.0[0], self.0[1], self.0[2]])
    }

    pub fn combine(&self, other: &Self) -> Self {
        Self([0, 1, 2, 3].map(|i| u32::max(self.0[i], other.0[i])))
    }

    pub fn get_cost(&self, pos: f64) -> f64 {
        self.0
            .iter()
            .map(|dist| 0.1 * *dist as f64 - pos)
            .enumerate()
            .map(|(idx, dist)| (idx + 1) as f64 / dist)
            .reduce(|a, b| a.max(b))
            .unwrap()
    }

    fn make_dist(dist: f64) -> u32 {
        (10.0 * dist) as u32
    }
}
