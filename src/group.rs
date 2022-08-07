use crate::math::{project_point_onto_curve, rot90};
use crate::util::{Direction, Interval};
use crate::Link;
use crate::{math::Point2d, LinkId};
use cgmath::InnerSpace;
use itertools::iproduct;
use smallvec::SmallVec;

/// A set of adjacent links.
/// Lane changing is only permitted between members of a link group.
#[derive(Clone)]
pub struct LinkGroup {
    links: SmallVec<[LinkId; 8]>,
    projections: Vec<LinkProjection>,
    lc_mask: u32,
}

#[derive(Clone)]
pub(crate) struct LinkProjection {
    dst: LinkId,
    inv_segment_len: f64,
    samples: Vec<ProjectedSample>,
}

/// Represents a vehicle's position relative to a link.
#[derive(Clone, Copy)]
pub struct Obstacle {
    /// The approximate longitudinal position of the obstacle in m.
    pub pos: f64,
    /// The approximate lateral extents of the obstacle in m.
    pub lat: Interval<f64>,
    /// The rear coordinates of the obstacle in world space.
    pub rear_coords: [Point2d; 2],
    /// The velocity of the obstacle in m/s.
    pub vel: f64,
}

#[derive(Clone, Copy)]
struct ProjectedSample {
    pos: f64,
    lat: f64,
}

impl LinkGroup {
    pub(crate) fn new(links: &[&Link], lc_mask: u32) -> Self {
        if links.len() < 2 {
            panic!("Link group must contain atleast two links");
        }

        let segment_len = 2.0;

        let projections = iproduct!(links, links)
            .filter(|(src, dst)| src.id() != dst.id())
            .map(|(src, dst)| LinkProjection {
                dst: dst.id(),
                inv_segment_len: 1.0 / segment_len,
                samples: (0..)
                    .map(|i| segment_len * i as f64)
                    .take_while(|pos| *pos < src.curve().length())
                    .map(|pos| src.curve().sample_centre(pos).pos)
                    .scan(Some(0.0), |pos, point| {
                        *pos = project_point_onto_curve(dst.curve(), point, 0.1, *pos);
                        pos.map(|pos| (point, pos))
                    })
                    .map(|(point, pos)| {
                        let s = dst.curve().sample_centre(pos);
                        let lat = (point - s.pos).dot(rot90(s.tan));
                        ProjectedSample { pos, lat }
                    })
                    .collect(),
            })
            .collect();

        Self {
            links: links.iter().map(|l| l.id()).collect(),
            projections,
            lc_mask,
        }
    }

    pub(crate) fn link_ids(&self) -> &[LinkId] {
        &self.links
    }

    pub(crate) fn projections(&self, src: LinkId) -> &[LinkProjection] {
        let idx = self.find_link(src).expect("Link is not in group.");
        let n = self.links.len() - 1;
        &self.projections[(n * idx)..(n * (idx + 1))]
    }

    /// Gets the index of the `link` in this link group.
    pub(crate) fn find_link(&self, link_id: LinkId) -> Option<usize> {
        self.links.iter().position(|id| *id == link_id)
    }

    /// Gets all the lanes reachable from the `src` lane by performing lane changes,
    /// including the `src` lane itself.
    pub(crate) fn reachable_lanes(&self, src: usize) -> &[LinkId] {
        let mut left = src;
        while left > 0 && self.lc_bit(left, Direction::Left) {
            left -= 1;
        }
        let mut right = src;
        while right < self.links.len() - 1 && self.lc_bit(right, Direction::Right) {
            right += 1;
        }
        &self.links[left..=right]
    }

    /// All the permissable lane changes in the link group, in a particular order.
    pub(crate) fn lane_changes(&self) -> impl Iterator<Item = (LinkId, LinkId)> + '_ {
        let right = (0..self.links.len() - 1)
            .rev()
            .filter(|idx| self.lc_bit(*idx, Direction::Right))
            .map(|idx| (self.links[idx], self.links[idx + 1]));
        let left = (1..self.links.len())
            .filter(|idx| self.lc_bit(*idx, Direction::Left))
            .map(|idx| (self.links[idx], self.links[idx - 1]));
        right.chain(left)
    }

    /// Gets the value of the lane change bit for the given index and direction.
    #[inline(always)]
    fn lc_bit(&self, idx: usize, dir: Direction) -> bool {
        let bit = (2 * idx)
            + match dir {
                Direction::Left => 0,
                Direction::Right => 1,
            };
        (self.lc_mask >> bit) & 1 != 0
    }
}

impl LinkProjection {
    pub fn link_id(&self) -> LinkId {
        self.dst
    }

    pub fn project(
        &self,
        rear_coords: [Point2d; 2],
        pos: f64,
        lat: Interval<f64>,
        vel: f64,
    ) -> Obstacle {
        let idx = usize::min(
            (pos * self.inv_segment_len) as usize,
            self.samples.len() - 1,
        );
        let sample = unsafe {
            // SAFETY: Way index is generated guarantees it's within bounds
            self.samples.get_unchecked(idx)
        };
        // Account for a negative `pos` value
        let behind = f64::min(pos, 0.0);
        Obstacle {
            rear_coords,
            pos: sample.pos + behind,
            lat: lat + sample.lat,
            vel,
        }
    }
}
