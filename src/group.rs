use crate::math::{project_point_onto_curve, rot90};
use crate::util::Interval;
use crate::Link;
use crate::{math::Point2d, LinkId};
use cgmath::InnerSpace;
use itertools::iproduct;
use serde::{Deserialize, Serialize};

/// A set of adjacent links.
/// Lane changing is only permitted between members of a link group.
#[derive(Clone)]
pub struct LinkGroup {
    num_links: usize,
    projections: Vec<LinkProjection>,
}

#[derive(Clone)]
pub(crate) struct LinkProjection {
    src: LinkId,
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

#[derive(Clone, Copy, Serialize, Deserialize)]
struct ProjectedSample {
    pos: f64,
    lat: f64,
}

impl LinkGroup {
    pub fn new(links: &[&Link]) -> Self {
        if links.len() < 2 {
            panic!("Link group must contain atleast two links");
        }

        let segment_len = 2.0;

        let projections = iproduct!(links, links)
            .filter(|(src, dst)| src.id() != dst.id())
            .map(|(src, dst)| LinkProjection {
                src: src.id(),
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
            num_links: links.len(),
            projections,
        }
    }

    pub(crate) fn link_ids(&self) -> impl Iterator<Item = LinkId> + '_ {
        let n = self.num_links - 1;
        (0..self.num_links).map(move |i| self.projections[n * i].src)
    }

    pub(crate) fn projections(&self, src_link: LinkId) -> &[LinkProjection] {
        let idx = self
            .link_ids()
            .position(|id| id == src_link)
            .expect("Link ID is not in group.");
        let n = self.num_links - 1;
        &self.projections[(n * idx)..(n * (idx + 1))]
    }

    // pub(crate) fn project(&self, rear_coords: [Point2d; 2], vel: f64, out: &mut GroupPosition) {
    //     // Update the segment index
    //     out.segment = self
    //         .segments
    //         .iter()
    //         .skip(out.segment)
    //         .position(|(pos, tan)| rear_coords.iter().any(|c| (c - pos).dot(*tan) <= 0.0))
    //         .map(|i| i + out.segment)
    //         .unwrap_or(self.segments.len())
    //         .saturating_sub(1);

    //     // Get the local coordinate system around the vehicle
    //     let (pos, tan) = unsafe {
    //         // SAFETY: Use of `position` above guarantees the index is in bounds.
    //         *self.segments.get_unchecked(out.segment)
    //     };

    //     // Project the vehicle's rear coordinates and save
    //     let proj = rear_coords.map(|c| project_local(c, pos, rot90(tan), tan));
    //     out.pos = f64::min(proj[0].y, proj[1].y);
    //     out.lat = Interval::new(proj[0].x, proj[1].x);
    //     out.vel = vel;
    // }
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
