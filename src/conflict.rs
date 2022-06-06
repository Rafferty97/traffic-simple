use crate::link::RelativeVehicle;
use crate::math::project_point_onto_curve;
use crate::{Link, LinkId, LinkSet, VehicleSet};
use cgmath::prelude::*;
use serde::{Deserialize, Serialize};
use std::ops::ControlFlow;

const LINK_RADIUS: f64 = 1.8; // m

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConflictPoint {
    /// Data associated with each of the conflicting links.
    links: [ConflictPointLink; 2],
    /// Whether the links run in the same general direction.
    same_dir: bool,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
struct ConflictPointLink {
    link_id: LinkId,
    min_pos: f64,
    max_pos: f64,
}

/// Represents a conflict with another link.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct LinkConflict {
    /// The other link.
    pub link_id: LinkId,
    /// The distance on this link at which the conflict is cleared.
    pub own_max_pos: f64,
    /// The minimum `pos` on the other link conflicting with this one.
    pub min_pos: f64,
    /// The maximum `pos` on the other link conflicting with this one.
    pub max_pos: f64,
}

impl ConflictPoint {
    pub(crate) fn new(a: &Link, b: &Link) -> Option<Self> {
        let conflict_a = ConflictPointLink::new(a, b)?;
        let conflict_b = ConflictPointLink::new(b, a)?;

        let tangents = [(a, conflict_a), (b, conflict_b)].map(|(l, conflict)| {
            let points =
                [conflict.min_pos, conflict.max_pos].map(|pos| l.curve().sample_centre(pos).pos);
            (points[1] - points[0]).normalize()
        });
        let same_dir = tangents[0].dot(tangents[1]) > 0.25;

        Some(Self {
            links: [conflict_a, conflict_b],
            same_dir,
        })
    }

    pub(crate) fn link_conflicts(&self) -> impl Iterator<Item = (LinkId, LinkConflict)> + '_ {
        [(0, 1), (1, 0)].into_iter().map(|(i, j)| {
            (
                self.links[i].link_id,
                LinkConflict {
                    link_id: self.links[j].link_id,
                    own_max_pos: self.links[i].max_pos,
                    min_pos: self.links[j].min_pos,
                    max_pos: self.links[j].max_pos,
                },
            )
        })
    }

    pub(crate) fn apply_accelerations<'a>(&self, links: &LinkSet, vehicles: &'a VehicleSet) {
        let mut buffer = vec![]; // todo
        for link in &self.links {
            let cnt = buffer.len();
            link.find_vehicles(links, vehicles, &mut buffer);
            if buffer.len() == cnt {
                return;
            }
        }
        buffer.sort_by(|a, b| b.pos_front().partial_cmp(&a.pos_front()).unwrap());

        buffer.windows(2).for_each(|vehs| {
            if let [leader, follower] = vehs {
                if leader.link_id(0) != follower.link_id(0) {
                    if self.same_dir && leader.pos_rear() > 0.0 {
                        follower.follow_obstacle(leader.rear_coords(), leader.vel());
                    } else {
                        follower.stop_at_line(0.0);
                    }
                }
            }
        })
    }
}

impl ConflictPointLink {
    fn new(link: &Link, other: &Link) -> Option<Self> {
        const STEP: f64 = 0.25;

        let mut own_pos = 0.0;
        let mut other_pos = None;
        let mut min_pos = None;
        let mut max_pos = None;

        while own_pos < link.curve().length() {
            let own_s = link.curve().sample_centre(own_pos);
            other_pos = project_point_onto_curve(other.curve(), own_s.pos, 0.1, other_pos);
            if let Some(other_pos) = other_pos {
                let other_s = other.curve().sample_centre(other_pos);
                let dist = own_s.pos.distance(other_s.pos);
                let dist = dist - LINK_RADIUS * own_s.tan.dot(other_s.tan).abs();
                if dist < LINK_RADIUS {
                    min_pos.get_or_insert(own_pos - STEP);
                    max_pos = Some(own_pos + STEP);
                }
            }
            own_pos += STEP;
        }

        if let (Some(min_pos), Some(max_pos)) = (min_pos, max_pos) {
            Some(Self {
                link_id: link.id(),
                min_pos,
                max_pos,
            })
        } else {
            None
        }
    }

    fn find_vehicles<'a>(
        &self,
        links: &LinkSet,
        vehicles: &'a VehicleSet,
        out: &mut Vec<RelativeVehicle<'a>>,
    ) {
        links[self.link_id].process_vehicles(
            links,
            vehicles,
            &mut |veh| {
                if veh.pos_rear() < self.max_pos - self.min_pos {
                    out.push(veh);
                }
                ControlFlow::Continue(())
            },
            (0, self.link_id),
            self.min_pos,
            0,
        )
    }
}
