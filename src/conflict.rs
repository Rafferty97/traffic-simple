use cgmath::prelude::*;
use itertools::{unfold, Itertools};
use crate::{LinkId, Link, LinkSet, VehicleSet, Vehicle};
use crate::math::project_point_onto_curve;

const LINK_RADIUS: f64 = 1.8; // m

#[derive(Clone, Copy, Debug)]
pub struct ConflictPoint {
    /// Data associated with each of the conflicting links.
    links: [ConflictingLink; 2],
    /// Whether the links run in the same general direction.
    same_dir: bool
}

#[derive(Clone, Copy, Debug)]
struct ConflictingLink {
    link_id: LinkId,
    min_pos: f64,
    max_pos: f64
}

#[derive(Clone, Copy)]
struct OffsetVehicle<'a> {
    vehicle: &'a Vehicle,
    offset: f64,
    link: LinkId,
    entered_at: usize
}

impl ConflictPoint {
    pub(crate) fn new(a: &Link, b: &Link) -> Option<Self> {
        let conflict_a = ConflictingLink::new(a, b)?;
        let conflict_b = ConflictingLink::new(b, a)?;

        let tangents = [(a, conflict_a), (b, conflict_b)].map(|(l, conflict)| {
            let points = [conflict.min_pos, conflict.max_pos].map(|pos| {
                l.curve().sample_centre(pos).pos
            });
            (points[1] - points[0]).normalize()
        });
        let same_dir = tangents[0].dot(tangents[1]) > 0.25;

        Some(Self {
            links: [conflict_a, conflict_b],
            same_dir
        })
    }

    pub(crate) fn apply_car_following(&self, links: &LinkSet, vehicles: &VehicleSet) {
        let v1 = self.links[0].iter_vehicles(links, vehicles);
        let v2 = self.links[1].iter_vehicles(links, vehicles);
        let merged = v1.merge_by(v2, |a, b| a.entered_at < b.entered_at);
        let mut vehicles = merged.take_while(|v| v.pos_mid() > -40.0); // TODO: Constant

        let mut prev = if let Some(v) = vehicles.next() { v } else { return };
        for curr in vehicles {
            if prev.link != curr.link {
                if self.same_dir && prev.pos_rear() > 0.0 {
                    curr.vehicle.follow_obstacle(prev.vehicle.rear_coords(), prev.vel());
                } else {
                    curr.stop_at_line(0.0);
                }
            }
            prev = curr;
        }
    }
}

impl ConflictingLink {
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
                max_pos
            })
        } else {
            None
        }
    }

    fn iter_vehicles<'a>(
        &'a self,
        links: &'a LinkSet,
        vehicles: &'a VehicleSet
    ) -> impl Iterator<Item=OffsetVehicle<'a>> + 'a {
        let init = (Some(&links[self.link_id]), self.min_pos);
        unfold(init, |(m_link, offset)| {
            if let Some(link) = m_link {
                let out = (*link, *offset);
                if let [prev] = link.links_in() {
                    *link = &links[*prev];
                    *offset += link.length();
                } else {
                    *m_link = None;
                }
                Some(out)
            } else {
                None
            }
        })
            .enumerate()
            .flat_map(move |(idx, (link, offset))| {
                link.iter_vehicles_rev()
                    .filter_map(move |id| {
                        let vehicle = &vehicles[id];
                        vehicle.route()
                            .get(idx)
                            .filter(|el| el.link == self.link_id)
                            .and_then(|el| el.entered_at)
                            .map(|entered_at| {
                                let link = self.link_id;
                                OffsetVehicle { vehicle, offset, link, entered_at }
                            })
                    })
            })
            .skip_while(|veh| veh.pos_rear() > self.max_pos - self.min_pos)
    }
}

impl<'a> OffsetVehicle<'a> {
    fn pos_front(&self) -> f64 {
        self.vehicle.pos_front() - self.offset
    }

    fn pos_mid(&self) -> f64 {
        self.vehicle.pos_mid() - self.offset
    }

    fn pos_rear(&self) -> f64 {
        self.vehicle.pos_rear() - self.offset
    }

    fn vel(&self) -> f64 {
        self.vehicle.vel()
    }

    fn stop_at_line(&self, pos: f64) {
        self.vehicle.stop_at_line(pos + self.offset);
    }
}