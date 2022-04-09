use cgmath::InnerSpace;
use itertools::{unfold, Itertools};
use crate::{LinkId, Link, math::{project_point_onto_curve, Point2d}, LinkSet, VehicleSet, Vehicle, vehicle::RouteElement};

const LINK_RADIUS: f64 = 3.0; // m

#[derive(Clone, Copy, Debug)]
pub struct ConflictPoint {
    links: [ConflictingLink; 2]
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
        let link_a = ConflictingLink::new(a, b)?;
        let link_b = ConflictingLink::new(b, a)?;
        Some(Self {
            links: [link_a, link_b]
        })
    }

    pub(crate) fn apply_car_following(&self, links: &LinkSet, vehicles: &VehicleSet) {
        let v1 = self.links[0].iter_vehicles(links, vehicles);
        let v2 = self.links[1].iter_vehicles(links, vehicles);
        let merged = v1.merge_by(v2, |a, b| a.entered_at < b.entered_at);
        let mut vehicles = merged.take_while(|v| v.pos_mid() > -20.0); // TODO: Constant

        let mut prev = if let Some(v) = vehicles.next() { v } else { return };
        for curr in vehicles {
            if prev.link != curr.link {
                curr.merge_vehicle(prev.pos_rear(), prev.vel(), 0.0);
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
                let dist = (own_s.pos - other_s.pos).perp_dot(other_s.tan).abs();
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
        let init = (Some(&links[self.link_id]), self.min_pos, 0);
        unfold(init, |(m_link, offset, idx)| {
            if let Some(link) = m_link {
                let out = (*link, *offset, *idx);
                if let [prev] = link.links_in() {
                    *link = &links[*prev];
                    *offset += link.length();
                    *idx += 1;
                } else {
                    *m_link = None;
                }
                Some(out)
            } else {
                None
            }
        })
            .flat_map(move |(link, offset, idx)| {
                link.iter_vehicles_rev()
                    .filter_map(move |id| {
                        let vehicle = &vehicles[id];
                        let RouteElement { link, entered_at } = vehicle.route()[idx];
                        if link != self.link_id {
                            return None;
                        }
                        entered_at.map(|entered_at| {
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

    fn merge_vehicle(&self, pos: f64, vel: f64, stop_line: f64) {
        self.vehicle.merge_vehicle(pos + self.offset, vel, stop_line + self.offset);
    }
}