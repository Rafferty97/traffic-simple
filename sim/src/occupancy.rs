//! Per-lane occupant lists, sorted by `s`, and leader search over them.
//!
//! A vehicle joins a lane when it advances onto it (its rear axle passes the end
//! of the previous lane) or changes lanes into it. It leaves once both rear
//! corners are outside the lane: past its end, or beside it. In between, its
//! record's `s` is the smaller of its rear corners' `s`. On a loop shorter than a
//! vehicle's rear overhang plus a lane, a vehicle can be in the same lane's list
//! twice, briefly, at two different `s`.
//!
//! A vehicle indicating a lane change also has a ghost record in the target
//! lane's list. Ghosts are kept up to date like any record but are only removed
//! explicitly, when the intent is cancelled or carried out.

use slotmap::SlotMap;

use crate::debug::PassCheck;
use crate::lane::LanePos;
use crate::models::lateral::LaneChange;
use crate::network::{LaneId, SimNetwork};
use crate::vehicle::{Vehicle, VehicleId};

/// How far ahead of its front bumper a vehicle looks for a leader, in meters.
const LEADER_HORIZON: f64 = 200.0;
/// The least room, in meters, a vehicle needs beside another to pass it.
const PASSING_CLEARANCE: f64 = 0.5;

/// A vehicle's record in one lane's occupant list.
#[derive(Clone, Copy, Debug)]
pub struct Occupant {
    /// The smaller of the vehicle's rear corners' `s` on this lane.
    pub s: f64,
    pub id: VehicleId,
    /// Lane coordinates of that rear corner, a hint for the next projection.
    hint: LanePos,
    /// Whether this is the ghost of a vehicle indicating a change into this lane.
    pub ghost: bool,
    /// The span the vehicle takes up across this lane, as followers here see it.
    pub lateral: Lateral,
}

/// The span of `t` a vehicle takes up across a lane it has a record on, as
/// followers there see it.
///
/// For a vehicle settled in its lane, that is its rear corners. For one changing
/// lanes, or the ghost of one indicating, it is stretched to cover where it will
/// end up as well, centred on the lane it is moving into: a vehicle moving in is
/// treated as already there, and one moving out still blocks the side it is
/// moving toward. A follower passes it only if it clears that span.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Lateral {
    pub right: f64,
    pub left: f64,
}

impl Lateral {
    /// The room between this span and a body spanning `body` across the lane
    /// (its `t`, right edge first), on whichever side there is more. Negative if
    /// they overlap.
    fn room(self, body: [f64; 2]) -> f64 {
        (body[0] - self.left).max(self.right - body[1])
    }

    /// Whether a follower whose body spans `body` can pass, with at least
    /// [`PASSING_CLEARANCE`] to spare. Leader search makes the same test itself,
    /// so that it can report the room.
    #[cfg(test)]
    fn can_pass(self, body: [f64; 2]) -> bool {
        self.room(body) >= PASSING_CLEARANCE
    }
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`.
    pub fn new(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, false).0
    }

    /// As [`Occupant::new`], as the ghost of a vehicle indicating a change into `lane`.
    pub fn ghost(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, true).0
    }

    /// As [`Occupant::new`] or [`Occupant::ghost`], also saying where each rear
    /// corner is relative to the lane.
    fn project(
        network: &SimNetwork,
        lane_id: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
        ghost: bool,
    ) -> (Self, [Corner; 2]) {
        let lane = network.lane(lane_id);
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = lane.world_to_lane_near(rear_left, hint);
        let right = lane.world_to_lane_near(rear_right, hint);
        let corner = |p: LanePos| Corner {
            past_end: p.s >= lane.length(),
            beside: p.t.abs() > 0.5 * lane.width(),
        };
        let hint = if left.s <= right.s { left } else { right };
        let (mut lo, mut hi) = (left.t.min(right.t), left.t.max(right.t));
        // Where it is heading, as `t` on this lane: the centerline of this lane for
        // a ghost, and of the lane it follows for a vehicle changing lanes, level
        // with it. Carried across through world space, so the lanes may lie any
        // way relative to each other.
        let heading_for = match (ghost, vehicle.lane_change) {
            (true, _) => Some(0.0),
            (false, None) => None,
            (false, Some(_)) if vehicle.path[0] == lane_id => Some(0.0),
            (false, Some(_)) => {
                let target = network.lane(vehicle.path[0]);
                let point = target.lane_to_world(target.lane_pos(vehicle.lane_pos.s, 0.0));
                Some(lane.world_to_lane_near(point, hint).t)
            }
        };
        if let Some(centre) = heading_for {
            let half_width = 0.5 * vehicle.spec.width;
            (lo, hi) = (lo.min(centre - half_width), hi.max(centre + half_width));
        }
        let lateral = Lateral { right: lo, left: hi };
        let occupant = Self { s: hint.s, id, hint, ghost, lateral };
        (occupant, [corner(left), corner(right)])
    }
}

/// Where a rear corner is relative to a lane.
#[derive(Clone, Copy, Debug)]
struct Corner {
    past_end: bool,
    /// Further from the centerline than half the lane's width.
    beside: bool,
}

/// Refreshes every occupant's `s`, drops those whose rear corners are both
/// outside their lane, and restores the order.
///
/// - Ghosts are refreshed but never dropped here.
/// - A record on the lane a vehicle is following is never dropped for being
///   beside it, only for being past its end.
/// - Vehicles that lose a record on a lane they aren't following, by its end or
///   its side, have their ids pushed to `departed`, so their old side can be
///   checked once every list is up to date.
pub fn update(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &mut [Vec<Occupant>],
    departed: &mut Vec<VehicleId>,
) {
    for (i, list) in occupants.iter_mut().enumerate() {
        let lane_id = LaneId(i as u32);
        list.retain_mut(|occupant| {
            let vehicle = &vehicles[occupant.id];
            let (id, hint, ghost) = (occupant.id, occupant.hint, occupant.ghost);
            let (updated, corners) = Occupant::project(network, lane_id, id, vehicle, hint, ghost);
            *occupant = updated;
            if occupant.ghost {
                return true;
            }
            let following = vehicle.path[0] == lane_id;
            let outside = |c: &Corner| c.past_end || (c.beside && !following);
            if !corners.iter().all(outside) {
                return true;
            }
            if !following {
                departed.push(occupant.id);
            }
            false
        });
        insertion_sort(list);
    }
}

/// Sorts by `s`. Linear on input that is already sorted, which it nearly always
/// is: vehicles rarely pass each other within a lane between substeps.
fn insertion_sort(list: &mut [Occupant]) {
    for i in 1..list.len() {
        let mut j = i;
        while j > 0 && list[j - 1].s > list[j].s {
            list.swap(j - 1, j);
            j -= 1;
        }
    }
}

/// Where a searching vehicle plans to be: its lane change's cubic, or else the
/// centerline, along the lanes it follows. Pure pursuit tracks that path
/// closely, so it says where the vehicle's body will be when it gets somewhere
/// ahead.
///
/// A search along other lanes, as the old-side search runs along the lanes beside
/// them, carries points on the course across through world space, so nothing is
/// assumed about how those lanes lie: they may be any distance apart, and
/// converge or spread.
#[derive(Clone, Copy, Debug)]
pub struct Course<'a> {
    /// The lanes it follows, the one it is on first.
    path: &'a [LaneId],
    /// The lane change it is following, in `s` along `path`.
    change: Option<LaneChange>,
}

impl<'a> Course<'a> {
    pub fn new(path: &'a [LaneId], change: Option<LaneChange>) -> Self {
        Self { path, change }
    }

    /// A vehicle's own planned path.
    pub fn of(vehicle: &'a Vehicle) -> Self {
        Self::new(&vehicle.path, vehicle.lane_change)
    }

    /// The span of `t` on a searched lane that a body covers from when its front
    /// reaches the point at `s` there: over its own length past that point, and
    /// out to where its lane change ends, on the centerline of the lane it follows
    /// level with that point. A body moving across the lane may stay alongside
    /// whatever is there for as long as their speeds keep them level, which can be
    /// the rest of the change; for one moving away, the end adds nothing.
    ///
    /// `lane` is the `k`th lane searched, starting `start` along the searched
    /// lanes, and is either the `k`th lane of the course's path or the lane beside
    /// it. `hint` is a position on `lane` near `s`. The cubic can bulge when it
    /// starts with a slope, so the middle is sampled as well as the ends.
    #[allow(clippy::too_many_arguments)]
    fn span(
        &self,
        network: &SimNetwork,
        k: usize,
        lane: LaneId,
        start: f64,
        s: f64,
        hint: LanePos,
        length: f64,
        half_width: f64,
    ) -> [f64; 2] {
        let offset_at = |at: f64| self.change.map_or(0.0, |c| c.offset_at(at));
        let Some(&level) = self.path.get(k) else {
            // Beyond the course: assume the body could be anywhere.
            return [f64::NEG_INFINITY, f64::INFINITY];
        };
        let searched = network.lane(lane);
        let own = level == lane;
        // Where the point is along the course's path.
        let along = if own {
            start + s
        } else {
            let beside = network.lane(level);
            let before: f64 = self.path[..k].iter().map(|&l| network.lane(l).length()).sum();
            let point = searched.lane_to_world(searched.lane_pos(s, 0.0));
            let guess = beside.lane_pos(s / searched.length() * beside.length(), 0.0);
            before + beside.world_to_lane_near(point, guess).s
        };
        let t_at = |at: f64, offset: f64| match own {
            true => offset,
            false => {
                searched.world_to_lane_near(network.point_along(self.path, at, offset), hint).t
            }
        };
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for f in [0.0, 0.5, 1.0] {
            let at = along + f * length;
            let t = t_at(at, offset_at(at));
            (lo, hi) = (lo.min(t), hi.max(t));
        }
        if self.change.is_some() {
            let t = t_at(along, 0.0);
            (lo, hi) = (lo.min(t), hi.max(t));
        }
        [lo - half_width, hi + half_width]
    }
}

/// How a leader search treats ghost records.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ghosts {
    /// Ghosts are passed over, but the nearest one before the leader is reported:
    /// the searching vehicle is that ghost's would-be follower.
    Report,
    /// Ghosts count as leaders, as when a vehicle weighs up changing lanes: a
    /// vehicle already indicating into the gap ahead goes first.
    Lead,
}

/// What a leader search found, each with the bumper-to-bumper gap to it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Search {
    pub leader: Option<(VehicleId, f64)>,
    /// The nearest ghost before the leader, with [`Ghosts::Report`].
    pub ghost: Option<(VehicleId, f64)>,
}

/// The nearest vehicle ahead along `lanes`, within the leader horizon, and the
/// bumper-to-bumper gap to it. `axle_s` is the vehicle's rear axle's `s` on the
/// first lane.
///
/// On the first lane, only occupants whose rear is ahead of the rear axle count.
/// On later lanes, every occupant counts. The vehicle's own records are skipped,
/// as are `skip`'s, for asking what if that vehicle weren't there.
///
/// A record is passed if its span across the lane (see [`Lateral`]) leaves at
/// least [`PASSING_CLEARANCE`] beside this vehicle's body, where that body will be
/// while passing it, along `course`. Each check is reported to `on_pass`.
#[allow(clippy::too_many_arguments)]
pub fn find_leader(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lanes: &[LaneId],
    axle_s: f64,
    course: Course<'_>,
    ghosts: Ghosts,
    skip: Option<VehicleId>,
    mut on_pass: impl FnMut(PassCheck),
) -> Search {
    let half_width = 0.5 * vehicle.spec.width;
    let front = axle_s + vehicle.spec.length - vehicle.spec.rear_axle;
    let mut found = Search::default();
    // Start of the current lane, measured along `lanes` from the start of the first.
    let mut offset = 0.0;
    let mut after = axle_s;
    for (k, &lane) in lanes.iter().enumerate() {
        if offset - front > LEADER_HORIZON {
            break;
        }
        let list = &occupants[lane.0 as usize];
        let start = list.partition_point(|o| o.s <= after);
        for o in &list[start..] {
            if o.id == id || Some(o.id) == skip {
                continue;
            }
            let gap = offset + o.s - front;
            if gap > LEADER_HORIZON {
                return found;
            }
            if o.ghost && ghosts == Ghosts::Report {
                found.ghost = found.ghost.or(Some((o.id, gap)));
                continue;
            }
            let length = vehicle.spec.length;
            let body = course.span(network, k, lane, offset, o.s, o.hint, length, half_width);
            let room = o.lateral.room(body);
            let passed = room >= PASSING_CLEARANCE;
            let (s, span) = (o.s, [o.lateral.right, o.lateral.left]);
            on_pass(PassCheck { leader: o.id, lane, s, span, body, room, passed });
            if passed {
                continue;
            }
            found.leader = Some((o.id, gap));
            return found;
        }
        offset += network.lane(lane).length();
        after = f64::NEG_INFINITY;
    }
    found
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::lane::Lane;

    #[test]
    fn a_course_is_carried_onto_lanes_that_spread_apart() {
        use glam::DVec2;
        // Lane 0 runs along the x axis. Lane 1 starts beside it, 3.5 m to the
        // left, and spreads to 5.5 m over 100 m: no fixed separation.
        let lane = |a: DVec2, b: DVec2| Lane::new(&[a, b], 3.5, 14.0).unwrap();
        let mut network = SimNetwork::new(vec![
            lane(DVec2::ZERO, DVec2::new(100.0, 0.0)),
            lane(DVec2::new(0.0, 3.5), DVec2::new(100.0, 5.5)),
        ]);
        network.set_adjacent(LaneId(1), LaneId(0), true, true);
        // A car centred on lane 0, checked from lane 1 at its s = 50.
        let path = [LaneId(0)];
        let course = Course::new(&path, None);
        let beside = network.lane(LaneId(1));
        let hint = beside.lane_pos(50.0, 0.0);
        let [right, left] = course.span(&network, 0, LaneId(1), 0.0, 50.0, hint, 4.6, 0.95);
        // Lane 1 is about 4.5 m away there and 4.6 m at the far end of the car,
        // so its centre is about 4.5 m to lane 1's right. A fixed separation
        // would have put it at 3.5 m.
        let centre = 0.5 * (left + right);
        assert!((centre + 4.55).abs() < 0.1, "centre at t = {centre}");
        assert!((left - right - 1.9).abs() < 0.2, "span {right}..{left}");
    }

    #[test]
    fn a_span_is_passed_with_room_on_either_side() {
        let body = |t: f64| [t - 0.95, t + 0.95];
        // A car settled on the centerline.
        let settled = Lateral { right: -0.95, left: 0.95 };
        assert!(!settled.can_pass(body(0.0)));
        assert!(settled.can_pass(body(2.45)), "0.55 m to its left");
        assert!(settled.can_pass(body(-2.45)), "0.55 m to its right");
        assert!(!settled.can_pass(body(2.3)), "less than the clearance");
        // A car moving left out of the lane, its right rear corner 1.6 m left of
        // the centerline, stretched to cover the lane beside: passed only on its
        // right.
        let leaving = Lateral { right: 1.6, left: 4.45 };
        assert!(leaving.can_pass(body(0.0)), "0.65 m to its right");
        assert!(!leaving.can_pass(body(0.2)), "less than the clearance");
        assert!(!leaving.can_pass(body(3.5)), "not where it is going");
        // A car moving in from the right, stretched to cover the centerline, as if
        // already there.
        let arriving = Lateral { right: -4.45, left: 0.95 };
        assert!(!arriving.can_pass(body(0.0)));
        assert!(arriving.can_pass(body(2.45)), "0.55 m to where it will be");
    }

    #[test]
    fn insertion_sort_orders_by_s() {
        let mut ids = SlotMap::<VehicleId, ()>::with_key();
        let lane = Lane::new(&[glam::DVec2::ZERO, glam::DVec2::X * 10.0], 3.5, 14.0).unwrap();
        let mut list: Vec<_> = [3.0, 1.0, 2.0, 0.5]
            .into_iter()
            .map(|s| Occupant {
                s,
                id: ids.insert(()),
                hint: lane.lane_pos(s, 0.0),
                ghost: false,
                lateral: Lateral { right: -1.0, left: 1.0 },
            })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
