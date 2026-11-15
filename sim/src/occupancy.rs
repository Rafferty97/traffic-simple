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

use crate::lane::{Lane, LanePos};
use crate::models::lateral::LaneChange;
use crate::network::{LaneId, Side, SimNetwork};
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
    /// How the vehicle sits across this lane, as followers here see it. Kept up
    /// to date by `update`; in the lane on a new record, which is the cautious way.
    pub lateral: Lateral,
}

/// How a vehicle sits across a lane it has a record on, as followers there see it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Lateral {
    /// Following the lane or moving into it. It blocks followers wherever it is
    /// across the lane: a vehicle moving in is treated as already there.
    InLane,
    /// Leaving the lane to the left after a lane change. `trailing` is the `t` of
    /// its right rear corner, the one still furthest into the lane.
    DivergingLeft { trailing: f64 },
    /// Leaving the lane to the right; `trailing` is its left rear corner's `t`.
    DivergingRight { trailing: f64 },
}

impl Lateral {
    /// Whether a follower whose body spans `body` across the lane (its `t`, right
    /// edge first) can pass, with at least [`PASSING_CLEARANCE`] to spare. It can
    /// only pass a diverging vehicle, and only on the side that vehicle is leaving.
    fn can_pass(self, body: [f64; 2]) -> bool {
        match self {
            Lateral::InLane => false,
            Lateral::DivergingLeft { trailing } => body[1] + PASSING_CLEARANCE <= trailing,
            Lateral::DivergingRight { trailing } => body[0] - PASSING_CLEARANCE >= trailing,
        }
    }
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`.
    pub fn new(lane: &Lane, id: VehicleId, vehicle: &Vehicle, hint: LanePos) -> Self {
        Self::project(lane, id, vehicle, hint).0
    }

    /// As [`Occupant::new`], as the ghost of a vehicle indicating a change into `lane`.
    pub fn ghost(lane: &Lane, id: VehicleId, vehicle: &Vehicle, hint: LanePos) -> Self {
        Self { ghost: true, ..Self::new(lane, id, vehicle, hint) }
    }

    /// As [`Occupant::new`], also saying where each rear corner is relative to the lane.
    fn project(
        lane: &Lane,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> (Self, [Corner; 2]) {
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = lane.world_to_lane_near(rear_left, hint);
        let right = lane.world_to_lane_near(rear_right, hint);
        let corner = |p: LanePos| Corner {
            t: p.t,
            past_end: p.s >= lane.length(),
            beside: p.t.abs() > 0.5 * lane.width(),
        };
        let hint = if left.s <= right.s { left } else { right };
        // Set by `update`, which knows how the vehicle relates to this lane.
        let lateral = Lateral::InLane;
        let occupant = Self { s: hint.s, id, hint, ghost: false, lateral };
        (occupant, [corner(left), corner(right)])
    }
}

/// Where a rear corner is relative to a lane.
#[derive(Clone, Copy, Debug)]
struct Corner {
    t: f64,
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
        let lane = network.lane(lane_id);
        list.retain_mut(|occupant| {
            let vehicle = &vehicles[occupant.id];
            let (updated, corners) = Occupant::project(lane, occupant.id, vehicle, occupant.hint);
            *occupant = Occupant { ghost: occupant.ghost, ..updated };
            if occupant.ghost {
                return true;
            }
            let following = vehicle.path[0] == lane_id;
            occupant.lateral = match following {
                true => Lateral::InLane,
                false => lateral(network, lane, vehicle, corners),
            };
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

/// How a vehicle that isn't following `lane` sits across it, from its rear
/// corners there, left then right.
///
/// It is diverging if `lane` is an old lane it is moving out of sideways: the
/// lane beside `lane`, on the side away from the vehicle's old side, is the lane
/// it follows or, just after a section boundary, the lane before it. Otherwise,
/// as for a record lingering past the end of the lane before, it is in the lane.
fn lateral(network: &SimNetwork, lane: &Lane, vehicle: &Vehicle, corners: [Corner; 2]) -> Lateral {
    let Some(old_side) = vehicle.old_side else {
        return Lateral::InLane;
    };
    let toward = old_side.opposite();
    let following = vehicle.path[0];
    let moving_out = lane.adj(toward).is_some_and(|n| {
        n.lane == following || network.lane(n.lane).successors().contains(&following)
    });
    let [left, right] = corners;
    match (moving_out, toward) {
        (false, _) => Lateral::InLane,
        (true, Side::Left) => Lateral::DivergingLeft { trailing: right.t },
        (true, Side::Right) => Lateral::DivergingRight { trailing: left.t },
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

/// Where a searching vehicle will be across the lanes it searches: its planned
/// path, which is its lane change's cubic or else its lane's centerline, shifted
/// onto the searched lanes. Pure pursuit tracks that path closely, so it says
/// where the vehicle's body will be when it gets somewhere ahead.
#[derive(Clone, Copy, Debug)]
pub struct Course {
    /// The lane change being followed, in the `s` of the lane it is on.
    change: Option<LaneChange>,
    /// That lane's `s` at the vehicle's rear axle.
    s: f64,
    /// The searched lanes' `t` of that lane's centerline: zero when searching the
    /// lanes it follows, a lane's separation when searching the lanes beside them.
    shift: f64,
}

impl Course {
    pub fn new(change: Option<LaneChange>, s: f64, shift: f64) -> Self {
        Self { change, s, shift }
    }

    /// A vehicle's own planned path, along the lanes it follows.
    pub fn of(vehicle: &Vehicle) -> Self {
        Self::new(vehicle.lane_change, vehicle.lane_pos.s, 0.0)
    }

    /// The planned `t` on the searched lanes, `ahead` meters past the rear axle.
    fn t_at(&self, ahead: f64) -> f64 {
        self.shift + self.change.map_or(0.0, |c| c.offset_at(self.s + ahead))
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
/// A vehicle diverging from a lane after a lane change is passed if its trailing
/// rear corner leaves room on the side it is leaving: at least
/// [`PASSING_CLEARANCE`] between that corner and this vehicle's body where it will
/// be while passing it, along `course`. Any other record blocks (see [`Lateral`]).
#[allow(clippy::too_many_arguments)]
pub fn find_leader(
    network: &SimNetwork,
    occupants: &[Vec<Occupant>],
    id: VehicleId,
    vehicle: &Vehicle,
    lanes: &[LaneId],
    axle_s: f64,
    course: Course,
    ghosts: Ghosts,
    skip: Option<VehicleId>,
) -> Search {
    let half_width = 0.5 * vehicle.spec.width;
    // The stretch of the searched lanes this vehicle's body covers while passing
    // a corner `ahead` of its rear axle: from its front reaching the corner to its
    // rear clearing it, along its course. The cubic can bulge with a starting
    // slope, so the middle is sampled as well as the ends.
    let body = |ahead: f64| {
        let length = vehicle.spec.length;
        let ts = [0.0, 0.5, 1.0].map(|f| course.t_at(ahead + f * length));
        let (lo, hi) = ts.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &t| {
            (lo.min(t), hi.max(t))
        });
        [lo - half_width, hi + half_width]
    };
    let front = axle_s + vehicle.spec.length - vehicle.spec.rear_axle;
    let mut found = Search::default();
    // Start of the current lane, measured along `lanes` from the start of the first.
    let mut offset = 0.0;
    let mut after = axle_s;
    for &lane in lanes {
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
            if o.lateral.can_pass(body(offset + o.s - axle_s)) {
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

    #[test]
    fn in_lane_never_passes() {
        // Even with the follower a whole lane width to the side.
        assert!(!Lateral::InLane.can_pass([-5.0, -3.0]));
        assert!(!Lateral::InLane.can_pass([3.0, 5.0]));
    }

    #[test]
    fn diverging_passes_only_on_the_side_it_leaves() {
        let body = |t: f64| [t - 0.95, t + 0.95];
        // Leaving to the left, its right rear corner 1.6 m left of the centerline:
        // a follower on the centerline has 0.65 m to its right.
        let left = Lateral::DivergingLeft { trailing: 1.6 };
        assert!(left.can_pass(body(0.0)));
        assert!(!left.can_pass(body(0.2)), "less than the clearance");
        assert!(
            !left.can_pass(body(3.0)),
            "never on the side it is moving to"
        );
        let right = Lateral::DivergingRight { trailing: -1.6 };
        assert!(right.can_pass(body(0.0)));
        assert!(!right.can_pass(body(-0.2)), "less than the clearance");
        assert!(
            !right.can_pass(body(-3.0)),
            "never on the side it is moving to"
        );
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
                lateral: Lateral::InLane,
            })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
