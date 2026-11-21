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

use std::ops::ControlFlow;

use glam::DVec2;
use slotmap::SlotMap;

use crate::debug::PassCheck;
use crate::lane::LanePos;
use crate::models::acceleration;
use crate::models::lateral::LaneChange;
use crate::network::{LaneId, Side, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::{Vehicle, VehicleId};

/// The least room, in meters, a vehicle needs beside another to pass it.
const PASSING_CLEARANCE: f64 = 0.5;

/// How far behind the other vehicle's rear, in meters, a passing check puts the
/// searching vehicle's nose, so a check that fails is drawn short of overlapping.
const NOSE_GAP: f64 = 0.1;

/// A vehicle's record in one lane's occupant list.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Occupant {
    pub id: VehicleId,
    /// Lane coordinates of the vehicle's rearmost corner on this lane: the
    /// record's position, and a hint for the next projection.
    pub(crate) hint: LanePos,
    /// Whether the vehicle is in this lane, converging on it, diverging from it
    /// or indicating into it.
    pub lateral: Lateral,
    /// How long, in seconds, the vehicle has been in this lane's traffic: since
    /// it changed lanes into it, entered a merge onto it or joined it from
    /// beside. Advancing from one lane to the next carries it over, and an
    /// inserted vehicle starts settled.
    /// Leader terms relax with it (see [`acceleration::relaxation`]).
    #[cfg_attr(feature = "serde", serde(default = "settled"))]
    pub age: f64,
}

/// The age of a record from before records had one: long settled.
#[cfg(feature = "serde")]
fn settled() -> f64 {
    acceleration::SETTLED
}

impl Occupant {
    /// The smaller of the vehicle's rear corners' `s` on this lane.
    pub fn s(&self) -> f64 {
        self.hint.s
    }

    /// Whether this is the ghost of a vehicle indicating a change into this lane.
    pub fn is_ghost(&self) -> bool {
        self.lateral == Lateral::Ghost
    }

    /// Whether this is `id`'s record, not its ghost.
    pub fn is_record_of(&self, id: VehicleId) -> bool {
        self.id == id && !self.is_ghost()
    }
}

/// How a vehicle relates to a lane it has a record on, as followers there see it
/// (see `docs/multilane-car-following.md`).
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub enum Lateral {
    /// In the lane: following it, settled, or lingering past the end of it
    /// after advancing.
    InLane,
    /// Its course may yet bring it into the lane, or closer to it: changing lanes
    /// into it, merging into it from a lane that leads into the same one, or
    /// leaving it on a course that may come back. It turns diverging toward
    /// `side` once its `s` reaches `turn`, which is `f64::MAX` if it never does
    /// (see [`crate::following::classify`]); finite, so snapshots serialize.
    Converging { turn: f64, side: Side },
    /// Moving away from the lane sideways, toward this side, and never coming
    /// back to it.
    Diverging(Side),
    /// The ghost of a vehicle indicating a change into the lane.
    Ghost,
}

impl Lateral {
    /// Converging, until classified again.
    pub const CONVERGING: Self = Self::Converging { turn: f64::MAX, side: Side::Left };

    pub fn is_converging(self) -> bool {
        matches!(self, Self::Converging { .. })
    }

    pub fn is_diverging(self) -> bool {
        matches!(self, Self::Diverging(_))
    }
}

/// The side of a vehicle ahead a follower might pass it on, or `None` if it may
/// never pass it, from how each relates to the lane searched: `own` the
/// follower, `leader` the vehicle ahead.
///
/// The passing check is a snapshot taken as the follower reaches the other
/// vehicle's rear, so it is only sound when neither moves toward the other
/// sideways after that: then that moment is the tightest, and the gap only
/// grows. So only a diverging leader passed by a follower in the lane, a
/// diverging follower passing a leader in the lane, and two diverging to
/// opposite sides are checked; every other pair follows.
fn passing_side(own: Lateral, leader: Lateral) -> Option<Side> {
    match (own, leader) {
        (Lateral::InLane, Lateral::Diverging(side)) => Some(side.opposite()),
        (Lateral::Diverging(side), Lateral::InLane) => Some(side),
        (Lateral::Diverging(a), Lateral::Diverging(b)) if a != b => Some(a),
        _ => None,
    }
}

/// The corners of a body of `spec`'s size with its rear axle at `pos`, heading
/// along `tangent`: rear left, rear right, front right, front left.
pub(crate) fn body(spec: &crate::vehicle::VehicleSpec, pos: DVec2, tangent: DVec2) -> [DVec2; 4] {
    let left = 0.5 * spec.width * tangent.perp();
    let rear = pos - spec.rear_axle * tangent;
    let front = rear + spec.length * tangent;
    [rear + left, rear - left, front - left, front + left]
}

/// How much room a follower would have passing a vehicle ahead on `side` of it,
/// if it drove straight ahead from `pos`, its rear axle, heading along `tangent`:
/// the lateral distance from its body's near edge to the nearer of the other
/// vehicle's rear corners, and that corner. Negative if they overlap.
///
/// Only the rear corners are measured, which assumes the other vehicle is
/// straight or turned away from the follower. That holds for the pairs
/// [`passing_side`] lets through: one diverging is turned away, and one in the
/// lane is passed only by a follower diverging from it.
fn clearance(
    side: Side,
    pos: DVec2,
    tangent: DVec2,
    half_width: f64,
    corners: [DVec2; 2],
) -> (f64, DVec2) {
    // Positive to the follower's left.
    let lateral = corners.map(|c| (c - pos).dot(tangent.perp()));
    match side {
        // Passing on its left, so it is to the follower's right.
        Side::Left => {
            let i = if lateral[0] >= lateral[1] { 0 } else { 1 };
            (-half_width - lateral[i], corners[i])
        }
        Side::Right => {
            let i = if lateral[0] <= lateral[1] { 0 } else { 1 };
            (lateral[i] - half_width, corners[i])
        }
    }
}

impl Occupant {
    /// Projects a vehicle's rear corners onto `lane`, searching from `hint`, for a
    /// record that is `lateral` to it. A record's lateral status is fixed when it
    /// is made, from how the lanes lie, not from where the vehicle is: on a lane
    /// it is joining sideways, its corners start out level with the centerline,
    /// where their side says nothing.
    pub fn new(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
        lateral: Lateral,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, lateral).0
    }

    /// As [`Occupant::new`], as the ghost of a vehicle indicating a change into `lane`.
    pub fn ghost(
        network: &SimNetwork,
        lane: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
    ) -> Self {
        Self::project(network, lane, id, vehicle, hint, Lateral::Ghost).0
    }

    /// As [`Occupant::new`] or [`Occupant::ghost`], also saying where each rear
    /// corner is relative to the lane.
    fn project(
        network: &SimNetwork,
        lane_id: LaneId,
        id: VehicleId,
        vehicle: &Vehicle,
        hint: LanePos,
        lateral: Lateral,
    ) -> (Self, [Corner; 2]) {
        let geometry = network.lane(lane_id).geometry();
        let [rear_left, rear_right, ..] = vehicle.corners();
        let left = geometry.world_to_lane_near(rear_left, hint);
        let right = geometry.world_to_lane_near(rear_right, hint);
        let corner = |p: LanePos| Corner {
            past_end: p.s >= geometry.length(),
            beside: !geometry.between_edges(p),
        };
        let hint = if left.s <= right.s { left } else { right };
        let occupant = Self { id, hint, lateral, age: 0.0 };
        (occupant, [corner(left), corner(right)])
    }
}

/// Whether `point` is on `geometry`'s lane, between its ends and its edges,
/// projecting from `hint`.
pub(crate) fn inside(geometry: &crate::lane::LaneGeometry, point: DVec2, hint: LanePos) -> bool {
    let p = geometry.world_to_lane_near(point, hint);
    p.s >= 0.0 && p.s < geometry.length() && geometry.between_edges(p)
}

/// Where a rear corner is relative to a lane.
#[derive(Clone, Copy, Debug)]
struct Corner {
    past_end: bool,
    /// Outside the lane's edges, where it is along the lane.
    beside: bool,
}

/// Refreshes every occupant's `s`, drops those whose rear corners are both
/// outside their lane, and restores the order.
///
/// - Ghosts are refreshed but never dropped here.
/// - A record on the lane a vehicle follows is in the lane, or converging while
///   it changes lanes. Elsewhere, a converging record turns diverging once its
///   `s` reaches where its classification said it would.
/// - Only a diverging record is dropped for being beside its lane. Any record is
///   dropped once both rear corners are past the lane's end.
/// - Vehicles that lose a record on a lane they aren't following, by its end or
///   its side, have their ids pushed to `departed`, so their old side can be
///   checked once every list is up to date.
pub fn update(
    network: &SimNetwork,
    vehicles: &SlotMap<VehicleId, Vehicle>,
    occupants: &mut [Vec<Occupant>],
    departed: &mut Vec<VehicleId>,
    dt: f64,
) {
    for (i, list) in occupants.iter_mut().enumerate() {
        let lane_id = LaneId(i as u32);
        list.retain_mut(|occupant| {
            occupant.age += dt;
            let vehicle = &vehicles[occupant.id];
            let Occupant { id, hint, lateral, .. } = *occupant;
            let (updated, corners) =
                Occupant::project(network, lane_id, id, vehicle, hint, lateral);
            occupant.hint = updated.hint;
            let following = vehicle.path[0] == lane_id;
            match occupant.lateral {
                Lateral::Ghost => return true,
                _ if following => {
                    occupant.lateral = match vehicle.lane_change {
                        Some(_) => Lateral::CONVERGING,
                        None => Lateral::InLane,
                    };
                }
                Lateral::Converging { turn, side } if occupant.hint.s >= turn => {
                    occupant.lateral = Lateral::Diverging(side);
                }
                _ => {}
            }
            // Only a vehicle diverging from the lane is ever dropped for being
            // beside it: one converging may yet come back.
            let diverging = matches!(occupant.lateral, Lateral::Diverging(_));
            let outside = |c: &Corner| c.past_end || (c.beside && diverging);
            if !corners.iter().all(outside) {
                return true;
            }
            // Beside the lane, the whole body must be clear of it: where the
            // lane bends toward it, a vehicle leaving it points back across it,
            // and its front can still be in the lane after its rear has left.
            if diverging && !corners.iter().all(|c| c.past_end) {
                let geometry = network.lane(lane_id).geometry();
                let [.., front_right, front_left] = vehicle.corners();
                let in_lane = |c: DVec2| inside(geometry, c, occupant.hint);
                if in_lane(front_left) || in_lane(front_right) {
                    return true;
                }
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
/// is: vehicles rarely pass each other within a lane between steps.
fn insertion_sort(list: &mut [Occupant]) {
    for i in 1..list.len() {
        let mut j = i;
        while j > 0 && list[j - 1].s() > list[j].s() {
            list.swap(j - 1, j);
            j -= 1;
        }
    }
}

/// Which lanes a leader search runs along, relative to the searching vehicle's
/// path, how the vehicle relates to them, and where on the first it starts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Along {
    /// Its own path, following its lane change, if any: from its record on the
    /// first lane, by index in that lane's occupants.
    Path(usize),
    /// The lanes beside its path on this side, which it is moving out of: the
    /// old-side search, while it straddles. From its record on the first, by
    /// index.
    Leaving(Side, usize),
    /// The lanes beside its path on this side, as if it were already settled on
    /// their centerlines: weighing up a lane change. From its rear, projected onto
    /// the first.
    Into(Side),
    /// Its own path, as Path, for a vehicle not yet inserted, so with no record:
    /// from where its rear is.
    Inserting,
}

impl Along {
    /// The `k`th lane searched: its own path's `k`th lane, or the lane beside it.
    /// `None` past the end of its path, or, beside it, where there is no lane
    /// there.
    pub fn lane(self, network: &SimNetwork, vehicle: &Vehicle, k: usize) -> Option<LaneId> {
        let own = *vehicle.path.get(k)?;
        match self {
            Along::Path(_) | Along::Inserting => Some(own),
            Along::Leaving(side, _) | Along::Into(side) => {
                network.lane(own).adj(side).map(|n| n.lane)
            }
        }
    }

    /// The lane change it follows: its own on its path, and none settled beside it.
    fn change(self, vehicle: &Vehicle) -> Option<LaneChange> {
        match self {
            Along::Path(_) | Along::Leaving(..) | Along::Inserting => vehicle.lane_change,
            Along::Into(_) => None,
        }
    }

    /// Its rear axle's position and heading, as a unit tangent, `ahead` meters
    /// further along where it plans to be, from `axle_s` along the lanes it
    /// follows: its path, with its lane change's cubic, or, settled beside it, the
    /// centerlines there. The heading follows the cubic's slope as well as the
    /// lanes' bends.
    pub(crate) fn pose(
        self,
        network: &SimNetwork,
        vehicle: &Vehicle,
        axle_s: f64,
        ahead: f64,
    ) -> (DVec2, DVec2) {
        let change = self.change(vehicle);
        let lanes = (0..).map_while(|k| match self {
            Along::Into(_) => self.lane(network, vehicle, k),
            _ => vehicle.path.get(k).copied(),
        });
        let point = |at: f64| {
            let offset = change.map_or(0.0, |c| c.offset_at(at));
            network.point_along(lanes.clone(), at, offset)
        };
        let at = axle_s + ahead.max(0.0);
        let pos = point(at);
        (pos, (point(at + 1.0) - pos).normalize())
    }

    /// Where it will be, as its rear axle's position and heading, when its nose
    /// is [`NOSE_GAP`] short of `point`, another vehicle's rearmost corner, at `s`
    /// on `lane`, the `k`th lane searched, with its rear axle at `axle_s` along the
    /// lanes it follows (see [`Along::pose`]).
    ///
    /// Along the lanes it follows, `gap`, how far its rear axle travels before its
    /// nose reaches `s`, is exact. Leaving the lanes searched, distances there
    /// differ from its own, so `point` is projected onto its own `k`th lane
    /// instead.
    ///
    /// If its nose is already there, there is no earlier point to check from, so
    /// it is where it is `now` rather than where its course would put it, which
    /// pure pursuit lags behind.
    #[allow(clippy::too_many_arguments)]
    fn reach(
        self,
        network: &SimNetwork,
        vehicle: &Vehicle,
        axle_s: f64,
        k: usize,
        lane: LaneId,
        gap: f64,
        point: DVec2,
        s: f64,
        now: (DVec2, DVec2),
    ) -> (DVec2, DVec2) {
        let nose = vehicle.spec.length - vehicle.spec.rear_axle;
        let ahead = match (self, vehicle.path.get(k)) {
            (Along::Leaving(..), Some(&own)) => {
                let (own_lane, searched) = (network.lane(own), network.lane(lane));
                let before: f64 =
                    vehicle.path[..k].iter().map(|&l| network.lane(l).geometry().length()).sum();
                // Adjacent lanes start and end together, so the same fraction is a good hint.
                let hint = own_lane.geometry().lane_pos(
                    s / searched.geometry().length() * own_lane.geometry().length(),
                    0.0,
                );
                before + own_lane.geometry().world_to_lane_near(point, hint).s - nose - axle_s
            }
            _ => gap,
        };
        match ahead - NOSE_GAP {
            ahead if ahead > 0.0 => self.pose(network, vehicle, axle_s, ahead),
            _ => now,
        }
    }
}

/// Where a vehicle's rear axle is along `lane`, a lane beside the one it follows,
/// projected onto it.
pub fn axle_beside(network: &SimNetwork, vehicle: &Vehicle, lane: LaneId) -> f64 {
    let (own, beside) = (network.lane(vehicle.path[0]), network.lane(lane));
    // Adjacent lanes start and end together, so the same fraction along it is a
    // good hint.
    let fraction = vehicle.lane_pos.s / own.geometry().length();
    let hint = beside.geometry().lane_pos(fraction * beside.geometry().length(), 0.0);
    beside.geometry().world_to_lane_near(vehicle.pos, hint).s
}

/// Something ahead that a leader search found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ahead {
    /// A vehicle it can't pass.
    Vehicle(VehicleId),
    /// The ghost of a vehicle indicating a change into the lane. What it means is
    /// the caller's choice: weighing up a change, it is a leader, since a vehicle
    /// already indicating into the gap ahead goes first; otherwise the vehicle
    /// may make room for the nearest one, if no leader comes first.
    Ghost(VehicleId),
    /// The end of the lanes it knows it will take, a stop line: a stationary
    /// virtual leader, unless it leaves the network there.
    PathEnd,
}

/// What a leader search finds, in the order it meets them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Found {
    /// Something ahead to react to, with the bumper-to-bumper gap to it, and the
    /// share of its time headway the searching vehicle keeps to it, as
    /// relaxation has it (see [`acceleration::relaxation`]): each is its own
    /// term in the longitudinal model, whose minimum binds.
    Ahead(Ahead, f64, f64),
    /// A check for passing a vehicle ahead, passed or not. One that fails is
    /// followed by that vehicle, as [`Found::Ahead`].
    Pass(PassCheck),
}

/// Searches for what is ahead of a vehicle, `along` some lanes, within its
/// [`acceleration::horizon`] of its front, that it must react to, calling `f`
/// with each thing in order of distance, nearest first. `f` returns whether to go
/// on: a caller can stop as soon as it has what it needs, and the search then
/// returns what `f` broke with. See [`for_each_leader`] for a caller that wants it all.
///
/// It measures from the vehicle's rear on the first lane searched: its record's
/// `s`, if it has one there, or else where its rear is, projected onto that lane
/// if it is beside its path. Its front is its length ahead of that. On that
/// lane, only records from just after its own count, or, with none, those at or
/// ahead of its rear: [`find_follower`] takes those behind, so between them they
/// find every vehicle, one alongside included. On later lanes, every occupant
/// counts.
///
/// The search doesn't know which vehicle is searching. On a loop shorter than
/// its horizon, a vehicle can find its own record ahead, which is geometrically
/// true; and searching a lane it is indicating into, it finds its own ghost,
/// which the caller passes over.
///
/// Leaving the lanes searched (see [`Along::Leaving`]), the horizon is where its
/// lane change ends, if that is nearer: after that the vehicle is wholly on the
/// lanes it follows, so nothing it would only reach later can be in its way
/// there. Beside its path, the end of the lanes searched isn't a stop line: they
/// end where the lanes beside its path do.
///
/// A vehicle diverging from a lane searched passes any record there that its
/// body will be wholly out of the lane by the time it reaches: it never comes
/// back. Some records may also be passed, by the rules in [`passing_side`]. One
/// of those is passed if, with this vehicle where it plans to be (see [`Along::pose`]) as its
/// front reaches that record's rear, driving straight ahead would leave at least
/// [`PASSING_CLEARANCE`] beside the other vehicle's rear corners. That reads the
/// other vehicle's corners from the state's vehicles, as does working out what a
/// leader shields. Each such check is passed to `f`, as [`Found::Pass`].
///
/// Which records it needs: a vehicle in the lane that this one can't pass, B,
/// blocks the lane's whole width, so this one can't reach anything beyond it
/// that B can't, and whatever in the lane beyond constrains B reaches this one
/// through B. So in-lane vehicles beyond B are skipped, on B's lane and on the
/// lanes after it, as far as B's path goes along the lanes searched (see
/// [`shields`]): the first lane B doesn't cover is a high-water mark,
/// the furthest any leader has reached. Ghosts only partly block a lane, and B
/// may pass a vehicle diverging from it that this one can't, so neither is ever
/// skipped, and neither shields anything. A vehicle converging onto the lane
/// after a lane change is in it, so it shields like any other.
///
/// Past the end of its path, it doesn't know where it is going, so it must stop
/// there, unless it leaves the network: the last thing found, since nothing
/// lies beyond.
pub fn find_leaders<B>(
    state: &SimState,
    vehicle: &Vehicle,
    along: Along,
    mut f: impl FnMut(Found) -> ControlFlow<B>,
) -> ControlFlow<B> {
    let network = &state.network;
    let lane_at = |k: usize| along.lane(network, vehicle, k);
    let Some(first) = lane_at(0) else {
        return ControlFlow::Continue(());
    };
    let list = |lane: LaneId| state.occupants[lane.0 as usize].as_slice();
    let after = |rear: f64| list(first).partition_point(|o| o.s() < rear);
    let rear_axle = vehicle.spec.rear_axle;
    // Where it starts on the first lane, its rear there, and its rear axle along
    // the lanes it follows (see `Along::pose`).
    let (start, rear, axle_s) = match along {
        Along::Path(own) | Along::Leaving(_, own) => {
            (own + 1, list(first)[own].s(), vehicle.lane_pos.s)
        }
        Along::Inserting => {
            let rear = vehicle.lane_pos.s - rear_axle;
            (after(rear), rear, vehicle.lane_pos.s)
        }
        Along::Into(_) => {
            let axle_s = axle_beside(network, vehicle, first);
            (after(axle_s - rear_axle), axle_s - rear_axle, axle_s)
        }
    };
    let front = rear + vehicle.spec.length;
    // Its own record on the first lane, if it searches from one.
    let own = match along {
        Along::Path(own) | Along::Leaving(_, own) => Some(list(first)[own]),
        _ => None,
    };
    // How long it has been in the lanes searched, which its leader terms relax
    // with. Weighing up a change or inserting, it is checked unrelaxed: a
    // driver doesn't change lanes counting on the gap it will settle for after.
    let own_age = own.map(|own| own.age);
    let horizon = acceleration::horizon(&vehicle.spec, vehicle.speed);
    // Wanting to change lanes, along its own lanes it looks as far as it keeps
    // room to get halfway through a change (see `lane_changing::halfway`).
    let horizon = match along {
        Along::Path(_) if crate::lane_changing::wants_change(vehicle) => horizon.max(
            vehicle.spec.min_gap
                + crate::lane_changing::halfway_from_rest(network, vehicle)
                + crate::lane_changing::SPARE_ROOM,
        ),
        _ => horizon,
    };
    let horizon = match (along, vehicle.lane_change) {
        // A gap is how far the rear axle travels before the nose reaches a rear.
        // Leaving its old lane, it can only reach a vehicle there while it is
        // still in it, which it is no longer once its lane change ends.
        (Along::Leaving(..), change) => {
            change.map_or(0.0, |c| (c.end() - axle_s).max(0.0)).min(horizon)
        }
        _ => horizon,
    };
    // How it relates to the `k`th lane searched: on its own path, in the lane or
    // converging while it changes lanes; settled beside it, in the lane; leaving
    // its old lanes, as its records there say, and converging where it has none.
    let own_lateral = |k: usize, lane: LaneId| match (along, own) {
        (Along::Leaving(..), Some(own)) if k == 0 => own.lateral,
        (Along::Leaving(..), Some(own)) => list(lane)
            .iter()
            .find(|o| o.is_record_of(own.id))
            .map_or(Lateral::CONVERGING, |o| o.lateral),
        (Along::Into(_), _) => Lateral::InLane,
        _ => match vehicle.lane_change {
            Some(_) => Lateral::CONVERGING,
            None => Lateral::InLane,
        },
    };
    // Settled beside its path, it is nowhere yet, so where it is now is where it
    // would be.
    let now = match along {
        Along::Into(_) => along.pose(network, vehicle, axle_s, 0.0),
        _ => (vehicle.pos, DVec2::from_angle(vehicle.heading)),
    };

    // Start of the `k`th lane, along the lanes searched from the start of the first.
    let mut offset = 0.0;
    // The first lane no leader in the lane shields: on the lanes before it, only
    // ghosts and vehicles diverging from them still count.
    let mut covered = 0;
    let mut k = 0;
    while let Some(lane) = lane_at(k) {
        if offset - front > horizon {
            return ControlFlow::Continue(());
        }
        let records = &list(lane)[if k == 0 { start } else { 0 }..];
        for o in records {
            let gap = offset + o.s() - front;
            if gap > horizon {
                return ControlFlow::Continue(());
            }
            if o.is_ghost() {
                f(Found::Ahead(Ahead::Ghost(o.id), gap, 1.0))?;
                continue;
            }
            // Converging on the lane, it can't be passed in it, whatever it
            // covers beyond, so what lies beyond it there can't be reached first.
            let in_lane = matches!(o.lateral, Lateral::InLane | Lateral::Converging { .. });
            if in_lane && k < covered {
                continue;
            }
            let mine = own_lateral(k, lane);
            // Diverging, it may pass anything it is out of the lane by the time
            // it reaches, since it never comes back: so the check is made for
            // any leader, on the side it leaves toward, if the table has none.
            let check = match (passing_side(mine, o.lateral), mine) {
                (Some(side), _) => Some((side, true)),
                (None, Lateral::Diverging(side)) => Some((side, false)),
                _ => None,
            };
            if let Some((side, by_table)) = check {
                let other = &state.vehicles[o.id];
                let geometry = network.lane(lane).geometry();
                // The record sits at the other vehicle's rearmost corner.
                let [rear_left, rear_right, ..] = other.corners();
                let (rearmost, corners) = (geometry.lane_to_world(o.hint), [rear_left, rear_right]);
                let (pos, tangent) =
                    along.reach(network, vehicle, axle_s, k, lane, gap, rearmost, o.s(), now);
                let half_width = 0.5 * vehicle.spec.width;
                let (clearance, corner) = clearance(side, pos, tangent, half_width, corners);
                let gone = || {
                    let body = body(&vehicle.spec, pos, tangent);
                    !body.iter().any(|&c| inside(geometry, c, o.hint))
                };
                let passed =
                    (by_table && clearance >= PASSING_CLEARANCE) || (mine.is_diverging() && gone());
                let leader = o.id;
                f(Found::Pass(PassCheck {
                    leader,
                    pos,
                    tangent,
                    side,
                    corner,
                    clearance,
                    passed,
                }))?;
                if passed {
                    continue;
                }
            }
            let headway = match own_age {
                Some(own) => acceleration::relaxation(own.min(o.age)),
                None => 1.0,
            };
            f(Found::Ahead(Ahead::Vehicle(o.id), gap, headway))?;
            if in_lane {
                covered = covered.max(shields(lane_at, k, &state.vehicles[o.id].path));
            }
        }
        offset += network.lane(lane).geometry().length();
        k += 1;
    }
    // Past the end of its path, with nowhere it knows to go.
    let own_path = matches!(along, Along::Path(_) | Along::Inserting);
    let ends = |&l: &LaneId| !network.lane(l).successors().is_empty();
    let gap = offset - front;
    if own_path && vehicle.path.last().is_some_and(ends) && gap <= horizon {
        f(Found::Ahead(Ahead::PathEnd, gap, 1.0))?;
    }
    ControlFlow::Continue(())
}

/// As [`find_leaders`], for a caller that wants everything it finds.
pub fn for_each_leader(
    state: &SimState,
    vehicle: &Vehicle,
    along: Along,
    mut f: impl FnMut(Found),
) {
    let ControlFlow::Continue(()) = find_leaders(state, vehicle, along, |found| {
        f(found);
        ControlFlow::<!>::Continue(())
    });
}

/// How far along the lanes searched, `lane_at` each by index, a vehicle in the
/// lane on the `k`th of them shields, with `path` its own path: the index of the
/// first lane it doesn't. It shields the rest of lane `k`, and the lanes after it
/// as far as its own path goes along them: where its path parts from them, or
/// ends, what lies beyond never constrains it.
///
/// Its path starts at the lane it follows: lane `k`, or, while it lingers past
/// the end of lane `k` after advancing, usually the next. Anything else, it
/// shields lane `k` alone.
fn shields(lane_at: impl Fn(usize) -> Option<LaneId>, k: usize, path: &[LaneId]) -> usize {
    let j = match path.first() {
        Some(&first) if lane_at(k) == Some(first) => k,
        Some(&first) if lane_at(k + 1) == Some(first) => k + 1,
        _ => return k + 1,
    };
    let shared = path.iter().enumerate().take_while(|&(n, &l)| lane_at(j + n) == Some(l));
    (j + shared.count()).max(k + 1)
}

/// The index of a vehicle's record, not its ghost, in `lane`'s occupants: where a
/// search from it along its path starts (see [`Along::Path`]).
pub fn record_index(state: &SimState, lane: LaneId, id: VehicleId) -> usize {
    state.occupants[lane.0 as usize]
        .iter()
        .position(|o| o.is_record_of(id))
        .expect("a vehicle has a record on the lane it follows")
}

/// The nearest vehicle behind `s` on `lane` that would end up behind a vehicle
/// there, and the gap from its front to `s`: searching back along `lane`'s
/// occupants, then the lanes leading into it.
///
/// Every vehicle on `lane` itself counts, whether settled, moving in or moving
/// out, since any of them can be in the way; on the lanes leading into it, one
/// counts if `lane` is on its path. Its leader search ends where its path does,
/// so one that doesn't know it will reach `lane` never sees what is there. That
/// bounds how far back the search goes: no vehicle's path reaches more than
/// `max_reach` beyond its front, so one further back can't have `lane` on it.
/// Ghosts and `skip` are passed over.
///
/// The first vehicle found is the follower if `s` is within its [`acceleration::horizon`]:
/// further back, its own leader search wouldn't reach that far, and nothing
/// behind it would either, since it is in the way.
///
/// It splits the lane at `s` with [`find_leaders`]: a record whose rear is at or
/// ahead of `s` is a leader, and one whose rear is behind is a follower, so a
/// vehicle alongside is found by one or the other, with a gap of zero or less.
///
/// This is the backward search: intermittent decisions and insertion use it to
/// find the vehicles they would affect, while every per-frame search looks only
/// forward.
pub fn find_follower(
    state: &SimState,
    max_reach: f64,
    lane: LaneId,
    s: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    behind(state, max_reach, lane, lane, s, 0.0, skip)
}

/// [`find_follower`]'s search of `lane`, from `s` back, where `s` is `travelled`
/// back from the point searched from, on `target`.
fn behind(
    state: &SimState,
    max_reach: f64,
    target: LaneId,
    lane: LaneId,
    s: f64,
    travelled: f64,
    skip: VehicleId,
) -> Option<(VehicleId, f64)> {
    let SimState { network, vehicles, occupants, .. } = state;
    let list = &occupants[lane.0 as usize];
    let end = list.partition_point(|o| o.s() < s);
    let counts = |o: &Occupant| {
        let through = || vehicles[o.id].path.contains(&target);
        !o.is_ghost() && o.id != skip && (lane == target || through())
    };
    if let Some(o) = list[..end].iter().rev().find(|o| counts(o)) {
        let follower = &vehicles[o.id];
        let gap = travelled + s - (o.s() + follower.spec.length);
        let reach = acceleration::horizon(&follower.spec, follower.speed);
        return (gap <= reach).then_some((o.id, gap));
    }
    // None on this lane: on to the ends of the lanes leading into it, as far back
    // as a vehicle could still have `target` on its path. On a loop a lane leads
    // into itself, which that ends too.
    let travelled = travelled + s;
    if travelled > max_reach {
        return None;
    }
    network
        .lane(lane)
        .predecessors()
        .iter()
        .filter_map(|&p| {
            let length = network.lane(p).geometry().length();
            behind(state, max_reach, target, p, length, travelled, skip)
        })
        .min_by(|a, b| a.1.total_cmp(&b.1))
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::debug::NoopSink;
    use crate::network::Lane;
    use crate::simulation::Simulation;
    use crate::vehicle::{Placement, VehicleSpec};

    #[test]
    fn only_pairs_that_move_apart_sideways_are_checked() {
        use Lateral::{Diverging, InLane};
        const CONVERGING: Lateral = Lateral::CONVERGING;
        use Side::{Left, Right};
        // Neither diverging: never.
        for (own, leader) in [(InLane, InLane), (InLane, CONVERGING), (CONVERGING, InLane)] {
            assert_eq!(passing_side(own, leader), None);
        }
        // A converging follower never passes, and a converging leader is never
        // passed.
        assert_eq!(passing_side(CONVERGING, Diverging(Left)), None);
        assert_eq!(passing_side(Diverging(Left), CONVERGING), None);
        // A diverging leader is passed on the side it is leaving.
        assert_eq!(passing_side(InLane, Diverging(Left)), Some(Right));
        // A diverging follower passes on the side it is leaving toward.
        assert_eq!(passing_side(Diverging(Right), InLane), Some(Right));
        // Diverging apart, yes; the same way, never.
        assert_eq!(passing_side(Diverging(Left), Diverging(Right)), Some(Left));
        assert_eq!(passing_side(Diverging(Left), Diverging(Left)), None);
    }

    #[test]
    fn clearance_is_measured_across_the_followers_heading() {
        let half_width = 0.95;
        // Heading along x, with the other car's rear corners 2.0 and 3.9 m to its
        // left: passing on that car's right leaves 1.05 m.
        let corners = [DVec2::new(5.0, 3.9), DVec2::new(5.0, 2.0)];
        let (room, corner) = clearance(Side::Right, DVec2::ZERO, DVec2::X, half_width, corners);
        assert!((room - 1.05).abs() < 1e-12);
        assert_eq!(corner, corners[1]);
        // Passing on its left instead would overlap it.
        assert!(clearance(Side::Left, DVec2::ZERO, DVec2::X, half_width, corners).0 < 0.0);
        // Heading 45 degrees to the left, toward it: much less room.
        let toward = DVec2::new(1.0, 1.0).normalize();
        let (room, _) = clearance(Side::Right, DVec2::ZERO, toward, half_width, corners);
        assert!(room < 0.0, "room {room}");
    }

    /// A car with its rear axle `s` along `lane` of `network`, not changing lanes.
    fn car_at(network: &SimNetwork, lane: u32, s: f64) -> Vehicle {
        let spec = VehicleSpec {
            length: 4.6,
            width: 1.9,
            max_speed: 30.0,
            speed_factor: 1.0,
            max_accel: 1.0,
            comfy_decel: 1.5,
            time_headway: 1.0,
            min_gap: 1.5,
            rear_axle: 1.0,
            front_axle: 3.7,
            courtesy: 1.0,
            politeness: 0.2,
            assertiveness: 0.0,
            accel_noise: 0.0,
        };
        let mut sim = Simulation::new(network.clone(), NoopSink);
        let id = sim.insert(
            spec,
            Placement {
                lane: LaneId(lane),
                s,
                speed: 10.0,
                route: crate::models::routing::Route::default(),
            },
        );
        sim.get(id).unwrap().clone()
    }

    #[test]
    fn a_vehicle_plans_to_follow_its_cubic() {
        // Straight along x, changing from 3.5 m left of the centerline back onto
        // it, from rest, as sharply as it can steer: halfway, it is 1.75 m left
        // and heading right of x.
        let lane = Lane::new(&[DVec2::ZERO, DVec2::new(100.0, 0.0)], 3.5, 14.0).unwrap();
        let network = SimNetwork::new(vec![lane]);
        let mut vehicle = car_at(&network, 0, 0.0);
        let change = LaneChange::new(0.0, 3.5, 0.0, 0.0, 2.7);
        vehicle.lane_change = Some(change);
        let along = Along::Path(0);
        let half = 0.5 * change.length;
        let (pos, tangent) = along.pose(&network, &vehicle, 0.0, half);
        assert!((pos - DVec2::new(half, 1.75)).length() < 1e-9, "at {pos}");
        assert!(tangent.y < -0.2, "heading {tangent}");
        // Not changing, it heads straight along the centerline.
        vehicle.lane_change = None;
        let (pos, tangent) = along.pose(&network, &vehicle, 0.0, 7.5);
        assert!((pos - DVec2::new(7.5, 0.0)).length() < 1e-9);
        assert!((tangent - DVec2::X).length() < 1e-9);
    }

    #[test]
    fn settled_into_the_lane_beside_it_plans_to_follow_its_centerline() {
        // On lane 1 of the bend, weighing up lane 0 on its left: as if already
        // there, on lane 0's centerline, with no cubic and not moving across.
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let along = Along::Into(Side::Left);
        assert_eq!(along.lane(&network, &vehicle, 0), Some(LaneId(0)));
        let axle_s = axle_beside(&network, &vehicle, LaneId(0));
        let (pos, _) = along.pose(&network, &vehicle, axle_s, 20.0);
        let inside = network.lane(LaneId(0));
        let on = inside
            .geometry()
            .world_to_lane_near(pos, inside.geometry().lane_pos(axle_s + 20.0, 0.0));
        assert!(on.t.abs() < 1e-6, "{on:?}");
        assert!((on.s - (axle_s + 20.0)).abs() < 1e-6, "{on:?}");
    }

    /// Two concentric half-circle lanes, curving left: lane 0 inside, of radius
    /// 150 m, and lane 1 outside it.
    fn bend() -> SimNetwork {
        let arc = |radius: f64| {
            let points: Vec<_> = (0..=400)
                .map(|i| {
                    let a = std::f64::consts::PI * i as f64 / 400.0 - std::f64::consts::FRAC_PI_2;
                    radius * DVec2::new(a.cos(), a.sin())
                })
                .collect();
            Lane::new(&points, 3.5, 14.0).unwrap()
        };
        let mut network = SimNetwork::new(vec![arc(150.0), arc(153.5)]);
        network.set_adjacent(LaneId(0), LaneId(1), true, true);
        network
    }

    /// Where a predicted rear axle at `pos`, heading along `tangent`, puts its
    /// nose, `nose` ahead, as `s` along `lane`.
    fn nose_along(
        network: &SimNetwork,
        lane: LaneId,
        pos: DVec2,
        tangent: DVec2,
        nose: f64,
        near: f64,
    ) -> f64 {
        let lane = network.lane(lane);
        lane.geometry()
            .world_to_lane_near(pos + tangent * nose, lane.geometry().lane_pos(near, 0.0))
            .s
    }

    /// The test car's nose, ahead of its rear axle.
    const NOSE: f64 = 3.6;
    /// Unused: `reach` shouldn't fall back to where the vehicle is.
    const NOWHERE: (DVec2, DVec2) = (DVec2::new(1e9, 1e9), DVec2::X);

    #[test]
    fn on_its_own_lane_the_nose_stops_just_short_of_the_other_rear() {
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let lane = network.lane(LaneId(1));
        // A rear corner 60 m along, off the centerline.
        let corner = lane.geometry().lane_to_world(lane.geometry().lane_pos(60.0, -0.9));
        let gap = 60.0 - (10.0 + NOSE);
        let (pos, tangent) = Along::Path(0).reach(
            &network,
            &vehicle,
            10.0,
            0,
            LaneId(1),
            gap,
            corner,
            60.0,
            NOWHERE,
        );
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, 60.0);
        assert!((nose - (60.0 - NOSE_GAP)).abs() < 0.02, "nose at {nose}");
    }

    #[test]
    fn on_the_old_side_the_other_rear_is_projected_onto_its_own_lane() {
        // Leaving lane 0 for lane 1: lane 0 is shorter round the bend, so its
        // distances aren't lane 1's, and the gap is ignored.
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let beside = network.lane(LaneId(0));
        let corner = beside.geometry().lane_to_world(beside.geometry().lane_pos(58.0, -1.2));
        let along = Along::Leaving(Side::Left, 0);
        let (pos, tangent) = along.reach(
            &network,
            &vehicle,
            10.0,
            0,
            LaneId(0),
            999.0,
            corner,
            58.0,
            NOWHERE,
        );
        let own = network.lane(LaneId(1));
        let target =
            own.geometry().world_to_lane_near(corner, own.geometry().lane_pos(58.0, 0.0)).s;
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, target);
        assert!(
            (nose - (target - NOSE_GAP)).abs() < 0.02,
            "nose at {nose}, corner at {target}"
        );
    }

    #[test]
    fn mid_change_the_nose_still_stops_short() {
        // Changing onto lane 1 from lane 0, so heading across the lane: the nose,
        // ahead along that heading, falls a little short, never past.
        let network = bend();
        let mut vehicle = car_at(&network, 1, 10.0);
        vehicle.lane_change = Some(LaneChange::new(10.0, 3.5, 0.0, 0.0, 2.7));
        let lane = network.lane(LaneId(1));
        let corner = lane.geometry().lane_to_world(lane.geometry().lane_pos(25.0, 0.0));
        let gap = 25.0 - (10.0 + NOSE);
        let (pos, tangent) = Along::Path(0).reach(
            &network,
            &vehicle,
            10.0,
            0,
            LaneId(1),
            gap,
            corner,
            25.0,
            NOWHERE,
        );
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, 25.0);
        assert!(
            nose < 25.0 && nose > 25.0 - NOSE_GAP - 0.3,
            "nose at {nose}"
        );
    }

    #[test]
    fn already_alongside_it_is_checked_where_it_is_now() {
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let lane = network.lane(LaneId(1));
        let now = (DVec2::new(1.0, 2.0), DVec2::Y);
        // The other rear is behind its nose already, and just short of it.
        for s in [11.0, 13.0, 10.0 + NOSE + 0.5 * NOSE_GAP] {
            let corner = lane.geometry().lane_to_world(lane.geometry().lane_pos(s, 0.0));
            let gap = s - (10.0 + NOSE);
            let reached =
                Along::Path(0).reach(&network, &vehicle, 10.0, 0, LaneId(1), gap, corner, s, now);
            assert_eq!(reached, now, "rear at {s}");
        }
    }

    #[test]
    fn insertion_sort_orders_by_s() {
        let mut ids = SlotMap::<VehicleId, ()>::with_key();
        let lane = Lane::new(&[glam::DVec2::ZERO, glam::DVec2::X * 10.0], 3.5, 14.0).unwrap();
        let mut list: Vec<_> = [3.0, 1.0, 2.0, 0.5]
            .into_iter()
            .map(|s| Occupant {
                id: ids.insert(()),
                hint: lane.geometry().lane_pos(s, 0.0),
                lateral: Lateral::InLane,
                age: 0.0,
            })
            .collect();
        insertion_sort(&mut list);
        let order: Vec<_> = list.iter().map(|o| o.s()).collect();
        assert_eq!(order, [0.5, 1.0, 2.0, 3.0]);
    }
}
