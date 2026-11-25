//! The pass check: whether a vehicle may pass another it finds ahead, from
//! where it plans to be as its front reaches the other's rear.

use glam::DVec2;

use super::leaders::Along;
use super::{Lateral, Occupant, inside};
use crate::debug::PassCheck;
use crate::motion::lateral::LaneChange;
use crate::network::{LaneId, Side, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::Vehicle;

/// The least room, in meters, a vehicle needs beside another to pass it.
pub const PASSING_CLEARANCE: f64 = 0.5;

/// The pass check (see "Pass check" in `docs/multilane-car-following.md`):
/// whether `vehicle`, searching `along` some lanes with its rear axle at
/// `axle_s` along the lanes it follows and its pose `now`, may pass the vehicle
/// whose record `o` it found `gap` ahead on `lane`, the `k`th lane searched,
/// where it is itself `mine` to that lane. `None` where no check is made, and
/// the other vehicle is simply followed.
///
/// A pair [`passing_side`] lets through is checked: with the follower where it
/// plans to be as its front reaches the other's rear (see [`Along::pose`]),
/// driving straight ahead must leave at least [`PASSING_CLEARANCE`] beside the
/// other vehicle's rear corners. A follower diverging from the lane passes any
/// record its body will be wholly out of the lane by then, since it never
/// comes back, so it is checked against every leader, on the side it leaves
/// toward.
#[allow(clippy::too_many_arguments)]
pub(super) fn check(
    state: &SimState,
    vehicle: &Vehicle,
    along: Along,
    axle_s: f64,
    now: (DVec2, DVec2),
    k: usize,
    lane: LaneId,
    gap: f64,
    o: &Occupant,
    mine: Lateral,
) -> Option<PassCheck> {
    let (side, by_table) = match (passing_side(mine, o.lateral), mine) {
        (Some(side), _) => (side, true),
        (None, Lateral::Diverging(side)) => (side, false),
        _ => return None,
    };
    let network = &state.network;
    let other = &state.vehicles[o.id];
    let geometry = network.lane(lane).geometry();
    // The record sits at the other vehicle's rearmost corner.
    let [rear_left, rear_right, ..] = other.corners();
    let (rearmost, corners) = (geometry.lane_to_world(o.hint), [rear_left, rear_right]);
    let (pos, tangent) = along.reach(network, vehicle, axle_s, k, lane, gap, rearmost, o.s(), now);
    let half_width = 0.5 * vehicle.spec.width;
    let (clearance, corner) = clearance(side, pos, tangent, half_width, corners);
    let gone = || {
        let body = body(&vehicle.spec, pos, tangent);
        !body.iter().any(|&c| inside(geometry, c, o.hint))
    };
    let passed = (by_table && clearance >= PASSING_CLEARANCE) || (mine.is_diverging() && gone());
    Some(PassCheck { leader: o.id, pos, tangent, side, corner, clearance, passed })
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

impl Along {
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
        let at = axle_s + ahead.max(0.0);
        let (offset, slope) = change.map_or((0.0, 0.0), |c| (c.offset_at(at), c.slope_at(at)));
        network.pose_along(lanes, at, offset, slope)
    }

    /// Where it will be, as its rear axle's position and heading, when its nose
    /// reaches `point`, another vehicle's rearmost corner, at `s`
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
    /// its rear axle, trailing its front, lags behind.
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
        match ahead > 0.0 {
            true => self.pose(network, vehicle, axle_s, ahead),
            false => now,
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::network::Lane;
    use crate::query::axle_beside;
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
            max_accel: 1.0,
            comfy_decel: 1.5,
            time_headway: 1.0,
            min_gap: 1.5,
            rear_axle: 1.0,
            front_axle: 3.7,
            class: crate::VehicleClass::Car,
            rail: false,
        };
        let mut sim = crate::simulation::test::new_sim(network.clone());
        let id = sim.insert(
            spec,
            crate::simulation::test::driver(),
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
        let lane = Lane::new(
            &[DVec2::ZERO, DVec2::new(100.0, 0.0)],
            3.5,
            14.0,
            crate::simulation::test::alone(),
        )
        .unwrap();
        let network = SimNetwork::new(vec![lane]);
        let mut vehicle = car_at(&network, 0, 0.0);
        let change = LaneChange::new(0.0, 3.5, 0.0, 0.0);
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
            Lane::new(&points, 3.5, 14.0, crate::simulation::test::alone()).unwrap()
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
        assert!((nose - 60.0).abs() < 0.02, "nose at {nose}");
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
            (nose - target).abs() < 0.02,
            "nose at {nose}, corner at {target}"
        );
    }

    #[test]
    fn mid_change_the_nose_still_stops_short() {
        // Changing onto lane 1 from lane 0, so heading across the lane: the nose,
        // ahead along that heading, falls a little short, never past.
        let network = bend();
        let mut vehicle = car_at(&network, 1, 10.0);
        vehicle.lane_change = Some(LaneChange::new(10.0, 3.5, 0.0, 0.0));
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
        assert!(nose <= 25.0 + 1e-9 && nose > 25.0 - 0.3, "nose at {nose}");
    }

    #[test]
    fn already_alongside_it_is_checked_where_it_is_now() {
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let lane = network.lane(LaneId(1));
        let now = (DVec2::new(1.0, 2.0), DVec2::Y);
        // The other rear is behind its nose already, or level with it.
        for s in [11.0, 13.0, 10.0 + NOSE] {
            let corner = lane.geometry().lane_to_world(lane.geometry().lane_pos(s, 0.0));
            let gap = s - (10.0 + NOSE);
            let reached =
                Along::Path(0).reach(&network, &vehicle, 10.0, 0, LaneId(1), gap, corner, s, now);
            assert_eq!(reached, now, "rear at {s}");
        }
    }
}
