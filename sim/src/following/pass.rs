//! The pass check: whether a vehicle may pass another it finds ahead, from
//! where it plans to be as its front reaches the other's rear.

use glam::DVec2;

use super::leaders::Along;
use super::{Lateral, Occupant, inside};
use crate::debug::PassCheck;
use crate::motion::kinematics;
use crate::motion::lateral::LaneChange;
use crate::network::{LaneId, Side, SimNetwork};
use crate::simulation::SimState;
use crate::vehicle::Vehicle;

/// The room, in meters, beside a vehicle that another is given to pass it, as
/// connectors are sized: it passes with less, more slowly (see
/// [`MIN_PASSING_CLEARANCE`]).
pub const PASSING_CLEARANCE: f64 = 0.5;

/// The least room, in meters, beside another vehicle's rear corners that a
/// vehicle passes it with, as its nose reaches the other's rear: its own room,
/// above the most its tail swings (see `following::classify`) and what is left
/// of where it plans to be (see [`Along::reach`]). How it goes by with the
/// room it has above that is the longitudinal model's (see
/// [`crate::models::acceleration::passing`]).
pub const MIN_PASSING_CLEARANCE: f64 = 0.3;

/// How many steps its rear axle trails its front in, planning where it will be
/// as its nose reaches another vehicle's rear (see [`Along::reach`]).
const REACH_STEPS: usize = 3;

/// The pass check (see "Pass check" in `docs/multilane-car-following.md`):
/// whether `vehicle`, searching `along` some lanes with its rear axle at
/// `axle_s` along the lanes it follows and its pose `now`, may pass the vehicle
/// whose record `o` it found `at` along the lanes searched, on `lane`, the `k`th
/// of them, where it is itself `mine` to that lane. `None` where no check is
/// made, and the other vehicle is simply followed.
///
/// A pair [`passing_side`] lets through is checked: with the follower where it
/// plans to be as its front reaches the other's rear (see [`Along::reach`]),
/// driving straight ahead must leave at least [`MIN_PASSING_CLEARANCE`] beside
/// the other vehicle's rear corners, and how it goes by with the room it has
/// above that is the longitudinal model's, which is the caller's to ask. A follower diverging from the lane
/// passes any record its body will be wholly out of the lane by then, freely,
/// since it never comes back, so it is checked against every leader, on the
/// side it leaves toward.
#[allow(clippy::too_many_arguments)]
pub(super) fn check(
    state: &SimState,
    vehicle: &Vehicle,
    along: Along,
    axle_s: f64,
    now: (DVec2, DVec2),
    k: usize,
    lane: LaneId,
    at: f64,
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
    let (pos, tangent, _) =
        along.reach(network, vehicle, axle_s, k, lane, at, rearmost, o.s(), now);
    let half_width = 0.5 * vehicle.spec.width;
    let (clearance, corner) = clearance(side, pos, tangent, half_width, corners);
    let gone = mine.is_diverging() && {
        let body = body(&vehicle.spec, pos, tangent);
        !body.iter().any(|&c| inside(geometry, c, o.hint))
    };
    let passed = gone || (by_table && clearance >= MIN_PASSING_CLEARANCE);
    Some(PassCheck {
        leader: o.id,
        pos,
        tangent,
        side,
        corner,
        clearance,
        gone,
        passed,
    })
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

    /// Its front axle's course, at `s` along the lanes it follows: its path, with
    /// its lane change's quintic, its corners rounded as it drives them, or,
    /// settled beside it, the centerlines there.
    fn course(self, network: &SimNetwork, vehicle: &Vehicle, s: f64) -> DVec2 {
        let change = self.change(vehicle);
        let offset = |s: f64| change.map_or(0.0, |c| c.offset_at(s));
        match self {
            Along::Into(_) => {
                let lanes = (0..).map_while(|k| self.lane(network, vehicle, k));
                network.pose_along(lanes, s, 0.0, 0.0).0
            }
            _ => network.smooth_point_along(&vehicle.path, s, offset),
        }
    }

    /// Its rear axle's position and heading, as a unit tangent, where it plans
    /// to be with its front axle `front_s` along the lanes it follows (see
    /// [`Along::course`]), with no pose to start from: its front axle on its
    /// course, heading toward its course a wheelbase behind that, as a body on
    /// rails would, its rear axle a wheelbase behind its front along that
    /// heading. A rear axle trailing its front cuts inside a bend, which this
    /// leaves out: at most about 15 cm by its nose across a lane change from
    /// rest, and under 1 cm at speed.
    pub(crate) fn pose(
        self,
        network: &SimNetwork,
        vehicle: &Vehicle,
        front_s: f64,
    ) -> (DVec2, DVec2) {
        let wheelbase = vehicle.spec.wheelbase();
        let front = self.course(network, vehicle, front_s);
        let tangent = (front - self.course(network, vehicle, front_s - wheelbase))
            .try_normalize()
            .unwrap_or(vehicle.dir);
        (front - wheelbase * tangent, tangent)
    }

    /// Its front axle's `s` along the lanes it follows: its own on its path, and,
    /// settled beside it, a wheelbase ahead of its rear axle there, at `axle_s`.
    pub(crate) fn front_s(self, vehicle: &Vehicle, axle_s: f64) -> f64 {
        match self {
            Along::Into(_) => axle_s + vehicle.spec.wheelbase(),
            _ => vehicle.front_s,
        }
    }

    /// Where it will be, as its rear axle's position and heading, when its nose
    /// reaches `point`, another vehicle's rearmost corner, `at` along the lanes
    /// searched, at `s` on `lane`, the `k`th of them, with its rear axle at
    /// `axle_s` along the lanes it follows and its pose `now`; and how far its
    /// front axle has to go to get there, along its course (see
    /// [`Along::course`]), to its front overhang short of the corner.
    ///
    /// Along the lanes it follows, `at` is where the corner is. Leaving the lanes
    /// searched, distances there differ from its own, so `point` is projected
    /// onto its own `k`th lane instead.
    ///
    /// It drives there from `now`, its front axle along its course, its rear axle
    /// trailing it as it moves, in [`REACH_STEPS`] equal steps: rough far
    /// ahead, where its heading hardly bends meanwhile, and closer to how it
    /// will go as it gets there, until where it plans to be is where it is. By
    /// its nose, across a lane change from rest, that is out by under 2 cm with
    /// 5 m to go, 5 cm with 10 m and about 12 cm with 15 m. If its front axle
    /// is already there, it is where it is now.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn reach(
        self,
        network: &SimNetwork,
        vehicle: &Vehicle,
        axle_s: f64,
        k: usize,
        lane: LaneId,
        at: f64,
        point: DVec2,
        s: f64,
        now: (DVec2, DVec2),
    ) -> (DVec2, DVec2, f64) {
        let overhang = vehicle.spec.length - vehicle.spec.front_axle;
        let at = match (self, vehicle.path.get(k)) {
            (Along::Leaving(..), Some(&own)) => {
                let (own_lane, searched) = (network.lane(own), network.lane(lane));
                let before: f64 =
                    vehicle.path[..k].iter().map(|&l| network.lane(l).geometry().length()).sum();
                // Adjacent lanes start and end together, so the same fraction is a good hint.
                let hint = own_lane.geometry().lane_pos(
                    s / searched.geometry().length() * own_lane.geometry().length(),
                    0.0,
                );
                before + own_lane.geometry().world_to_lane_near(point, hint).s
            }
            _ => at,
        };
        let from = self.front_s(vehicle, axle_s);
        let ahead = (at - overhang - from).max(0.0);
        let wheelbase = vehicle.spec.wheelbase();
        let (mut rear, mut dir) = now;
        if ahead > 0.0 {
            for i in 1..=REACH_STEPS {
                let front = self.course(
                    network,
                    vehicle,
                    from + ahead * i as f64 / REACH_STEPS as f64,
                );
                (rear, dir) = kinematics::trail(rear, dir, front, wheelbase);
            }
        }
        (rear, dir, ahead)
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
    fn a_vehicle_plans_its_front_axle_along_its_quintic() {
        // Straight along x, changing from 3.5 m left of the centerline back onto
        // it, from rest, as sharply as it can steer: with its front axle halfway,
        // that is 1.75 m left, heading right of x, its rear axle a wheelbase
        // behind.
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
        let wheelbase = vehicle.spec.wheelbase();
        let half = 0.5 * change.length;
        let (pos, tangent) = along.pose(&network, &vehicle, half);
        let front = pos + wheelbase * tangent;
        assert!(
            (front - DVec2::new(half, 1.75)).length() < 1e-9,
            "at {front}"
        );
        assert!(tangent.y < -0.2, "heading {tangent}");
        // Not changing, it heads straight along the centerline.
        vehicle.lane_change = None;
        let (pos, tangent) = along.pose(&network, &vehicle, 7.5 + wheelbase);
        assert!((pos - DVec2::new(7.5, 0.0)).length() < 1e-9);
        assert!((tangent - DVec2::X).length() < 1e-9);
    }

    #[test]
    fn settled_into_the_lane_beside_it_plans_to_follow_its_centerline() {
        // On lane 1 of the bend, weighing up lane 0 on its left: as if already
        // there, its front axle on lane 0's centerline a wheelbase ahead of its
        // rear axle, with no quintic and not moving across, and its rear axle
        // within the chord's sagitta of it, under a centimetre.
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let along = Along::Into(Side::Left);
        assert_eq!(along.lane(&network, &vehicle, 0), Some(LaneId(0)));
        let axle_s = axle_beside(&network, &vehicle, LaneId(0));
        let wheelbase = vehicle.spec.wheelbase();
        let (pos, tangent) = along.pose(&network, &vehicle, axle_s + wheelbase + 20.0);
        let inside = network.lane(LaneId(0)).geometry();
        let front = pos + wheelbase * tangent;
        let s = axle_s + wheelbase + 20.0;
        let on = inside.world_to_lane_near(front, inside.lane_pos(s, 0.0));
        assert!(on.t.abs() < 1e-6, "{on:?}");
        assert!((on.s - s).abs() < 1e-6, "{on:?}");
        let on = inside.world_to_lane_near(pos, inside.lane_pos(axle_s + 20.0, 0.0));
        assert!(on.t.abs() < 0.01, "{on:?}");
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

    #[test]
    fn on_its_own_lane_the_nose_stops_just_short_of_the_other_rear() {
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let lane = network.lane(LaneId(1));
        // A rear corner 60 m along, off the centerline.
        let corner = lane.geometry().lane_to_world(lane.geometry().lane_pos(60.0, -0.9));
        let (pos, tangent, _) = Along::Path(0).reach(
            &network,
            &vehicle,
            10.0,
            0,
            LaneId(1),
            60.0,
            corner,
            60.0,
            (vehicle.pos, vehicle.dir),
        );
        let nose = nose_along(&network, LaneId(1), pos, tangent, NOSE, 60.0);
        assert!((nose - 60.0).abs() < 0.02, "nose at {nose}");
    }

    #[test]
    fn on_the_old_side_the_other_rear_is_projected_onto_its_own_lane() {
        // Leaving lane 0 for lane 1: lane 0 is shorter round the bend, so its
        // distances aren't lane 1's, and where it is along lane 0 is ignored.
        let network = bend();
        let vehicle = car_at(&network, 1, 10.0);
        let beside = network.lane(LaneId(0));
        let corner = beside.geometry().lane_to_world(beside.geometry().lane_pos(58.0, -1.2));
        let along = Along::Leaving(Side::Left, 0);
        let (pos, tangent, _) = along.reach(
            &network,
            &vehicle,
            10.0,
            0,
            LaneId(0),
            999.0,
            corner,
            58.0,
            (vehicle.pos, vehicle.dir),
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
        let (pos, tangent, _) = Along::Path(0).reach(
            &network,
            &vehicle,
            10.0,
            0,
            LaneId(1),
            25.0,
            corner,
            25.0,
            (vehicle.pos, vehicle.dir),
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
            let reached =
                Along::Path(0).reach(&network, &vehicle, 10.0, 0, LaneId(1), s, corner, s, now);
            assert_eq!(reached, (now.0, now.1, 0.0), "rear at {s}");
        }
    }
}
