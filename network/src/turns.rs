//! Which way each connection through an intersection goes: straight on, or a
//! turn, near-side or far-side. Arrows, priorities and signal movements all
//! class connections this way.

use glam::DVec2;
use traffic_sim::Side;

/// The most a connection turns, either way, and still goes straight on: 30°,
/// as SUMO classes a link's direction by the angle between its lanes.
const STRAIGHT_ON: f64 = std::f64::consts::PI / 6.0;

/// Which way a connection goes, by the angle between the lane it leaves and
/// the lane it joins.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Turn {
    Straight,
    Left,
    Right,
}

impl Turn {
    /// The turn from heading along `arriving` to heading along `leaving`.
    pub(crate) fn between(arriving: DVec2, leaving: DVec2) -> Turn {
        match arriving.angle_to(leaving) {
            a if a.abs() <= STRAIGHT_ON => Turn::Straight,
            a if a > 0.0 => Turn::Left,
            _ => Turn::Right,
        }
    }

    /// Whether it turns across oncoming traffic, where traffic drives on
    /// `drives_on`: right where it drives on the left, as in Australia.
    pub fn far_side(self, drives_on: Side) -> bool {
        matches!(
            (self, drives_on),
            (Turn::Right, Side::Left) | (Turn::Left, Side::Right)
        )
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn within_30_degrees_is_straight_on_and_beyond_it_a_turn() {
        let ahead = DVec2::X;
        let at = |deg: f64| DVec2::from_angle(deg.to_radians());
        assert_eq!(Turn::between(ahead, at(29.0)), Turn::Straight);
        assert_eq!(Turn::between(ahead, at(-29.0)), Turn::Straight);
        assert_eq!(Turn::between(ahead, at(31.0)), Turn::Left);
        assert_eq!(Turn::between(ahead, at(-90.0)), Turn::Right);
    }

    #[test]
    fn a_far_side_turn_crosses_oncoming_traffic() {
        assert!(Turn::Right.far_side(Side::Left));
        assert!(!Turn::Left.far_side(Side::Left));
        assert!(Turn::Left.far_side(Side::Right));
        assert!(!Turn::Straight.far_side(Side::Left));
    }
}
