//! A road's lanes: each a profile across the road, varying along it.

/// A lane of a road: its profile across the road, by distance along the
/// road's path, as stations. Between each two stations its offset and widths
/// ease (smoothstep) from one's to the other's; before the first and after the
/// last they are those stations'. Every lane runs the road's whole length: a
/// change in how many lanes there are is another road. In its serde form, a
/// lane the same all along is `{ offset, left, right }`, a dropped lane
/// `{ stations, dropped: true }`, and any other its list of stations.
#[derive(Clone, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(from = "LaneForm", into = "LaneForm"))]
pub struct RoadLane {
    pub stations: Vec<Station>,
    /// Whether it is dropped where it eases in over another lane of its
    /// road, rather than the two zipping together: drivers on it change out
    /// of it before it does, where they can, and the lane going on is marked
    /// beside it with a short dashed line. Where two lanes ease together and
    /// neither is dropped, they zip, with no line between them.
    pub dropped: bool,
}

/// A lane's serde form: the same all along, its stations, or a dropped
/// lane's stations.
#[cfg(feature = "serde")]
#[derive(serde::Serialize, serde::Deserialize)]
#[serde(untagged)]
enum LaneForm {
    Constant { offset: f64, left: f64, right: f64 },
    Stations(Vec<Station>),
    Dropped { stations: Vec<Station>, dropped: bool },
}

#[cfg(feature = "serde")]
impl From<LaneForm> for RoadLane {
    fn from(form: LaneForm) -> Self {
        match form {
            LaneForm::Constant { offset, left, right } => {
                RoadLane::through(vec![Station { s: 0.0, offset, left, right }])
            }
            LaneForm::Stations(stations) => RoadLane::through(stations),
            LaneForm::Dropped { stations, dropped } => RoadLane { stations, dropped },
        }
    }
}

#[cfg(feature = "serde")]
impl From<RoadLane> for LaneForm {
    fn from(lane: RoadLane) -> Self {
        match (lane.dropped, &lane.stations[..]) {
            (true, _) => LaneForm::Dropped { stations: lane.stations, dropped: true },
            (false, &[Station { offset, left, right, .. }]) => {
                LaneForm::Constant { offset, left, right }
            }
            (false, _) => LaneForm::Stations(lane.stations),
        }
    }
}

/// A lane's profile across its road at a distance along its path, in meters.
/// In its serde form it is `[s, offset, left, right]`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serde", serde(from = "[f64; 4]", into = "[f64; 4]"))]
pub struct Station {
    /// How far along the road's path it is.
    pub s: f64,
    /// From the road's path to the lane's centreline, positive to the left of
    /// the path's direction, whichever way the lane runs.
    pub offset: f64,
    /// From the centreline to the lane's left edge, the left of the way the
    /// lane runs.
    pub left: f64,
    /// From the centreline to the lane's right edge, the right of the way the
    /// lane runs.
    pub right: f64,
}

impl Station {
    /// A station `s` along the path, `offset` from it, `width` wide about its
    /// centreline.
    pub const fn new(s: f64, offset: f64, width: f64) -> Self {
        Self { s, offset, left: 0.5 * width, right: 0.5 * width }
    }
}

impl From<[f64; 4]> for Station {
    fn from([s, offset, left, right]: [f64; 4]) -> Self {
        Self { s, offset, left, right }
    }
}

impl From<Station> for [f64; 4] {
    fn from(station: Station) -> Self {
        [station.s, station.offset, station.left, station.right]
    }
}

impl RoadLane {
    /// A lane `width` wide all along, `offset` from the path.
    pub fn new(offset: f64, width: f64) -> Self {
        Self::through(vec![Station::new(0.0, offset, width)])
    }

    /// A lane through `stations`, in order along the path.
    pub fn through(stations: Vec<Station>) -> Self {
        Self { stations, dropped: false }
    }

    /// It, dropped where it eases in over another lane (see
    /// [`RoadLane::dropped`]).
    pub fn dropped(self) -> Self {
        Self { dropped: true, ..self }
    }

    /// Its profile `s` along the path: its offset and widths there.
    pub fn at(&self, s: f64) -> Station {
        let i = self.stations.partition_point(|st| st.s <= s);
        let (a, b) = match i {
            0 => (self.stations[0], self.stations[0]),
            i if i == self.stations.len() => (self.stations[i - 1], self.stations[i - 1]),
            i => (self.stations[i - 1], self.stations[i]),
        };
        if a.s == b.s {
            return Station { s, ..a };
        }
        let x = ((s - a.s) / (b.s - a.s)).clamp(0.0, 1.0);
        let t = x * x * (3.0 - 2.0 * x);
        let ease = |from: f64, to: f64| from + (to - from) * t;
        Station {
            s,
            offset: ease(a.offset, b.offset),
            left: ease(a.left, b.left),
            right: ease(a.right, b.right),
        }
    }

    /// Whether it can be evaluated: it has a station, every value finite, and
    /// its stations run forward along the path.
    pub(crate) fn is_valid(&self) -> bool {
        let finite =
            |st: &Station| [st.s, st.offset, st.left, st.right].iter().all(|v| v.is_finite());
        !self.stations.is_empty()
            && self.stations.iter().all(finite)
            && self.stations.windows(2).all(|w| w[0].s < w[1].s)
    }

    /// Its edges `s` along the path, right then left of the path's direction,
    /// as offsets from it.
    pub(crate) fn edges(&self, s: f64, reverse: bool) -> [f64; 2] {
        let Station { offset, left, right, .. } = self.at(s);
        match reverse {
            false => [offset - right, offset + left],
            true => [offset - left, offset + right],
        }
    }

    /// Where along the path a polyline needs vertices so that, straight
    /// between them, its centreline and edges stray at most `tolerance` from
    /// the profile: smoothstep's second derivative is at most `6·Δ / L²`, so
    /// straight between `n` even steps it strays at most `3·Δ / (4·n²)`, each
    /// edge easing by its offset's change and its width's together.
    pub(crate) fn vertices(&self, tolerance: f64) -> Vec<f64> {
        let mut out = Vec::new();
        for w in self.stations.windows(2) {
            let (a, b) = (w[0], w[1]);
            let offset = b.offset - a.offset;
            let change = [
                offset,
                offset + (b.left - a.left),
                offset - (b.right - a.right),
            ]
            .into_iter()
            .fold(0.0, |most: f64, d| most.max(d.abs()));
            if change == 0.0 {
                continue;
            }
            let n = (0.75 * change / tolerance).sqrt().ceil().max(1.0) as usize;
            out.extend((0..=n).map(|i| a.s + (b.s - a.s) * i as f64 / n as f64));
        }
        out
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn between_stations_a_lane_eases_and_beyond_them_holds() {
        let lane = RoadLane::through(vec![
            Station::new(10.0, 1.0, 3.0),
            Station::new(20.0, 3.0, 4.0),
        ]);
        let offset = |s| lane.at(s).offset;
        assert_eq!((offset(0.0), offset(10.0), offset(15.0)), (1.0, 1.0, 2.0));
        assert_eq!((offset(20.0), offset(30.0)), (3.0, 3.0));
        assert_eq!(lane.at(15.0).left, 1.75);
        let backward = RoadLane::through(vec![
            Station::new(20.0, 0.0, 3.0),
            Station::new(10.0, 1.0, 3.0),
        ]);
        assert!(!backward.is_valid());
        assert!(!RoadLane::through(Vec::new()).is_valid());
    }

    #[test]
    fn straight_between_its_vertices_a_lane_strays_within_tolerance() {
        let lane = RoadLane::through(vec![
            Station::new(0.0, 0.0, 3.5),
            Station::new(80.0, 3.5, 2.0),
            Station::new(100.0, 3.5, 2.0),
        ]);
        let vertices = lane.vertices(0.01);
        assert!(
            vertices.iter().all(|&s| s <= 80.0),
            "none needed where it holds"
        );
        for w in vertices.windows(2) {
            for i in 1..10 {
                let s = w[0] + (w[1] - w[0]) * i as f64 / 10.0;
                let x = (s - w[0]) / (w[1] - w[0]);
                for side in [0, 1] {
                    let [a, b] = [lane.edges(w[0], false)[side], lane.edges(w[1], false)[side]];
                    let straight = a + x * (b - a);
                    assert!(
                        (straight - lane.edges(s, false)[side]).abs() <= 0.01,
                        "at {s}"
                    );
                }
            }
        }
    }

    #[cfg(feature = "serde")]
    #[test]
    fn a_lane_s_serde_form_is_its_stations_as_tuples_or_its_profile_if_it_never_changes() {
        let lane = RoadLane::through(vec![
            Station::new(0.0, -1.75, 3.5),
            Station::new(50.0, -5.25, 3.5),
        ]);
        let json = serde_json::to_string(&lane).unwrap();
        assert_eq!(json, "[[0.0,-1.75,1.75,1.75],[50.0,-5.25,1.75,1.75]]");
        assert_eq!(serde_json::from_str::<RoadLane>(&json).unwrap(), lane);
        // One the same all along is its offset and widths.
        let constant = RoadLane::new(-1.75, 3.5);
        let json = serde_json::to_string(&constant).unwrap();
        assert_eq!(json, r#"{"offset":-1.75,"left":1.75,"right":1.75}"#);
        assert_eq!(serde_json::from_str::<RoadLane>(&json).unwrap(), constant);
        // A dropped one says so.
        let dropped = lane.dropped();
        let json = serde_json::to_string(&dropped).unwrap();
        assert_eq!(
            json,
            r#"{"stations":[[0.0,-1.75,1.75,1.75],[50.0,-5.25,1.75,1.75]],"dropped":true}"#
        );
        assert_eq!(serde_json::from_str::<RoadLane>(&json).unwrap(), dropped);
    }
}
