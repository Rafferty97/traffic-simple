//! Replays a log saved by the UI's Capture button, reproducing the captured
//! simulation exactly. The simulation is deterministic, so the same calls in the
//! same order give the same state, bit for bit. The format is described in
//! `ui/src/replay.rs`, which writes it. Needs the `serde` feature, for the
//! network.
//!
//! To turn a capture into a test, copy it into `sim/tests/captures/` and:
//!
//! ```ignore
//! let replay = replay(include_str!("captures/capture-123.replay"), Terms::default());
//! let mut sim = replay.sim;
//! let selected = replay.selected.unwrap();
//! // …step on from the captured moment and assert what should happen.
//! ```

use traffic_sim::{
    DebugSink, LaneId, Placement, Route, Side, SimNetwork, Simulation, VehicleId, VehicleSpec,
};

/// A replayed simulation at the moment it was captured.
pub struct Replay<D: DebugSink> {
    pub sim: Simulation<D>,
    /// Every vehicle inserted, in order: the log's names for them.
    pub ids: Vec<VehicleId>,
    /// The vehicle selected in the UI when it was captured.
    pub selected: Option<VehicleId>,
}

/// Replays `log` into a new simulation with `sink`. Panics on a malformed line,
/// or if a call's outcome differs from the one logged, which means the replay
/// has diverged from the original run.
pub fn replay<D: DebugSink>(log: &str, sink: D) -> Replay<D> {
    let mut lines = log.lines().enumerate().filter(|(_, l)| !l.trim().is_empty());
    let mut next = || lines.find(|(_, l)| !l.starts_with('#'));

    let (_, header) = next().expect("empty replay");
    let json = header.strip_prefix("network ").expect("a replay starts with its network");
    let network: SimNetwork = serde_json::from_str(json).expect("the network is JSON");
    let mut sim = Simulation::new(network, sink);
    let mut ids = Vec::new();

    while let Some((number, line)) = next() {
        let at = || format!("line {}: {line}", number + 1);
        let mut words = line.split_whitespace();
        let call = words.next().unwrap();
        let mut vehicle = || -> VehicleId {
            let index: usize = words
                .next()
                .and_then(|w| w.parse().ok())
                .unwrap_or_else(|| panic!("malformed {}", at()));
            ids[index]
        };
        match call {
            "step" => sim.step(
                words
                    .next()
                    .and_then(|w| w.parse().ok())
                    .unwrap_or_else(|| panic!("malformed {}", at())),
            ),
            "try_insert" => {
                let outcome = words.next().unwrap_or_else(|| panic!("malformed {}", at()));
                let f = Fields::new(words);
                let spec = VehicleSpec {
                    length: f.get("length"),
                    width: f.get("width"),
                    max_speed: f.get("max_speed"),
                    speed_factor: f.get("speed_factor"),
                    max_accel: f.get("max_accel"),
                    comfy_decel: f.get("comfy_decel"),
                    time_headway: f.get("time_headway"),
                    min_gap: f.get("min_gap"),
                    rear_axle: f.get("rear_axle"),
                    front_axle: f.get("front_axle"),
                    courtesy: f.get("courtesy"),
                    politeness: f.get("politeness"),
                };
                let lane = LaneId(f.get("lane") as u32);
                let at_ = Placement {
                    lane,
                    s: f.get("s"),
                    speed: f.get("speed"),
                    route: Route::random(f.get_u64("seed")),
                };
                let result = sim.try_insert(spec, at_);
                assert_eq!(result.is_ok(), outcome == "ok", "diverged at {}", at());
                ids.extend(result.ok());
            }
            "remove" => {
                let id = vehicle();
                sim.remove(id).unwrap_or_else(|| panic!("malformed {}", at()));
            }
            "hold" => {
                let id = vehicle();
                sim.hold(id);
            }
            "cap" => {
                let id = vehicle();
                let max_speed = words.next().and_then(|w| w.parse().ok());
                sim.cap_speed(
                    id,
                    max_speed.unwrap_or_else(|| panic!("malformed {}", at())),
                );
            }
            "release" => {
                let id = vehicle();
                sim.release(id);
            }
            "change_lane" => {
                let id = vehicle();
                let side = match words.next() {
                    Some("left") => Side::Left,
                    Some("right") => Side::Right,
                    _ => panic!("bad side at {}", at()),
                };
                let ok = words.next() == Some("ok");
                let result = sim.change_lane(id, side);
                assert_eq!(result.is_ok(), ok, "diverged at {}", at());
            }
            _ => panic!("unknown call at {}", at()),
        }
    }

    let selected = log
        .lines()
        .find_map(|l| l.strip_prefix("# selected="))
        .map(|index| ids[index.trim().parse::<usize>().unwrap()]);
    Replay { sim, ids, selected }
}

/// A line's `key=value` fields.
struct Fields<'a>(Vec<(&'a str, &'a str)>);

impl<'a> Fields<'a> {
    fn new(words: impl Iterator<Item = &'a str>) -> Self {
        Self(words.map(|w| w.split_once('=').expect("expected key=value")).collect())
    }

    fn get(&self, key: &str) -> f64 {
        let (_, value) = self.0.iter().find(|(k, _)| *k == key).expect(key);
        value.parse().unwrap_or_else(|_| panic!("bad {key}: {value}"))
    }

    fn get_u64(&self, key: &str) -> u64 {
        let (_, value) = self.0.iter().find(|(k, _)| *k == key).expect(key);
        value.parse().unwrap_or_else(|_| panic!("bad {key}: {value}"))
    }
}
