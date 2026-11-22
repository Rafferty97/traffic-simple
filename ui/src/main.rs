//! egui app with a pretty view and a debug view of the simulation.

mod glyphs;
mod replay;

/// The simulation's test helpers, for checking that replays read back exactly.
#[cfg(test)]
#[path = "../../sim/tests/common/mod.rs"]
// Its freeze-frame loader is behind `sim`'s `serde` feature, which this crate
// doesn't have, so it is left out here.
#[allow(unexpected_cfgs)]
mod common;

/// The simulation's replay reader, for checking that replays read back exactly.
#[cfg(test)]
#[path = "../../sim/tests/common/replay.rs"]
#[allow(dead_code)]
mod replay_reader;

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::path::PathBuf;

use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use glam::DVec2;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{RngExt, SeedableRng};
use replay::Recorded;
use traffic_network::{
    Arrow, MarkingStyle, RenderGeometry, RoadNetwork, Strip, compile, scenarios,
};
use traffic_sim::{
    AccelSource, BarrierDecision, BarrierKind, ConflictKind, DebugEvent, DebugSink,
    LaneChangeError, LaneChangeEval, LaneId, Leaving, Light, PassCheck, Placement, Route, Side,
    Simulation, Vehicle, VehicleId, VehicleSpec,
};

/// The inner lane's circumference: about 160 m across, roomy enough for
/// vehicles to accelerate, settle into gaps and change lanes.
const RING_CIRCUMFERENCE: f64 = 1000.0;
const RING_LANES: usize = 3;
const SPEED_LIMIT: f64 = 50.0 / 3.6;

/// Most spawns or despawns per simulated second, while the count is off target.
const CHANGE_RATE: f64 = 2.0;
/// How wide the inspector's column of acceleration values is, in points.
const TERM_WIDTH: f32 = 80.0;

/// The speed the inspector's slow button imposes, in m/s: 20 km/h.
const SLOW_CAP: f64 = 20.0 / 3.6;
/// How far outside a vehicle a click still selects it, in screen points.
const PICK_TOLERANCE: f32 = 6.0;

const SELECTED: Color32 = Color32::from_rgb(250, 210, 60);
/// With a vehicle selected, those it doesn't relate to.
const UNRELATED: Color32 = Color32::from_gray(235);
/// The colours of the vehicles the selected one relates to, in turn.
const RELATED: [Color32; 8] = [
    Color32::from_rgb(70, 140, 255),
    Color32::from_rgb(230, 60, 200),
    Color32::from_rgb(40, 200, 200),
    Color32::from_rgb(255, 130, 30),
    Color32::from_rgb(150, 90, 255),
    Color32::from_rgb(60, 200, 80),
    Color32::from_rgb(255, 90, 110),
    Color32::from_rgb(140, 110, 60),
];

/// A vehicle's number, as the UI shows it: its slot's index, unique among the
/// vehicles there at once.
fn number(id: VehicleId) -> String {
    id.index().to_string()
}

/// The vehicle an acceleration term is for, if any.
fn named(source: AccelSource) -> Option<VehicleId> {
    match source {
        AccelSource::Leader { id, .. }
        | AccelSource::Yield { id, .. }
        | AccelSource::Junction { id, .. }
        | AccelSource::Sync { id, .. }
        | AccelSource::Merge { id, .. } => Some(id),
        _ => None,
    }
}

/// The vehicles `id`'s acceleration terms name, each with its colour, in the
/// order its terms name them.
fn related(sim: &Simulation<DebugState>, id: VehicleId) -> Vec<(VehicleId, Color32)> {
    let mut out: Vec<(VehicleId, Color32)> = Vec::new();
    for (source, _) in sim.sink().terms(id) {
        if let Some(other) = named(source)
            && other != id
            && !out.iter().any(|&(v, _)| v == other)
        {
            out.push((other, RELATED[out.len() % RELATED.len()]));
        }
    }
    out
}

/// What colour to draw `id`'s body: `plain` with nobody selected; otherwise
/// the selected vehicle's colour, its related vehicles' colours, or white.
fn body_colour(
    id: VehicleId,
    selected: Option<VehicleId>,
    related: &[(VehicleId, Color32)],
    plain: Color32,
) -> Color32 {
    match selected {
        None => plain,
        Some(s) if s == id => SELECTED,
        Some(_) => related.iter().find(|&&(v, _)| v == id).map_or(UNRELATED, |&(_, c)| c),
    }
}

/// Runs the UI, starting on the scenario named on the command line, by its
/// key (see [`Scenario::key`]), or on the ring.
fn main() -> eframe::Result {
    let scenario = match std::env::args().nth(1) {
        None => Scenario::Ring,
        Some(key) => match Scenario::ALL.into_iter().find(|s| s.key() == key) {
            Some(scenario) => scenario,
            None => {
                eprintln!("No scenario called {key:?}. The scenarios are:");
                for s in Scenario::ALL {
                    eprintln!("  {:<16} {}", s.key(), s.name());
                }
                std::process::exit(2);
            }
        },
    };
    eframe::run_native(
        "Traffic Simulator",
        eframe::NativeOptions::default(),
        Box::new(move |_cc| Ok(Box::new(App::new(scenario)))),
    )
}

/// A network to run, and how vehicles come and go on it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    Ring,
    /// Two two-way roads crossing, with this many lanes each way.
    Crossroads(usize),
    /// As `Crossroads`, signalised.
    Signals(usize),
    /// As `Signals`, with two lanes each way, the roads crossing at 60°.
    ObliqueSignals,
    /// A freeway with an on-ramp and an off-ramp, the acceleration lane left as
    /// this says.
    Ramps(Leaving),
}

impl Scenario {
    const ALL: [Scenario; 9] = [
        Scenario::Ring,
        Scenario::Crossroads(1),
        Scenario::Crossroads(2),
        Scenario::Crossroads(3),
        Scenario::Signals(1),
        Scenario::Signals(2),
        Scenario::ObliqueSignals,
        Scenario::Ramps(Leaving::Zipper),
        Scenario::Ramps(Leaving::Early),
    ];

    /// What to call it on the command line.
    fn key(self) -> &'static str {
        match self {
            Scenario::Ring => "ring",
            Scenario::Crossroads(1) => "crossroads-1",
            Scenario::Crossroads(2) => "crossroads-2",
            Scenario::Crossroads(_) => "crossroads-3",
            Scenario::Signals(1) => "signals-1",
            Scenario::Signals(_) => "signals-2",
            Scenario::ObliqueSignals => "oblique-signals",
            Scenario::Ramps(Leaving::Zipper) => "ramps-zipper",
            Scenario::Ramps(Leaving::Early) => "ramps-early",
        }
    }

    fn name(self) -> &'static str {
        match self {
            Scenario::Ring => "Three-lane ring",
            Scenario::Crossroads(1) => "Crossroads of 2-lane roads",
            Scenario::Crossroads(2) => "Crossroads of 4-lane roads",
            Scenario::Crossroads(_) => "Crossroads of 6-lane roads",
            Scenario::Signals(1) => "Signalised crossroads of 2-lane roads",
            Scenario::Signals(_) => "Signalised crossroads of 4-lane roads",
            Scenario::ObliqueSignals => "Signalised 4-lane roads crossing at 60°",
            Scenario::Ramps(Leaving::Zipper) => "Freeway with ramps, zipper merge",
            Scenario::Ramps(Leaving::Early) => "Freeway with ramps, early merge",
        }
    }

    fn network(self) -> RoadNetwork {
        match self {
            Scenario::Ring => scenarios::ring(RING_CIRCUMFERENCE, RING_LANES, SPEED_LIMIT),
            Scenario::Crossroads(lanes) => scenarios::crossroads(lanes, SPEED_LIMIT),
            Scenario::Signals(lanes) => scenarios::signalised_crossroads(lanes, SPEED_LIMIT),
            Scenario::ObliqueSignals => {
                scenarios::signalised_oblique_crossroads(2, SPEED_LIMIT, 60f64.to_radians())
            }
            Scenario::Ramps(leaving) => scenarios::ramps_leaving(100.0 / 3.6, 80.0 / 3.6, leaving),
        }
    }

    /// Whether the network is closed: vehicles neither enter nor leave it.
    fn closed(self) -> bool {
        matches!(self, Scenario::Ring)
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    Pretty,
    Debug,
}

struct App {
    sim: Recorded<DebugState>,
    /// What to draw for the network.
    render: RenderGeometry,
    scenario: Scenario,
    view: View,
    paused: bool,
    /// Simulated seconds per real second.
    speed: f64,
    /// Visible region of the world, in scene coordinates (see [`to_scene`]).
    scene_rect: Rect,
    selected: Option<VehicleId>,
    /// Which debug view layers are drawn.
    layers: Layers,
    /// Why the last lane-change request for the selected vehicle was refused.
    lane_change_error: Option<LaneChangeError>,
    /// Where the last capture was saved, or why it failed.
    capture_status: Option<String>,
    harness: Harness,
    /// Each vehicle's corners before the last step, to draw it between there and
    /// where it is now (see [`Poses`]).
    poses: Poses,
}

/// Where vehicles were before the simulation's last fixed step, so they can be
/// drawn between there and where they are by how far into the next step the
/// frame has reached ([`Simulation::alpha`]), rather than jumping a step at a
/// time: interpolation, as in "Fix Your Timestep". Drawn a step behind.
#[derive(Default)]
struct Poses {
    before: HashMap<VehicleId, [DVec2; 4]>,
}

impl Poses {
    /// Where to draw `v`'s corners, `alpha` of the way from before the last step
    /// to now. A vehicle that wasn't there before is drawn where it is.
    fn corners(&self, id: VehicleId, v: &Vehicle, alpha: f64) -> [DVec2; 4] {
        let now = v.corners();
        match self.before.get(&id) {
            Some(before) => std::array::from_fn(|i| before[i].lerp(now[i], alpha)),
            None => now,
        }
    }
}

/// Keeps vehicles coming. On a closed network, such as the ring, it keeps a
/// target number on the road, spawning and despawning anywhere; on an open one,
/// it spawns at a flow into each lane that nothing leads into, and vehicles
/// leave where lanes lead nowhere.
struct Harness {
    /// Whether the network is closed: nowhere to come in or leave.
    closed: bool,
    /// How many vehicles it keeps on a closed network.
    target: usize,
    /// Vehicles per hour into each entry lane of an open network.
    flow: f64,
    /// Spawns or despawns owed: grows at [`CHANGE_RATE`], or at the flow, and a
    /// refused spawn waits here.
    credit: f64,
    spawned: u32,
    despawned: u32,
    /// Draws drivers and their routes. Seeded, so every run spawns the same.
    rng: Xoshiro256PlusPlus,
}

impl Default for Harness {
    fn default() -> Self {
        Self {
            closed: true,
            target: 60,
            flow: 600.0,
            credit: 0.0,
            spawned: 0,
            despawned: 0,
            rng: Xoshiro256PlusPlus::seed_from_u64(0),
        }
    }
}

impl Harness {
    /// Runs after each simulation step of `dt` simulated seconds.
    fn step(&mut self, sim: &mut Recorded<DebugState>, selected: Option<VehicleId>, dt: f64) {
        match self.closed {
            true => self.keep_count(sim, selected, dt),
            false => self.keep_flowing(sim, dt),
        }
    }

    /// While the count is off target, spawns or despawns at up to
    /// [`CHANGE_RATE`], each at a random lane and point, or a random vehicle
    /// other than the `selected` one.
    fn keep_count(&mut self, sim: &mut Recorded<DebugState>, selected: Option<VehicleId>, dt: f64) {
        let count = sim.vehicles().count();
        if count == self.target {
            self.credit = 0.0;
            return;
        }
        // At most one owed: a backlog would arrive as a burst.
        self.credit = (self.credit + CHANGE_RATE * dt).min(1.0);
        if self.credit < 1.0 {
            return;
        }
        if count < self.target {
            let lanes = sim.network().lanes().count() as u32;
            let lane = LaneId(self.rng.random_range(0..lanes));
            let s = self.rng.random::<f64>() * sim.network().lane(lane).geometry().length();
            self.spawn(sim, lane, s);
        } else {
            // Never the one being inspected.
            let others = || sim.vehicles().map(|(id, _)| id).filter(|&id| Some(id) != selected);
            let n = others().count();
            if n > 0 {
                let id = others().nth(self.rng.random_range(0..n)).unwrap();
                sim.remove(id);
                self.credit -= 1.0;
                self.despawned += 1;
            }
        }
    }

    /// Spawns at the flow, each into a random entry lane, near its start.
    fn keep_flowing(&mut self, sim: &mut Recorded<DebugState>, dt: f64) {
        let entries: Vec<LaneId> = sim
            .network()
            .lanes()
            .filter(|(_, lane)| lane.predecessors().is_empty())
            .map(|(id, _)| id)
            .collect();
        if entries.is_empty() {
            return;
        }
        let rate = self.flow / 3600.0 * entries.len() as f64;
        // A few owed at most: a long backlog would arrive as a burst.
        self.credit = (self.credit + rate * dt).min(entries.len() as f64);
        if self.credit >= 1.0 {
            let lane = entries[self.rng.random_range(0..entries.len())];
            self.spawn(sim, lane, 5.0);
        }
    }

    /// Tries to spawn a driver at `s` on `lane`, at its desired speed, taking
    /// one from the credit if it fits.
    fn spawn(&mut self, sim: &mut Recorded<DebugState>, lane: LaneId, s: f64) {
        // Drivers vary in how hard they'll brake to let someone in, in how much
        // they weigh others when changing lanes, and in how readily they change:
        // mostly cautious, about one in ten above 0.7.
        let car = VehicleSpec {
            speed_factor: self.speed_factor(),
            courtesy: 2.0 * self.rng.random::<f64>(),
            politeness: 0.1 + 0.3 * self.rng.random::<f64>(),
            assertiveness: self.rng.random::<f64>().powi(3),
            ..demo_car()
        };
        let limit = sim.network().lane(lane).speed_limit();
        let at = Placement {
            lane,
            s,
            speed: car.desired_speed(limit),
            route: Route::random(self.rng.random()),
        };
        // Refused where there is no safe room: the credit waits for another try.
        if sim.try_insert(car, at).is_ok() {
            self.credit -= 1.0;
            self.spawned += 1;
        }
    }

    /// A speed factor from a normal distribution with mean 1 and deviation 0.1,
    /// as SUMO's default, clamped to [0.8, 1.2].
    fn speed_factor(&mut self) -> f64 {
        // Box-Muller on two uniform samples, u in (0, 1] so its log is finite.
        let u = 1.0 - self.rng.random::<f64>();
        let v = self.rng.random::<f64>();
        let normal = (-2.0 * u.ln()).sqrt() * (TAU * v).cos();
        (1.0 + 0.1 * normal).clamp(0.8, 1.2)
    }

    fn controls(&mut self, ui: &mut egui::Ui, on_road: usize) {
        ui.horizontal(|ui| {
            if self.closed {
                ui.add(egui::Slider::new(&mut self.target, 0..=300).text("target vehicles"));
            } else {
                let slider = egui::Slider::new(&mut self.flow, 0.0..=1800.0);
                ui.add(slider.text("vehicles per hour per entry lane"));
            }
            ui.separator();
            ui.label(format!(
                "spawned {}, despawned {}, on the road {on_road}",
                self.spawned, self.despawned
            ));
        });
    }
}

impl App {
    fn new(scenario: Scenario) -> Self {
        let (sim, render) = new_sim(scenario);
        let mut app = Self {
            sim,
            render,
            scenario,
            view: View::Pretty,
            paused: false,
            speed: 1.0,
            scene_rect: Rect::NOTHING,
            selected: None,
            layers: Layers::default(),
            lane_change_error: None,
            capture_status: None,
            harness: Harness::default(),
            poses: Poses::default(),
        };
        app.load(scenario);
        app
    }

    /// Starts the scenario afresh, with the harness's settings kept, and fits
    /// the view to it.
    fn load(&mut self, scenario: Scenario) {
        (self.sim, self.render) = new_sim(scenario);
        self.scenario = scenario;
        self.selected = None;
        self.capture_status = None;
        self.poses = Poses::default();
        let Harness { target, flow, .. } = self.harness;
        self.harness = Harness {
            closed: scenario.closed(),
            target,
            flow,
            ..Harness::default()
        };
        let road_points = self.render.roads.iter().flat_map(|r| r.left.iter().chain(&r.right));
        let points: Vec<Pos2> = road_points.map(|&p| to_scene(p)).collect();
        self.scene_rect = Rect::from_points(&points).expand(10.0);
    }

    fn step(&mut self, dt: f64) {
        let before: HashMap<_, _> = self.sim.vehicles().map(|(id, v)| (id, v.corners())).collect();
        if self.sim.step_variable(dt) > 0 {
            self.poses.before = before;
        }
        self.harness.step(&mut self.sim, self.selected, dt);
    }

    /// Saves a replay of the session so far to `captures/` in the workspace,
    /// named by the wall-clock time, and returns its path.
    fn capture(&self) -> std::io::Result<PathBuf> {
        let dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../captures"));
        std::fs::create_dir_all(&dir)?;
        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH);
        let stamp = now.map_or(0, |d| d.as_secs());
        let path = dir.join(format!("capture-{stamp}.replay"));
        std::fs::write(&path, self.sim.capture(self.selected))?;
        Ok(path.canonicalize().unwrap_or(path))
    }

    /// Saves a freeze frame of the simulation as it is now, with the selected
    /// vehicle, to `captures/` in the workspace, named by the wall-clock time, and
    /// returns its path. `sim/tests/common/frame.rs` loads it back.
    fn freeze_frame(&self) -> std::io::Result<PathBuf> {
        let dir = PathBuf::from(concat!(env!("CARGO_MANIFEST_DIR"), "/../captures"));
        std::fs::create_dir_all(&dir)?;
        let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH);
        let stamp = now.map_or(0, |d| d.as_secs());
        let path = dir.join(format!("frame-{stamp}.json"));
        let frame = serde_json::json!({
            "selected": self.selected,
            "snapshot": self.sim.state(),
        });
        std::fs::write(&path, serde_json::to_string_pretty(&frame)?)?;
        Ok(path.canonicalize().unwrap_or(path))
    }

    fn controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.view, View::Pretty, "Pretty");
            ui.selectable_value(&mut self.view, View::Debug, "Debug");
            ui.separator();
            let label = if self.paused { "Play" } else { "Pause" };
            if ui.button(label).clicked() {
                self.paused = !self.paused;
            }
            if ui.add_enabled(self.paused, egui::Button::new("Step")).clicked() {
                self.step(1.0 / 60.0);
            }
            for (speed, label) in [
                (0.1, "0.1×"),
                (0.5, "0.5×"),
                (1.0, "1×"),
                (2.0, "2×"),
                (5.0, "5×"),
                (10.0, "10×"),
                (20.0, "20×"),
            ] {
                ui.selectable_value(&mut self.speed, speed, label);
            }
            ui.separator();
            let mut scenario = self.scenario;
            egui::ComboBox::from_id_salt("scenario").selected_text(scenario.name()).show_ui(
                ui,
                |ui| {
                    for option in Scenario::ALL {
                        ui.selectable_value(&mut scenario, option, option.name());
                    }
                },
            );
            if scenario != self.scenario || ui.button("Reset").clicked() {
                self.load(scenario);
            }
            if ui
                .button("Capture")
                .on_hover_text("Save a replay of this session, for a test case")
                .clicked()
            {
                self.capture_status = Some(match self.capture() {
                    Ok(path) => format!("Saved {}", path.display()),
                    Err(error) => format!("Capture failed: {error}"),
                });
            }
            if ui
                .button("Freeze frame")
                .on_hover_text("Save the simulation exactly as it is now, for a test case")
                .clicked()
            {
                self.capture_status = Some(match self.freeze_frame() {
                    Ok(path) => format!("Saved {}", path.display()),
                    Err(error) => format!("Freeze frame failed: {error}"),
                });
            }
        });
        if let Some(status) = &self.capture_status {
            ui.small(status);
        }
        let on_road = self.sim.vehicles().count();
        self.harness.controls(ui, on_road);
    }

    fn inspector(&mut self, ui: &mut egui::Ui, id: VehicleId) {
        let Some(v) = self.sim.get(id) else {
            ui.label("Vehicle has left the network.");
            return;
        };
        let pos = v.lane_pos();
        egui::Grid::new("inspector").num_columns(2).show(ui, |ui| {
            ui.label("id");
            ui.label(egui::RichText::new(number(id)).monospace().color(SELECTED));
            ui.end_row();
            let mut row = |name: &str, value: String| {
                ui.label(name);
                ui.monospace(value);
                ui.end_row();
            };
            row("lane", format!("{}", v.lane().0));
            let neighbor = |side| match self.sim.network().lane(v.lane()).adj(side) {
                Some(n) if n.can_change => format!("{}", n.lane.0),
                Some(n) => format!("{} (no change)", n.lane.0),
                None => "none".into(),
            };
            row("left", neighbor(Side::Left));
            row("right", neighbor(Side::Right));
            let side = |side: Option<Side>| match side {
                Some(Side::Left) => "left".to_string(),
                Some(Side::Right) => "right".to_string(),
                None => "none".to_string(),
            };
            row("old side", side(v.old_side()));
            match v.lane_change() {
                Some(change) => {
                    let done = (pos.s - change.s0) / change.length;
                    row(
                        "changing",
                        format!("{:.0}% of {:.0} m", 100.0 * done, change.length),
                    )
                }
                None => row("changing", "no".into()),
            }
            match v.intent() {
                Some(intent) => {
                    let since = self.sim.time() - intent.since;
                    row(
                        "indicating",
                        format!("{} for {since:.1} s", side(Some(intent.side))),
                    );
                }
                None => row("indicating", "no".into()),
            }
            let path: Vec<_> = v.path().iter().map(|l| l.0.to_string()).collect();
            row("path", path.join(" → "));
            row("s", format!("{:.2} m", pos.s));
            row("t", format!("{:+.3} m", pos.t));
            row(
                "speed",
                format!("{:.1} km/h ({:.2} m/s)", v.speed() * 3.6, v.speed()),
            );
            row("accel", format!("{:+.2} m/s²", v.accel()));
            row("curvature", format!("{:+.4} /m", v.curvature()));
        });

        ui.separator();
        ui.label("Lane-change desire (LMRS)");
        let evals = self.sim.sink().decisions.get(&id).cloned().unwrap_or_default();
        {
            for eval in &evals {
                let side = match eval.side {
                    Side::Left => "left",
                    Side::Right => "right",
                };
                // Only when the gap was checked: its own acceleration there and
                // its new follower's.
                let gap = match eval.own_after {
                    None => String::new(),
                    Some(own) => format!(
                        ", gap: own {own:+.2}, follower {}, accepts {:.1} m/s²{}{}",
                        eval.new_follower.map_or("none".into(), |x| format!("{x:+.2}")),
                        eval.accepted_braking,
                        if eval.safe { "" } else { ", unsafe" },
                        if eval.room { "" } else { ", no room" },
                    ),
                };
                let i = eval.incentives;
                ui.label(format!("{side}: {:?}", eval.outcome));
                let detail = egui::RichText::new(format!(
                    "desire {:+.2} (free {:.2}, sync {:.2}, coop {:.2}): route {:+.2}, speed {:+.2}, keep {:+.2}, courtesy {:+.2}{gap}",
                    eval.desire,
                    traffic_sim::D_FREE,
                    traffic_sim::D_SYNC,
                    traffic_sim::D_COOP,
                    i.route,
                    i.speed,
                    i.keep,
                    i.courtesy,
                ))
                .small();
                ui.add(egui::Label::new(detail).wrap());
            }
        }
        if evals.is_empty() {
            ui.small("No permitted lane to either side.");
        }

        ui.separator();
        ui.label(format!("Committed to {} barriers ahead", v.committed()));
        let decisions = self.sim.sink().barriers.iter().filter(|b| b.0 == id);
        for &(_, lane, index, gap, decision) in decisions {
            let what = match decision {
                BarrierDecision::Commit => "commits".to_string(),
                BarrierDecision::Pass => "passes for now".to_string(),
                BarrierDecision::Hold(accel) => format!("held, {accel:+.2} m/s²"),
            };
            ui.small(format!(
                "Barrier {index} on lane {}, gap {gap:.1} m: {what}",
                lane.0
            ));
        }

        ui.separator();
        ui.label("Acceleration terms (minimum binds)");
        let terms: Vec<_> = self.sim.sink().terms(id).collect();
        let related = related(&self.sim, id);
        for &(source, accel) in &terms {
            // Its description, either side of the vehicle it names, if any.
            let (before, after): (String, String) = match source {
                AccelSource::FreeRoad { desired_speed } => (
                    format!(
                        "free road, v0 {:.0} km/h ({desired_speed:.1} m/s)",
                        desired_speed * 3.6
                    ),
                    String::new(),
                ),
                AccelSource::Leader { gap, headway, .. } if headway < 1.0 => (
                    "leader ".into(),
                    format!(", gap {gap:.2} m, headway ×{headway:.2}"),
                ),
                AccelSource::Leader { gap, .. } => ("leader ".into(), format!(", gap {gap:.2} m")),
                AccelSource::Cap { max_speed: 0.0 } => ("held to a stop".into(), String::new()),
                AccelSource::Cap { max_speed } => (
                    format!("capped at {:.0} km/h", max_speed * 3.6),
                    String::new(),
                ),
                AccelSource::Yield { gap, .. } => {
                    ("making room for ".into(), format!(", gap {gap:.2} m"))
                }
                AccelSource::PathEnd { gap } => {
                    (format!("end of known path, gap {gap:.2} m"), String::new())
                }
                AccelSource::Barrier { lane, gap, .. } => (
                    format!("held at a barrier on lane {}, gap {gap:.2} m", lane.0),
                    String::new(),
                ),
                AccelSource::Junction { kind: ConflictKind::Cross, gap, .. } => {
                    ("crossing after ".into(), format!(", gap {gap:.2} m"))
                }
                AccelSource::Junction { kind: ConflictKind::Merge, gap, .. } => {
                    ("merging after ".into(), format!(", gap {gap:.2} m"))
                }
                AccelSource::Sync { gap, .. } => {
                    ("synchronising with ".into(), format!(", gap {gap:.2} m"))
                }
                AccelSource::Merge { gap, .. } => {
                    ("merge order after ".into(), format!(", gap {gap:.2} m"))
                }
            };
            let font = egui::TextStyle::Body.resolve(ui.style());
            let plain = egui::TextFormat::simple(font.clone(), ui.visuals().text_color());
            let mut name = egui::text::LayoutJob::default();
            name.append(&before, 0.0, plain.clone());
            if let Some(other) = named(source) {
                let colour =
                    related.iter().find(|&&(v, _)| v == other).map_or(SELECTED, |&(_, c)| c);
                name.append(&number(other), 0.0, egui::TextFormat::simple(font, colour));
            }
            name.append(&after, 0.0, plain);
            let text = egui::RichText::new(format!("{accel:+.2} m/s²")).monospace();
            let text = match accel == v.accel() {
                true => text.strong().color(SELECTED),
                false => text,
            };
            // Its description wrapped on the left, its value aligned on the
            // right, so neither runs out of the panel.
            ui.horizontal_top(|ui| {
                let value = egui::Label::new(text);
                let width = ui.available_width() - TERM_WIDTH - ui.spacing().item_spacing.x;
                ui.allocate_ui(egui::vec2(width, 0.0), |ui| {
                    ui.set_width(width);
                    ui.add(egui::Label::new(name).wrap());
                });
                ui.with_layout(egui::Layout::right_to_left(egui::Align::TOP), |ui| {
                    ui.add(value);
                });
            });
        }
        // The vehicle's acceleration is the minimum of its terms, so if none
        // matches, an input to the model wasn't recorded.
        if !terms.is_empty() && !terms.iter().any(|&(_, accel)| accel == v.accel()) {
            ui.colored_label(Color32::RED, "No recorded term matches the acceleration.");
        }
    }

    /// The selected vehicle's controls, in two rows of equal-width buttons: lane
    /// changes, then speed caps.
    fn inspector_actions(&mut self, ui: &mut egui::Ui, id: VehicleId) {
        if let Some(error) = self.lane_change_error {
            ui.colored_label(Color32::RED, error.to_string());
        }
        let cap = self.sim.speed_cap(id);
        let gap = ui.spacing().item_spacing.x;
        let size = egui::vec2(
            (ui.available_width() - gap) / 2.0,
            ui.spacing().interact_size.y,
        );
        ui.horizontal(|ui| {
            if ui.add_sized(size, egui::Button::new("Change left")).clicked() {
                self.lane_change_error = self.sim.change_lane(id, Side::Left).err();
            }
            if ui.add_sized(size, egui::Button::new("Change right")).clicked() {
                self.lane_change_error = self.sim.change_lane(id, Side::Right).err();
            }
        });
        ui.horizontal(|ui| {
            // Each a toggle, drawn as an ordinary button when off and highlighted
            // when on; turning one on replaces the other.
            for (label, max_speed) in [("Stop", 0.0), ("20 km/h", SLOW_CAP)] {
                let on = cap == Some(max_speed);
                if ui.add_sized(size, egui::Button::new(label).selected(on)).clicked() {
                    if on {
                        self.sim.release(id);
                    } else {
                        self.sim.cap_speed(id, max_speed);
                    }
                }
            }
        });
    }

    /// Selects the vehicle under `point` (scene coordinates), or clears the selection.
    fn pick(&mut self, point: Pos2, tolerance: f64) {
        self.lane_change_error = None;
        let world = DVec2::new(point.x as f64, -point.y as f64);
        self.selected = self.sim.vehicle_at(world, tolerance);
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Space toggles pause, unless a text field is being edited. Consumed
        // before the panels are drawn, so a focused button doesn't also click.
        let typing = ui.ctx().egui_wants_keyboard_input();
        if !typing && ui.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Space)) {
            self.paused = !self.paused;
        }
        // The number keys pick a speed: 1×, 2×, 5× and 20×.
        for (key, speed) in [
            (egui::Key::Num1, 1.0),
            (egui::Key::Num2, 2.0),
            (egui::Key::Num3, 5.0),
            (egui::Key::Num4, 20.0),
        ] {
            if !typing && ui.input_mut(|i| i.consume_key(egui::Modifiers::NONE, key)) {
                self.speed = speed;
            }
        }

        egui::Panel::top("controls").show(ui, |ui| self.controls(ui));

        // Always shown at a fixed width, so selecting a vehicle doesn't move the scene.
        egui::Panel::right("inspector")
            .resizable(false)
            .exact_size(320.0)
            .show(ui, |ui| {
                // Content that is too wide scrolls instead of widening the panel,
                // which would shift the scene.
                if let Some(id) = self.selected {
                    // Pinned to the bottom, so the buttons stay put as the
                    // details above them grow and shrink.
                    if self.sim.get(id).is_some() {
                        egui::Panel::bottom("vehicle actions")
                            .frame(egui::Frame::new().inner_margin(egui::Margin::symmetric(0, 6)))
                            .show(ui, |ui| self.inspector_actions(ui, id));
                    }
                    egui::ScrollArea::both().auto_shrink(false).show(ui, |ui| {
                        ui.heading("Vehicle");
                        self.inspector(ui, id);
                    });
                } else {
                    // Vertical only, so the descriptions wrap.
                    egui::ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                        ui.heading("Debug view layers");
                        ui.small("Select a vehicle to show only its marks.");
                        self.layers.legend(ui);
                    });
                }
            });

        if !self.paused {
            let dt = ui.input(|i| i.stable_dt).min(0.1) as f64;
            self.step(dt * self.speed);
            ui.ctx().request_repaint();
        }

        egui::CentralPanel::default().show(ui, |ui| {
            let background = match self.view {
                View::Pretty => Color32::from_rgb(70, 110, 60),
                View::Debug => Color32::from_gray(20),
            };
            ui.painter().rect_filled(ui.max_rect(), 0.0, background);
            let scene = egui::Scene::new().zoom_range(0.1..=200.0).scroll_zooms(true);
            let related = self.selected.map(|id| related(&self.sim, id)).unwrap_or_default();
            let response = scene
                .show(ui, &mut self.scene_rect, |ui| match self.view {
                    View::Pretty => draw_pretty(
                        ui.painter(),
                        &self.sim,
                        &self.render,
                        self.selected,
                        &related,
                        &self.poses,
                    ),
                    View::Debug => draw_debug(
                        ui.painter(),
                        &self.sim,
                        self.selected,
                        &related,
                        &self.layers,
                        &self.poses,
                    ),
                })
                .response;
            if response.clicked()
                && let Some(point) = response.hover_pos()
            {
                let to_global = ui.ctx().layer_transform_to_global(response.layer_id);
                let scale = to_global.map_or(1.0, |t| t.scaling);
                self.pick(point, (PICK_TOLERANCE / scale) as f64);
            }
        });
    }
}

fn draw_pretty(
    painter: &egui::Painter,
    sim: &Simulation<DebugState>,
    render: &RenderGeometry,
    selected: Option<VehicleId>,
    related: &[(VehicleId, Color32)],
    poses: &Poses,
) {
    let asphalt = Color32::from_gray(60);
    for surface in &render.intersections {
        painter.add(fan_mesh(surface, asphalt));
    }
    for strip in &render.roads {
        painter.add(strip_mesh(strip, asphalt));
    }
    let white = Stroke::new(0.15, Color32::from_gray(230));
    let yellow = Stroke::new(0.15, Color32::from_rgb(240, 200, 40));
    let stop = Stroke::new(0.3, Color32::from_gray(230));
    for marking in &render.markings {
        let points: Vec<Pos2> = marking.points.iter().map(|&p| to_scene(p)).collect();
        match marking.style {
            MarkingStyle::Edge => drop(painter.add(Shape::line(points, white))),
            MarkingStyle::Centre => drop(painter.add(Shape::line(points, yellow))),
            MarkingStyle::Stop => drop(painter.add(Shape::line(points, stop))),
            // 3 m dashes every 9 m.
            MarkingStyle::Dashed => {
                for dash in dashes(&marking.points, 3.0, 9.0) {
                    painter.add(Shape::line(dash.into_iter().map(to_scene).collect(), white));
                }
            }
        }
    }
    painter.add(arrows_mesh(&render.arrows, Color32::from_gray(230)));
    draw_lights(painter, sim);
    for (id, v) in sim.vehicles() {
        // Where it is drawn, between steps: the indicators go with the body.
        let drawn = poses.corners(id, v, sim.alpha());
        let corners = drawn.map(to_scene).to_vec();
        let outline = if Some(id) == selected {
            Stroke::new(0.25, SELECTED)
        } else {
            Stroke::NONE
        };
        let colour = body_colour(id, selected, related, Color32::from_rgb(200, 40, 40));
        painter.add(Shape::convex_polygon(corners, colour, outline));
        draw_indicators(painter, sim, v, drawn);
    }
}

/// How far in from the body's front and rear ends the indicators sit, in
/// meters, and their radius.
const INDICATOR_INSET: f64 = 0.4;
const INDICATOR_RADIUS: f32 = 0.28;

/// Blinking indicators on the side a vehicle is indicating to, or changing
/// toward, at about 1.5 Hz, on its body as drawn, `corners`: rear left, rear
/// right, front right, front left.
fn draw_indicators(
    painter: &egui::Painter,
    sim: &Simulation<DebugState>,
    v: &Vehicle,
    corners: [DVec2; 4],
) {
    let side = v
        .intent()
        .map(|i| i.side)
        .or(v.lane_change().and(v.old_side()).map(Side::opposite));
    let Some(side) = side else {
        return;
    };
    if (sim.time() * 1.5).fract() >= 0.5 {
        return;
    }
    let [rear_left, rear_right, front_right, front_left] = corners;
    let (rear, front) = match side {
        Side::Left => (rear_left, front_left),
        Side::Right => (rear_right, front_right),
    };
    // A little in from each end, along that side.
    let along = (front - rear).normalize_or_zero() * INDICATOR_INSET;
    let color = Color32::from_rgb(255, 170, 0);
    for p in [rear + along, front - along] {
        painter.circle_filled(to_scene(p), INDICATOR_RADIUS, color);
    }
}

fn draw_debug(
    painter: &egui::Painter,
    sim: &Simulation<DebugState>,
    selected: Option<VehicleId>,
    related: &[(VehicleId, Color32)],
    layers: &Layers,
    poses: &Poses,
) {
    // With a vehicle selected, only its own marks are drawn; otherwise everyone's.
    // Vehicle bodies and the network are always drawn.
    let shown = |id: VehicleId| selected.is_none_or(|s| s == id);
    let thin = Stroke::new(0.08, OUTLINE);
    for (_, lane) in sim.network().lanes() {
        if layers.on(Layer::LaneOutlines) {
            let geometry = lane.geometry();
            // Its sides solid, and its ends dashed and faint, where it meets the
            // lanes before and after it.
            let outline: Vec<_> = geometry.outline().collect();
            let n = outline.len();
            let (left, right) = outline.split_at(n / 2);
            for side in [left, right] {
                painter.add(Shape::line(
                    side.iter().map(|&p| to_scene(p)).collect(),
                    thin,
                ));
            }
            let faint = Stroke::new(0.08, OUTLINE.gamma_multiply(0.5));
            for ends in [
                [outline[n - 1], outline[0]],
                [left[left.len() - 1], right[0]],
            ] {
                painter.extend(Shape::dashed_line(&ends.map(to_scene), faint, 0.4, 0.3));
            }
        }

        if layers.on(Layer::Centerlines) {
            let centerline: Vec<_> = lane.geometry().centerline().map(to_scene).collect();
            for &p in &centerline {
                painter.circle_filled(p, 0.1, VERTEX);
            }
            painter.add(Shape::line(centerline, Stroke::new(0.04, CENTERLINE)));
        }

        if layers.on(Layer::Ticks) {
            let geometry = lane.geometry();
            for i in 0..=(geometry.length() / 10.0) as u32 {
                let s = 10.0 * i as f64;
                let pos = geometry.lane_pos(s, 0.0);
                let left = geometry.half_width(pos, Side::Left);
                let right = -geometry.half_width(pos, Side::Right);
                let ends = [left, right]
                    .map(|t| to_scene(geometry.lane_to_world(geometry.lane_pos(s, t))));
                painter.line_segment(ends, thin);
            }
        }

        // Every 20 m, from the centerline to each side's edge.
        if layers.on(Layer::Neighbors) {
            for side in [Side::Left, Side::Right] {
                let Some(n) = lane.adj(side) else { continue };
                let color = if n.can_change {
                    MAY_CHANGE
                } else {
                    MAY_NOT_CHANGE
                };
                let geometry = lane.geometry();
                let sign = if side == Side::Left { 1.0 } else { -1.0 };
                for i in 0..(geometry.length() / 20.0) as u32 {
                    let s = 20.0 * i as f64 + 5.0;
                    // Short of the edge, so neighbors' markers don't meet.
                    let edge = 0.9 * sign * geometry.half_width(geometry.lane_pos(s, 0.0), side);
                    let ends = [0.0, edge]
                        .map(|t| to_scene(geometry.lane_to_world(geometry.lane_pos(s, t))));
                    painter.line_segment(ends, Stroke::new(0.06, color));
                }
            }
        }
    }

    // Solid, over every lane's dashed ends.
    if layers.on(Layer::Barriers) {
        for (_, lane) in sim.network().lanes() {
            let geometry = lane.geometry();
            for barrier in lane.barriers() {
                let colour = match barrier.kind {
                    BarrierKind::Signal { movement } => match sim.light(movement).light {
                        Light::Green => LIGHT_GREEN,
                        Light::Amber => LIGHT_AMBER,
                        Light::Red => LIGHT_RED,
                    },
                    BarrierKind::GiveWay => GIVE_WAY,
                    BarrierKind::AllWayStop => ALL_WAY,
                };
                let pos = geometry.lane_pos(barrier.s, 0.0);
                let left = geometry.half_width(pos, Side::Left);
                let right = -geometry.half_width(pos, Side::Right);
                let edges =
                    [left, right].map(|t| geometry.lane_to_world(geometry.lane_pos(barrier.s, t)));
                // Along its line, if it has one, from edge to edge, the edges
                // taken as running straight on from `s`; and faintly square
                // across at `s`, where it is safe to stop.
                let ends = match barrier.line {
                    Some(line) => {
                        let tangent = geometry.tangent_at(pos);
                        let along = tangent.dot(line.normal);
                        edges.map(|e| e - line.short_of(e) / along * tangent)
                    }
                    None => edges,
                };
                if barrier.line.is_some() {
                    let faint = Stroke::new(0.08, colour.gamma_multiply(0.5));
                    painter.line_segment(edges.map(to_scene), faint);
                }
                painter.line_segment(ends.map(to_scene), Stroke::new(0.3, colour));
            }
        }
    }

    // Every mark on a vehicle is placed on its body as drawn, between steps, so
    // they stay together.
    let drawn = |id: VehicleId, v: &Vehicle| Drawn(poses.corners(id, v, sim.alpha()));
    for (id, v) in sim.vehicles() {
        if layers.on(Layer::Vehicles) {
            let color = body_colour(id, selected, related, VEHICLE);
            let body = drawn(id, v);
            painter.add(Shape::convex_polygon(
                body.0.map(to_scene).to_vec(),
                color,
                Stroke::NONE,
            ));
            // Its rear axle, across the body.
            let axle = body.axle(v.spec().rear_axle);
            painter.line_segment(axle.map(to_scene), Stroke::new(0.12, AXLE));
        }

        // From the vehicle's stored cubic, drawn on its current lane only, so it
        // stops short if it crosses a lane's end.
        if layers.on(Layer::LaneChanges)
            && shown(id)
            && let Some(change) = v.lane_change()
        {
            let lane = sim.network().lane(v.lane());
            let samples = (0..=40).map(|i| change.s0 + change.length * i as f64 / 40.0);
            let points = samples
                .take_while(|&s| s <= lane.geometry().length())
                .map(|s| {
                    to_scene(
                        lane.geometry()
                            .lane_to_world(lane.geometry().lane_pos(s, change.offset_at(s))),
                    )
                })
                .collect();
            painter.add(Shape::line(points, Stroke::new(0.08, LANE_CHANGE)));
        }
    }

    if layers.on(Layer::Occupants) {
        for (lane_id, lane) in sim.network().lanes() {
            for occupant in sim.occupants(lane_id).iter().filter(|o| shown(o.id)) {
                let p = lane.geometry().lane_to_world(lane.geometry().lane_pos(occupant.s(), 0.0));
                let color = if occupant.is_ghost() { GHOST } else { OCCUPANT };
                painter.circle_filled(to_scene(p), 0.25, color);
            }
        }
    }

    // From the front bumper to the leader's rear bumper.
    if layers.on(Layer::Leaders) {
        for &(id, source, accel) in sim.sink().accel_terms.iter().filter(|t| shown(t.0)) {
            let AccelSource::Leader { id: leader, .. } = source else {
                continue;
            };
            let (Some(v), Some(l)) = (sim.get(id), sim.get(leader)) else {
                continue;
            };
            let color = if accel == v.accel() {
                LEADER_BINDING
            } else {
                LEADER_OTHER
            };
            painter.line_segment(
                [
                    to_scene(drawn(id, v).front()),
                    to_scene(drawn(leader, l).rear()),
                ],
                Stroke::new(0.08, color),
            );
        }
    }

    // Dashed, from the front bumper to the rear bumper of the vehicle before it in
    // a merge's order, which it keeps behind as if on its own lane.
    if layers.on(Layer::Leaders) {
        for &(id, source, accel) in sim.sink().accel_terms.iter().filter(|t| shown(t.0)) {
            let AccelSource::Merge { id: leader, .. } = source else {
                continue;
            };
            let (Some(v), Some(l)) = (sim.get(id), sim.get(leader)) else {
                continue;
            };
            let color = if accel == v.accel() {
                LEADER_BINDING
            } else {
                LEADER_OTHER
            };
            let ends = [
                to_scene(drawn(id, v).front()),
                to_scene(drawn(leader, l).rear()),
            ];
            painter.extend(Shape::dashed_line(
                &ends,
                Stroke::new(0.08, color),
                0.6,
                0.4,
            ));
        }
    }

    // In world space, as the check is made: the follower's predicted body as its
    // front reaches the other vehicle's rear, that vehicle's nearer rear corner,
    // and the gap between them across the follower's heading.
    if layers.on(Layer::PassChecks) {
        for &(id, check) in sim.sink().pass_checks.iter().filter(|c| shown(c.0)) {
            let Some(v) = sim.get(id) else { continue };
            let (u, n) = (check.tangent, check.tangent.perp());
            let (back, ahead) = (-v.spec().rear_axle, v.spec().length - v.spec().rear_axle);
            let half = 0.5 * v.spec().width;
            let body = [(back, half), (ahead, half), (ahead, -half), (back, -half)]
                .map(|(x, y)| to_scene(check.pos + u * x + n * y))
                .to_vec();
            painter.add(Shape::closed_line(body, Stroke::new(0.08, PASS_BODY)));
            painter.circle_filled(to_scene(check.corner), 0.25, PASS_CORNER);
            // From the corner straight across to the body's near edge, extended
            // ahead: a negative gap is an overlap, drawn the same way in red.
            let along = (check.corner - check.pos).dot(u);
            let edge = match check.side {
                Side::Left => -half,
                Side::Right => half,
            };
            let foot = check.pos + u * along + n * edge;
            let color = if check.passed { PASS_OK } else { PASS_BLOCKED };
            painter.line_segment(
                [to_scene(check.corner), to_scene(foot)],
                Stroke::new(0.1, color),
            );
            painter.line_segment(
                [
                    to_scene(drawn(id, v).front()),
                    to_scene(check.pos + u * ahead),
                ],
                Stroke::new(0.03, PASS_BODY),
            );
        }
    }

    if layers.on(Layer::Lookahead) {
        for &(id, point) in sim.sink().lookaheads.iter().filter(|l| shown(l.0)) {
            let Some(v) = sim.get(id) else { continue };
            let axle = drawn(id, v).axle_mid(v.spec().rear_axle);
            painter.line_segment(
                [to_scene(axle), to_scene(point)],
                Stroke::new(0.04, LOOKAHEAD),
            );
            painter.circle_filled(to_scene(point), 0.2, LOOKAHEAD);
        }
    }
}

// Debug view colors, shared by the drawing and the legend.
const OUTLINE: Color32 = Color32::from_gray(160);
const VERTEX: Color32 = Color32::from_gray(200);
const CENTERLINE: Color32 = Color32::from_gray(110);
const MAY_CHANGE: Color32 = Color32::from_rgb(80, 200, 80);
const MAY_NOT_CHANGE: Color32 = Color32::from_rgb(220, 70, 70);
const OCCUPANT: Color32 = Color32::from_rgb(200, 160, 255);
const GHOST: Color32 = Color32::from_rgb(255, 150, 40);
const VEHICLE: Color32 = Color32::from_rgb(220, 40, 40);
/// Half-transparent white.
const AXLE: Color32 = Color32::from_rgba_premultiplied(128, 128, 128, 128);
const LANE_CHANGE: Color32 = Color32::from_rgb(250, 160, 60);
const LEADER_BINDING: Color32 = Color32::from_rgb(120, 230, 120);
const LEADER_OTHER: Color32 = Color32::from_rgb(60, 110, 60);
const LOOKAHEAD: Color32 = Color32::from_rgb(90, 200, 240);
const LIGHT_GREEN: Color32 = Color32::from_rgb(60, 220, 90);
const LIGHT_AMBER: Color32 = Color32::from_rgb(250, 190, 40);
const LIGHT_RED: Color32 = Color32::from_rgb(230, 60, 50);
const GIVE_WAY: Color32 = Color32::from_rgb(240, 240, 240);
const ALL_WAY: Color32 = Color32::from_rgb(190, 120, 255);
const PASS_CORNER: Color32 = Color32::from_rgb(255, 120, 200);
const PASS_BODY: Color32 = Color32::from_rgb(120, 170, 255);
const PASS_OK: Color32 = Color32::from_rgb(80, 220, 80);
const PASS_BLOCKED: Color32 = Color32::from_rgb(230, 60, 60);

/// A debug view layer that can be turned on and off.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Layer {
    LaneOutlines,
    Centerlines,
    Ticks,
    Neighbors,
    Occupants,
    Vehicles,
    LaneChanges,
    Leaders,
    PassChecks,
    Lookahead,
    Barriers,
}

impl Layer {
    const ALL: [Layer; 11] = [
        Layer::LaneOutlines,
        Layer::Centerlines,
        Layer::Ticks,
        Layer::Neighbors,
        Layer::Occupants,
        Layer::Vehicles,
        Layer::LaneChanges,
        Layer::Leaders,
        Layer::PassChecks,
        Layer::Lookahead,
        Layer::Barriers,
    ];

    /// Its name, its colors, and what they mean, for the legend.
    fn legend(self) -> (&'static str, &'static [Color32], &'static str) {
        match self {
            Layer::LaneOutlines => (
                "Lane outlines",
                &[OUTLINE],
                "Each lane's edges, and its ends dashed and fainter",
            ),
            Layer::Centerlines => (
                "Centerlines",
                &[CENTERLINE, VERTEX],
                "Each lane's centerline and its vertices",
            ),
            Layer::Ticks => ("Ticks", &[OUTLINE], "Across each lane every 10 m of s"),
            Layer::Neighbors => (
                "Neighbors",
                &[MAY_CHANGE, MAY_NOT_CHANGE],
                "Links to each side's neighbor: green where changing into it is allowed, red where not",
            ),
            Layer::Occupants => (
                "Occupants",
                &[OCCUPANT, GHOST],
                "Each occupant record, on the centerline at its s; orange for the ghost of a vehicle indicating into that lane",
            ),
            Layer::Vehicles => (
                "Vehicles",
                &[VEHICLE, SELECTED],
                "Solid bodies, each with its rear axle across it; the selected vehicle in yellow, those its acceleration terms name each in its own colour, as the inspector names them, and the rest white",
            ),
            Layer::LaneChanges => (
                "Lane changes",
                &[LANE_CHANGE],
                "The cubic path of each lane change in progress",
            ),
            Layer::Leaders => (
                "Leaders",
                &[LEADER_BINDING, LEADER_OTHER],
                "Front bumper to each leader's rear: bright when that leader sets the acceleration",
            ),
            Layer::PassChecks => (
                "Passing checks",
                &[PASS_CORNER, PASS_BODY, PASS_OK, PASS_BLOCKED],
                "Where a follower checked whether it can pass a vehicle ahead: its predicted body as its front reaches that vehicle's rear, that vehicle's nearer rear corner, and the gap from the corner across to the follower's edge, green if wide enough to pass and red if not. Only pairs where the follower could pass are checked: a vehicle moving out of the lane, or one the follower is moving away from. A fine line runs from the follower to its predicted body",
            ),
            Layer::Barriers => (
                "Barriers",
                &[LIGHT_GREEN, LIGHT_AMBER, LIGHT_RED, GIVE_WAY, ALL_WAY],
                "Each barrier across its lane: a signal in its light's colour, a give-way line in white, an all-way stop line in violet",
            ),
            Layer::Lookahead => (
                "Lookahead",
                &[LOOKAHEAD],
                "Rear axle to the point pure pursuit is steering toward",
            ),
        }
    }
}

/// Which debug view layers are on.
struct Layers([bool; Layer::ALL.len()]);

impl Default for Layers {
    fn default() -> Self {
        let mut layers = [false; Layer::ALL.len()];
        for layer in [
            Layer::LaneOutlines,
            Layer::Vehicles,
            Layer::LaneChanges,
            Layer::PassChecks,
            Layer::Leaders,
            Layer::Barriers,
        ] {
            layers[layer as usize] = true;
        }
        Self(layers)
    }
}

impl Layers {
    fn on(&self, layer: Layer) -> bool {
        self.0[layer as usize]
    }

    /// A checkbox per layer, with its colors and what they mean.
    fn legend(&mut self, ui: &mut egui::Ui) {
        for layer in Layer::ALL {
            let (name, colors, meaning) = layer.legend();
            ui.horizontal(|ui| {
                ui.checkbox(&mut self.0[layer as usize], name);
                for &color in colors {
                    let (rect, _) =
                        ui.allocate_exact_size(egui::vec2(12.0, 12.0), egui::Sense::hover());
                    ui.painter().rect_filled(rect, 2.0, color);
                }
            });
            ui.indent(name, |ui| ui.small(meaning));
        }
    }
}

/// Accumulates the simulation's debug events. Each group is cleared by its reset
/// event, so it always holds exactly what the last update of that group used.
#[derive(Default)]
struct DebugState {
    /// The last longitudinal update's terms: vehicle, what it reacted to, acceleration.
    accel_terms: Vec<(VehicleId, AccelSource, f64)>,
    /// The last motion update's lookahead points.
    lookaheads: Vec<(VehicleId, DVec2)>,
    /// Each vehicle's evaluations from its last lane-change decision.
    decisions: HashMap<VehicleId, Vec<LaneChangeEval>>,
    /// The last longitudinal update's checks for passing a diverging vehicle.
    pass_checks: Vec<(VehicleId, PassCheck)>,
    /// The last longitudinal update's barrier decisions, by vehicle: the lane,
    /// the barrier's index there, the gap to it, and what it did.
    barriers: Vec<(VehicleId, LaneId, u16, f64, BarrierDecision)>,
}

impl DebugSink for DebugState {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::RecomputeAccels => {
                self.accel_terms.clear();
                self.pass_checks.clear();
                self.barriers.clear();
            }
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.accel_terms.push((vehicle, source, accel))
            }
            DebugEvent::RecomputeMotion => self.lookaheads.clear(),
            DebugEvent::Lookahead { vehicle, point } => self.lookaheads.push((vehicle, point)),
            DebugEvent::RecomputeDecision { vehicle } => {
                self.decisions.entry(vehicle).or_default().clear()
            }
            DebugEvent::LaneChangeEval { vehicle, eval } => {
                self.decisions.entry(vehicle).or_default().push(eval)
            }
            DebugEvent::VehicleRemoved { vehicle } => {
                self.decisions.remove(&vehicle);
            }
            DebugEvent::PassCheck { vehicle, check } => self.pass_checks.push((vehicle, check)),
            DebugEvent::Barrier { vehicle, lane, index, gap, decision } => {
                self.barriers.push((vehicle, lane, index, gap, decision))
            }
        }
    }
}

impl DebugState {
    /// One vehicle's terms from the last longitudinal update.
    fn terms(&self, id: VehicleId) -> impl Iterator<Item = (AccelSource, f64)> + '_ {
        self.accel_terms
            .iter()
            .filter(move |t| t.0 == id)
            .map(|&(_, source, accel)| (source, accel))
    }
}

/// A vehicle's corners where it is drawn: rear left, rear right, front right,
/// front left.
#[derive(Clone, Copy)]
struct Drawn([DVec2; 4]);

impl Drawn {
    /// Midpoint of the front bumper.
    fn front(self) -> DVec2 {
        0.5 * (self.0[2] + self.0[3])
    }

    /// Midpoint of the rear bumper.
    fn rear(self) -> DVec2 {
        0.5 * (self.0[0] + self.0[1])
    }

    /// The rear axle's ends, at the sides of the body, `rear_axle` ahead of the
    /// rear bumper.
    fn axle(self, rear_axle: f64) -> [DVec2; 2] {
        let length = self.0[0].distance(self.0[3]);
        let ahead = if length > 0.0 {
            rear_axle / length
        } else {
            0.0
        };
        [
            self.0[0].lerp(self.0[3], ahead),
            self.0[1].lerp(self.0[2], ahead),
        ]
    }

    /// Midpoint of the rear axle.
    fn axle_mid(self, rear_axle: f64) -> DVec2 {
        let [left, right] = self.axle(rear_axle);
        0.5 * (left + right)
    }
}

/// A polygon every point of which can see its centroid, as a fan of triangles
/// from there.
fn fan_mesh(points: &[DVec2], color: Color32) -> Shape {
    let mut mesh = egui::Mesh::default();
    if points.is_empty() {
        return Shape::mesh(mesh);
    }
    let centre = points.iter().sum::<DVec2>() / points.len() as f64;
    mesh.colored_vertex(to_scene(centre), color);
    for &p in points {
        mesh.colored_vertex(to_scene(p), color);
    }
    let n = points.len() as u32;
    for i in 0..n {
        mesh.add_triangle(0, 1 + i, 1 + (i + 1) % n);
    }
    Shape::mesh(mesh)
}

/// Each signal barrier's lane, from the barrier on, in its movement's light's
/// colour while green or amber, and nothing while red, as the old visualiser
/// drew them: the ways through the junction open now. A connector split where
/// it waits for a gap, a far-side turn's, goes on in white from its give-way
/// line: open, but giving way.
fn draw_lights(painter: &egui::Painter, sim: &Simulation<DebugState>) {
    let network = sim.network();
    let line = |lane: &traffic_sim::Lane, from: f64, colour: Color32| {
        let geometry = lane.geometry();
        let ahead = geometry.vertex_s().zip(geometry.centerline());
        let points: Vec<Pos2> =
            ahead.filter(|&(s, _)| s >= from).map(|(_, p)| to_scene(p)).collect();
        painter.add(Shape::line(points, Stroke::new(0.6, colour)));
    };
    for (_, lane) in network.lanes() {
        for barrier in lane.barriers() {
            let BarrierKind::Signal { movement } = barrier.kind else {
                continue;
            };
            let colour = match sim.light(movement).light {
                Light::Green => Color32::from_rgba_unmultiplied(60, 220, 90, 150),
                Light::Amber => Color32::from_rgba_unmultiplied(250, 190, 40, 170),
                Light::Red => continue,
            };
            line(lane, barrier.s, colour);
            // On through the rest of its connector, the lanes after it in the
            // junction, each the only way on from the last.
            let (mut on, mut colour) = (lane, colour);
            while let [next] = on.successors()[..] {
                let next_lane = network.lane(next);
                if next_lane.junction().is_none() || next_lane.junction() != on.junction() {
                    break;
                }
                let gives_way = next_lane.barriers().iter().any(|b| b.kind == BarrierKind::GiveWay);
                if gives_way {
                    colour = Color32::from_rgba_unmultiplied(240, 240, 240, 170);
                }
                line(next_lane, 0.0, colour);
                on = next_lane;
            }
        }
    }
}

/// Turn arrows, each its glyph turned to its heading and placed at its anchor.
fn arrows_mesh(arrows: &[Arrow], color: Color32) -> Shape {
    let mut mesh = egui::Mesh::default();
    for arrow in arrows {
        let glyph = match (arrow.left, arrow.straight, arrow.right) {
            (true, false, false) => &glyphs::TL,
            (false, false, true) => &glyphs::TR,
            (true, false, true) => &glyphs::TLR,
            (true, true, false) => &glyphs::TSL,
            (false, true, true) => &glyphs::TSR,
            (true, true, true) => &glyphs::TSLR,
            (false, true, false) => &glyphs::TS,
            (false, false, false) => continue,
        };
        // The glyph's +y along the heading, and its +x to the right of it.
        let ahead = DVec2::from_angle(arrow.heading);
        let right = DVec2::new(ahead.y, -ahead.x);
        let base = mesh.vertices.len() as u32;
        for &[x, y] in glyph.vertices {
            let p = arrow.pos + x as f64 * right + y as f64 * ahead;
            mesh.colored_vertex(to_scene(p), color);
        }
        for &[a, b, c] in glyph.triangles {
            mesh.add_triangle(base + a as u32, base + b as u32, base + c as u32);
        }
    }
    Shape::mesh(mesh)
}

/// A road's surface as a mesh of quads between its edges.
fn strip_mesh(strip: &Strip, color: Color32) -> Shape {
    let mut mesh = egui::Mesh::default();
    for (&l, &r) in strip.left.iter().zip(&strip.right) {
        mesh.colored_vertex(to_scene(l), color);
        mesh.colored_vertex(to_scene(r), color);
    }
    for i in 0..strip.left.len().saturating_sub(1) as u32 {
        let (l0, r0, l1, r1) = (2 * i, 2 * i + 1, 2 * i + 2, 2 * i + 3);
        mesh.add_triangle(l0, r0, r1);
        mesh.add_triangle(l0, r1, l1);
    }
    Shape::mesh(mesh)
}

/// Dashes `on` meters long every `period` meters along a polyline, each as a
/// polyline of its own, following bends.
fn dashes(points: &[DVec2], on: f64, period: f64) -> Vec<Vec<DVec2>> {
    let mut out: Vec<Vec<DVec2>> = Vec::new();
    let mut s = 0.0;
    for w in points.windows(2) {
        let length = w[0].distance(w[1]);
        if length == 0.0 {
            continue;
        }
        let at = |d: f64| w[0] + (w[1] - w[0]) * ((d - s) / length);
        // Each dash that overlaps this segment, cut to it.
        let mut k = (s / period).floor();
        while k * period < s + length {
            let (a, b) = ((k * period).max(s), (k * period + on).min(s + length));
            if a < b {
                let continues = out.last().is_some_and(|d| d.last() == Some(&at(a)));
                if !continues {
                    out.push(vec![at(a)]);
                }
                out.last_mut().unwrap().push(at(b));
            }
            k += 1.0;
        }
        s += length;
    }
    out
}

/// World meters (y up) to scene coordinates (y down).
fn to_scene(p: DVec2) -> Pos2 {
    Pos2::new(p.x as f32, -p.y as f32)
}

/// An empty simulation of a scenario's network, and what to draw for it; the
/// harness fills it.
fn new_sim(scenario: Scenario) -> (Recorded<DebugState>, RenderGeometry) {
    let compiled = compile(&scenario.network()).expect("every scenario compiles");
    let sim = Recorded::new(Simulation::new(compiled.sim, DebugState::default()));
    (sim, compiled.render)
}

fn demo_car() -> VehicleSpec {
    VehicleSpec::passenger_car()
}
