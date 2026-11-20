//! egui app with a pretty view and a debug view of the simulation.

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
use traffic_network::{MarkingStyle, RenderGeometry, RoadNetwork, Strip, compile, scenarios};
use traffic_sim::{
    AccelSource, ConflictKind, DebugEvent, DebugSink, LaneChangeError, LaneChangeEval, LaneId,
    PassCheck, Placement, Route, Side, Simulation, Vehicle, VehicleId, VehicleSpec,
};

/// The inner lane's circumference: about 160 m across, roomy enough for
/// vehicles to accelerate, settle into gaps and change lanes.
const RING_CIRCUMFERENCE: f64 = 1000.0;
const RING_LANES: usize = 3;
const SPEED_LIMIT: f64 = 50.0 / 3.6;

/// Most spawns or despawns per simulated second, while the count is off target.
const CHANGE_RATE: f64 = 2.0;
/// The speed the inspector's slow button imposes, in m/s: 20 km/h.
const SLOW_CAP: f64 = 20.0 / 3.6;
/// How far outside a vehicle a click still selects it, in screen points.
const PICK_TOLERANCE: f32 = 6.0;

const SELECTED: Color32 = Color32::from_rgb(250, 210, 60);

fn main() -> eframe::Result {
    eframe::run_native(
        "Traffic Simulator",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(App::new()))),
    )
}

/// A network to run, and how vehicles come and go on it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Scenario {
    Ring,
    /// Two two-way roads crossing, with this many lanes each way.
    Crossroads(usize),
    /// A freeway with an on-ramp and an off-ramp.
    Ramps,
}

impl Scenario {
    const ALL: [Scenario; 5] = [
        Scenario::Ring,
        Scenario::Crossroads(1),
        Scenario::Crossroads(2),
        Scenario::Crossroads(3),
        Scenario::Ramps,
    ];

    fn name(self) -> &'static str {
        match self {
            Scenario::Ring => "Three-lane ring",
            Scenario::Crossroads(1) => "Crossroads of 2-lane roads",
            Scenario::Crossroads(2) => "Crossroads of 4-lane roads",
            Scenario::Crossroads(_) => "Crossroads of 6-lane roads",
            Scenario::Ramps => "Freeway with ramps",
        }
    }

    fn network(self) -> RoadNetwork {
        match self {
            Scenario::Ring => scenarios::ring(RING_CIRCUMFERENCE, RING_LANES, SPEED_LIMIT),
            Scenario::Crossroads(lanes) => scenarios::crossroads(lanes, SPEED_LIMIT),
            Scenario::Ramps => scenarios::ramps(100.0 / 3.6, 80.0 / 3.6),
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
        // Drivers vary in how hard they'll brake to let someone in, and in how
        // much they weigh others when changing lanes.
        let car = VehicleSpec {
            speed_factor: self.speed_factor(),
            courtesy: 2.0 * self.rng.random::<f64>(),
            politeness: 0.1 + 0.3 * self.rng.random::<f64>(),
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
    fn new() -> Self {
        let scenario = Scenario::Ring;
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
        self.sim.step(dt);
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
            "snapshot": self.sim.snapshot(),
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
                (3.0, "3×"),
                (4.0, "4×"),
                (5.0, "5×"),
                (10.0, "10×"),
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
            let mut row = |name: &str, value: String| {
                ui.label(name);
                ui.monospace(value);
                ui.end_row();
            };
            row("id", format!("{id:?}"));
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
                format!("{:.2} m/s ({:.0} km/h)", v.speed(), v.speed() * 3.6),
            );
            row("accel", format!("{:+.2} m/s²", v.accel()));
            row("curvature", format!("{:+.4} /m", v.curvature()));
        });

        ui.separator();
        ui.label("Last lane-change decision (MOBIL)");
        let evals = self.sim.sink().decisions.get(&id).cloned().unwrap_or_default();
        egui::Grid::new("decision").num_columns(2).show(ui, |ui| {
            for eval in &evals {
                let side = match eval.side {
                    Side::Left => "left",
                    Side::Right => "right",
                };
                // Only while indicating: the change for the new and old followers,
                // if any is in reach.
                let replies = match (eval.new_follower, eval.old_follower) {
                    (None, None) => String::new(),
                    (new, old) => format!(
                        ", followers {} / {}",
                        new.map_or("none".into(), |x| format!("{x:+.2}")),
                        old.map_or("none".into(), |x| format!("{x:+.2}"))
                    ),
                };
                ui.label(format!("{side}: {:?}", eval.outcome));
                ui.small(format!(
                    "gain {:+.2}{replies}, net {:+.2}, built up {:.2}/{:.2}, frustrated {:.1} s, accepts {:.1} m/s²{}",
                    eval.own_gain,
                    eval.incentive,
                    eval.advantage,
                    traffic_sim::LANE_CHANGE_INTENT,
                    eval.frustration,
                    eval.accepted_braking,
                    if eval.safe { "" } else { ", unsafe" }
                ));
                ui.end_row();
            }
        });
        if evals.is_empty() {
            ui.small("No permitted lane to either side.");
        }

        ui.separator();
        ui.label("Acceleration terms (minimum binds)");
        let terms: Vec<_> = self.sim.sink().terms(id).collect();
        egui::Grid::new("terms").num_columns(2).show(ui, |ui| {
            for &(source, accel) in &terms {
                let name = match source {
                    AccelSource::FreeRoad { desired_speed } => {
                        format!("free road, v0 {desired_speed:.1} m/s")
                    }
                    AccelSource::Leader { id, gap } => format!("leader {id:?}, gap {gap:.2} m"),
                    AccelSource::Cap { max_speed: 0.0 } => "held to a stop".into(),
                    AccelSource::Cap { max_speed } => {
                        format!("capped at {:.0} km/h", max_speed * 3.6)
                    }
                    AccelSource::Yield { id, gap } => {
                        format!("making room for {id:?}, gap {gap:.2} m")
                    }
                    AccelSource::PathEnd { gap } => format!("end of known path, gap {gap:.2} m"),
                    AccelSource::Junction { id, kind: ConflictKind::Cross, gap } => {
                        format!("crossing after {id:?}, gap {gap:.2} m")
                    }
                    AccelSource::Junction { id, kind: ConflictKind::Merge, gap } => {
                        format!("merging after {id:?}, gap {gap:.2} m")
                    }
                    AccelSource::Merge { id, gap } => {
                        format!("merge order after {id:?}, gap {gap:.2} m")
                    }
                };
                let text = egui::RichText::new(format!("{accel:+.2} m/s²")).monospace();
                ui.label(name);
                if accel == v.accel() {
                    ui.label(text.strong().color(SELECTED));
                } else {
                    ui.label(text);
                }
                ui.end_row();
            }
        });
        // The vehicle's acceleration is the minimum of its terms, so if none
        // matches, an input to the model wasn't recorded.
        if !terms.is_empty() && !terms.iter().any(|&(_, accel)| accel == v.accel()) {
            ui.colored_label(Color32::RED, "No recorded term matches the acceleration.");
        }
    }

    /// The selected vehicle's controls, in one row of equal-width buttons.
    fn inspector_actions(&mut self, ui: &mut egui::Ui, id: VehicleId) {
        if let Some(error) = self.lane_change_error {
            ui.colored_label(Color32::RED, error.to_string());
        }
        let cap = self.sim.speed_cap(id);
        let gap = ui.spacing().item_spacing.x;
        let size = egui::vec2(
            (ui.available_width() - 3.0 * gap) / 4.0,
            ui.spacing().interact_size.y,
        );
        ui.horizontal(|ui| {
            if ui.add_sized(size, egui::Button::new("Change left")).clicked() {
                self.lane_change_error = self.sim.change_lane(id, Side::Left).err();
            }
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
            if ui.add_sized(size, egui::Button::new("Change right")).clicked() {
                self.lane_change_error = self.sim.change_lane(id, Side::Right).err();
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
            let response = scene
                .show(ui, &mut self.scene_rect, |ui| match self.view {
                    View::Pretty => {
                        draw_pretty(ui.painter(), &self.sim, &self.render, self.selected)
                    }
                    View::Debug => draw_debug(ui.painter(), &self.sim, self.selected, &self.layers),
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
    for marking in &render.markings {
        let points: Vec<Pos2> = marking.points.iter().map(|&p| to_scene(p)).collect();
        match marking.style {
            MarkingStyle::Edge => drop(painter.add(Shape::line(points, white))),
            MarkingStyle::Centre => drop(painter.add(Shape::line(points, yellow))),
            // 3 m dashes every 9 m.
            MarkingStyle::Dashed => {
                for dash in dashes(&marking.points, 3.0, 9.0) {
                    painter.add(Shape::line(dash.into_iter().map(to_scene).collect(), white));
                }
            }
        }
    }
    for (id, v) in sim.vehicles() {
        let corners = v.corners().map(to_scene).to_vec();
        let outline = if Some(id) == selected {
            Stroke::new(0.25, SELECTED)
        } else {
            Stroke::NONE
        };
        painter.add(Shape::convex_polygon(
            corners,
            Color32::from_rgb(200, 40, 40),
            outline,
        ));
        draw_indicators(painter, sim, v);
    }
}

/// Blinking indicators on the side a vehicle is indicating to, or changing
/// toward, at about 1.5 Hz.
fn draw_indicators(painter: &egui::Painter, sim: &Simulation<DebugState>, v: &Vehicle) {
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
    // Corners: rear left, rear right, front right, front left.
    let [rear_left, rear_right, front_right, front_left] = v.corners();
    let (rear, front) = match side {
        Side::Left => (rear_left, front_left),
        Side::Right => (rear_right, front_right),
    };
    let color = Color32::from_rgb(255, 170, 0);
    for p in [rear, front] {
        painter.circle_filled(to_scene(p), 0.35, color);
    }
}

fn draw_debug(
    painter: &egui::Painter,
    sim: &Simulation<DebugState>,
    selected: Option<VehicleId>,
    layers: &Layers,
) {
    // With a vehicle selected, only its own marks are drawn; otherwise everyone's.
    // Vehicle bodies and the network are always drawn.
    let shown = |id: VehicleId| selected.is_none_or(|s| s == id);
    let thin = Stroke::new(0.08, OUTLINE);
    for (lane_id, lane) in sim.network().lanes() {
        if layers.on(Layer::LaneOutlines) {
            painter.add(Shape::closed_line(
                lane.geometry().outline().map(to_scene).collect(),
                thin,
            ));
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

        if layers.on(Layer::Occupants) {
            for occupant in sim.occupants(lane_id).iter().filter(|o| shown(o.id)) {
                let p = lane.geometry().lane_to_world(lane.geometry().lane_pos(occupant.s(), 0.0));
                let color = if occupant.is_ghost() { GHOST } else { OCCUPANT };
                painter.circle_filled(to_scene(p), 0.25, color);
            }
        }
    }

    for (id, v) in sim.vehicles() {
        if layers.on(Layer::Vehicles) {
            let color = if Some(id) == selected {
                SELECTED
            } else {
                VEHICLE
            };
            let corners = v.corners().map(to_scene).to_vec();
            painter.add(Shape::closed_line(corners, Stroke::new(0.1, color)));
            painter.circle_filled(to_scene(v.pos()), 0.2, color);
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

    // From the front bumper to the leader's rear bumper.
    if layers.on(Layer::Leaders) {
        for &(id, source, accel) in sim.sink().accel_terms.iter().filter(|t| shown(t.0)) {
            let AccelSource::Leader { id: leader, .. } = source else {
                continue;
            };
            let (Some(v), Some(leader)) = (sim.get(id), sim.get(leader)) else {
                continue;
            };
            let color = if accel == v.accel() {
                LEADER_BINDING
            } else {
                LEADER_OTHER
            };
            painter.line_segment(
                [to_scene(front(v)), to_scene(rear(leader))],
                Stroke::new(0.08, color),
            );
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
                [to_scene(front(v)), to_scene(check.pos + u * ahead)],
                Stroke::new(0.03, PASS_BODY),
            );
        }
    }

    if layers.on(Layer::Lookahead) {
        for &(id, point) in sim.sink().lookaheads.iter().filter(|l| shown(l.0)) {
            let Some(v) = sim.get(id) else { continue };
            painter.line_segment(
                [to_scene(v.pos()), to_scene(point)],
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
const VEHICLE: Color32 = Color32::from_rgb(240, 90, 90);
const LANE_CHANGE: Color32 = Color32::from_rgb(250, 160, 60);
const LEADER_BINDING: Color32 = Color32::from_rgb(120, 230, 120);
const LEADER_OTHER: Color32 = Color32::from_rgb(60, 110, 60);
const LOOKAHEAD: Color32 = Color32::from_rgb(90, 200, 240);
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
}

impl Layer {
    const ALL: [Layer; 10] = [
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
    ];

    /// Its name, its colors, and what they mean, for the legend.
    fn legend(self) -> (&'static str, &'static [Color32], &'static str) {
        match self {
            Layer::LaneOutlines => ("Lane outlines", &[OUTLINE], "Each lane's edges"),
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
                "Bodies and rear axles; the selected vehicle in yellow",
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
}

impl DebugSink for DebugState {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::RecomputeAccels => {
                self.accel_terms.clear();
                self.pass_checks.clear();
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

/// Midpoint of the front bumper.
fn front(v: &Vehicle) -> DVec2 {
    let [_, _, front_right, front_left] = v.corners();
    0.5 * (front_right + front_left)
}

/// Midpoint of the rear bumper.
fn rear(v: &Vehicle) -> DVec2 {
    let [rear_left, rear_right, ..] = v.corners();
    0.5 * (rear_left + rear_right)
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
    VehicleSpec {
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
    }
}
