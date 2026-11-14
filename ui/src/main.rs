//! egui app with a pretty view and a debug view of the simulation.

use std::collections::HashMap;
use std::f64::consts::TAU;

use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use glam::DVec2;
use traffic_sim::{
    AccelSource, DebugEvent, DebugSink, Lane, LaneChangeError, LaneId, Placement, Side, SimNetwork,
    Simulation, Vehicle, VehicleId, VehicleSpec,
};

/// The inner lane's circumference, as in Sugiyama et al. (2008).
const RING_CIRCUMFERENCE: f64 = 230.0;
const RING_LANES: usize = 3;
const LANE_WIDTH: f64 = 3.5;
const SPEED_LIMIT: f64 = 50.0 / 3.6;

/// Vehicles enter the outer (rightmost) lane and leave from the inner (leftmost)
/// one, so they have to change lanes leftward to leave.
const SPAWN_LANE: LaneId = LaneId(RING_LANES as u32 - 1);
const DESPAWN_LANE: LaneId = LaneId(0);
/// Where the spawn and despawn points are, as fractions of their lanes' lengths:
/// half a lap apart.
const SPAWN_AT: f64 = 0.25;
const DESPAWN_AT: f64 = 0.75;
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

#[derive(Clone, Copy, PartialEq, Eq)]
enum View {
    Pretty,
    Debug,
}

struct App {
    sim: Simulation<DebugState>,
    view: View,
    paused: bool,
    /// Simulated seconds per real second.
    speed: f64,
    /// Visible region of the world, in scene coordinates (see [`to_scene`]).
    scene_rect: Rect,
    selected: Option<VehicleId>,
    /// Whether the inspector's brake button is held down.
    braking: bool,
    /// Which debug view layers are drawn.
    layers: Layers,
    /// Why the last lane-change request for the selected vehicle was refused.
    lane_change_error: Option<LaneChangeError>,
    harness: Harness,
}

/// Spawns vehicles into the outer lane and despawns them from the inner lane,
/// each at a rate.
struct Harness {
    /// Spawns per simulated second.
    spawn_rate: f64,
    /// Despawns per simulated second.
    despawn_rate: f64,
    /// Spawns owed: grows at the spawn rate, and a refused spawn waits here.
    spawn_credit: f64,
    /// Despawns owed: grows at the despawn rate, spent as inner-lane vehicles
    /// pass the despawn point.
    despawn_credit: f64,
    spawned: u32,
    despawned: u32,
    /// Each vehicle's `s` last step, to see who crossed the despawn point.
    last_s: HashMap<VehicleId, f64>,
    /// State of a xorshift random number generator, for drivers' speed factors.
    rng: u64,
}

impl Default for Harness {
    fn default() -> Self {
        Self {
            spawn_rate: 0.5,
            despawn_rate: 0.3,
            spawn_credit: 0.0,
            despawn_credit: 0.0,
            spawned: 0,
            despawned: 0,
            last_s: HashMap::new(),
            rng: 0x9E37_79B9_7F4A_7C15,
        }
    }
}

impl Harness {
    /// Runs after each simulation step of `dt` simulated seconds.
    fn step(&mut self, sim: &mut Simulation<DebugState>, dt: f64) {
        // At most one spawn owed: a backlog would arrive as a burst.
        self.spawn_credit = (self.spawn_credit + self.spawn_rate * dt).min(1.0);
        if self.spawn_credit >= 1.0 {
            let lane = sim.network().lane(SPAWN_LANE);
            let car = VehicleSpec { speed_factor: self.speed_factor(), ..demo_car() };
            let s = SPAWN_AT * lane.length();
            let at = Placement { lane: SPAWN_LANE, s, speed: car.desired_speed(SPEED_LIMIT) };
            if sim.try_insert(car, at).is_ok() {
                self.spawn_credit -= 1.0;
                self.spawned += 1;
            }
        }

        self.despawn_credit = (self.despawn_credit + self.despawn_rate * dt).min(3.0);
        let despawn_s = DESPAWN_AT * sim.network().lane(DESPAWN_LANE).length();
        let crossed: Vec<_> = sim
            .vehicles()
            .filter(|(_, v)| v.lane() == DESPAWN_LANE)
            .filter(|(id, v)| {
                let last = self.last_s.get(id).copied().unwrap_or(f64::INFINITY);
                last < despawn_s && v.lane_pos().s >= despawn_s
            })
            .map(|(id, _)| id)
            .collect();
        for id in crossed {
            if self.despawn_credit >= 1.0 {
                sim.remove(id);
                self.despawn_credit -= 1.0;
                self.despawned += 1;
            }
        }

        self.last_s.clear();
        self.last_s.extend(sim.vehicles().map(|(id, v)| (id, v.lane_pos().s)));
    }

    /// A speed factor from a normal distribution with mean 1 and deviation 0.1,
    /// as SUMO's default, clamped to [0.8, 1.2].
    fn speed_factor(&mut self) -> f64 {
        // Box-Muller on two uniform samples in (0, 1].
        let (u, v) = (self.uniform(), self.uniform());
        let normal = (-2.0 * u.ln()).sqrt() * (TAU * v).cos();
        (1.0 + 0.1 * normal).clamp(0.8, 1.2)
    }

    /// A uniform sample in (0, 1], from xorshift64.
    fn uniform(&mut self) -> f64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        ((self.rng >> 11) + 1) as f64 / (1u64 << 53) as f64
    }

    fn controls(&mut self, ui: &mut egui::Ui, on_road: usize) {
        ui.horizontal(|ui| {
            ui.add(egui::Slider::new(&mut self.spawn_rate, 0.0..=2.0).text("spawns/s"));
            ui.add(egui::Slider::new(&mut self.despawn_rate, 0.0..=2.0).text("despawns/s"));
            ui.separator();
            ui.label(format!(
                "spawned {}, despawned {}, on the road {on_road}",
                self.spawned, self.despawned
            ));
        });
    }

    /// Ticks across the lanes at the spawn (green) and despawn (red) points.
    fn draw(painter: &egui::Painter, sim: &Simulation<DebugState>) {
        for (lane_id, at, color) in [
            (SPAWN_LANE, SPAWN_AT, Color32::from_rgb(80, 220, 80)),
            (DESPAWN_LANE, DESPAWN_AT, Color32::from_rgb(230, 70, 70)),
        ] {
            let lane = sim.network().lane(lane_id);
            let s = at * lane.length();
            let hw = 0.5 * lane.width();
            let ends = [hw, -hw].map(|t| to_scene(lane.lane_to_world(lane.lane_pos(s, t))));
            painter.line_segment(ends, Stroke::new(0.4, color));
        }
    }
}

impl App {
    fn new() -> Self {
        let sim = new_sim();
        let outlines = sim.network().lanes().flat_map(|(_, lane)| lane.outline().map(to_scene));
        let scene_rect = Rect::from_points(&outlines.collect::<Vec<_>>());
        Self {
            sim,
            view: View::Pretty,
            paused: false,
            speed: 1.0,
            scene_rect: scene_rect.expand(10.0),
            selected: None,
            braking: false,
            layers: Layers::default(),
            lane_change_error: None,
            harness: Harness::default(),
        }
    }

    fn step(&mut self, dt: f64) {
        self.sim.step(dt);
        self.harness.step(&mut self.sim, dt);
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
            ui.add(egui::Slider::new(&mut self.speed, 0.1..=10.0).logarithmic(true).text("×"));
            ui.separator();
            if ui.button("Reset").clicked() {
                self.sim = new_sim();
                self.selected = None;
                self.braking = false;
                let rates = (self.harness.spawn_rate, self.harness.despawn_rate);
                self.harness = Harness::default();
                (self.harness.spawn_rate, self.harness.despawn_rate) = rates;
            }
        });
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
        ui.label("Acceleration terms (minimum binds)");
        let terms: Vec<_> = self.sim.sink().terms(id).collect();
        egui::Grid::new("terms").num_columns(2).show(ui, |ui| {
            for &(source, accel) in &terms {
                let name = match source {
                    AccelSource::FreeRoad { desired_speed } => {
                        format!("free road, v0 {desired_speed:.1} m/s")
                    }
                    AccelSource::Leader { id, gap } => format!("leader {id:?}, gap {gap:.2} m"),
                    AccelSource::Hold => "hold".into(),
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

        ui.separator();
        // A click-only button stops counting as pressed once the press is too long
        // to be a click (0.8 s); sensing drags keeps it pressed until release.
        let button = egui::Button::new("Hold to brake").sense(egui::Sense::click_and_drag());
        let response = ui.add(button);
        let down = response.is_pointer_button_down_on();
        if down != self.braking {
            self.braking = down;
            if down {
                self.sim.hold(id);
            } else {
                self.sim.release(id);
            }
        }

        ui.horizontal(|ui| {
            for (label, side) in [("Change left", Side::Left), ("Change right", Side::Right)] {
                if ui.button(label).clicked() {
                    self.lane_change_error = self.sim.change_lane(id, side).err();
                }
            }
        });
        if let Some(error) = self.lane_change_error {
            ui.colored_label(Color32::RED, error.to_string());
        }
    }

    /// Selects the vehicle under `point` (scene coordinates), or clears the selection.
    fn pick(&mut self, point: Pos2, tolerance: f64) {
        if let Some(id) = self.selected.take() {
            self.sim.release(id);
            self.braking = false;
        }
        self.lane_change_error = None;
        let world = DVec2::new(point.x as f64, -point.y as f64);
        self.selected = self.sim.vehicle_at(world, tolerance);
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("controls").show(ui, |ui| self.controls(ui));

        // Always shown at a fixed width, so selecting a vehicle doesn't move the scene.
        egui::Panel::right("inspector")
            .resizable(false)
            .exact_size(320.0)
            .show(ui, |ui| {
                // Content that is too wide scrolls instead of widening the panel,
                // which would shift the scene.
                if let Some(id) = self.selected {
                    egui::ScrollArea::both().auto_shrink(false).show(ui, |ui| {
                        ui.heading("Vehicle");
                        self.inspector(ui, id);
                    });
                } else {
                    // Vertical only, so the descriptions wrap.
                    egui::ScrollArea::vertical().auto_shrink(false).show(ui, |ui| {
                        ui.heading("Debug view layers");
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
                .show(ui, &mut self.scene_rect, |ui| {
                    match self.view {
                        View::Pretty => draw_pretty(ui.painter(), &self.sim, self.selected),
                        View::Debug => {
                            draw_debug(ui.painter(), &self.sim, self.selected, &self.layers)
                        }
                    }
                    Harness::draw(ui.painter(), &self.sim);
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

fn draw_pretty(painter: &egui::Painter, sim: &Simulation<DebugState>, selected: Option<VehicleId>) {
    for (_, lane) in sim.network().lanes() {
        painter.add(lane_surface(lane, Color32::from_gray(60)));
    }
    // Lane lines, drawn once per pair along the right edge of the left lane:
    // dashed where changes are allowed both ways, solid otherwise.
    let marking = Stroke::new(0.15, Color32::from_gray(230));
    for (_, lane) in sim.network().lanes() {
        let Some(right) = lane.adj_right() else {
            continue;
        };
        let back = sim.network().lane(right.lane).adj_left();
        let dashed = right.can_change && back.is_some_and(|n| n.can_change);
        let edge = |s: f64| to_scene(lane.lane_to_world(lane.lane_pos(s, -0.5 * lane.width())));
        if dashed {
            // 3 m dashes every 9 m.
            for i in 0..(lane.length() / 9.0) as u32 {
                let s = 9.0 * i as f64;
                painter.line_segment([edge(s), edge(s + 3.0)], marking);
            }
        } else {
            let points = (0..=(lane.length() / 2.0) as u32).map(|i| edge(2.0 * i as f64));
            painter.add(Shape::line(points.collect(), marking));
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
    }
}

fn draw_debug(
    painter: &egui::Painter,
    sim: &Simulation<DebugState>,
    selected: Option<VehicleId>,
    layers: &Layers,
) {
    let thin = Stroke::new(0.08, OUTLINE);
    for (lane_id, lane) in sim.network().lanes() {
        if layers.on(Layer::LaneOutlines) {
            painter.add(Shape::closed_line(
                lane.outline().map(to_scene).collect(),
                thin,
            ));
        }

        if layers.on(Layer::Centerlines) {
            let centerline: Vec<_> = lane.centerline().map(to_scene).collect();
            for &p in &centerline {
                painter.circle_filled(p, 0.1, VERTEX);
            }
            painter.add(Shape::line(centerline, Stroke::new(0.04, CENTERLINE)));
        }

        if layers.on(Layer::Ticks) {
            let hw = 0.5 * lane.width();
            for i in 0..=(lane.length() / 10.0) as u32 {
                let s = 10.0 * i as f64;
                let ends = [hw, -hw].map(|t| to_scene(lane.lane_to_world(lane.lane_pos(s, t))));
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
                let edge = if side == Side::Left { 0.45 } else { -0.45 } * lane.width();
                for i in 0..(lane.length() / 20.0) as u32 {
                    let s = 20.0 * i as f64 + 5.0;
                    let ends =
                        [0.0, edge].map(|t| to_scene(lane.lane_to_world(lane.lane_pos(s, t))));
                    painter.line_segment(ends, Stroke::new(0.06, color));
                }
            }
        }

        if layers.on(Layer::Occupants) {
            for occupant in sim.occupants(lane_id) {
                let p = lane.lane_to_world(lane.lane_pos(occupant.s, 0.0));
                painter.circle_filled(to_scene(p), 0.25, OCCUPANT);
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
            && let Some(change) = v.lane_change()
        {
            let lane = sim.network().lane(v.lane());
            let samples = (0..=40).map(|i| change.s0 + change.length * i as f64 / 40.0);
            let points = samples
                .take_while(|&s| s <= lane.length())
                .map(|s| to_scene(lane.lane_to_world(lane.lane_pos(s, change.offset_at(s)))))
                .collect();
            painter.add(Shape::line(points, Stroke::new(0.08, LANE_CHANGE)));
        }
    }

    // From the front bumper to the leader's rear bumper.
    if layers.on(Layer::Leaders) {
        for &(id, source, accel) in &sim.sink().accel_terms {
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

    if layers.on(Layer::Lookahead) {
        for &(id, point) in &sim.sink().lookaheads {
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
const VEHICLE: Color32 = Color32::from_rgb(240, 90, 90);
const LANE_CHANGE: Color32 = Color32::from_rgb(250, 160, 60);
const LEADER_BINDING: Color32 = Color32::from_rgb(120, 230, 120);
const LEADER_OTHER: Color32 = Color32::from_rgb(60, 110, 60);
const LOOKAHEAD: Color32 = Color32::from_rgb(90, 200, 240);

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
    Lookahead,
}

impl Layer {
    const ALL: [Layer; 9] = [
        Layer::LaneOutlines,
        Layer::Centerlines,
        Layer::Ticks,
        Layer::Neighbors,
        Layer::Occupants,
        Layer::Vehicles,
        Layer::LaneChanges,
        Layer::Leaders,
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
                &[OCCUPANT],
                "Each occupant record, on the centerline at its s",
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
        Self([true; Layer::ALL.len()])
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
}

impl DebugSink for DebugState {
    fn record(&mut self, event: impl FnOnce() -> DebugEvent) {
        match event() {
            DebugEvent::RecomputeAccels => self.accel_terms.clear(),
            DebugEvent::AccelTerm { vehicle, source, accel } => {
                self.accel_terms.push((vehicle, source, accel))
            }
            DebugEvent::RecomputeMotion => self.lookaheads.clear(),
            DebugEvent::Lookahead { vehicle, point } => self.lookaheads.push((vehicle, point)),
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

/// The lane's surface as a triangle strip between its left and right edges.
///
/// [`Lane::outline`] runs forward along the left edge and back along the right,
/// so the rib at vertex `i` joins `outline[i]` and `outline[n - 1 - i]`. Each
/// quad between ribs is convex, which a filled polygon of the whole outline
/// would not be on a bend.
fn lane_surface(lane: &Lane, color: Color32) -> Shape {
    let outline: Vec<_> = lane.outline().map(to_scene).collect();
    let n = outline.len() as u32;
    let mut mesh = egui::Mesh::default();
    for &p in &outline {
        mesh.colored_vertex(p, color);
    }
    for i in 0..n / 2 - 1 {
        let (l0, l1, r0, r1) = (i, i + 1, n - 1 - i, n - 2 - i);
        mesh.add_triangle(l0, l1, r1);
        mesh.add_triangle(l0, r1, r0);
    }
    Shape::mesh(mesh)
}

/// World meters (y up) to scene coordinates (y down).
fn to_scene(p: DVec2) -> Pos2 {
    Pos2::new(p.x as f32, -p.y as f32)
}

/// An empty three-lane ring; the harness fills it.
fn new_sim() -> Simulation<DebugState> {
    let network = ring_road(RING_CIRCUMFERENCE, RING_LANES, SPEED_LIMIT);
    Simulation::new(network, DebugState::default())
}

/// A ring road of concentric lanes, each its own successor, with lane changes
/// allowed both ways between neighbors. Lane 0 is the innermost, with the given
/// circumference. Travel is counterclockwise, so lane `i` is on the left of lane `i + 1`.
///
/// Chords are sized for at most 1 cm of sagitta. Every lane starts and ends at
/// angle 0, at the midpoint of a chord, so there is no kink at the seam and the
/// lanes start and end together.
fn ring_road(circumference: f64, lanes: usize, speed_limit: f64) -> SimNetwork {
    let lane = |radius: f64| {
        let max_chord = (8.0 * radius * 0.01).sqrt();
        let n = (TAU * radius / max_chord).ceil() as usize;
        let vertex = |i: usize| radius * DVec2::from_angle(TAU * (i as f64 + 0.5) / n as f64);
        let seam = 0.5 * (vertex(n - 1) + vertex(0));
        let mut points = vec![seam];
        points.extend((0..n).map(vertex));
        points.push(seam);
        Lane::new(&points, LANE_WIDTH, speed_limit).expect("ring lane is valid")
    };
    let inner = circumference / TAU;
    let radii = (0..lanes).map(|i| inner + LANE_WIDTH * i as f64);
    let mut network = SimNetwork::new(radii.map(lane).collect());
    for i in 0..lanes as u32 {
        network.connect(LaneId(i), LaneId(i));
        if i > 0 {
            network.set_adjacent(LaneId(i - 1), LaneId(i), true, true);
        }
    }
    network
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
    }
}
