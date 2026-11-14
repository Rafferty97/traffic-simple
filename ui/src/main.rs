//! egui app with a pretty view and a debug view of the simulation.

use std::f64::consts::TAU;

use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use glam::DVec2;
use traffic_sim::{
    AccelSource, DebugEvent, DebugSink, Lane, LaneId, Placement, SimNetwork, Simulation, Vehicle,
    VehicleId, VehicleSpec,
};

/// Sugiyama et al. (2008): 22 cars on a 230 m ring.
const RING_CIRCUMFERENCE: f64 = 230.0;
const RING_CARS: usize = 22;
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
}

impl App {
    fn new() -> Self {
        let sim = new_sim();
        let lane = sim.network().lane(LaneId(0));
        let scene_rect = Rect::from_points(&lane.outline().map(to_scene).collect::<Vec<_>>());
        Self {
            sim,
            view: View::Pretty,
            paused: false,
            speed: 1.0,
            scene_rect: scene_rect.expand(10.0),
            selected: None,
            braking: false,
        }
    }

    fn step(&mut self, dt: f64) {
        self.sim.step(dt);
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
            }
        });
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
            let path: Vec<_> = v.path().iter().map(|l| l.0.to_string()).collect();
            row("path", path.join(" → "));
            row("s", format!("{:.2} m", pos.s));
            row("t", format!("{:+.3} m", pos.t));
            row(
                "speed",
                format!("{:.2} m/s ({:.0} km/h)", v.speed(), v.speed() * 3.6),
            );
            row("accel", format!("{:+.2} m/s²", v.accel()));
            row("steer", format!("{:+.2}°", v.steer().to_degrees()));
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
    }

    /// Selects the vehicle under `point` (scene coordinates), or clears the selection.
    fn pick(&mut self, point: Pos2, tolerance: f64) {
        if let Some(id) = self.selected.take() {
            self.sim.release(id);
            self.braking = false;
        }
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
                egui::ScrollArea::both().auto_shrink(false).show(ui, |ui| {
                    if let Some(id) = self.selected {
                        ui.heading("Vehicle");
                        self.inspector(ui, id);
                    }
                });
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
                    View::Pretty => draw_pretty(ui.painter(), &self.sim, self.selected),
                    View::Debug => draw_debug(ui.painter(), &self.sim, self.selected),
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

fn draw_debug(painter: &egui::Painter, sim: &Simulation<DebugState>, selected: Option<VehicleId>) {
    let thin = Stroke::new(0.08, Color32::from_gray(160));
    for (lane_id, lane) in sim.network().lanes() {
        painter.add(Shape::closed_line(
            lane.outline().map(to_scene).collect(),
            thin,
        ));
        let centreline: Vec<_> = lane.centreline().map(to_scene).collect();
        for &p in &centreline {
            painter.circle_filled(p, 0.1, Color32::from_gray(200));
        }
        painter.add(Shape::line(
            centreline,
            Stroke::new(0.04, Color32::from_gray(110)),
        ));

        // A tick across the lane every 10 m.
        let hw = 0.5 * lane.width();
        for i in 0..=(lane.length() / 10.0) as u32 {
            let s = 10.0 * i as f64;
            let ends = [hw, -hw].map(|t| to_scene(lane.lane_to_world(lane.lane_pos(s, t))));
            painter.line_segment(ends, thin);
        }

        // Each occupant record, on the centreline at its `s`.
        for occupant in sim.occupants(lane_id) {
            let p = lane.lane_to_world(lane.lane_pos(occupant.s, 0.0));
            painter.circle_filled(to_scene(p), 0.25, Color32::from_rgb(200, 160, 255));
        }
    }

    for (id, v) in sim.vehicles() {
        let colour = if Some(id) == selected {
            SELECTED
        } else {
            Color32::from_rgb(240, 90, 90)
        };
        let corners = v.corners().map(to_scene).to_vec();
        painter.add(Shape::closed_line(corners, Stroke::new(0.1, colour)));
        painter.circle_filled(to_scene(v.pos()), 0.2, colour);
    }

    // Leader links, from the front bumper to the leader's rear bumper. Bright
    // when that leader's term is the one the acceleration came from.
    for &(id, source, accel) in &sim.sink().accel_terms {
        let AccelSource::Leader { id: leader, .. } = source else {
            continue;
        };
        let (Some(v), Some(leader)) = (sim.get(id), sim.get(leader)) else {
            continue;
        };
        let colour = if accel == v.accel() {
            Color32::from_rgb(120, 230, 120)
        } else {
            Color32::from_rgb(60, 110, 60)
        };
        painter.line_segment(
            [to_scene(front(v)), to_scene(rear(leader))],
            Stroke::new(0.08, colour),
        );
    }

    for &(id, point) in &sim.sink().lookaheads {
        let Some(v) = sim.get(id) else { continue };
        let colour = Color32::from_rgb(90, 200, 240);
        painter.line_segment(
            [to_scene(v.pos()), to_scene(point)],
            Stroke::new(0.04, colour),
        );
        painter.circle_filled(to_scene(point), 0.2, colour);
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
fn lane_surface(lane: &Lane, colour: Color32) -> Shape {
    let outline: Vec<_> = lane.outline().map(to_scene).collect();
    let n = outline.len() as u32;
    let mut mesh = egui::Mesh::default();
    for &p in &outline {
        mesh.colored_vertex(p, colour);
    }
    for i in 0..n / 2 - 1 {
        let (l0, l1, r0, r1) = (i, i + 1, n - 1 - i, n - 2 - i);
        mesh.add_triangle(l0, l1, r1);
        mesh.add_triangle(l0, r1, r0);
    }
    Shape::mesh(mesh)
}

/// World metres (y up) to scene coordinates (y down).
fn to_scene(p: DVec2) -> Pos2 {
    Pos2::new(p.x as f32, -p.y as f32)
}

/// The Sugiyama ring with its cars evenly spaced and at rest.
fn new_sim() -> Simulation<DebugState> {
    let mut sim = Simulation::new(ring(RING_CIRCUMFERENCE, 50.0 / 3.6), DebugState::default());
    let length = sim.network().lane(LaneId(0)).length();
    for i in 0..RING_CARS {
        let s = length * i as f64 / RING_CARS as f64;
        sim.insert(demo_car(), Placement { lane: LaneId(0), s, speed: 0.0 });
    }
    sim
}

/// A circular lane that is its own successor.
///
/// Chords are sized for at most 1 cm of sagitta. The lane starts and ends at the
/// midpoint of one chord, so there is no kink at the seam.
fn ring(circumference: f64, speed_limit: f64) -> SimNetwork {
    let radius = circumference / TAU;
    let max_chord = (8.0 * radius * 0.01).sqrt();
    let n = (circumference / max_chord).ceil() as usize;
    let vertex = |i: usize| radius * DVec2::from_angle(TAU * i as f64 / n as f64);
    let seam = 0.5 * (vertex(n - 1) + vertex(0));
    let mut points = vec![seam];
    points.extend((0..n).map(vertex));
    points.push(seam);
    let lane = Lane::new(&points, 3.5, speed_limit).expect("ring lane is valid");
    let mut network = SimNetwork::new(vec![lane]);
    network.connect(LaneId(0), LaneId(0));
    network
}

fn demo_car() -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        max_accel: 1.0,
        comfy_decel: 1.5,
        time_headway: 1.0,
        min_gap: 1.5,
        rear_axle: 1.0,
        front_axle: 3.7,
    }
}
