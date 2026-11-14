//! egui app with a pretty view and a debug view of the simulation.

use eframe::egui::{self, Color32, Pos2, Rect, Shape, Stroke};
use glam::DVec2;
use traffic_sim::{DebugEvent, Lane, LaneId, Placement, SimNetwork, Simulation, VehicleSpec};

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
    sim: Simulation<Vec<DebugEvent>>,
    view: View,
    paused: bool,
    /// Simulated seconds per real second.
    speed: f64,
    /// Visible region of the world, in scene coordinates (see [`to_scene`]).
    scene_rect: Rect,
}

impl App {
    fn new() -> Self {
        let lane = demo_lane();
        let scene_rect = Rect::from_points(&lane.outline().map(to_scene).collect::<Vec<_>>());
        Self {
            sim: Simulation::new(SimNetwork::new(vec![lane]), Vec::new()),
            view: View::Pretty,
            paused: false,
            speed: 1.0,
            scene_rect: scene_rect.expand(20.0),
        }
    }

    fn step(&mut self, dt: f64) {
        // Keep the demo looping: a new car once the last one has left.
        if self.sim.vehicles().next().is_none() {
            // Rear bumper at the start of the lane.
            let car = demo_car();
            let at = Placement { lane: LaneId(0), s: car.rear_axle, speed: 0.0 };
            self.sim.insert(car, at);
        }
        self.sim.sink_mut().clear();
        self.sim.step(dt);
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("controls").show(ui, |ui| {
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
                if let Some((_, v)) = self.sim.vehicles().next() {
                    ui.separator();
                    let pos = v.lane_pos();
                    ui.monospace(format!(
                        "s {:6.1} m  t {:+.2} m  v {:4.1} m/s  a {:+.2} m/s²",
                        pos.s,
                        pos.t,
                        v.speed(),
                        v.accel()
                    ));
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
            scene.show(ui, &mut self.scene_rect, |ui| match self.view {
                View::Pretty => draw_pretty(ui.painter(), &self.sim),
                View::Debug => draw_debug(ui.painter(), &self.sim),
            });
        });
    }
}

fn draw_pretty(painter: &egui::Painter, sim: &Simulation<Vec<DebugEvent>>) {
    for (_, lane) in sim.network().lanes() {
        painter.add(lane_surface(lane, Color32::from_gray(60)));
    }
    for (_, v) in sim.vehicles() {
        let corners = v.corners().map(to_scene).to_vec();
        painter.add(Shape::convex_polygon(
            corners,
            Color32::from_rgb(200, 40, 40),
            Stroke::NONE,
        ));
    }
}

fn draw_debug(painter: &egui::Painter, sim: &Simulation<Vec<DebugEvent>>) {
    let thin = Stroke::new(0.08, Color32::from_gray(160));
    for (_, lane) in sim.network().lanes() {
        painter.add(Shape::closed_line(
            lane.outline().map(to_scene).collect(),
            thin,
        ));
        let centreline: Vec<_> = lane.centreline().map(to_scene).collect();
        for &p in &centreline {
            painter.circle_filled(p, 0.15, Color32::from_gray(200));
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
    }

    for (_, v) in sim.vehicles() {
        let corners = v.corners().map(to_scene).to_vec();
        painter.add(Shape::closed_line(
            corners,
            Stroke::new(0.1, Color32::from_rgb(240, 90, 90)),
        ));
        painter.circle_filled(to_scene(v.pos()), 0.2, Color32::from_rgb(240, 90, 90));
    }

    for event in sim.sink() {
        match *event {
            DebugEvent::Lookahead { vehicle, point } => {
                let Some(v) = sim.get(vehicle) else { continue };
                let colour = Color32::from_rgb(90, 200, 240);
                painter.line_segment(
                    [to_scene(v.pos()), to_scene(point)],
                    Stroke::new(0.06, colour),
                );
                painter.circle_filled(to_scene(point), 0.3, colour);
            }
        }
    }
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

/// 100 m straight, a 60° bend of radius about 100 m, then 100 m straight.
fn demo_lane() -> Lane {
    let (steps, turn) = (20, 60f64.to_radians());
    let mut p = DVec2::new(100.0, 0.0);
    let mut pts = vec![DVec2::ZERO, p];
    for i in 0..steps {
        p += 5.0 * DVec2::from_angle(turn * (i as f64 + 0.5) / steps as f64);
        pts.push(p);
    }
    pts.push(p + 100.0 * DVec2::from_angle(turn));
    Lane::new(&pts, 3.5, 50.0 / 3.6).expect("demo lane is valid")
}

fn demo_car() -> VehicleSpec {
    VehicleSpec {
        length: 4.6,
        width: 1.9,
        max_speed: 30.0,
        max_accel: 1.5,
        comfy_decel: 2.0,
        rear_axle: 1.0,
        front_axle: 3.7,
    }
}
