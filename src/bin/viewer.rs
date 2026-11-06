use eframe::egui::{self, Color32, Pos2, Sense, Stroke};
use egui::{lerp, Painter, Rect, Ui, Vec2};
use glam::{DAffine2, DVec2};
use itertools::Itertools;
use traffic_sim::{
    debug::{DebugSink, ObjectSink},
    load_network_file,
    simulation::Simulation,
};

fn main() -> eframe::Result {
    eframe::run_native(
        "Traffic sim viewer",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(Viewer::default()))),
    )
}

struct Viewer {
    sim: Simulation,
    running: bool,
    speed: f64,
    accum: f64,
    center: Vec2, // world-space point at screen centre
    scale: f32,   // pixels per world unit
    error: Option<String>,
}

impl Default for Viewer {
    fn default() -> Self {
        Self {
            sim: Simulation::new(),
            running: true,
            speed: 1.0,
            accum: 0.0,
            center: Vec2::new(0.0, 0.0),
            scale: 5.0,
            error: None,
        }
    }
}

impl eframe::App for Viewer {
    fn logic(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if self.running {
            self.accum += self.speed * (ctx.input(|i| i.stable_dt) as f64);
            const DT: f64 = 1.0 / 60.0;
            while self.accum >= DT {
                self.sim.step(DT);
                self.accum -= DT;
            }
            ctx.request_repaint();
        }
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::Panel::top("controls").show(ui, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Load network").clicked() {
                    match load_network_file("networks/basic.json") {
                        Ok(sim) => {
                            self.sim = sim;
                        }
                        Err(err) => {
                            self.error = Some(err.to_string());
                        }
                    }
                }

                if ui.button("Insert vehicle").clicked() {
                    match self.sim.insert_vehicle() {
                        Ok(_) => {}
                        Err(err) => {
                            self.error = Some(err.to_string());
                        }
                    }
                }

                let label = match self.running {
                    true => "Stop",
                    false => "Start",
                };
                if ui.button(label).clicked() {
                    self.running = !self.running;
                }

                ui.add(egui::Slider::new(&mut self.speed, 0.5..=5.0).text("Speed"));
            });
        });

        egui::CentralPanel::default().show(ui, |ui| {
            let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
            let rect = resp.rect;

            // Pan and zoom
            self.center -= resp.drag_delta() / self.scale;
            if resp.hovered() {
                let z = ui.input(|i| i.smooth_scroll_delta.y);
                self.scale *= (z * 0.01).exp();
            }

            // Grid lines
            for (s, e, c) in grid_lines(rect, self.center, self.scale, 10.0) {
                painter.line_segment([s, e], Stroke::new(1.0, c));
            }

            // Debug geometry
            let translation = rect.center() - self.scale * self.center;
            let transform = DAffine2::from_scale_angle_translation(
                DVec2::splat(self.scale as f64),
                0.0,
                DVec2::new(translation.x as f64, translation.y as f64),
            );
            self.sim
                .debug(&mut EguiDebug { ui, painter: &painter, transform, label: None });

            // Roads
            // for road in self.sim.roads() {
            //     painter.line_segment(
            //         [to_screen(road.start.into()), to_screen(road.end.into())],
            //         Stroke::new(3.0, Color32::DARK_GRAY),
            //     );
            // }
            // Vehicles
            // for (i, v) in [(0, Pos2::new(10.0, 10.0))] {
            //     let col = Color32::LIGHT_BLUE;
            //     painter.circle_filled(to_screen(v.into()), 4.0, col);
            // }

            // Click to select the nearest vehicle
            // if resp.clicked() {
            //     if let Some(p) = resp.interact_pointer_pos() {
            //         self.selected = self
            //             .sim
            //             .vehicles()
            //             .enumerate()
            //             .min_by(|(_, a), (_, b)| {
            //                 let da = (to_screen(a.pos.into()) - p).length_sq();
            //                 let db = (to_screen(b.pos.into()) - p).length_sq();
            //                 da.total_cmp(&db)
            //             })
            //             .map(|(i, _)| i);
            //     }
            // }
        });

        let mut dismiss = false;
        if let Some(msg) = &self.error {
            egui::Window::new("Error")
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .show(ui, |ui| {
                    ui.colored_label(ui.visuals().error_fg_color, msg);
                    if ui.button("OK").clicked() {
                        dismiss = true;
                    }
                });
        }
        if dismiss {
            self.error = None;
        }
    }
}

fn grid_lines(
    rect: Rect,
    center: Vec2,
    scale: f32,
    base_spacing: f32,
) -> impl Iterator<Item = (Pos2, Pos2, Color32)> {
    static MIN_PX: f32 = 10.0;

    // Coarsen by 10x until lines are far enough apart on screen
    let mut spacing = base_spacing;
    while spacing * scale < MIN_PX {
        spacing *= 5.0;
    }

    // Line colors
    let t = (0.5 * ((spacing * scale / MIN_PX) - 1.0)).min(1.0);
    let minor_lines = Color32::from_white_alpha(lerp(0.0..=20.0, t) as u8);
    let major_lines = Color32::from_white_alpha(20);

    // Visible world-space bounds
    let half = rect.size() * 0.5 / scale;
    let (lo, hi) = (center - half, center + half);

    // Integer multiples of `spacing` inside the bounds
    let range =
        move |lo: f32, hi: f32| (lo / spacing).ceil() as isize..=(hi / spacing).floor() as isize;

    let h_lines = range(lo.y, hi.y).map(move |i| {
        let color = match i % 5 {
            0 => major_lines,
            _ => minor_lines,
        };
        let y = rect.center().y + (spacing * i as f32 - center.y) * scale;
        (Pos2::new(rect.left(), y), Pos2::new(rect.right(), y), color)
    });

    let v_lines = range(lo.x, hi.x).map(move |i| {
        let color = match i % 5 {
            0 => major_lines,
            _ => minor_lines,
        };
        let x = rect.center().x + (spacing * i as f32 - center.x) * scale;
        (Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom()), color)
    });

    h_lines.chain(v_lines)
}

struct EguiDebug<'a> {
    ui: &'a mut Ui,
    painter: &'a Painter,
    transform: DAffine2,
    label: Option<String>,
}

impl<'a> DebugSink for EguiDebug<'a> {
    type Object<'b>
        = EguiDebug<'b>
    where
        Self: 'b;

    fn advance_frame(&mut self) {}

    fn object<'b>(&'b mut self, label: &str) -> EguiDebug<'b>
    where
        Self: 'b,
    {
        EguiDebug {
            ui: &mut *self.ui,
            painter: self.painter,
            transform: self.transform,
            label: Some(label.into()),
        }
    }
}

impl<'a> ObjectSink for EguiDebug<'a> {
    fn polyline(&mut self, points: &[glam::prelude::DVec2], closed: bool) {
        let mut points = points
            .into_iter()
            .map(|&p| self.transform.transform_point2(p))
            .map(|DVec2 { x, y }| Pos2::new(x as f32, y as f32))
            .collect_vec();
        if closed {
            points.push(*points.first().unwrap());
        }
        let color = match self
            .label
            .as_deref()
            .and_then(|l| l.split(':').next())
            .unwrap_or_default()
        {
            "link" => Color32::WHITE,
            "veh" => Color32::CYAN,
            _ => Color32::RED,
        };
        self.painter.line(points, Stroke::new(1.0, color));
    }

    fn point(&mut self, point: glam::prelude::DVec2, label: Option<&str>) {
        // todo
    }

    fn datum(&mut self, key: &str, value: &str) {
        // todo
    }
}
