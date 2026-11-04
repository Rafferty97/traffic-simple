use eframe::egui::{self, Color32, Pos2, Sense, Stroke};
use egui::{lerp, Rect, Vec2};

fn main() -> eframe::Result {
    eframe::run_native(
        "Traffic sim viewer",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(Viewer::default()))),
    )
}

struct Viewer {
    center: Vec2, // world-space point at screen centre
    scale: f32,   // pixels per world unit
}

impl Default for Viewer {
    fn default() -> Self {
        Self { center: Vec2::new(0.0, 0.0), scale: 1.0 }
    }
}

impl eframe::App for Viewer {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ui, |ui| {
            let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
            let rect = resp.rect;

            // Pan and zoom
            self.center -= resp.drag_delta() / self.scale;
            if resp.hovered() {
                let z = ui.input(|i| i.smooth_scroll_delta.y);
                self.scale *= (z * 0.01).exp();
            }

            let to_screen = |p: Pos2| rect.center() + (p.to_vec2() - self.center) * self.scale;

            // Grid lines
            for (s, e, c) in grid_lines(rect, self.center, self.scale, 10.0) {
                painter.line_segment([s, e], Stroke::new(1.0, c));
            }

            // Roads
            // for road in self.sim.roads() {
            //     painter.line_segment(
            //         [to_screen(road.start.into()), to_screen(road.end.into())],
            //         Stroke::new(3.0, Color32::DARK_GRAY),
            //     );
            // }
            // Vehicles
            for (i, v) in [(0, Pos2::new(10.0, 10.0))] {
                let col = Color32::LIGHT_BLUE;
                painter.circle_filled(to_screen(v.into()), 4.0, col);
            }

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
