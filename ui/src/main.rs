//! egui app with a pretty view and a debug view of the simulation.

use eframe::egui;
// Not used yet; keeps the dependency on the core in place.
use traffic_sim as _;

fn main() -> eframe::Result {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "Traffic Simulator",
        options,
        Box::new(|_cc| Ok(Box::new(App::default()))),
    )
}

#[derive(Default)]
struct App {}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ui, |ui| {
            ui.heading("Traffic Simulator");
        });
    }
}
