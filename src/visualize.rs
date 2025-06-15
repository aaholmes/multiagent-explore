use eframe::egui;
use eframe::App;
use multiagent_explore::robot_node::RobotNode;
use multiagent_explore::types::CellState;

pub fn visualize(history: &Vec<Vec<RobotNode>>, map_width: usize, map_height: usize) {
    let app = VisualizeApp::new(history.clone(), map_width, map_height);
    let native_options = eframe::NativeOptions::default();
    eframe::run_native(
        "Multi-Robot Exploration Visualization",
        native_options,
        Box::new(|_cc| Box::new(app)),
    ).unwrap();
}

struct VisualizeApp {
    history: Vec<Vec<RobotNode>>,
    map_width: usize,
    map_height: usize,
    tick: usize,
}

impl VisualizeApp {
    pub fn new(history: Vec<Vec<RobotNode>>, map_width: usize, map_height: usize) -> Self {
        Self { history, map_width, map_height, tick: 0 }
    }
}

impl App for VisualizeApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading(format!("Tick: {}", self.tick));
            ui.add(egui::Slider::new(&mut self.tick, 0..=self.history.len().saturating_sub(1)).text("Tick"));
            ui.add_space(20.0);
            ui.horizontal(|ui| {
                for (i, robot) in self.history[self.tick].iter().enumerate() {
                    ui.vertical(|ui| {
                        ui.label(format!("Robot {}", i + 1));
                        let map_pixel_height = (self.map_height as f32) * 20.0;
                        let available_height = ui.available_height();
                        let y_offset = (available_height - map_pixel_height).max(0.0) / 2.0 + 40.0; // 40 for header/slider
                        let (rect, _response) = ui.allocate_exact_size(
                            egui::vec2((self.map_width as f32) * 20.0, map_pixel_height),
                            egui::Sense::hover(),
                        );
                        let painter = ui.painter_at(rect);
                        // Draw map (only empty and obstacles)
                        for y in 0..self.map_height {
                            for x in 0..self.map_width {
                                let idx = y * self.map_width + x;
                                let cell = robot.state.map.cells[idx];
                                let color = match cell {
                                    CellState::Obstacle => Some(egui::Color32::BLACK),
                                    CellState::Empty => Some(egui::Color32::WHITE),
                                    CellState::Unexplored => None,
                                };
                                if let Some(color) = color {
                                    let x0 = rect.left_top().x + x as f32 * 20.0;
                                    let y0 = rect.left_top().y + y as f32 * 20.0 + y_offset;
                                    let x1 = x0 + 20.0;
                                    let y1 = y0 + 20.0;
                                    painter.rect_filled(
                                        egui::Rect::from_min_max(
                                            egui::pos2(x0, y0),
                                            egui::pos2(x1, y1),
                                        ),
                                        0.0,
                                        color,
                                    );
                                }
                            }
                        }
                        // Draw robot as a colored circle with number
                        let rx = robot.state.pose.position.x as f32;
                        let ry = robot.state.pose.position.y as f32;
                        let center = egui::pos2(
                            rect.left_top().x + rx * 20.0 + 10.0,
                            rect.left_top().y + ry * 20.0 + y_offset + 10.0,
                        );
                        let robot_color = if i == 0 { egui::Color32::from_rgb(0, 120, 255) } else { egui::Color32::from_rgb(255, 80, 0) };
                        painter.circle_filled(center, 9.0, robot_color);
                        painter.text(
                            center,
                            egui::Align2::CENTER_CENTER,
                            format!("{}", i + 1),
                            egui::FontId::proportional(14.0),
                            egui::Color32::WHITE,
                        );
                    });
                }
            });
        });
    }
} 