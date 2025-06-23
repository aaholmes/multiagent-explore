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
    playing: bool,
    frame_counter: usize,
    frames_per_tick: usize,
}

impl VisualizeApp {
    pub fn new(history: Vec<Vec<RobotNode>>, map_width: usize, map_height: usize) -> Self {
        Self {
            history,
            map_width,
            map_height,
            tick: 0,
            playing: false, // User must hit Play
            frame_counter: 0,
            frames_per_tick: 10, // Slower animation (higher = slower)
        }
    }
}

impl App for VisualizeApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading(format!("Tick: {}", self.tick));
            ui.horizontal(|ui| {
                if ui.button(if self.playing { "Pause" } else { "Play" }).clicked() {
                    self.playing = !self.playing;
                }
                ui.add(egui::Slider::new(&mut self.tick, 0..=self.history.len().saturating_sub(1)).text("Tick"));
            });
            ui.add_space(20.0);

            // Auto-play logic (slowed down)
            if self.playing && self.tick < self.history.len().saturating_sub(1) {
                self.frame_counter += 1;
                if self.frame_counter >= self.frames_per_tick {
                    self.tick += 1;
                    self.frame_counter = 0;
                }
                ctx.request_repaint();
            } else {
                self.frame_counter = 0;
            }

            // Calculate map display size
            let map_pixel_height = (self.map_height as f32) * 20.0;
            let map_pixel_width = (self.map_width as f32) * 20.0;
            let n_maps = self.history[self.tick].len();
            let total_width = map_pixel_width * n_maps as f32 + 40.0 * (n_maps as f32 - 1.0);
            let total_height = map_pixel_height;

            // Reserve space for the maps and get the rect
            let (_rect, _response) = ui.allocate_exact_size(
                egui::vec2(total_width, total_height),
                egui::Sense::hover(),
            );

            // Center the maps in the available space
            let available_rect = ui.max_rect();
            let center_x = available_rect.center().x - total_width / 2.0;
            let center_y = available_rect.center().y - total_height / 2.0;

            // Draw each map
            for (i, robot) in self.history[self.tick].iter().enumerate() {
                let x = center_x + i as f32 * (map_pixel_width + 40.0);
                let y = center_y;
                let map_rect = egui::Rect::from_min_max(
                    egui::pos2(x, y),
                    egui::pos2(x + map_pixel_width, y + map_pixel_height),
                );
                let painter = ui.painter_at(map_rect);
                // Draw map (only empty and obstacles)
                for y in 0..self.map_height {
                    for x in 0..self.map_width {
                        let idx = y * self.map_width + x;
                        let cell = robot.state.map.cells[idx];
                        match cell {
                            CellState::Obstacle => {
                                let x0 = map_rect.left_top().x + x as f32 * 20.0;
                                let y0 = map_rect.left_top().y + y as f32 * 20.0;
                                let x1 = x0 + 20.0;
                                let y1 = y0 + 20.0;
                                // Black fill
                                painter.rect_filled(
                                    egui::Rect::from_min_max(
                                        egui::pos2(x0, y0),
                                        egui::pos2(x1, y1),
                                    ),
                                    0.0,
                                    egui::Color32::BLACK,
                                );
                                // White border
                                painter.rect_stroke(
                                    egui::Rect::from_min_max(
                                        egui::pos2(x0, y0),
                                        egui::pos2(x1, y1),
                                    ),
                                    0.0,
                                    egui::Stroke::new(1.5, egui::Color32::WHITE),
                                );
                            }
                            CellState::Empty => {
                                let x0 = map_rect.left_top().x + x as f32 * 20.0;
                                let y0 = map_rect.left_top().y + y as f32 * 20.0;
                                let x1 = x0 + 20.0;
                                let y1 = y0 + 20.0;
                                painter.rect_filled(
                                    egui::Rect::from_min_max(
                                        egui::pos2(x0, y0),
                                        egui::pos2(x1, y1),
                                    ),
                                    0.0,
                                    egui::Color32::WHITE,
                                );
                            }
                            CellState::Unexplored => {}
                        }
                    }
                }
                // Draw robot as a colored circle with number
                let rx = robot.state.pose.position.x as f32;
                let ry = robot.state.pose.position.y as f32;
                let center = egui::pos2(
                    map_rect.left_top().x + rx * 20.0 + 10.0,
                    map_rect.left_top().y + ry * 20.0 + 10.0,
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
                // Draw label above each map
                let label_pos = egui::pos2(map_rect.center().x, map_rect.top() - 24.0);
                painter.text(
                    label_pos,
                    egui::Align2::CENTER_CENTER,
                    format!("Robot {}", i + 1),
                    egui::FontId::proportional(16.0),
                    egui::Color32::DARK_GRAY,
                );
            }
        });
    }
} 