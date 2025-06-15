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
            ui.horizontal(|ui| {
                for (i, robot) in self.history[self.tick].iter().enumerate() {
                    ui.vertical(|ui| {
                        ui.label(format!("Robot {}", i));
                        egui::Grid::new(format!("robot_map_{}", i)).show(ui, |ui| {
                            for y in 0..self.map_height {
                                for x in 0..self.map_width {
                                    let idx = y * self.map_width + x;
                                    let ch = if robot.state.pose.position.x == x as i32 && robot.state.pose.position.y == y as i32 {
                                        "R"
                                    } else {
                                        match robot.state.map.cells[idx] {
                                            CellState::Obstacle => "#",
                                            CellState::Empty => ".",
                                            CellState::Unexplored => " ",
                                        }
                                    };
                                    ui.label(ch);
                                }
                                ui.end_row();
                            }
                        });
                    });
                }
            });
        });
    }
} 