use crate::types::*;
use crate::robot_node::RobotNode;
use crate::map_loader;

/// Manages the simulation environment, robots, and clock.
pub struct SimulationManager {
    // The environment grid, robots, and other state
    pub map: GridMap,
    pub robots: Vec<RobotNode>,
    // Add simulation clock or other fields as needed
}

impl SimulationManager {
    /// Initializes the simulation with a map and two robots.
    pub fn new(map: GridMap, robot_states: Vec<RobotState>) -> Self {
        let robots = robot_states.into_iter().map(|state| RobotNode { state }).collect();
        Self { map, robots }
    }

    /// Loads a map from a file and initializes two robots at (0,0) and (1,0) with default state.
    pub fn from_map_file(path: &str) -> std::io::Result<Self> {
        let map = map_loader::load_map_from_file(path)?;
        let robot_states = vec![
            RobotState {
                id: 0,
                pose: Pose { position: Point { x: 0, y: 0 }, orientation_rad: 0.0 },
                phase: RobotPhase::InitialWallFind,
                map: map.clone(),
                scout_depth_n: 3,
                partner_id: 1,
                last_known_partner_pose: None,
                loop_analysis_data: None,
                travel_direction_before_island: None,
            },
            RobotState {
                id: 1,
                pose: Pose { position: Point { x: 1, y: 0 }, orientation_rad: 0.0 },
                phase: RobotPhase::InitialWallFind,
                map: map.clone(),
                scout_depth_n: 3,
                partner_id: 0,
                last_known_partner_pose: None,
                loop_analysis_data: None,
                travel_direction_before_island: None,
            },
        ];
        Ok(Self::new(map, robot_states))
    }

    /// Advances the simulation by one tick.
    pub fn tick(&mut self) {
        for robot in &mut self.robots {
            robot.tick();
        }
        // TODO: Add global termination checks, communication, etc.
    }
} 