use crate::types::*;
use crate::robot_node::RobotNode;

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

    /// Advances the simulation by one tick.
    pub fn tick(&mut self) {
        for robot in &mut self.robots {
            robot.tick();
        }
        // TODO: Add global termination checks, communication, etc.
    }
} 