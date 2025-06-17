use crate::types::*;
use crate::robot_node::RobotNode;
use crate::map_loader;
use rand::seq::SliceRandom;
use rand::{SeedableRng, rngs::StdRng};
use std::f64::consts::PI;

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
        let robots = robot_states.into_iter().map(|state| RobotNode::new(state)).collect();
        Self { map, robots }
    }

    /// Loads a map from a file and initializes two robots at random adjacent empty cells (left-to-right), both facing up (-Y direction).
    /// Accepts a random seed for reproducibility.
    pub fn from_map_file(path: &str, seed: u64) -> std::io::Result<Self> {
        let map = map_loader::load_map_from_file(path)?;
        let mut rng = StdRng::seed_from_u64(seed);
        let mut pairs = Vec::new();
        let w = map.width as i32;
        let h = map.height as i32;
        // Only consider left-to-right pairs
        for y in 1..h-1 {
            for x in 1..w-2 {
                let idx = (y as usize) * map.width + (x as usize);
                let nidx = (y as usize) * map.width + ((x+1) as usize);
                if map.cells[idx] == CellState::Empty && map.cells[nidx] == CellState::Empty {
                    pairs.push(((x, y), (x+1, y)));
                }
            }
        }
        let &((x0, y0), (x1, y1)) = pairs.choose(&mut rng).ok_or_else(|| std::io::Error::new(std::io::ErrorKind::Other, "No valid adjacent left-right empty start positions found"))?;
        let mut robot_states = vec![
            RobotState {
                id: 0,
                pose: Pose { position: Point { x: x0, y: y0 }, orientation_rad: -PI/2.0 },
                phase: RobotPhase::InitialWallFind,
                map: GridMap {
                    width: map.width,
                    height: map.height,
                    cells: vec![CellState::Unexplored; map.width * map.height],
                },
                scout_depth_n: 3,
                partner_id: 1,
                last_known_partner_pose: None,
                loop_analysis_data: None,
                travel_direction_before_island: None,
                boundary_scout: None,
            },
            RobotState {
                id: 1,
                pose: Pose { position: Point { x: x1, y: y1 }, orientation_rad: -PI/2.0 },
                phase: RobotPhase::InitialWallFind,
                map: GridMap {
                    width: map.width,
                    height: map.height,
                    cells: vec![CellState::Unexplored; map.width * map.height],
                },
                scout_depth_n: 3,
                partner_id: 0,
                last_known_partner_pose: None,
                loop_analysis_data: None,
                travel_direction_before_island: None,
                boundary_scout: None,
            },
        ];
        // After setting positions, update each robot's local map with initial surroundings
        for robot in &mut robot_states {
            let mut node = RobotNode::new(robot.clone());
            node.update_local_map(&map);
            *robot = node.state;
        }
        Ok(Self::new(map, robot_states))
    }

    /// Advances the simulation by one tick.
    pub fn tick(&mut self) {
        // Clone robots for read-only reference to pass to each tick
        let robots_snapshot = self.robots.clone();
        for robot in &mut self.robots {
            robot.tick(&robots_snapshot, &self.map);
        }
        // TODO: Add global termination checks, communication, etc.
    }

    /// Print all robots' maps for inspection.
    pub fn print_all_maps(&self) {
        for robot in &self.robots {
            robot.print_map();
        }
    }
} 