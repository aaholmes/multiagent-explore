/// Initial wall finding phase implementation

use crate::types::*;
use crate::robot_node::phase_trait::*;

/// Phase 1: Move in a straight line until a wall is seen directly in front
#[derive(Debug, Clone)]
pub struct WallFindPhase;

impl RobotPhaseBehavior for WallFindPhase {
    fn execute(&mut self, robot_state: &mut RobotState, _context: &PhaseContext) -> PhaseTransition {
        let (next_pos, cell) = Self::sense_front(robot_state);
        
        match cell {
            Some(CellState::Obstacle) => {
                println!("Robot {} sees obstacle in front at ({}, {}), stopping and starting boundary scouting.", 
                         robot_state.id, next_pos.x, next_pos.y);
                PhaseTransition::Transition(RobotPhase::BoundaryScouting)
            }
            _ => {
                // Move forward (-Y direction)
                println!("Robot {} moves from ({}, {}) to ({}, {})", 
                         robot_state.id, robot_state.pose.position.x, robot_state.pose.position.y, 
                         next_pos.x, next_pos.y);
                robot_state.pose.position = next_pos;
                
                // Update local map (mark as empty)
                let idx = (next_pos.y as usize) * robot_state.map.width + (next_pos.x as usize);
                robot_state.map.cells[idx] = CellState::Empty;
                
                PhaseTransition::Continue
            }
        }
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::InitialWallFind
    }
}

impl WallFindPhase {
    /// Sense the cell directly in front of the robot
    fn sense_front(robot_state: &RobotState) -> (Point, Option<CellState>) {
        let next_pos = Point {
            x: robot_state.pose.position.x,
            y: robot_state.pose.position.y - 1, // Move north (-Y direction)
        };
        
        let width = robot_state.map.width as i32;
        let height = robot_state.map.height as i32;
        
        if next_pos.x < 0 || next_pos.x >= width || next_pos.y < 0 || next_pos.y >= height {
            (next_pos, Some(CellState::Obstacle))
        } else {
            let idx = (next_pos.y as usize) * robot_state.map.width + (next_pos.x as usize);
            (next_pos, Some(robot_state.map.cells[idx]))
        }
    }
}