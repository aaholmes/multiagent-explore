/// Interior sweep phase implementation

use crate::types::*;
use crate::constants::*;
use crate::robot_node::phase_trait::*;
use crate::robot_node::wall_following::WallFollower;

/// Phase 4B: Interior Sweep - scan for large interior obstacles
#[derive(Debug, Clone)]
pub struct InteriorSweepPhase;

impl RobotPhaseBehavior for InteriorSweepPhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        println!("Robot {} executing interior sweep", robot_state.id);
        
        // Divide the interior area and perform a straight-line sweep
        if let Some(ref scout_state) = robot_state.boundary_scout {
            let room_bounds = Self::calculate_room_bounds(&scout_state.path, robot_state.pose.position);
            let sweep_target = Self::calculate_sweep_target(room_bounds, robot_state.pose.position);
            
            // Move towards the sweep target
            let direction = Self::calculate_direction_to_target(robot_state.pose.position, sweep_target);
            let (dx, dy) = direction;
            let next_pos = Point {
                x: robot_state.pose.position.x + dx,
                y: robot_state.pose.position.y + dy,
            };
            
            if WallFollower::is_position_valid_and_empty(next_pos, context.global_map) {
                let prev_pos = robot_state.pose.position;
                robot_state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
                robot_state.pose.position = next_pos;
                println!("Robot {} sweeping interior, moved to ({}, {})", robot_state.id, next_pos.x, next_pos.y);
                
                // Check if we hit an obstacle during sweep
                let forward_pos = Point {
                    x: next_pos.x + dx,
                    y: next_pos.y + dy,
                };
                
                if !WallFollower::is_position_valid_and_empty(forward_pos, context.global_map) {
                    println!("Robot {} found interior obstacle during sweep. Starting boundary trace.", robot_state.id);
                    robot_state.scout_depth_n = INITIAL_SCOUT_DEPTH;
                    robot_state.boundary_scout = None;
                    return PhaseTransition::Transition(RobotPhase::BoundaryScouting);
                }
                
                PhaseTransition::Continue
            } else {
                println!("Robot {} completed interior sweep area", robot_state.id);
                PhaseTransition::Complete
            }
        } else {
            println!("Robot {} has no boundary data for interior sweep. Returning to wall finding.", robot_state.id);
            PhaseTransition::Transition(RobotPhase::InitialWallFind)
        }
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::InteriorSweep
    }
}

impl InteriorSweepPhase {
    /// Calculate room bounds from exterior wall path
    fn calculate_room_bounds(path: &[Point], fallback: Point) -> (Point, Point) {
        if path.is_empty() {
            return (fallback, fallback);
        }
        
        let min_x = path.iter().map(|p| p.x).min().unwrap_or(0);
        let max_x = path.iter().map(|p| p.x).max().unwrap_or(0);
        let min_y = path.iter().map(|p| p.y).min().unwrap_or(0);
        let max_y = path.iter().map(|p| p.y).max().unwrap_or(0);
        
        (Point { x: min_x, y: min_y }, Point { x: max_x, y: max_y })
    }

    /// Calculate sweep target for interior exploration
    fn calculate_sweep_target(bounds: (Point, Point), current_pos: Point) -> Point {
        let (min_bound, max_bound) = bounds;
        
        // Simple strategy: sweep to opposite corner of the room
        if current_pos.x - min_bound.x < max_bound.x - current_pos.x {
            // Closer to left, sweep right
            Point { x: max_bound.x - 1, y: (min_bound.y + max_bound.y) / 2 }
        } else {
            // Closer to right, sweep left
            Point { x: min_bound.x + 1, y: (min_bound.y + max_bound.y) / 2 }
        }
    }

    /// Calculate direction to move towards target
    fn calculate_direction_to_target(current_pos: Point, target: Point) -> (i32, i32) {
        let dx = target.x - current_pos.x;
        let dy = target.y - current_pos.y;
        
        // Normalize to unit direction, prioritize larger difference
        if dx.abs() > dy.abs() {
            (if dx > 0 { 1 } else { -1 }, 0)
        } else if dy != 0 {
            (0, if dy > 0 { 1 } else { -1 })
        } else {
            (0, 0) // At target
        }
    }
}