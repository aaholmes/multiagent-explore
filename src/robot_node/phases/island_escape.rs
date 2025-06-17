/// Island escape phase implementation

use crate::types::*;
use crate::constants::*;
use crate::robot_node::phase_trait::*;
use crate::robot_node::wall_following::WallFollower;

/// Phase 4A: Island Escape - move away from detected island to find exterior wall
#[derive(Debug, Clone)]
pub struct IslandEscapePhase;

impl RobotPhaseBehavior for IslandEscapePhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        println!("Robot {} executing island escape", robot_state.id);
        
        // Move in a straight line away from the island to find the exterior wall
        if let Some(ref scout_state) = robot_state.boundary_scout {
            let island_center = Self::calculate_path_centroid(&scout_state.path, robot_state.pose.position);
            let escape_direction = Self::calculate_escape_direction(robot_state.pose.position, island_center);
            
            // Try to move in the escape direction
            let (dx, dy) = escape_direction;
            let next_pos = Point {
                x: robot_state.pose.position.x + dx,
                y: robot_state.pose.position.y + dy,
            };
            
            // Check bounds and obstacles
            if WallFollower::is_position_valid_and_empty(next_pos, context.global_map) {
                let prev_pos = robot_state.pose.position;
                robot_state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
                robot_state.pose.position = next_pos;
                println!("Robot {} escaping island, moved to ({}, {})", robot_state.id, next_pos.x, next_pos.y);
                
                // After moving away, look for a new wall to start boundary scouting again
                let forward_pos = Point {
                    x: next_pos.x + dx,
                    y: next_pos.y + dy,
                };
                
                if !WallFollower::is_position_valid_and_empty(forward_pos, context.global_map) {
                    println!("Robot {} found new wall during island escape. Restarting boundary scouting.", robot_state.id);
                    robot_state.scout_depth_n = INITIAL_SCOUT_DEPTH;
                    robot_state.boundary_scout = None;
                    return PhaseTransition::Transition(RobotPhase::BoundaryScouting);
                }
                
                PhaseTransition::Continue
            } else {
                println!("Robot {} cannot move in escape direction, exploring alternatives", robot_state.id);
                Self::try_alternative_escape_directions(robot_state, context.global_map);
                PhaseTransition::Continue
            }
        } else {
            println!("Robot {} has no boundary scout data for island escape. Returning to wall finding.", robot_state.id);
            PhaseTransition::Transition(RobotPhase::InitialWallFind)
        }
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::IslandEscape
    }
}

impl IslandEscapePhase {
    /// Calculate the centroid (center point) of a path
    fn calculate_path_centroid(path: &[Point], fallback: Point) -> Point {
        if path.is_empty() {
            return fallback;
        }
        
        let sum_x: i32 = path.iter().map(|p| p.x).sum();
        let sum_y: i32 = path.iter().map(|p| p.y).sum();
        let len = path.len() as i32;
        
        Point {
            x: sum_x / len,
            y: sum_y / len,
        }
    }

    /// Calculate direction to escape from an island center
    fn calculate_escape_direction(current_pos: Point, island_center: Point) -> (i32, i32) {
        let dx = current_pos.x - island_center.x;
        let dy = current_pos.y - island_center.y;
        
        // Normalize to unit direction
        if dx.abs() > dy.abs() {
            (if dx > 0 { 1 } else { -1 }, 0)
        } else {
            (0, if dy > 0 { 1 } else { -1 })
        }
    }

    /// Try alternative escape directions if primary direction is blocked
    fn try_alternative_escape_directions(robot_state: &mut RobotState, global_map: &GridMap) {
        let directions = [NORTH, EAST, SOUTH, WEST];
        
        for (dx, dy) in &directions {
            let next_pos = Point {
                x: robot_state.pose.position.x + dx,
                y: robot_state.pose.position.y + dy,
            };
            
            if WallFollower::is_position_valid_and_empty(next_pos, global_map) {
                let prev_pos = robot_state.pose.position;
                robot_state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
                robot_state.pose.position = next_pos;
                println!("Robot {} found alternative escape direction to ({}, {})", 
                         robot_state.id, next_pos.x, next_pos.y);
                return;
            }
        }
        
        println!("Robot {} is trapped, cannot find escape direction", robot_state.id);
    }
}