/// Interior sweep phase implementation - coordinated frontier exploration

use crate::types::*;
use crate::constants::*;
use crate::robot_node::phase_trait::*;
use crate::robot_node::wall_following::WallFollower;

/// Phase 5: Interior Sweep - systematic coordinated exploration of interior
#[derive(Debug, Clone)]
pub struct InteriorSweepPhase;

impl RobotPhaseBehavior for InteriorSweepPhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        println!("Robot {} executing interior sweep", robot_state.id);
        
        // Ensure both robots are in interior sweep phase before proceeding
        let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
        if partner.state.phase != RobotPhase::InteriorSweep {
            println!("Robot {} waiting for partner to enter interior sweep phase", robot_state.id);
            return PhaseTransition::Continue;
        }
        
        // Check if exploration is complete
        if Self::is_exploration_complete(robot_state) {
            println!("Robot {} completed interior exploration - no more unexplored areas!", robot_state.id);
            return PhaseTransition::Transition(RobotPhase::Idle);
        }
        
        // Initialize interior sweep state if needed
        if robot_state.loop_analysis_data.is_none() {
            Self::initialize_interior_sweep(robot_state, partner);
        }
        
        // Execute coordinated sweep movement
        Self::execute_sweep_movement(robot_state, partner, context.global_map)
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::InteriorSweep
    }
}

impl InteriorSweepPhase {
    /// Initialize the interior sweep by setting up sweep parameters
    fn initialize_interior_sweep(robot_state: &mut RobotState, _partner: &crate::robot_node::RobotNode) {
        println!("Robot {} initializing interior sweep", robot_state.id);
        
        // Find the boundary that was just traced
        let boundary_path = if let Some(ref scout_state) = robot_state.boundary_scout {
            scout_state.path.clone()
        } else {
            Vec::new()
        };
        
        robot_state.loop_analysis_data = Some(LoopAnalysisData {
            path_traced: boundary_path,
            total_angular_displacement: 0.0,
            loop_start_position: None,
            loop_closed: Some(true),
            total_loop_length: None,
            midpoint_direction: None,
            target_position: None,
        });
        
        println!("Robot {} interior sweep initialized", robot_state.id);
    }
    
    /// Execute coordinated sweep movement along the frontier
    fn execute_sweep_movement(robot_state: &mut RobotState, partner: &crate::robot_node::RobotNode, global_map: &GridMap) -> PhaseTransition {
        // Find frontier cells (explored cells adjacent to unexplored)
        let frontier_cells = Self::find_frontier_cells(&robot_state.map);
        
        if frontier_cells.is_empty() {
            println!("Robot {} found no frontier cells - exploration complete", robot_state.id);
            return PhaseTransition::Transition(RobotPhase::Idle);
        }
        
        // Check if we're in a sweep iteration or need to start a new one
        let current_pos = robot_state.pose.position;
        
        // Initialize sweep state if needed
        if robot_state.loop_analysis_data.as_ref().unwrap().target_position.is_none() {
            Self::initialize_sweep_iteration(robot_state, partner, &frontier_cells);
        }
        
        // Get target position for this sweep iteration
        let target_pos = robot_state.loop_analysis_data.as_ref().unwrap().target_position.unwrap();
        
        // Check if we've reached our target position
        if current_pos == target_pos {
            println!("Robot {} reached target position ({}, {})", robot_state.id, target_pos.x, target_pos.y);
            
            // Check if partner has also reached their target
            if Self::both_robots_reached_targets(robot_state, partner) {
                println!("Robot {} - both robots completed sweep iteration, moving inward", robot_state.id);
                Self::start_next_sweep_iteration(robot_state, partner, &frontier_cells);
                return PhaseTransition::Continue;
            } else {
                // Wait for partner to reach their target
                println!("Robot {} waiting for partner to complete sweep leg", robot_state.id);
                return PhaseTransition::Continue;
            }
        }
        
        // Move toward target position using frontier-following
        if let Some(next_pos) = Self::move_toward_target_along_frontier(robot_state, target_pos, global_map) {
            robot_state.pose.position = next_pos;
            robot_state.pose.orientation_rad = WallFollower::update_orientation(current_pos, next_pos);
            
            println!("Robot {} swept to ({}, {}) toward target ({}, {})", 
                     robot_state.id, next_pos.x, next_pos.y, target_pos.x, target_pos.y);
            
            PhaseTransition::Continue
        } else {
            println!("Robot {} cannot reach target - exploration complete", robot_state.id);
            PhaseTransition::Transition(RobotPhase::Idle)
        }
    }
    
    /// Find frontier cells (explored cells adjacent to unexplored cells)
    fn find_frontier_cells(map: &GridMap) -> Vec<Point> {
        let mut frontier = Vec::new();
        let directions = [NORTH, SOUTH, EAST, WEST];
        
        for y in 0..map.height {
            for x in 0..map.width {
                let idx = y * map.width + x;
                if map.cells[idx] == CellState::Empty {
                    let pos = Point { x: x as i32, y: y as i32 };
                    
                    // Check if adjacent to unexplored
                    for (dx, dy) in &directions {
                        let neighbor = Point { x: pos.x + dx, y: pos.y + dy };
                        if neighbor.x >= 0 && neighbor.x < map.width as i32 && 
                           neighbor.y >= 0 && neighbor.y < map.height as i32 {
                            let neighbor_idx = (neighbor.y as usize) * map.width + (neighbor.x as usize);
                            if map.cells[neighbor_idx] == CellState::Unexplored {
                                frontier.push(pos);
                                break;
                            }
                        }
                    }
                }
            }
        }
        frontier
    }
    
    /// Initialize a new sweep iteration by assigning target positions
    fn initialize_sweep_iteration(robot_state: &mut RobotState, _partner: &crate::robot_node::RobotNode, frontier_cells: &[Point]) {
        println!("Robot {} initializing new sweep iteration", robot_state.id);
        
        // Find the frontier boundary as a connected path
        let frontier_path = Self::trace_frontier_boundary(frontier_cells);
        
        if frontier_path.len() < 2 {
            // Not enough frontier for coordinated sweep
            if let Some(&first_frontier) = frontier_cells.first() {
                robot_state.loop_analysis_data.as_mut().unwrap().target_position = Some(first_frontier);
            }
            return;
        }
        
        // Assign opposite ends of the frontier, but pick reasonable targets
        let current_pos = robot_state.pose.position;
        
        // Find the best target for this robot based on its role and current position
        let target = if robot_state.id == ROBOT_LEFT_HAND {
            // Robot 0 goes to the leftmost frontier point that's not too far
            frontier_path.iter()
                .take(frontier_path.len() / 2 + 1) // Consider first half + middle
                .min_by_key(|&&p| Self::manhattan_distance(current_pos, p))
                .copied()
                .unwrap_or(frontier_path[0])
        } else {
            // Robot 1 goes to the rightmost frontier point that's not too far  
            frontier_path.iter()
                .skip(frontier_path.len() / 2) // Consider second half
                .min_by_key(|&&p| Self::manhattan_distance(current_pos, p))
                .copied()
                .unwrap_or(frontier_path[frontier_path.len() - 1])
        };
        
        robot_state.loop_analysis_data.as_mut().unwrap().target_position = Some(target);
        
        println!("Robot {} assigned target position ({}, {})", robot_state.id, target.x, target.y);
    }
    
    /// Trace the frontier boundary to create a connected path
    fn trace_frontier_boundary(frontier_cells: &[Point]) -> Vec<Point> {
        if frontier_cells.is_empty() {
            return Vec::new();
        }
        
        if frontier_cells.len() == 1 {
            return frontier_cells.to_vec();
        }
        
        // Find the leftmost and rightmost frontier cells for better separation
        let mut sorted_frontier = frontier_cells.to_vec();
        
        // Sort first by X coordinate (left to right), then by Y 
        sorted_frontier.sort_by_key(|p| (p.x, p.y));
        
        // Return the sorted path from leftmost to rightmost
        sorted_frontier
    }
    
    /// Check if both robots have reached their target positions
    fn both_robots_reached_targets(robot_state: &RobotState, partner: &crate::robot_node::RobotNode) -> bool {
        let robot_reached = robot_state.loop_analysis_data.as_ref()
            .and_then(|data| data.target_position)
            .map(|target| robot_state.pose.position == target)
            .unwrap_or(false);
            
        let partner_reached = partner.state.loop_analysis_data.as_ref()
            .and_then(|data| data.target_position)
            .map(|target| partner.state.pose.position == target)
            .unwrap_or(false);
            
        robot_reached && partner_reached
    }
    
    /// Start the next sweep iteration by moving inward
    fn start_next_sweep_iteration(robot_state: &mut RobotState, _partner: &crate::robot_node::RobotNode, _frontier_cells: &[Point]) {
        println!("Robot {} starting next sweep iteration (moving inward)", robot_state.id);
        
        // Clear current target to trigger re-initialization
        robot_state.loop_analysis_data.as_mut().unwrap().target_position = None;
        
        // Find new frontier after exploration progress
        // This will be handled in the next call to execute_sweep_movement
    }
    
    /// Move toward target position along the frontier
    fn move_toward_target_along_frontier(robot_state: &RobotState, target: Point, global_map: &GridMap) -> Option<Point> {
        let current_pos = robot_state.pose.position;
        let directions = [NORTH, SOUTH, EAST, WEST];
        
        // Find the best direction towards target that follows frontier
        let mut best_move = None;
        let mut best_distance = i32::MAX;
        
        for (dx, dy) in &directions {
            let next_pos = Point { x: current_pos.x + dx, y: current_pos.y + dy };
            
            // Check if move is valid and gets us closer to target
            if Self::is_valid_move(next_pos, global_map) {
                let distance = Self::manhattan_distance(next_pos, target);
                if distance < best_distance {
                    best_distance = distance;
                    best_move = Some(next_pos);
                }
            }
        }
        
        best_move
    }
    
    /// Check if a move is valid (not obstacle, within bounds)
    fn is_valid_move(pos: Point, global_map: &GridMap) -> bool {
        if pos.x < 0 || pos.x >= global_map.width as i32 || 
           pos.y < 0 || pos.y >= global_map.height as i32 {
            return false;
        }
        
        let idx = (pos.y as usize) * global_map.width + (pos.x as usize);
        global_map.cells[idx] != CellState::Obstacle
    }
    
    /// Calculate Manhattan distance between two points
    fn manhattan_distance(a: Point, b: Point) -> i32 {
        (a.x - b.x).abs() + (a.y - b.y).abs()
    }
    
    /// Check if exploration is complete (no more unexplored areas)
    fn is_exploration_complete(robot_state: &RobotState) -> bool {
        let unexplored_count = robot_state.map.cells.iter()
            .filter(|&&cell| cell == CellState::Unexplored)
            .count();
        
        // Consider exploration complete if very few unexplored cells remain
        unexplored_count < 5
    }
}