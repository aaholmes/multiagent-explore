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
        
        // Determine sweep direction based on robot ID
        let sweep_direction = if robot_state.id == ROBOT_LEFT_HAND {
            LEFT_HAND_RULE  // Robot 0 sweeps left
        } else {
            RIGHT_HAND_RULE // Robot 1 sweeps right
        };
        
        // Find the closest frontier cell
        let current_pos = robot_state.pose.position;
        let target_frontier = frontier_cells.iter()
            .min_by_key(|&&p| Self::manhattan_distance(current_pos, p))
            .copied();
        
        if let Some(target) = target_frontier {
            // Move towards the frontier using wall-following behavior
            if let Some(next_pos) = Self::move_towards_frontier(robot_state, target, global_map, sweep_direction) {
                robot_state.pose.position = next_pos;
                robot_state.pose.orientation_rad = WallFollower::update_orientation(current_pos, next_pos);
                
                println!("Robot {} swept to ({}, {}) towards frontier", robot_state.id, next_pos.x, next_pos.y);
                
                // Check if robots should rendezvous
                if Self::should_rendezvous(robot_state, partner) {
                    println!("Robot {} completed sweep leg - ready for next iteration", robot_state.id);
                }
                
                PhaseTransition::Continue
            } else {
                println!("Robot {} cannot reach frontier - exploration complete", robot_state.id);
                PhaseTransition::Transition(RobotPhase::Idle)
            }
        } else {
            println!("Robot {} found no reachable frontier - exploration complete", robot_state.id);
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
    
    /// Move towards a frontier cell using coordinated movement
    fn move_towards_frontier(robot_state: &RobotState, target: Point, global_map: &GridMap, _sweep_direction: i8) -> Option<Point> {
        let current_pos = robot_state.pose.position;
        let directions = [NORTH, SOUTH, EAST, WEST];
        
        // Find the best direction towards target
        let mut best_move = None;
        let mut best_distance = i32::MAX;
        
        for (dx, dy) in &directions {
            let next_pos = Point { x: current_pos.x + dx, y: current_pos.y + dy };
            
            // Check if move is valid
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
    
    /// Check if robots should rendezvous (close to each other)
    fn should_rendezvous(robot_state: &RobotState, partner: &crate::robot_node::RobotNode) -> bool {
        let distance = Self::manhattan_distance(robot_state.pose.position, partner.state.pose.position);
        distance <= COMMUNICATION_RANGE
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