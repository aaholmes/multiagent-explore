/// Central scan phase implementation - FUTURE ENHANCEMENT
/// This phase represents iterative interior exploration using virtual boundaries.
/// Currently transitions directly to InteriorSweep as a foundation for future development.

use crate::types::*;
use crate::constants::*;
use crate::robot_node::phase_trait::*;
use crate::robot_node::wall_following::WallFollower;

/// Phase 4: Central Scan - FUTURE ENHANCEMENT
/// Reserved for iterative interior exploration using virtual boundaries
#[derive(Debug, Clone)]
pub struct CentralScanPhase;

impl RobotPhaseBehavior for CentralScanPhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        println!("Robot {} executing central scan", robot_state.id);
        
        // Check partner phase - if partner is Idle, we should also complete
        let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
        if partner.state.phase == RobotPhase::Idle {
            println!("Robot {} completing central scan because partner is already Idle", robot_state.id);
            return PhaseTransition::Transition(RobotPhase::Idle);
        }
        if partner.state.phase != RobotPhase::CentralScan {
            println!("Robot {} waiting for partner to enter central scan phase", robot_state.id);
            return PhaseTransition::Continue;
        }
        
        // Initialize central scan state if first time
        if robot_state.central_scan.is_none() {
            Self::initialize_central_scan(robot_state);
        }
        
        // Execute the central scan logic
        Self::execute_central_scan_step(robot_state, context)
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::CentralScan
    }
}

impl CentralScanPhase {
    /// Initialize central scan state from completed boundary scout data
    fn initialize_central_scan(robot_state: &mut RobotState) {
        if let Some(ref boundary_scout) = robot_state.boundary_scout {
            let virtual_boundary = boundary_scout.path.clone();
            
            println!("Robot {} initializing central scan with {} boundary points", 
                     robot_state.id, virtual_boundary.len());
            
            robot_state.central_scan = Some(CentralScanState {
                virtual_boundary: virtual_boundary.clone(),
                scan_iteration: 0,
                completed_loops: vec![virtual_boundary],
            });
            
            // Move robot one step inward from current position
            Self::move_robot_inward(robot_state);
            
            // Reset boundary scouting for virtual boundary tracing
            Self::reset_boundary_scout_for_virtual_tracing(robot_state);
        } else {
            println!("Robot {} has no boundary scout data, cannot initialize central scan", robot_state.id);
        }
    }
    
    /// Move robot one step inward from the boundary
    fn move_robot_inward(robot_state: &mut RobotState) {
        let current_pos = robot_state.pose.position;
        
        // Try to move inward (towards unexplored areas)
        // For simplicity, try all four cardinal directions and pick the first valid one
        let inward_directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]; // S, N, E, W
        
        for (dx, dy) in &inward_directions {
            let next_pos = Point {
                x: current_pos.x + dx,
                y: current_pos.y + dy,
            };
            
            // Check if this position would be valid and not part of existing boundaries
            if let Some(ref central_scan) = robot_state.central_scan {
                if Self::is_position_valid_for_inward_movement(next_pos, &robot_state.map, &central_scan.completed_loops) {
                    println!("Robot {} moved inward from ({}, {}) to ({}, {})", 
                             robot_state.id, current_pos.x, current_pos.y, next_pos.x, next_pos.y);
                    robot_state.pose.position = next_pos;
                    return;
                }
            }
        }
        
        println!("Robot {} could not move inward from ({}, {})", 
                 robot_state.id, current_pos.x, current_pos.y);
    }
    
    /// Check if a position is valid for inward movement
    fn is_position_valid_for_inward_movement(pos: Point, robot_map: &GridMap, completed_loops: &[Vec<Point>]) -> bool {
        // Check bounds
        if pos.x < 0 || pos.y < 0 || pos.x >= robot_map.width as i32 || pos.y >= robot_map.height as i32 {
            return false;
        }
        
        // Check if position is empty or unexplored (not an obstacle)
        let idx = (pos.y as usize) * robot_map.width + (pos.x as usize);
        if idx >= robot_map.cells.len() {
            return false;
        }
        
        let cell_state = robot_map.cells[idx];
        if cell_state == CellState::Obstacle {
            return false;
        }
        
        // Check if position is not part of any completed boundary
        for boundary in completed_loops {
            if boundary.contains(&pos) {
                return false;
            }
        }
        
        true
    }
    
    /// Reset boundary scout state for virtual boundary tracing (no scouting missions)
    fn reset_boundary_scout_for_virtual_tracing(robot_state: &mut RobotState) {
        // Determine tracing direction based on robot ID (opposite directions)
        let tracing_direction = if robot_state.id == ROBOT_LEFT_HAND { 
            RIGHT_HAND_RULE
        } else { 
            LEFT_HAND_RULE 
        };
        
        // For central scan, we don't need scouting missions - just trace until rendezvous
        robot_state.boundary_scout = Some(BoundaryScoutState {
            tracing_direction,
            steps_taken: 0,
            steps_taken_this_scouting_mission: 0,
            returning: false,
            path: vec![robot_state.pose.position],
            first_move: true,
            initial_scouting_direction: None,
            total_rotation_steps: 0,
        });
        
        println!("Robot {} reset for virtual boundary tracing (no scouting missions), direction {}", 
                 robot_state.id, tracing_direction);
    }
    
    /// Execute one step of the central scan
    fn execute_central_scan_step(robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
        
        // Get virtual boundaries from central scan state
        let virtual_boundaries = if let Some(ref central_scan) = robot_state.central_scan {
            central_scan.completed_loops.clone()
        } else {
            return PhaseTransition::Continue;
        };
        
        // Execute direct virtual boundary tracing (no scouting missions)
        let trace_result = Self::execute_direct_virtual_tracing(robot_state, context, &virtual_boundaries);
        
        match trace_result {
            VirtualTraceResult::Continue => PhaseTransition::Continue,
            VirtualTraceResult::LoopCompleted => {
                // Analyze the completed loop
                Self::handle_completed_virtual_loop(robot_state, partner)
            },
            VirtualTraceResult::NoProgress => {
                // Central scan complete - no more areas to explore
                println!("*** CENTRAL SCAN DEBUG: Robot {} central scan complete - no more explorable areas", robot_state.id);
                PhaseTransition::Transition(RobotPhase::Idle)
            }
        }
    }
    
    /// Execute direct virtual boundary tracing (no scouting missions)
    fn execute_direct_virtual_tracing(robot_state: &mut RobotState, context: &PhaseContext, virtual_boundaries: &[Vec<Point>]) -> VirtualTraceResult {
        let (tracing_direction, first_move) = if let Some(scout) = robot_state.boundary_scout.as_ref() {
            (scout.tracing_direction, scout.first_move)
        } else {
            return VirtualTraceResult::Continue;
        };
        
        let next_pos = if first_move {
            // First move: find direction along virtual boundary
            let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
            Self::calculate_first_virtual_move(robot_state, partner, virtual_boundaries)
        } else {
            // Regular virtual boundary following
            WallFollower::wall_follow_step_virtual(
                robot_state.pose.position,
                robot_state.pose.orientation_rad,
                context.global_map,
                virtual_boundaries,
                tracing_direction,
            )
        };
        
        if let Some(next_pos) = next_pos {
            let prev_pos = robot_state.pose.position;
            robot_state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
            robot_state.pose.position = next_pos;
            
            println!("Robot {} virtual trace: moved to ({}, {})", robot_state.id, next_pos.x, next_pos.y);
            
            // Update boundary scout state
            if let Some(scout) = robot_state.boundary_scout.as_mut() {
                scout.path.push(next_pos);
                scout.steps_taken += 1;
                scout.first_move = false;
            }
            
            // Check for rendezvous during tracing
            let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
            if !first_move && Self::within_comm_range(&robot_state.pose.position, &partner.state.pose.position) 
               && robot_state.pose.position != partner.state.pose.position {
                println!("Robot {} virtual rendezvous with partner - loop completed", robot_state.id);
                return VirtualTraceResult::LoopCompleted;
            }
            
            VirtualTraceResult::Continue
        } else {
            println!("Robot {} cannot continue virtual boundary tracing", robot_state.id);
            VirtualTraceResult::NoProgress
        }
    }
    
    /// Calculate first move for virtual boundary tracing
    fn calculate_first_virtual_move(robot_state: &RobotState, _partner: &crate::robot_node::RobotNode, virtual_boundaries: &[Vec<Point>]) -> Option<Point> {
        // Try to find a direction that leads along a virtual boundary
        let current_pos = robot_state.pose.position;
        let directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]; // S, N, E, W
        
        for (dx, dy) in &directions {
            let next_pos = Point {
                x: current_pos.x + dx,
                y: current_pos.y + dy,
            };
            
            if WallFollower::is_position_valid_and_empty_virtual(next_pos, &robot_state.map, virtual_boundaries) {
                // Check if this move would be along a virtual boundary (adjacent to virtual wall)
                if Self::is_adjacent_to_virtual_boundary(next_pos, virtual_boundaries) {
                    return Some(next_pos);
                }
            }
        }
        
        // Fallback: any valid move
        for (dx, dy) in &directions {
            let next_pos = Point {
                x: current_pos.x + dx,
                y: current_pos.y + dy,
            };
            
            if WallFollower::is_position_valid_and_empty_virtual(next_pos, &robot_state.map, virtual_boundaries) {
                return Some(next_pos);
            }
        }
        
        None
    }
    
    /// Check if a position is adjacent to any virtual boundary
    fn is_adjacent_to_virtual_boundary(pos: Point, virtual_boundaries: &[Vec<Point>]) -> bool {
        let adjacent_positions = [
            Point { x: pos.x + 1, y: pos.y },
            Point { x: pos.x - 1, y: pos.y },
            Point { x: pos.x, y: pos.y + 1 },
            Point { x: pos.x, y: pos.y - 1 },
        ];
        
        for adj_pos in &adjacent_positions {
            if WallFollower::is_virtual_wall(*adj_pos, virtual_boundaries) {
                return true;
            }
        }
        
        false
    }
    
    /// Handle completed virtual loop
    fn handle_completed_virtual_loop(robot_state: &mut RobotState, _partner: &crate::robot_node::RobotNode) -> PhaseTransition {
        // Analyze the completed virtual loop
        let contains_unexplored = Self::loop_contains_unexplored_area(robot_state);
        
        if contains_unexplored {
            // Valid interior loop - add to completed loops and continue to next layer
            if let Some(ref mut central_scan) = robot_state.central_scan {
                if let Some(ref scout) = robot_state.boundary_scout {
                    let new_virtual_boundary = scout.path.clone();
                    central_scan.completed_loops.push(new_virtual_boundary.clone());
                    central_scan.virtual_boundary = new_virtual_boundary;
                    central_scan.scan_iteration += 1;
                    
                    println!("Robot {} completed virtual loop {}, moving to next layer", 
                             robot_state.id, central_scan.scan_iteration);
                    
                    // Move inward and reset for next iteration
                    Self::move_robot_inward(robot_state);
                    Self::reset_boundary_scout_for_virtual_tracing(robot_state);
                    
                    PhaseTransition::Continue
                } else {
                    PhaseTransition::Continue
                }
            } else {
                PhaseTransition::Continue
            }
        } else {
            // No more unexplored area inside - central scan complete
            println!("*** CENTRAL SCAN DEBUG: Robot {} completed central scan - no more unexplored areas", robot_state.id);
            PhaseTransition::Transition(RobotPhase::Idle)
        }
    }
    
    /// Check if the current loop contains unexplored areas
    fn loop_contains_unexplored_area(robot_state: &RobotState) -> bool {
        // Simple heuristic: check if there are unexplored cells in the map
        // In a more sophisticated implementation, we would check specifically inside the loop
        for cell in &robot_state.map.cells {
            if *cell == CellState::Unexplored {
                return true;
            }
        }
        false
    }
    
    /// Check if two positions are within communication range
    fn within_comm_range(a: &Point, b: &Point) -> bool {
        (a.x - b.x).abs() + (a.y - b.y).abs() <= COMMUNICATION_RANGE
    }
}

/// Result of virtual boundary tracing step
#[derive(Debug)]
enum VirtualTraceResult {
    Continue,        // Keep tracing
    LoopCompleted,   // Robots met, loop is complete
    NoProgress,      // Cannot continue, central scan done
}