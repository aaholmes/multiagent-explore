/// Boundary scouting phase implementation

use crate::types::*;
use crate::constants::*;
use crate::robot_node::phase_trait::*;
use crate::robot_node::wall_following::{WallFollower, RotationTracker};

/// Phase 2: Iterative boundary scouting with exponentially increasing depth
#[derive(Debug, Clone)]
pub struct BoundaryScoutingPhase;

impl RobotPhaseBehavior for BoundaryScoutingPhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        // Initialize state if first tick in this phase
        if robot_state.boundary_scout.is_none() {
            Self::initialize_boundary_scout(robot_state);
        }

        let scout_n = robot_state.scout_depth_n;
        let steps_taken_this_scouting_mission = robot_state.boundary_scout
            .as_ref()
            .map(|s| s.steps_taken_this_scouting_mission)
            .unwrap_or(0);
        let returning = robot_state.boundary_scout
            .as_ref()
            .map(|s| s.returning)
            .unwrap_or(false);

        // Handle return journey
        if returning {
            return Self::handle_return_journey(robot_state, scout_n);
        }

        // Handle forward scouting
        if steps_taken_this_scouting_mission < scout_n {
            Self::execute_forward_scouting(robot_state, context)
        } else {
            // Reached scouting depth, start returning
            if let Some(scout) = robot_state.boundary_scout.as_mut() {
                scout.returning = true;
                scout.steps_taken_this_scouting_mission = 0;
            }
            println!("Robot {} finished {} steps, returning.", robot_state.id, scout_n);
            PhaseTransition::Continue
        }
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::BoundaryScouting
    }
}

impl BoundaryScoutingPhase {
    fn initialize_boundary_scout(robot_state: &mut RobotState) {
        // Determine tracing direction based on which way robot will turn
        // This is temporary - we'll determine the actual direction after first move
        let tracing_direction = if robot_state.id == ROBOT_LEFT_HAND { 
            RIGHT_HAND_RULE  // Will be set correctly after first move
        } else { 
            LEFT_HAND_RULE   // Will be set correctly after first move
        };
        
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
        
        println!("Robot {} begins boundary scouting, tracing_direction {}", 
                 robot_state.id, tracing_direction);
    }

    fn handle_return_journey(robot_state: &mut RobotState, scout_n: u32) -> PhaseTransition {
        if let Some(scout) = robot_state.boundary_scout.as_ref() {
            let path_length = scout.path.len();
            if scout.steps_taken_this_scouting_mission + 1 < path_length as u32 {
                // Continue returning
                let target_index = path_length - 2 - scout.steps_taken_this_scouting_mission as usize;
                let next_pos = scout.path[target_index];
                
                println!("Robot {} returns to ({}, {}). Path length remaining: {}", 
                         robot_state.id, next_pos.x, next_pos.y, 
                         path_length - 1 - scout.steps_taken_this_scouting_mission as usize);
                
                let prev_pos = robot_state.pose.position;
                robot_state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
                robot_state.pose.position = next_pos;
                
                if let Some(scout) = robot_state.boundary_scout.as_mut() {
                    scout.steps_taken_this_scouting_mission += 1;
                }
                
                PhaseTransition::Continue
            } else {
                // Robot has returned to the start of the leg
                println!("Robot {} completed return scan. Doubling scout_depth_n ({} -> {}) and starting next leg.", 
                         robot_state.id, scout_n, scout_n * 2);
                robot_state.scout_depth_n *= 2;
                
                if let Some(scout) = robot_state.boundary_scout.as_mut() {
                    scout.steps_taken_this_scouting_mission = 0;
                    scout.returning = false;
                    scout.path.clear();
                    scout.path.push(robot_state.pose.position);
                    scout.first_move = true;
                }
                
                PhaseTransition::Continue
            }
        } else {
            PhaseTransition::Continue
        }
    }

    fn execute_forward_scouting(robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        let (tracing_direction, first_move) = if let Some(scout) = robot_state.boundary_scout.as_ref() {
            (scout.tracing_direction, scout.first_move)
        } else {
            return PhaseTransition::Continue;
        };

        let next = if first_move {
            // Check if we have a stored initial scouting direction
            let stored_direction = robot_state.boundary_scout.as_ref().and_then(|s| s.initial_scouting_direction);
            
            let next_pos = if let Some(direction) = stored_direction {
                // Use the stored initial scouting direction
                let next = Point {
                    x: robot_state.pose.position.x + direction.x,
                    y: robot_state.pose.position.y + direction.y,
                };
                
                if WallFollower::is_position_valid_and_empty(next, context.global_map) {
                    Some(next)
                } else {
                    // Fallback to wall following if stored direction is blocked
                    WallFollower::wall_follow_step(
                        robot_state.pose.position,
                        robot_state.pose.orientation_rad,
                        context.global_map,
                        tracing_direction,
                    )
                }
            } else {
                // First time - calculate direction based on partner position
                let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
                let next_pos = WallFollower::wall_follow_step_first_move(
                    robot_state.pose.position,
                    robot_state.pose.orientation_rad,
                    context.global_map,
                    partner.state.pose.position,
                );
                
                // Store the initial direction and set correct tracing direction
                if let Some(pos) = next_pos {
                    let direction = Point {
                        x: pos.x - robot_state.pose.position.x,
                        y: pos.y - robot_state.pose.position.y,
                    };
                    
                    let current_pos = robot_state.pose.position;
                    let (current_dx, current_dy) = WallFollower::get_direction_vector(robot_state.pose.orientation_rad);
                    let move_dx = pos.x - current_pos.x;
                    let move_dy = pos.y - current_pos.y;
                    
                    // Check if we turned left or right
                    let cross_product = current_dx * move_dy - current_dy * move_dx;
                    let turned_left = cross_product > 0;
                    
                    // If turned left, follow wall on right (RIGHT_HAND_RULE)
                    // If turned right, follow wall on left (LEFT_HAND_RULE)
                    let correct_tracing_direction = if turned_left {
                        RIGHT_HAND_RULE
                    } else {
                        LEFT_HAND_RULE
                    };
                    
                    if let Some(scout) = robot_state.boundary_scout.as_mut() {
                        scout.initial_scouting_direction = Some(direction);
                        scout.tracing_direction = correct_tracing_direction;
                    }
                }
                
                next_pos
            };
            
            next_pos
        } else {
            WallFollower::wall_follow_step(
                robot_state.pose.position,
                robot_state.pose.orientation_rad,
                context.global_map,
                tracing_direction,
            )
        };

        if let Some(next_pos) = next {
            let steps_taken_this_scouting_mission = robot_state.boundary_scout
                .as_ref()
                .map(|s| s.steps_taken_this_scouting_mission)
                .unwrap_or(0);
            let scout_n = robot_state.scout_depth_n;
            
            println!("Robot {} wall-follows to ({}, {}) [step {}/{}]", 
                     robot_state.id, next_pos.x, next_pos.y, 
                     steps_taken_this_scouting_mission + 1, scout_n);
            
            let prev_pos = robot_state.pose.position;
            let prev_orientation = robot_state.pose.orientation_rad;
            robot_state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
            robot_state.pose.position = next_pos;
            
            // Track rotation change for boundary analysis
            let rotation_steps = RotationTracker::calculate_rotation_steps(prev_orientation, robot_state.pose.orientation_rad);
            if let Some(scout) = robot_state.boundary_scout.as_mut() {
                scout.total_rotation_steps += rotation_steps;
                scout.path.push(next_pos);
                scout.steps_taken += 1;
                scout.steps_taken_this_scouting_mission += 1;
                
                // Reset rotation tracking after first move of a new leg
                if scout.first_move {
                    scout.total_rotation_steps = 0;
                }
                scout.first_move = false;
            }

            // Check for rendezvous during active scouting leg
            let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
            
            if !first_move && Self::within_comm_range(&robot_state.pose.position, &partner.state.pose.position) 
               && robot_state.pose.position != partner.state.pose.position {
                println!("Robot {} rendezvous with partner {} during scouting leg. Transitioning to BOUNDARY_ANALYSIS.", 
                         robot_state.id, partner.state.id);
                return PhaseTransition::Transition(RobotPhase::BoundaryAnalysis);
            }

            PhaseTransition::Continue
        } else {
            println!("Robot {} cannot wall-follow, stays at ({}, {})", 
                     robot_state.id, robot_state.pose.position.x, robot_state.pose.position.y);
            PhaseTransition::Continue
        }
    }

    /// Returns true if two positions are within communication range
    fn within_comm_range(a: &Point, b: &Point) -> bool {
        (a.x - b.x).abs() + (a.y - b.y).abs() <= COMMUNICATION_RANGE
    }
}