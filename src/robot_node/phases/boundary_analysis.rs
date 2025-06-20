/// Boundary analysis phase implementation

use crate::types::*;
use crate::constants::*;
use crate::robot_node::phase_trait::*;
use crate::robot_node::boundary_analysis::BoundaryAnalyzer;

/// Phase 3: Analyze the boundary trace to determine if it's a closed loop (island)
#[derive(Debug, Clone)]
pub struct BoundaryAnalysisPhase;

impl RobotPhaseBehavior for BoundaryAnalysisPhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        println!("Robot {} executing boundary analysis", robot_state.id);
        
        // Check if both robots are in boundary analysis phase
        let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
        if partner.state.phase == RobotPhase::BoundaryAnalysis {
            // Use rotation-based analysis
            let rotation_analysis = Self::analyze_boundary_by_rotation(robot_state, partner);
            
            match rotation_analysis {
                BoundaryAnalysisResult::Island => {
                    println!("Robot {} detected ISLAND (obstacle) via rotation analysis. Transitioning to IslandEscape to find exterior wall.", 
                             robot_state.id);
                    PhaseTransition::Transition(RobotPhase::IslandEscape)
                },
                BoundaryAnalysisResult::ExteriorWall => {
                    println!("Robot {} detected EXTERIOR WALL via rotation analysis. Transitioning to CentralScan for interior exploration.", 
                             robot_state.id);
                    PhaseTransition::Transition(RobotPhase::CentralScan)
                },
                BoundaryAnalysisResult::Incomplete => {
                    println!("Robot {} rotation analysis incomplete. Continuing analysis.", robot_state.id);
                    PhaseTransition::Continue
                }
            }
        } else {
            PhaseTransition::Continue
        }
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::BoundaryAnalysis
    }
}

impl BoundaryAnalysisPhase {
    /// Analyze boundary type using rotation difference between robots
    fn analyze_boundary_by_rotation(robot_state: &RobotState, partner: &crate::robot_node::RobotNode) -> BoundaryAnalysisResult {
        // Get rotation data from both robots
        let robot0_rotation = if robot_state.id == ROBOT_LEFT_HAND {
            robot_state.boundary_scout.as_ref().map(|s| s.total_rotation_steps)
        } else {
            partner.state.boundary_scout.as_ref().map(|s| s.total_rotation_steps)
        };
        
        let robot1_rotation = if robot_state.id == ROBOT_RIGHT_HAND {
            robot_state.boundary_scout.as_ref().map(|s| s.total_rotation_steps)
        } else {
            partner.state.boundary_scout.as_ref().map(|s| s.total_rotation_steps)
        };
        
        BoundaryAnalyzer::analyze_boundary_by_rotation(robot0_rotation, robot1_rotation)
    }
}