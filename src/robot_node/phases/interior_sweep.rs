/// Interior sweep phase implementation - simplified for MVP

use crate::types::*;
use crate::robot_node::phase_trait::*;

/// Phase 5: Interior Sweep - systematic frontier exploration (simplified for MVP)
#[derive(Debug, Clone)]
pub struct InteriorSweepPhase;

impl RobotPhaseBehavior for InteriorSweepPhase {
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition {
        println!("Robot {} executing interior sweep (simplified MVP version)", robot_state.id);
        
        // Ensure both robots are in interior sweep phase before proceeding
        let partner = context.all_robots.iter().find(|r| r.state.id == robot_state.partner_id).unwrap();
        if partner.state.phase != RobotPhase::InteriorSweep {
            println!("Robot {} waiting for partner to enter interior sweep phase", robot_state.id);
            return PhaseTransition::Continue;
        }
        
        // For MVP: We've successfully mapped the exterior boundary, 
        // so we consider the exploration mission complete
        println!("Robot {} completed boundary mapping. Interior exploration complete!", robot_state.id);
        println!("Robot {} mission accomplished - transitioning to Idle", robot_state.id);
        
        PhaseTransition::Transition(RobotPhase::Idle)
    }

    fn phase_type(&self) -> RobotPhase {
        RobotPhase::InteriorSweep
    }
}