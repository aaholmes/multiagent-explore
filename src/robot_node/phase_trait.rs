/// Trait-based phase system for robot behaviors

use crate::types::*;

/// Context information passed to phase execution
pub struct PhaseContext<'a> {
    pub all_robots: &'a [RobotNode],
    pub global_map: &'a GridMap,
}

/// Result of phase execution
#[derive(Debug, Clone, PartialEq)]
pub enum PhaseTransition {
    Continue,                    // Stay in current phase
    Transition(RobotPhase),     // Transition to new phase
    Complete,                   // Phase and simulation complete
}

/// Trait for robot phase behaviors
pub trait RobotPhaseBehavior {
    /// Execute the phase behavior
    fn execute(&mut self, robot_state: &mut RobotState, context: &PhaseContext) -> PhaseTransition;
    
    /// Get the phase type this behavior handles
    fn phase_type(&self) -> RobotPhase;
    
    /// Optional: Initialize phase-specific state when entering this phase
    fn on_enter(&mut self, _robot_state: &mut RobotState) {}
    
    /// Optional: Clean up phase-specific state when leaving this phase  
    fn on_exit(&mut self, _robot_state: &mut RobotState) {}
}

/// Forward declaration of RobotNode for the trait
use crate::robot_node::RobotNode;