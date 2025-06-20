/// Modular robot node implementation

pub mod phase_trait;
pub mod wall_following;
pub mod boundary_analysis;
pub mod phases;

use crate::types::*;
use crate::constants::*;
use phase_trait::*;
use phases::*;

/// Main robot logic node, encapsulating state and behavior.
#[derive(Debug, Clone)]
pub struct RobotNode {
    pub state: RobotState,
    // Phase implementations
    wall_find: WallFindPhase,
    boundary_scouting: BoundaryScoutingPhase,
    boundary_analysis: BoundaryAnalysisPhase,
    island_escape: IslandEscapePhase,
    interior_sweep: InteriorSweepPhase,
    central_scan: CentralScanPhase,
}

impl RobotNode {
    /// Create a new robot node
    pub fn new(state: RobotState) -> Self {
        Self {
            state,
            wall_find: WallFindPhase,
            boundary_scouting: BoundaryScoutingPhase,
            boundary_analysis: BoundaryAnalysisPhase,
            island_escape: IslandEscapePhase,
            interior_sweep: InteriorSweepPhase,
            central_scan: CentralScanPhase,
        }
    }

    /// The main decision-making loop, called on each simulation tick.
    /// Only performs movement; sensing and communication are handled externally.
    pub fn tick(&mut self, all_robots: &[RobotNode], global_map: &GridMap) {
        let context = PhaseContext {
            all_robots,
            global_map,
        };

        let transition = match self.state.phase {
            RobotPhase::InitialWallFind => self.wall_find.execute(&mut self.state, &context),
            RobotPhase::BoundaryScouting => self.boundary_scouting.execute(&mut self.state, &context),
            RobotPhase::BoundaryAnalysis => self.boundary_analysis.execute(&mut self.state, &context),
            RobotPhase::IslandEscape => self.island_escape.execute(&mut self.state, &context),
            RobotPhase::InteriorSweep => self.interior_sweep.execute(&mut self.state, &context),
            RobotPhase::CentralScan => self.central_scan.execute(&mut self.state, &context),
            _ => PhaseTransition::Continue,
        };

        // Handle phase transitions
        match transition {
            PhaseTransition::Transition(new_phase) => {
                println!("Robot {} transitioning from {:?} to {:?}", self.state.id, self.state.phase, new_phase);
                self.state.phase = new_phase;
            },
            PhaseTransition::Complete => {
                println!("Robot {} completed all phases", self.state.id);
            },
            PhaseTransition::Continue => {
                // Stay in current phase
            }
        }
    }

    /// Update the robot's local map with its current cell and four neighbors from the global map.
    pub fn update_local_map(&mut self, global_map: &GridMap) {
        let width = self.state.map.width as i32;
        let height = self.state.map.height as i32;
        let pos = self.state.pose.position;
        let mut to_update = vec![pos];
        let dirs = [NORTH, SOUTH, EAST, WEST];
        for (dx, dy) in &dirs {
            let neighbor = Point { x: pos.x + dx, y: pos.y + dy };
            if neighbor.x >= 0 && neighbor.x < width && neighbor.y >= 0 && neighbor.y < height {
                to_update.push(neighbor);
            }
        }

        for point in to_update {
            let global_idx = (point.y as usize) * global_map.width + (point.x as usize);
            let local_idx = (point.y as usize) * self.state.map.width + (point.x as usize);
            if global_idx < global_map.cells.len() && local_idx < self.state.map.cells.len() {
                self.state.map.cells[local_idx] = global_map.cells[global_idx];
            }
        }
    }

    /// Returns true if two positions are within communication range
    pub fn within_comm_range(a: &Point, b: &Point) -> bool {
        (a.x - b.x).abs() + (a.y - b.y).abs() <= COMMUNICATION_RANGE
    }

    /// Print the robot's current map for inspection.
    pub fn print_map(&self) {
        println!("Robot {}'s map:", self.state.id);
        for y in 0..self.state.map.height {
            for x in 0..self.state.map.width {
                let idx = y * self.state.map.width + x;
                let ch = match self.state.map.cells[idx] {
                    CellState::Obstacle => '#',
                    CellState::Empty => '.',
                    CellState::Unexplored => ' ',
                };
                if self.state.pose.position.x == x as i32 && self.state.pose.position.y == y as i32 {
                    print!("R");
                } else {
                    print!("{}", ch);
                }
            }
            println!("");
        }
    }

    /// Merge another robot's map into this robot's map.
    pub fn merge_map(&mut self, partner_map: &GridMap) {
        for (i, &cell) in partner_map.cells.iter().enumerate() {
            if cell != CellState::Unexplored {
                self.state.map.cells[i] = cell;
            }
        }
    }
}