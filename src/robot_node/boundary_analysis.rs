/// Boundary analysis logic for distinguishing obstacles from exterior walls

use crate::types::*;
use crate::constants::*;

/// Boundary analysis utilities
pub struct BoundaryAnalyzer;

impl BoundaryAnalyzer {
    /// Analyze boundary type using rotation difference between robots
    pub fn analyze_boundary_by_rotation(
        robot0_rotation: Option<i32>,
        robot1_rotation: Option<i32>,
    ) -> BoundaryAnalysisResult {
        match (robot0_rotation, robot1_rotation) {
            (Some(rot0), Some(rot1)) => {
                let rotation_difference = rot0 - rot1;
                println!("Boundary analysis: Robot0={} steps, Robot1={} steps, Difference={} steps", 
                         rot0, rot1, rotation_difference);
                
                // Check if difference is ±4 steps (4 * 90° = ±360°)
                if rotation_difference == -EXPECTED_ROTATION_DIFFERENCE {
                    // -4 steps indicates exterior wall
                    BoundaryAnalysisResult::ExteriorWall
                } else if rotation_difference == EXPECTED_ROTATION_DIFFERENCE {
                    // +4 steps indicates interior island
                    BoundaryAnalysisResult::Island
                } else {
                    println!("Rotation difference {} steps doesn't match expected ±{} pattern", 
                             rotation_difference, EXPECTED_ROTATION_DIFFERENCE);
                    BoundaryAnalysisResult::Incomplete
                }
            },
            _ => {
                println!("Missing rotation data for analysis");
                BoundaryAnalysisResult::Incomplete
            }
        }
    }

    /// Analyze if a boundary path forms a closed loop
    pub fn is_boundary_closed_loop(path: &[Point]) -> bool {
        if path.len() < 3 {
            return false;
        }
        
        // Check if the path returns to the starting point
        path.first() == path.last()
    }

    /// Determine if a closed boundary is an island (obstacle) rather than exterior wall
    /// Returns true if it's an island, false if it's an exterior wall
    pub fn is_island_not_exterior(path: &[Point], map_width: usize, map_height: usize) -> bool {
        // If any point in the path touches the map boundaries, it's an exterior wall
        for point in path {
            if point.x == 0 || point.x == (map_width as i32 - 1) ||
               point.y == 0 || point.y == (map_height as i32 - 1) {
                return false; // Touches boundary -> exterior wall
            }
        }
        
        true // No boundary contact -> island/obstacle
    }

    /// Legacy path-based analysis method (for backwards compatibility)
    pub fn analyze_boundary_by_path(path: &[Point], map_width: usize, map_height: usize) -> BoundaryAnalysisResult {
        // For iterative boundary scouting, check if any point touches map boundaries
        if Self::is_island_not_exterior(path, map_width, map_height) {
            // No boundary contact in the path -> likely an island
            BoundaryAnalysisResult::Island
        } else {
            // Path touches boundary -> likely exterior wall
            BoundaryAnalysisResult::ExteriorWall
        }
    }
}