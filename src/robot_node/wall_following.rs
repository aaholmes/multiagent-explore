/// Wall-following algorithms and utilities

use crate::types::*;
use crate::constants::*;
use std::f64::consts::PI;

/// Wall-following utilities for robot navigation
pub struct WallFollower;

impl WallFollower {
    /// First move after hitting wall: robots turn away from each other
    pub fn wall_follow_step_first_move(
        current_pos: Point,
        orientation: f64,
        global_map: &GridMap,
        partner_pos: Point,
    ) -> Option<Point> {
        let (current_dx, current_dy) = Self::get_direction_vector(orientation);

        // Calculate direction away from partner
        let to_partner_x = partner_pos.x - current_pos.x;
        let to_partner_y = partner_pos.y - current_pos.y;
        
        // Determine which side partner is on relative to current orientation
        // If partner is to the right, turn left; if partner is to the left, turn right
        let cross_product = current_dx * to_partner_y - current_dy * to_partner_x;
        let turn_left = cross_product > 0; // Partner is to the right, so turn left
        
        let priorities = if turn_left {
            // Turn left (away from partner on right)
            vec![
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (current_dx, current_dy),   // Forward
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (-current_dx, -current_dy), // Back
            ]
        } else {
            // Turn right (away from partner on left)
            vec![
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (current_dx, current_dy),   // Forward
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (-current_dx, -current_dy), // Back
            ]
        };

        // Try each direction in priority order
        for (dx, dy) in priorities {
            let next_pos = Point {
                x: current_pos.x + dx,
                y: current_pos.y + dy,
            };

            if Self::is_position_valid_and_empty(next_pos, global_map) {
                return Some(next_pos);
            }
        }

        None
    }

    /// Wall-following step: try to move forward, else turn (left/right) to follow wall.
    /// Returns the next position if a move is possible.
    pub fn wall_follow_step(
        current_pos: Point,
        orientation: f64,
        global_map: &GridMap,
        tracing_direction: i8,
    ) -> Option<Point> {
        // Get current direction vector
        let (current_dx, current_dy) = Self::get_direction_vector(orientation);

        // Priority order depends on tracing direction - proper wall following
        let priorities = if tracing_direction == LEFT_HAND_RULE {
            // Left-hand rule: keep wall on left, so try right first
            vec![
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (current_dx, current_dy),   // Forward
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (-current_dx, -current_dy), // Back
            ]
        } else {
            // Right-hand rule: keep wall on right, so try left first
            vec![
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (current_dx, current_dy),   // Forward
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (-current_dx, -current_dy), // Back
            ]
        };

        // Try each direction in priority order
        for (dx, dy) in priorities {
            let next_pos = Point {
                x: current_pos.x + dx,
                y: current_pos.y + dy,
            };

            if Self::is_position_valid_and_empty(next_pos, global_map) {
                return Some(next_pos);
            }
        }

        None
    }

    /// Translates orientation (radians) into a (dx, dy) movement vector
    pub fn get_direction_vector(orientation_rad: f64) -> (i32, i32) {
        // Normalize angle to be between -PI and PI
        let angle = orientation_rad.rem_euclid(2.0 * PI);
        let angle_deg = angle.to_degrees().round() as i32;

        // Using rounded degrees to avoid floating point comparison issues
        // North (-Y): -90 or 270
        // South (+Y): 90
        // East (+X): 0
        // West (-X): 180 or -180

        if angle_deg == 0 { // East
            EAST
        } else if angle_deg == 90 { // South
            SOUTH
        } else if angle_deg == 180 || angle_deg == -180 { // West
            WEST
        } else if angle_deg == 270 || angle_deg == -90 { // North
            NORTH
        } else {
            // Default to North if orientation is not one of the cardinal directions
            println!("Warning: Non-cardinal orientation: {}. Defaulting to North.", orientation_rad.to_degrees());
            NORTH
        }
    }

    /// Check if a position is valid and empty
    pub fn is_position_valid_and_empty(pos: Point, global_map: &GridMap) -> bool {
        if pos.x < 0 || pos.y < 0 || 
           pos.x >= global_map.width as i32 || pos.y >= global_map.height as i32 {
            return false;
        }
        
        let idx = (pos.y as usize) * global_map.width + (pos.x as usize);
        global_map.cells[idx] != CellState::Obstacle
    }

    /// Check if a position is part of any virtual boundary (completed loop)
    pub fn is_virtual_wall(pos: Point, virtual_boundaries: &[Vec<Point>]) -> bool {
        for boundary in virtual_boundaries {
            if boundary.contains(&pos) {
                return true;
            }
        }
        false
    }

    /// Check if a position is valid and empty, treating virtual boundaries as walls
    pub fn is_position_valid_and_empty_virtual(pos: Point, global_map: &GridMap, virtual_boundaries: &[Vec<Point>]) -> bool {
        // First check if it's a valid empty position
        if !Self::is_position_valid_and_empty(pos, global_map) {
            return false;
        }
        // Then check if it's not part of a virtual boundary
        !Self::is_virtual_wall(pos, virtual_boundaries)
    }

    /// Wall-following step with virtual boundary support
    pub fn wall_follow_step_virtual(
        current_pos: Point,
        orientation: f64,
        global_map: &GridMap,
        virtual_boundaries: &[Vec<Point>],
        tracing_direction: i8,
    ) -> Option<Point> {
        let (current_dx, current_dy) = Self::get_direction_vector(orientation);

        let priorities = if tracing_direction == LEFT_HAND_RULE {
            // Left-hand rule: try left, forward, right, back
            [
                (-current_dy, current_dx),   // Left (90° counterclockwise)
                (current_dx, current_dy),    // Forward
                (current_dy, -current_dx),   // Right (90° clockwise)
                (-current_dx, -current_dy),  // Backward
            ]
        } else {
            // Right-hand rule: try right, forward, left, back
            [
                (current_dy, -current_dx),   // Right (90° clockwise)
                (current_dx, current_dy),    // Forward
                (-current_dy, current_dx),   // Left (90° counterclockwise)
                (-current_dx, -current_dy),  // Backward
            ]
        };

        for (dx, dy) in &priorities {
            let next_pos = Point {
                x: current_pos.x + dx,
                y: current_pos.y + dy,
            };

            if Self::is_position_valid_and_empty_virtual(next_pos, global_map, virtual_boundaries) {
                return Some(next_pos);
            }
        }

        None
    }

    /// Updates robot orientation based on previous position and new position
    pub fn update_orientation(prev_pos: Point, next_pos: Point) -> f64 {
        let dx = next_pos.x - prev_pos.x;
        let dy = next_pos.y - prev_pos.y;
        
        match (dx, dy) {
            (1, 0) => EAST_RAD,    // East
            (-1, 0) => WEST_RAD,   // West
            (0, 1) => SOUTH_RAD,   // South
            (0, -1) => NORTH_RAD,  // North
            _ => {
                println!("Warning: Invalid move vector ({}, {})", dx, dy);
                NORTH_RAD // Default
            }
        }
    }
}

/// Rotation tracking utilities
pub struct RotationTracker;

impl RotationTracker {
    /// Calculate the rotation change in 90-degree steps
    pub fn calculate_rotation_steps(prev_orientation: f64, new_orientation: f64) -> i32 {
        // Convert orientations to 90-degree steps
        let prev_step = Self::orientation_to_step(prev_orientation);
        let new_step = Self::orientation_to_step(new_orientation);
        
        // Calculate the shortest rotation (handling wrap-around)
        let mut diff = new_step - prev_step;
        if diff > 2 {
            diff -= 4;
        } else if diff < -2 {
            diff += 4;
        }
        
        diff
    }
    
    /// Convert orientation (radians) to 90-degree step (0=East, 1=South, 2=West, 3=North)
    pub fn orientation_to_step(orientation: f64) -> i32 {
        // Normalize angle to [0, 2π)
        let normalized = orientation.rem_euclid(2.0 * PI);
        
        // Convert to steps: 0=East(0°), 1=South(90°), 2=West(180°), 3=North(270°)
        ((normalized + PI / 4.0) / (PI / 2.0)).floor() as i32 % 4
    }
}