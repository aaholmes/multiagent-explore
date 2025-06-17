/// Constants used throughout the multiagent exploration system

use std::f64::consts::PI;

/// Communication and coordination constants
pub const COMMUNICATION_RANGE: i32 = 2; // Manhattan distance for robot communication
pub const INITIAL_SCOUT_DEPTH: u32 = 3; // Initial boundary scouting depth

/// Rotation and orientation constants
pub const ROTATION_TOLERANCE: f64 = 0.5; // Tolerance for rotation-based analysis
pub const EXPECTED_ROTATION_DIFFERENCE: i32 = 4; // Expected 90-degree step difference for boundary analysis

/// Cardinal directions in radians
pub const EAST_RAD: f64 = 0.0;
pub const SOUTH_RAD: f64 = PI / 2.0;
pub const WEST_RAD: f64 = PI;
pub const NORTH_RAD: f64 = -PI / 2.0;

/// Direction vectors for movement (dx, dy)
pub const NORTH: (i32, i32) = (0, -1);
pub const SOUTH: (i32, i32) = (0, 1);
pub const EAST: (i32, i32) = (1, 0);
pub const WEST: (i32, i32) = (-1, 0);

/// Cardinal directions for orientation mapping
pub const ORIENTATION_STEPS: [(f64, i32); 4] = [
    (EAST_RAD, 0),   // East = 0 steps
    (SOUTH_RAD, 1),  // South = 1 step
    (WEST_RAD, 2),   // West = 2 steps  
    (NORTH_RAD, 3),  // North = 3 steps
];

/// Simulation limits
pub const MAX_SIMULATION_TICKS: usize = 500;
pub const VISUALIZATION_UPDATE_INTERVAL: u64 = 100; // milliseconds

/// Robot identification
pub const ROBOT_LEFT_HAND: u8 = 0;  // Robot using left-hand wall following
pub const ROBOT_RIGHT_HAND: u8 = 1; // Robot using right-hand wall following

/// Tracing directions
pub const LEFT_HAND_RULE: i8 = -1;  // Counterclockwise tracing
pub const RIGHT_HAND_RULE: i8 = 1;  // Clockwise tracing