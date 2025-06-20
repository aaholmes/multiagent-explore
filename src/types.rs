/// Represents the state of a cell in the grid map.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CellState {
    Unexplored,
    Empty,
    Obstacle,
}

/// Represents the current operational phase of a robot.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum RobotPhase {
    Idle,
    InitialWallFind,
    BoundaryScouting,
    BoundaryAnalysis,
    IslandEscape,
    CentralScan,
    InteriorSweep,
}

/// Result of boundary analysis
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum BoundaryAnalysisResult {
    Incomplete,  // Boundary trace is not complete
    Island,      // Closed loop that doesn't touch map boundaries (obstacle)
    ExteriorWall, // Closed loop that touches map boundaries (room perimeter)
}

/// Simple 2D integer coordinates for the grid.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

/// Robot's position and orientation (orientation as a simple 2D vector).
#[derive(Copy, Clone, Debug)]
pub struct Pose {
    pub position: Point,
    pub orientation_rad: f64, // Angle in radians
}

/// The shared map representation.
#[derive(Clone, Debug)]
pub struct GridMap {
    pub width: usize,
    pub height: usize,
    pub cells: Vec<CellState>,
    // Optionally, add costmap or other fields as needed
}

/// State for boundary scouting phase
#[derive(Clone, Debug)]
pub struct BoundaryScoutState {
    pub tracing_direction: i8, // -1 for left, +1 for right
    pub steps_taken: u32, // Total steps taken in the current phase (BoundaryScouting)
    pub steps_taken_this_scouting_mission: u32, // Steps taken in the current scouting leg
    pub returning: bool,
    pub path: Vec<Point>,
    pub first_move: bool,
    pub initial_scouting_direction: Option<Point>, // Store the first direction taken when hitting wall
    pub total_rotation_steps: i32, // Total rotation in 90-degree steps (positive = counterclockwise)
}

/// State for central scan phase
#[derive(Clone, Debug)]
pub struct CentralScanState {
    pub virtual_boundary: Vec<Point>,     // Previous loop becomes new "wall"
    pub scan_iteration: u32,              // How many layers deep we've gone
    pub completed_loops: Vec<Vec<Point>>, // All completed boundary loops
}

/// State information for a single robot.
#[derive(Clone, Debug)]
pub struct RobotState {
    pub id: u8,
    pub pose: Pose,
    pub phase: RobotPhase,
    pub map: GridMap,
    pub scout_depth_n: u32,
    pub partner_id: u8,
    pub last_known_partner_pose: Option<Pose>,
    pub loop_analysis_data: Option<LoopAnalysisData>,
    pub travel_direction_before_island: Option<f64>,
    pub boundary_scout: Option<BoundaryScoutState>,
    pub central_scan: Option<CentralScanState>,
}

/// Data collected during a boundary trace to analyze a closed loop.
#[derive(Clone, Debug)]
pub struct LoopAnalysisData {
    pub path_traced: Vec<Point>,
    pub total_angular_displacement: f64,
    // Interior sweep fields
    pub loop_start_position: Option<Point>,
    pub loop_closed: Option<bool>,
    pub total_loop_length: Option<u32>,
    pub midpoint_direction: Option<Point>,
    pub target_position: Option<Point>,
} 