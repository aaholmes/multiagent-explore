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
    pub steps_taken: u32,
    pub returning: bool,
    pub path: Vec<Point>,
    pub first_move: bool,
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
}

/// Data collected during a boundary trace to analyze a closed loop.
#[derive(Clone, Debug)]
pub struct LoopAnalysisData {
    pub path_traced: Vec<Point>,
    pub total_angular_displacement: f64,
} 