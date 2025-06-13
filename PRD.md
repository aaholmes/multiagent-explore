# Design Document: Multi-Robot Iterative Exploration Algorithm

1. Project Overview & Goal

This project is an implementation of a coordinated multi-robot exploration algorithm for mapping unknown 2D grid environments. Two robots collaborate to systematically map the area using an "Iterative Boundary Trace and Coordinated Sweep" strategy. The final product will be a simulation demonstrating the algorithm's effectiveness, suitable for a public portfolio.

2. Core Components & Architecture (ROS2-inspired)

The system will consist of three main components within a simulation environment:

Simulation Manager: A central node responsible for setting up the environment (loading the map/obstacles), spawning the robots, managing the simulation clock, and detecting global termination conditions.
Robot Node (x2): Each robot runs an instance of this node. It contains all the logic for sensing, decision-making, planning, and acting according to the algorithm's phases.
Communication Channel: A mechanism for robots to exchange messages, specifically their map data and current state. In ROS2, this would be handled via Topics or Services.
3. Core Data Structures & Enums

Rust

// --- Enums ---

// Represents the state of a cell in the grid map
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum CellState {
    UNEXPLORED,
    EMPTY,
    OBSTACLE,
}

// Represents the current operational phase of a robot
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum RobotPhase {
    IDLE,
    INITIAL_WALL_FIND,
    BOUNDARY_SCOUTING, // The iterative part with increasing 'n'
    BOUNDARY_ANALYSIS, // After closing a loop, deciding if it's an island
    ISLAND_ESCAPE,     // Moving past a detected island
    CENTRAL_SCAN,      // Proactive scan for central islands
    INTERIOR_SWEEP,    // Coordinated inward sweep of a confirmed area
}

// --- Structs ---

// Simple 2D integer coordinates for the grid
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

// Robot's position and orientation (orientation as a simple 2D vector)
#[derive(Copy, Clone, Debug)]
pub struct Pose {
    pub position: Point,
    pub orientation_rad: f64, // Angle in radians
}

// The shared map representation
pub struct GridMap {
    pub width: usize,
    pub height: usize,
    pub cells: Vec<CellState>,
    // Potentially a costmap for pathfinding
}

// State information for a single robot
pub struct RobotState {
    pub id: u8,
    pub pose: Pose,
    pub phase: RobotPhase,
    pub map: GridMap,
    // --- Algorithm-specific state ---
    pub scout_depth_n: u32,
    pub partner_id: u8,
    pub last_known_partner_pose: Option<Pose>,
    pub loop_analysis_data: Option<LoopAnalysisData>,
    pub travel_direction_before_island: Option<f64>,
}

// Data collected during a boundary trace to analyze a closed loop
pub struct LoopAnalysisData {
    pub path_traced: Vec<Point>,
    pub total_angular_displacement: f64, // Used to distinguish island from outer wall
}
4. Classes & High-Level API

This API is designed to be modular, aligning with your goal of creating a reusable toolbox.

Rust

// --- Main Robot Logic ---
pub struct RobotNode {
    pub state: RobotState,
    // other fields like communication handles
}

impl RobotNode {
    /// The main decision-making loop, called on each simulation tick.
    /// It acts as a state machine based on the current RobotPhase.
    pub fn tick(&mut self) {
        // match self.state.phase { ... }
    }

    /// PHASE 1: Move in a straight line until a wall is hit.
    fn execute_initial_wall_find(&mut self);

    /// PHASE 2: Perform one leg of the iterative boundary scouting.
    /// Moves 'n' steps, performs the adjacent scan, and attempts to reconvene.
    fn execute_boundary_scouting_leg(&mut self);

    /// PHASE 3: Analyze a completed loop to determine if it's an island.
    /// Updates phase to ISLAND_ESCAPE or CENTRAL_SCAN accordingly.
    fn analyze_completed_loop(&mut self);

    /// PHASE 4: Perform a coordinated scan across the middle of a known area.
    fn execute_central_island_scan(&mut self);

    /// PHASE 5: Perform one leg of the coordinated inward sweep.
    fn execute_interior_sweep_leg(&mut self);

    /// Merges a map received from a partner robot into this robot's own map.
    fn merge_map(&mut self, partner_map: &GridMap);
}

// --- Map Management Module ---
pub mod map_manager {
    /// Determines if a completed loop is an island or an outer boundary.
    /// Rule: An island has OBSTACLE cells inside and EMPTY/UNEXPLORED outside.
    /// The opposite is true for an outer wall boundary.
    pub fn is_loop_an_island(map: &GridMap, loop_path: &[Point]) -> bool;

    /// Finds all frontier cells (EMPTY cells adjacent to UNEXPLORED cells).
    pub fn find_frontier_cells(map: &GridMap) -> Vec<Point>;
}

// --- Path Planning Module ---
pub mod path_planner {
    /// Implements A* search algorithm to find a path from start to goal.
    pub fn find_path(start: Point, goal: Point, map: &GridMap) -> Option<Vec<Point>>;
}
5. Algorithm Logic Flow

This section details the step-by-step logic for the AI assistant.

Initialization:

SimulationManager creates the environment grid from a predefined map file.
It spawns two RobotNode instances at their starting positions.
Initial RobotPhase for both is INITIAL_WALL_FIND.
Initial scout_depth_n is set to a small value (e.g., 3).

Phase 1: Initial Wall Find

Both robots move together in a pre-determined direction (e.g., +X).
Upon one robot detecting an OBSTACLE, it stops and communicates this to its partner. They both transition to BOUNDARY_SCOUTING.

Phase 2: Iterative Boundary Scouting

The robots agree on "opposite" directions (e.g., Robot 1 keeps the wall to its left, Robot 2 to its right).
In each iteration:
Each robot plans a path n steps along the wall.
They execute the path, updating their maps with local sensor data. During this, they record their path and angular turns for loop analysis.
If they detect each other within sensor range during this path execution, a loop is complete. They transition to BOUNDARY_ANALYSIS.
If the path is completed without meeting, each robot plans a "return scan" path: n steps along the adjacent row of cells, back towards the start of their scouting leg.
They reconvene, share maps, and n is doubled for the next iteration.

Phase 3: Loop Analysis & Island Handling

Once a loop is closed, the robots merge their maps and analyze the completed loop path.
They use map_manager::is_loop_an_island() and/or the total angular displacement to classify the loop.
If Island:
Set phase to ISLAND_ESCAPE.
Robots identify their original travel direction before starting the loop trace.
They plan a path in that direction away from the island and revert to INITIAL_WALL_FIND or BOUNDARY_SCOUTING upon hitting the next wall.
If Outer Wall:
The boundary is considered mapped. They transition to CENTRAL_SCAN.

Phase 4: Central Island Scan

The robots identify a central line/path through the newly mapped area.
They move together along this path to the other side, mapping any large central obstacles they find.
Upon completion, they transition to INTERIOR_SWEEP.

Phase 5: Coordinated Interior Sweeps

The robots identify the current complete frontier between the explored interior and unexplored interior.
They divide the frontier (e.g., each takes the half closest to it).
Each robot follows its frontier segment in an "inward sweeping" motion.
They continue until their paths bring them into sensor range of each other, at which point they meet, share maps, and re-plan the next sweep on the new, smaller frontier. This repeats until no significant frontier remains.
