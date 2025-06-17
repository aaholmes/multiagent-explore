use multiagent_explore::robot_node::RobotNode;
use multiagent_explore::robot_node::wall_following::WallFollower;
use multiagent_explore::types::*;
use multiagent_explore::constants::*;
use std::f64::consts::PI;

/// Helper function to create a test map from ASCII representation
fn create_test_map(ascii_grid: &[&str]) -> GridMap {
    let height = ascii_grid.len();
    let width = ascii_grid[0].len();
    let mut map_cells = vec![CellState::Empty; width * height];
    
    for (y, row) in ascii_grid.iter().enumerate() {
        for (x, ch) in row.chars().enumerate() {
            let idx = y * width + x;
            map_cells[idx] = match ch {
                '#' => CellState::Obstacle,
                '.' => CellState::Empty,
                _ => CellState::Unexplored,
            };
        }
    }
    GridMap { width, height, cells: map_cells }
}

#[test]
fn test_robot0_wall_follow_left_hand_simple_wall() {
    // Map: 
    // # # #
    // # . .
    // # # #
    let map = create_test_map(&[
        "###",
        "#..",
        "###",
    ]);

    // Robot 0 (left-hand rule), starts at (1,1) facing North (-Y)
    let robot0_state = RobotState {
        id: 0,
        pose: Pose { position: Point { x: 1, y: 1 }, orientation_rad: -PI / 2.0 }, // Facing North
        phase: RobotPhase::BoundaryScouting,
        map: map.clone(), // Robot's initial knowledge of the map
        scout_depth_n: 10,
        partner_id: 1,
        last_known_partner_pose: None,
        loop_analysis_data: None,
        travel_direction_before_island: None,
        boundary_scout: Some(BoundaryScoutState {
            tracing_direction: LEFT_HAND_RULE,
            steps_taken: 0,
            steps_taken_this_scouting_mission: 0,
            returning: false,
            path: vec![Point { x: 1, y: 1 }],
            first_move: true,
            initial_scouting_direction: None,
            total_rotation_steps: 0,
        }),
    };
    let mut robot0 = RobotNode::new(robot0_state);

    // Step 1: Robot 0 (left-hand rule), at (1,1) facing North. Wall to the left (West).
    // Prioritize: Right (East), Forward (North), Left (West), Back (South).
    // From (1,1) facing North: 
    // Right (East): (2,1) - Empty. Should move to (2,1), facing East.
    let next_pos = WallFollower::wall_follow_step(
        robot0.state.pose.position,
        robot0.state.pose.orientation_rad,
        &map,
        LEFT_HAND_RULE
    ).unwrap();
    assert_eq!(next_pos, Point { x: 2, y: 1 });
    let prev_pos = robot0.state.pose.position;
    robot0.state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
    robot0.state.pose.position = next_pos;
    assert_eq!(robot0.state.pose.orientation_rad.to_degrees().round() as i32, 0); // Facing East

    // Step 2: Robot 0 at (2,1) facing East. Wall to the left (North).
    // Prioritize: Right (South), Forward (East), Left (North), Back (West).
    // From (2,1) facing East:
    // Right (South): (2,2) - Obstacle (#)
    // Forward (East): (3,1) - Out of bounds
    // Left (North): (2,0) - Obstacle (#)
    // Back (West): (1,1) - Empty (.). Should move to (1,1), facing West.
    let next_pos = WallFollower::wall_follow_step(
        robot0.state.pose.position,
        robot0.state.pose.orientation_rad,
        &map,
        LEFT_HAND_RULE
    ).unwrap();
    assert_eq!(next_pos, Point { x: 1, y: 1 });
    let prev_pos = robot0.state.pose.position;
    robot0.state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
    robot0.state.pose.position = next_pos;
    assert_eq!(robot0.state.pose.orientation_rad.to_degrees().round() as i32, 180); // Facing West
}

#[test]
fn test_robot1_wall_follow_right_hand_simple_wall() {
    // Map: 
    // # # #
    // . . #
    // # # #
    let map = create_test_map(&[
        "###",
        "..#",
        "###",
    ]);

    // Robot 1 (right-hand rule), starts at (1,1) facing North (-Y)
    let robot1_state = RobotState {
        id: 1,
        pose: Pose { position: Point { x: 1, y: 1 }, orientation_rad: -PI / 2.0 }, // Facing North
        phase: RobotPhase::BoundaryScouting,
        map: map.clone(), // Robot's initial knowledge of the map
        scout_depth_n: 10,
        partner_id: 0,
        last_known_partner_pose: None,
        loop_analysis_data: None,
        travel_direction_before_island: None,
        boundary_scout: Some(BoundaryScoutState {
            tracing_direction: RIGHT_HAND_RULE,
            steps_taken: 0,
            steps_taken_this_scouting_mission: 0,
            returning: false,
            path: vec![Point { x: 1, y: 1 }],
            first_move: true,
            initial_scouting_direction: None,
            total_rotation_steps: 0,
        }),
    };
    let mut robot1 = RobotNode::new(robot1_state);

    // Step 1: Robot 1 (right-hand rule), at (1,1) facing North. Wall to the right (East).
    // Prioritize: Left (West), Forward (North), Right (East), Back (South).
    // From (1,1) facing North:
    // Left (West): (0,1) - Empty (.). Should move to (0,1), facing West.
    let next_pos = WallFollower::wall_follow_step(
        robot1.state.pose.position,
        robot1.state.pose.orientation_rad,
        &map,
        RIGHT_HAND_RULE
    ).unwrap();
    assert_eq!(next_pos, Point { x: 0, y: 1 });
    let prev_pos = robot1.state.pose.position;
    robot1.state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
    robot1.state.pose.position = next_pos;
    assert_eq!(robot1.state.pose.orientation_rad.to_degrees().round() as i32, 180); // Facing West

    // Step 2: Robot 1 at (0,1) facing West. Wall to the right (North).
    // Prioritize: Left (South), Forward (West), Right (North), Back (East).
    // From (0,1) facing West:
    // Left (South): (0,2) - Obstacle (#)
    // Forward (West): (-1,1) - Out of bounds
    // Right (North): (0,0) - Obstacle (#)
    // Back (East): (1,1) - Empty (.). Should move to (1,1), facing East.
    let next_pos = WallFollower::wall_follow_step(
        robot1.state.pose.position,
        robot1.state.pose.orientation_rad,
        &map,
        RIGHT_HAND_RULE
    ).unwrap();
    assert_eq!(next_pos, Point { x: 1, y: 1 });
    let prev_pos = robot1.state.pose.position;
    robot1.state.pose.orientation_rad = WallFollower::update_orientation(prev_pos, next_pos);
    robot1.state.pose.position = next_pos;
    assert_eq!(robot1.state.pose.orientation_rad.to_degrees().round() as i32, 0); // Facing East
}

#[test]
fn test_rotation_based_boundary_analysis() {
    // Create two robots with boundary scout data
    let robot0_state = RobotState {
        id: ROBOT_LEFT_HAND,
        pose: Pose { position: Point { x: 5, y: 5 }, orientation_rad: 0.0 },
        phase: RobotPhase::BoundaryAnalysis,
        map: create_test_map(&["...", "...", "..."]),
        scout_depth_n: 3,
        partner_id: ROBOT_RIGHT_HAND,
        last_known_partner_pose: None,
        loop_analysis_data: None,
        travel_direction_before_island: None,
        boundary_scout: Some(BoundaryScoutState {
            tracing_direction: LEFT_HAND_RULE,
            steps_taken: 0,
            steps_taken_this_scouting_mission: 0,
            returning: false,
            path: vec![Point { x: 5, y: 5 }],
            first_move: false,
            initial_scouting_direction: None,
            total_rotation_steps: -2, // Robot 0 rotated -2 steps (clockwise)
        }),
    };

    let robot1_state = RobotState {
        id: ROBOT_RIGHT_HAND,
        pose: Pose { position: Point { x: 6, y: 5 }, orientation_rad: 0.0 },
        phase: RobotPhase::BoundaryAnalysis,
        map: create_test_map(&["...", "...", "..."]),
        scout_depth_n: 3,
        partner_id: ROBOT_LEFT_HAND,
        last_known_partner_pose: None,
        loop_analysis_data: None,
        travel_direction_before_island: None,
        boundary_scout: Some(BoundaryScoutState {
            tracing_direction: RIGHT_HAND_RULE,
            steps_taken: 0,
            steps_taken_this_scouting_mission: 0,
            returning: false,
            path: vec![Point { x: 6, y: 5 }],
            first_move: false,
            initial_scouting_direction: None,
            total_rotation_steps: 2, // Robot 1 rotated +2 steps (counter-clockwise)
        }),
    };

    // Test exterior wall case: -2 - 2 = -4 (exterior wall)
    use multiagent_explore::robot_node::boundary_analysis::BoundaryAnalyzer;
    let result = BoundaryAnalyzer::analyze_boundary_by_rotation(Some(-2), Some(2));
    assert_eq!(result, BoundaryAnalysisResult::ExteriorWall);

    // Test island case: +2 - (-2) = +4 (island)
    let result = BoundaryAnalyzer::analyze_boundary_by_rotation(Some(2), Some(-2));
    assert_eq!(result, BoundaryAnalysisResult::Island);

    // Test incomplete case: missing data
    let result = BoundaryAnalyzer::analyze_boundary_by_rotation(None, Some(2));
    assert_eq!(result, BoundaryAnalysisResult::Incomplete);
}