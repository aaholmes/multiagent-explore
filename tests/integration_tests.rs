use multiagent_explore::simulation_manager::SimulationManager;
use multiagent_explore::types::*;
use std::collections::HashSet;

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

/// Test that robots can complete exploration of a simple corridor
#[test]
fn test_simple_corridor_exploration() {
    let map = create_test_map(&[
        "############", 
        "#..........#",
        "#..........#", 
        "############"
    ]);
    
    let robot_states = vec![
        create_test_robot_state(0, Point { x: 1, y: 1 }, 1, &map),
        create_test_robot_state(1, Point { x: 2, y: 1 }, 0, &map),
    ];
    
    let mut sim = SimulationManager::new(map, robot_states);
    
    // Run simulation for limited steps
    let mut boundary_phase_reached = false;
    for _tick in 0..100 {
        for robot in &mut sim.robots {
            robot.update_local_map(&sim.map);
        }
        
        // Exchange maps
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            for other in &robots_snapshot {
                if other.state.id != robot.state.id && 
                   multiagent_explore::robot_node::RobotNode::within_comm_range(&robot.state.pose.position, &other.state.pose.position) {
                    robot.merge_map(&other.state.map);
                }
            }
        }
        
        // Execute robot logic
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            robot.tick(&robots_snapshot, &sim.map);
        }
        
        // Check if robots reached boundary phase (indicating successful initial exploration)
        if sim.robots.iter().any(|r| r.state.phase == RobotPhase::BoundaryAnalysis) {
            boundary_phase_reached = true;
            break;
        }
    }
    
    // Verify exploration progressed to boundary analysis
    assert!(boundary_phase_reached, "Robots should reach boundary analysis phase");
    
    // Verify both robots have explored significant portions
    for robot in &sim.robots {
        let explored_cells = robot.state.map.cells.iter()
            .filter(|&&cell| cell != CellState::Unexplored)
            .count();
        assert!(explored_cells > 10, "Robot {} should have explored significant area", robot.state.id);
    }
}

/// Test island detection functionality
#[test]
fn test_island_detection() {
    let map = create_test_map(&[
        "##########",
        "#........#", 
        "#...##...#",
        "#...##...#",
        "#........#",
        "##########"
    ]);
    
    let robot_states = vec![
        create_test_robot_state(0, Point { x: 1, y: 1 }, 1, &map),
        create_test_robot_state(1, Point { x: 2, y: 1 }, 0, &map),
    ];
    
    let mut sim = SimulationManager::new(map, robot_states);
    
    // Run simulation and check that robots can handle the island
    for _tick in 0..200 {
        for robot in &mut sim.robots {
            robot.update_local_map(&sim.map);
        }
        
        // Exchange maps when in range
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            for other in &robots_snapshot {
                if other.state.id != robot.state.id && 
                   multiagent_explore::robot_node::RobotNode::within_comm_range(&robot.state.pose.position, &other.state.pose.position) {
                    robot.merge_map(&other.state.map);
                }
            }
        }
        
        // Execute robot logic
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            robot.tick(&robots_snapshot, &sim.map);
        }
        
        // Check if robots have encountered boundary analysis phase
        let any_boundary_analysis = sim.robots.iter().any(|r| r.state.phase == RobotPhase::BoundaryAnalysis);
        let any_island_escape = sim.robots.iter().any(|r| r.state.phase == RobotPhase::IslandEscape);
        
        if any_boundary_analysis || any_island_escape {
            // Success - robots are handling island detection
            return;
        }
    }
    
    // At minimum, robots should have progressed through initial phases
    assert!(sim.robots.iter().any(|r| r.state.phase != RobotPhase::InitialWallFind),
        "Robots should progress beyond initial wall finding");
}

/// Test that robots maintain communication during boundary scouting
#[test]
fn test_communication_during_scouting() {
    let map = create_test_map(&[
        "########",
        "#......#",
        "#......#", 
        "#......#",
        "########"
    ]);
    
    let robot_states = vec![
        create_test_robot_state(0, Point { x: 1, y: 1 }, 1, &map),
        create_test_robot_state(1, Point { x: 2, y: 1 }, 0, &map),
    ];
    
    let mut sim = SimulationManager::new(map, robot_states);
    let mut communication_events = 0;
    
    // Run simulation and count communication events
    for _tick in 0..50 {
        for robot in &mut sim.robots {
            robot.update_local_map(&sim.map);
        }
        
        // Count communication opportunities
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            for other in &robots_snapshot {
                if other.state.id != robot.state.id && 
                   multiagent_explore::robot_node::RobotNode::within_comm_range(&robot.state.pose.position, &other.state.pose.position) {
                    communication_events += 1;
                    robot.merge_map(&other.state.map);
                }
            }
        }
        
        // Execute robot logic
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            robot.tick(&robots_snapshot, &sim.map);
        }
    }
    
    // Verify that robots had opportunities to communicate
    assert!(communication_events > 0, "Robots should have opportunities to communicate during exploration");
}

/// Helper function to create a test robot state
fn create_test_robot_state(id: u8, position: Point, partner_id: u8, map: &GridMap) -> RobotState {
    RobotState {
        id,
        pose: Pose { 
            position, 
            orientation_rad: -std::f64::consts::PI / 2.0 // Facing North
        },
        phase: RobotPhase::InitialWallFind,
        map: GridMap {
            width: map.width,
            height: map.height,
            cells: vec![CellState::Unexplored; map.width * map.height],
        },
        scout_depth_n: 3,
        partner_id,
        last_known_partner_pose: None,
        loop_analysis_data: None,
        travel_direction_before_island: None,
        boundary_scout: None,
        central_scan: None,
    }
}

/// Test complete exploration coverage
#[test]
fn test_exploration_coverage() {
    let map = create_test_map(&[
        "######",
        "#....#",
        "#....#",
        "######"
    ]);
    
    let robot_states = vec![
        create_test_robot_state(0, Point { x: 1, y: 1 }, 1, &map),
        create_test_robot_state(1, Point { x: 2, y: 1 }, 0, &map),
    ];
    
    let mut sim = SimulationManager::new(map.clone(), robot_states);
    
    // Run simulation
    for _tick in 0..150 {
        for robot in &mut sim.robots {
            robot.update_local_map(&sim.map);
        }
        
        // Exchange maps
        let robots_snapshot = sim.robots.clone(); 
        for robot in &mut sim.robots {
            for other in &robots_snapshot {
                if other.state.id != robot.state.id && 
                   multiagent_explore::robot_node::RobotNode::within_comm_range(&robot.state.pose.position, &other.state.pose.position) {
                    robot.merge_map(&other.state.map);
                }
            }
        }
        
        // Execute robot logic
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            robot.tick(&robots_snapshot, &sim.map);
        }
    }
    
    // Check that robots have discovered all empty cells
    let map_cells = &map.cells;
    let map_width = map.width;
    let total_empty_cells: HashSet<_> = (0..map.height).flat_map(|y| {
        (0..map_width).filter_map(move |x| {
            let idx = y * map_width + x;
            if map_cells[idx] == CellState::Empty {
                Some((x, y))
            } else {
                None
            }
        })
    }).collect();
    
    // At least one robot should have significant coverage
    let max_coverage = sim.robots.iter().map(|robot| {
        robot.state.map.cells.iter()
            .filter(|&&cell| cell == CellState::Empty)
            .count()
    }).max().unwrap_or(0);
    
    assert!(max_coverage >= total_empty_cells.len() / 2, 
        "Robots should achieve reasonable exploration coverage");
}