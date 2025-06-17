use multiagent_explore::simulation_manager::SimulationManager;
use multiagent_explore::robot_node::RobotNode;
use multiagent_explore::types::RobotPhase;
use std::env;

mod visualize;

/// Main entry point for the multi-robot exploration simulation.
fn main() {
    // Parse map file path and optional seed from command line arguments
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <map_file> [seed]", args[0]);
        std::process::exit(1);
    }
    let map_file = &args[1];
    let seed: u64 = if args.len() > 2 {
        args[2].parse().unwrap_or(42)
    } else {
        42
    };

    // Load map and initialize simulation
    let mut sim = match SimulationManager::from_map_file(map_file, seed) {
        Ok(sim) => sim,
        Err(e) => {
            eprintln!("Failed to load map: {}", e);
            std::process::exit(1);
        }
    };

    // Store history for visualization
    let mut history = Vec::new();
    let mut tick = 0;
    loop {
        println!("=== Tick {} ===", tick);
        for (i, robot) in sim.robots.iter().enumerate() {
            println!("Robot {}: pos=({}, {}), phase={:?}",
                i,
                robot.state.pose.position.x,
                robot.state.pose.position.y,
                robot.state.phase
            );
        }
        // 1. See surroundings
        for robot in &mut sim.robots {
            robot.update_local_map(&sim.map);
        }
        // 2. Exchange maps if possible
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            for other in &robots_snapshot {
                if other.state.id != robot.state.id && RobotNode::within_comm_range(&robot.state.pose.position, &other.state.pose.position) {
                    robot.merge_map(&other.state.map);
                }
            }
        }
        // 3. Display maps
        sim.print_all_maps();
        // Save state for visualization
        history.push(sim.robots.clone());
        // 4. Move
        let robots_snapshot = sim.robots.clone();
        for robot in &mut sim.robots {
            robot.tick(&robots_snapshot, &sim.map);
        }

        // Loop closure detection is now handled within individual robot logic
        // No need for global loop closure detection here

        // Check for Phase 2 completion: both robots have transitioned to BoundaryAnalysis
        let all_boundary_analyzed = sim.robots.iter().all(|r|
            r.state.phase == RobotPhase::BoundaryAnalysis
        );
        if all_boundary_analyzed {
            println!("Phase 2 (Boundary Scouting) completed and loop analyzed.");
            break;
        }
        tick += 1;
        if tick > 500 { // safety limit
            println!("Simulation stopped after 500 ticks.");
            break;
        }
    }
    println!("Simulation complete. Launching visualization...");
    let map_width = sim.map.width;
    let map_height = sim.map.height;
    visualize::visualize(&history, map_width, map_height);
} 