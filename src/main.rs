use multiagent_explore::simulation_manager::SimulationManager;
use multiagent_explore::robot_node::RobotNode;
use multiagent_explore::types::{RobotPhase, CellState};
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
    let mut phase2_completed = false;
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
        let mut all_robots_completed = true;
        for robot in &mut sim.robots {
            let completed = robot.tick(&robots_snapshot, &sim.map);
            if completed {
                println!("*** DEBUG: Robot {} returned completion status at tick {}", robot.state.id, tick);
            }
            if !completed {
                all_robots_completed = false;
            }
        }
        
        if all_robots_completed {
            println!("*** DEBUG: All robots completed at tick {}", tick);
        }

        // Loop closure detection is now handled within individual robot logic
        // No need for global loop closure detection here

        // Check for simulation completion conditions
        let all_boundary_analyzed = sim.robots.iter().all(|r|
            r.state.phase == RobotPhase::BoundaryAnalysis
        );
        let all_in_advanced_phases = sim.robots.iter().all(|r|
            matches!(r.state.phase, RobotPhase::IslandEscape | RobotPhase::InteriorSweep)
        );
        let both_robots_idle = sim.robots.iter().all(|r|
            r.state.phase == RobotPhase::Idle
        );
        
        if both_robots_idle {
            println!("*** DEBUG: Both robots are in Idle phase at tick {}", tick);
        }
        
        // Check current robot phases every tick after 150
        if tick >= 150 {
            for robot in &sim.robots {
                println!("*** DEBUG: Tick {}: Robot {} in phase {:?}", tick, robot.state.id, robot.state.phase);
            }
        }
        
        if both_robots_idle {
            println!("Exploration complete - both robots in Idle phase! Final maps:");
            sim.print_all_maps();
            break;
        } else if all_robots_completed {
            // All robots have completed their exploration tasks
            println!("All robots completed exploration! Final maps:");
            sim.print_all_maps();
            break;
        } else if all_boundary_analyzed {
            if phase2_completed {
                println!("Phase 2 (Boundary Scouting) completed and loop analyzed.");
                break;
            } else {
                phase2_completed = true;
            }
        } else if all_in_advanced_phases {
            // Continue running - let the advanced phases execute
            // Check if any robots have completed (transitioned to Idle or other completion state)
            let any_completed = sim.robots.iter().any(|r| 
                matches!(r.state.phase, RobotPhase::Idle)
            );
            if any_completed {
                println!("Interior sweep completed successfully!");
                break;
            }
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