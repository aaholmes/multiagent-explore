use multiagent_explore::simulation_manager::SimulationManager;
use std::env;

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

    // Run the simulation for a fixed number of ticks (e.g., 10)
    for tick in 0..10 {
        println!("=== Tick {} ===", tick);
        for (i, robot) in sim.robots.iter().enumerate() {
            println!("Robot {}: pos=({}, {}), phase={:?}",
                i,
                robot.state.pose.position.x,
                robot.state.pose.position.y,
                robot.state.phase
            );
        }
        sim.print_all_maps();
        sim.tick();
    }
    println!("Simulation complete.");
} 