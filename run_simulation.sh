#!/bin/bash

# Multi-Robot Exploration Simulation Runner
# Usage: ./run_simulation.sh [--map_file <path>] [--seed <number>] [--help]

set -e

# Default values
MAP_FILE="maps/sample_room.map"
SEED=42

# Help message
show_help() {
    echo "Multi-Robot Exploration Simulation"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --map_file <path>   Path to the map file (default: maps/sample_room.map)"
    echo "  --seed <number>     Random seed for reproducible runs (default: 42)"
    echo "  --help              Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                                    # Run with default settings"
    echo "  $0 --map_file maps/complex_office.map"
    echo "  $0 --seed 123"
    echo "  $0 --map_file maps/maze.map --seed 456"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --map_file)
            MAP_FILE="$2"
            shift 2
            ;;
        --seed)
            SEED="$2"
            shift 2
            ;;
        --help)
            show_help
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if map file exists
if [[ ! -f "$MAP_FILE" ]]; then
    echo "Error: Map file '$MAP_FILE' not found!"
    echo "Available maps:"
    ls -1 maps/ 2>/dev/null || echo "  No maps directory found"
    exit 1
fi

# Build the project
echo "Building multi-robot exploration simulation..."
cargo build --release

# Run the simulation
echo "Running simulation with map: $MAP_FILE, seed: $SEED"
echo "Starting exploration..."
echo ""

./target/release/multiagent_explore "$MAP_FILE" "$SEED"