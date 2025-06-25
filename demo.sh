#!/bin/bash

# Multi-Robot Exploration Demo Script
# Shows the key features of the coordinated exploration algorithm

set -e

echo "🤖 Multi-Robot Exploration Algorithm Demo"
echo "=========================================="
echo ""

# Build the project
echo "🔨 Building project..."
cargo build --release
echo ""

echo "🎯 Demo 1: Simple Corridor (Fast completion)"
echo "Shows efficient boundary tracing and rapid completion"
echo "Press Enter to start..."
read -r
./run_simulation.sh --map_file maps/simple_corridor.map --seed 42
echo ""

echo "🏝️ Demo 2: Island Detection (Advanced coordination)"  
echo "Demonstrates island vs exterior wall detection"
echo "Press Enter to start..."
read -r
./run_simulation.sh --map_file maps/island_room.map --seed 42
echo ""

echo "🏠 Demo 3: Complex Room (Full algorithm)"
echo "Complete 4-phase exploration with coordinated sweeping"
echo "Press Enter to start..."
read -r  
./run_simulation.sh --map_file maps/sample_room.map --seed 42
echo ""

echo "✨ Demo Complete!"
echo "Each simulation launched an interactive visualization window"
echo "showing real-time robot movement and map discovery."
echo ""
echo "For more scenarios, see: ./run_simulation.sh --help"