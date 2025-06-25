#!/bin/bash

# GIF Creation Script for Multi-Robot Exploration
# Creates animated GIFs showing the algorithm in action

set -e

echo "🎬 Multi-Robot Exploration GIF Creator"
echo "====================================="
echo ""

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 is required but not installed"
    exit 1
fi

# Install matplotlib if needed
echo "📦 Checking Python dependencies..."
python3 -c "import matplotlib, matplotlib.animation" 2>/dev/null || {
    echo "📥 Installing matplotlib..."
    pip3 install matplotlib
}

echo "✅ Dependencies ready"
echo ""

# Build the Rust project
echo "🔨 Building simulation..."
cargo build --release
echo ""

# Create GIFs for different scenarios
echo "🎯 Creating GIF: Simple Corridor"
cd python_viz
python3 visualizer.py --map_file ../maps/simple_corridor.map --seed 42 --output corridor_exploration.gif
cd ..
echo ""

echo "🏝️ Creating GIF: Island Detection"
cd python_viz  
python3 visualizer.py --map_file ../maps/island_room.map --seed 42 --output island_exploration.gif
cd ..
echo ""

echo "🏠 Creating GIF: Complex Room (Full Algorithm)"
cd python_viz
python3 visualizer.py --map_file ../maps/sample_room.map --seed 42 --output complex_room_exploration.gif
cd ..
echo ""

echo "✨ GIF Creation Complete!"
echo ""
echo "Generated files:"
echo "  📁 python_viz/corridor_exploration.gif"
echo "  📁 python_viz/island_exploration.gif" 
echo "  📁 python_viz/complex_room_exploration.gif"
echo ""
echo "You can now add these GIFs to your README or documentation!"