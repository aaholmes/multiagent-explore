# User Guide: Coordinated Multi-Robot Exploration

## 🎯 Quick Overview

This simulation demonstrates how two autonomous robots can collaboratively explore and map unknown environments using the **Iterative Boundary Trace & Coordinated Sweep** algorithm.

## 🚀 Running Simulations

### Basic Usage

```bash
# Default simulation (sample room)
./run_simulation.sh

# Specify a different map
./run_simulation.sh --map_file maps/island_room.map

# Use a specific random seed for reproducible results
./run_simulation.sh --seed 42

# Combine options
./run_simulation.sh --map_file maps/l_shaped_room.map --seed 123
```

### Available Scenarios

| Map File | Description | Best For |
|----------|-------------|----------|
| `sample_room.map` | Basic room with scattered obstacles | General algorithm demonstration |
| `simple_corridor.map` | Long narrow corridor | Boundary tracing behavior |
| `island_room.map` | Room with central island obstacle | Island detection algorithm |
| `l_shaped_room.map` | Non-convex L-shaped geometry | Complex boundary handling |

## 🎮 Understanding the Visualization

### Robot Representation
- **R**: Robot position on the map
- **Different positions**: Shows real-time robot movement
- **Map colors**: 
  - `#`: Obstacles (walls)
  - `.`: Explored empty space
  - ` `: Unexplored areas

### Console Output
The simulation prints detailed information about:
- **Tick numbers**: Simulation time steps
- **Robot positions**: Current (x, y) coordinates
- **Phase transitions**: Algorithm progression
- **Communication events**: When robots exchange map data
- **Completion status**: Final exploration results

## 🔬 Algorithm Phases Explained

### Phase 1: Initial Wall Find
- **Goal**: Find the first obstacle/boundary
- **Behavior**: Robots move together until hitting a wall
- **Transition**: Both switch to boundary scouting

### Phase 2: Boundary Scouting
- **Goal**: Trace the boundary with iterative deepening
- **Behavior**: 
  - Robot 0 follows left-hand rule
  - Robot 1 follows right-hand rule  
  - Progressive depth increase (3, 6, 12, 24... steps)
  - Periodic reconvening to share maps
- **Transition**: Complete when robots meet after closing a loop

### Phase 3: Boundary Analysis
- **Goal**: Determine if traced boundary is an island or exterior wall
- **Method**: Analyze rotation patterns during boundary trace
- **Decision**:
  - **Island**: Transition to Island Escape
  - **Exterior Wall**: Transition to Interior Sweep

### Phase 4: Island Escape (if needed)
- **Goal**: Navigate around detected island obstacle
- **Behavior**: Return to original travel direction and continue exploration
- **Transition**: Back to Wall Find or Boundary Scouting

### Phase 5: Interior Sweep
- **Goal**: Complete interior exploration using frontier-based approach
- **Behavior**: Systematic coverage of remaining unexplored areas
- **Completion**: All accessible areas mapped

## 📊 Interpreting Results

### Success Indicators
- **Phase Progression**: Robots advance through all phases
- **Map Completeness**: High percentage of explorable area mapped
- **Coordination**: Successful rendezvous and map merging events
- **Termination**: Clean completion with "Exploration complete" message

### Common Patterns
- **Simple Rooms**: Quick boundary trace → interior sweep
- **Complex Geometry**: Multiple scouting iterations → careful analysis
- **Island Scenarios**: Initial island detection → escape → exterior mapping

## 🛠️ Customization Options

### Creating Custom Maps
Maps use simple ASCII format:
```
##########
#........#  
#...##...#  ← Room with central obstacle
#........#
##########
```
- `#`: Obstacles/walls
- `.`: Empty walkable space
- Must be rectangular
- Recommended: surround with walls for clear boundaries

### Adjusting Parameters
Key parameters in `src/constants.rs`:
- `COMMUNICATION_RANGE`: How close robots must be to exchange data
- `SCOUT_DEPTH_INITIAL`: Starting depth for boundary scouting
- Various robot behavior constants

### Random Seeds
- **seed=42**: Default, good general demonstration
- **seed=123**: Alternative starting positions
- **Custom seeds**: Experiment with different robot placements

## 🐛 Troubleshooting

### Simulation Doesn't Start
- Check map file exists: `ls maps/`
- Verify map format (rectangular, proper characters)
- Ensure Rust toolchain installed: `rustc --version`

### Robots Get Stuck
- Increase iteration limit in main.rs if needed
- Check for map connectivity issues
- Very complex maps may need algorithm tuning

### Performance Issues
- Use `cargo build --release` for optimized builds
- Large maps may take longer to complete
- Monitor console output for progress indicators

## 🧪 Testing & Development

### Running Tests
```bash
# All tests
cargo test

# Specific test suites
cargo test --test robot_node      # Core algorithm tests
cargo test --test integration_tests  # Full simulation tests
cargo test --test types          # Data structure tests
```

### Development Mode
```bash
# Debug build (faster compilation)
cargo build

# Run with debug output
RUST_LOG=debug ./target/debug/multiagent_explore maps/sample_room.map
```

## 📈 Performance Expectations

### Typical Completion Times
- **Simple Corridor**: ~20-50 ticks
- **Basic Room**: ~50-150 ticks  
- **Complex Geometry**: ~100-300 ticks
- **Island Scenarios**: ~150-400 ticks

### Algorithm Efficiency
- **Communication Events**: Should see regular rendezvous
- **Coverage Rate**: Progressive map completion
- **Phase Balance**: Most time in boundary scouting and interior sweep

## 🎯 Best Practices

1. **Start Simple**: Begin with `simple_corridor.map` to understand basics
2. **Observe Phases**: Watch console output to understand algorithm progression
3. **Experiment**: Try different seeds and maps to see behavioral variations
4. **Monitor Communication**: Verify robots are coordinating effectively
5. **Check Coverage**: Ensure final maps show complete exploration

## 🚀 Advanced Usage

### Integration with ROS2 (Future)
The codebase architecture supports future ROS2 integration:
- Modular phase system maps to ROS2 nodes
- Message passing design ready for ROS2 topics
- State management compatible with ROS2 lifecycle

### Performance Analysis
```bash
# Timing analysis
time ./run_simulation.sh --map_file maps/island_room.map

# Memory usage monitoring
/usr/bin/time -l ./target/release/multiagent_explore maps/sample_room.map
```

### Batch Processing
```bash
# Run multiple scenarios
for map in maps/*.map; do
    echo "Testing $map"
    ./run_simulation.sh --map_file "$map" --seed 42
done
```