# Coordinated Multi-Robot Exploration

A simulation of a coordinated multi-robot exploration and mapping algorithm in Rust. This project explores a robust, phased strategy for two robots to systematically map an unknown 2D grid environment under realistic constraints.

## Overview
Autonomous exploration of unknown environments is a fundamental problem in robotics, with applications ranging from search and rescue to planetary exploration. This project implements and simulates a novel multi-robot exploration strategy designed to be scalable, complete, and robust to common real-world constraints.

The core of this project is the implementation of the "Iterative Boundary Trace & Coordinated Sweep" algorithm, a deterministic strategy designed to solve the specific problem outlined below.

### The Problem: Coordinated Exploration Under Constraints

The algorithm is designed to solve the following problem:

Objective: A team of two autonomous robots must collaboratively explore and create a complete map of an unknown 2D environment.
Initial Conditions: The robots begin adjacent to each other at a single, arbitrary starting location. They possess no prior map information.

Environment: The space is represented as a 2D grid containing obstacles of arbitrary size, shape, and complexity (e.g., non-convex structures, dense clusters).

Primary Constraint: The robots have a limited communication range (R). They can only exchange data, such as their individual map discoveries and future plans, when they are within this range of each other.
The goal is to develop a comprehensive strategy that enables the robots to efficiently and completely map the entire connected area while operating under these conditions.

## The Challenge
The specific constraints of the problem lead to several key challenges that the algorithm must overcome:

The Explore-vs-Coordinate Dilemma: The limited communication range creates a fundamental tension. To explore new areas quickly, robots must move apart, but to coordinate effectively and avoid redundant work, they must remain close enough to communicate.
Scalability in the Unknown: Without knowing the environment's size beforehand, a naive exploration strategy could lead to robots getting lost on extremely long paths or becoming permanently separated.
Information Asymmetry: For periods when the robots are out of communication range, each robot will build a different map of the world. The algorithm must have a robust strategy for merging this information and reconciling different worldviews upon reconvening.

Ensuring Completeness: The strategy must guarantee that no areas are missed, requiring systematic methods to handle complex features like large open spaces, non-convex "courtyards," and dense clusters of obstacles that can act as mazes.

## The Algorithm: Iterative Boundary Trace & Coordinated Sweep
Our solution is a deterministic, multi-phase algorithm designed to address these challenges systematically.

### Phase 1: Initial Wall Discovery
The exploration begins with the two robots moving in a coordinated fashion from their starting point until they encounter the first major obstacle or boundary wall. This establishes an initial anchor point for the mapping process.

### Phase 2: Iterative Boundary Scouting
To avoid committing to a potentially massive and time-consuming perimeter trace, the robots first scout the boundary iteratively:

Starting with a small search depth n, the two robots follow the wall in opposite directions for n steps.
They then perform a local scan by retracing a path adjacent to the one just taken to reconvene, share map data, and synchronize. This frequent reconvening is key to managing the limited communication range.
The search depth n is progressively doubled for subsequent scouting missions.
This phase continues until the robots meet, having successfully traced a complete, closed loop. This adaptive approach ensures that large perimeters are explored incrementally without risk of over-commitment.

### Phase 3: Loop Analysis (Island vs. Outer Wall)
Once a loop is closed, the robots analyze it to determine if they have mapped an internal "island" (an obstacle cluster) or the true outer boundary of the area. This is achieved by analyzing the geometry of the traced path and the contents of the map inside versus outside the loop. If an island is detected, the robots use their last known heading to "escape" the island and continue searching for the true outer wall.

### Phase 4: Central Island Scan
After confirming the main outer boundary, the robots perform a quick, coordinated scan across a central line of the newly defined area. This is a heuristic designed to efficiently discover any large, simple obstacles or "courtyards" in the middle of the space that could complicate the next phase.

### Phase 5: Coordinated Interior Sweeps
With the boundary confirmed and major central obstacles noted, the robots begin the final mapping phase:

They collaboratively sweep the interior space by moving inwards from the known frontier.
They move in coordinated "opposite directions," ensuring systematic coverage.
They periodically meet on the other side of the sweep area to merge their maps, ensuring consistency and efficiently planning the next inward sweep on the new, smaller frontier. This process repeats until the entire interior is mapped.

## Key Features & Concepts
Adaptive Exploration: Uses iterative deepening ("scouting missions") to safely probe large environments.
Communication-Aware: The frequent rendezvous points during the scouting phase are designed to work within the constraint of limited communication range.
Systematic Coverage: Employs deterministic phases to ensure no part of the environment is missed.
Explicit Coordination: Relies on planned rendezvous points for map merging and re-synchronization.
Robust Island Handling: Includes specific logic to detect and navigate around complex internal obstacles without abandoning the goal of finding the true perimeter.

## Implementation Details
Language: Rust. Chosen for its performance, memory safety, and strong concurrency support, making it ideal for building reliable, high-performance robotics systems.
Ecosystem: Intended for integration with the ROS2 framework and the Gazebo simulator to provide a realistic demonstration of the algorithm's performance in a standard robotics environment.

## How to Build & Run

### Clone the repository
git clone https://github.com/aaholmes/multiagent-explore.git
cd multiagent-explore

### Build instructions (e.g., using 'cargo' or 'colcon build' for ROS2)
cargo build --release

### Run simulation instructions
./run_simulation.sh --map_file maps/complex_office.map
License
This project is licensed under the MIT License.
