use crate::types::*;
use std::f64::consts::PI;

/// Main robot logic node, encapsulating state and behavior.
#[derive(Debug, Clone)]
pub struct RobotNode {
    pub state: RobotState,
    // Add communication handles or other fields as needed
}

impl RobotNode {
    /// The main decision-making loop, called on each simulation tick.
    /// Only performs movement; sensing and communication are handled externally.
    pub fn tick(&mut self, all_robots: &[RobotNode], global_map: &GridMap) {
        match self.state.phase {
            RobotPhase::InitialWallFind => self.execute_initial_wall_find(all_robots, global_map),
            RobotPhase::BoundaryScouting => self.execute_boundary_scouting_leg(all_robots, global_map),
            // TODO: Add other phases
            _ => {}
        }
    }

    /// Update the robot's local map with its current cell and four neighbors from the global map.
    pub fn update_local_map(&mut self, global_map: &GridMap) {
        let width = self.state.map.width as i32;
        let height = self.state.map.height as i32;
        let pos = self.state.pose.position;
        let mut to_update = vec![pos];
        let dirs = [(0, -1), (0, 1), (1, 0), (-1, 0)];
        for (dx, dy) in &dirs {
            let nx = pos.x + dx;
            let ny = pos.y + dy;
            if nx >= 0 && nx < width && ny >= 0 && ny < height {
                to_update.push(Point { x: nx, y: ny });
            }
        }
        for p in to_update {
            let idx = (p.y as usize) * self.state.map.width + (p.x as usize);
            self.state.map.cells[idx] = global_map.cells[idx];
        }
    }

    /// Returns the state of the four adjacent cells (N, S, E, W) in the robot's local map.
    pub fn sense_neighbors(&self) -> [(Point, Option<CellState>); 4] {
        let dirs = [
            (0, -1), // North
            (0, 1),  // South
            (1, 0),  // East
            (-1, 0), // West
        ];
        let mut result = [(self.state.pose.position, None); 4];
        let width = self.state.map.width as i32;
        let height = self.state.map.height as i32;
        for (i, (dx, dy)) in dirs.iter().enumerate() {
            let nx = self.state.pose.position.x + dx;
            let ny = self.state.pose.position.y + dy;
            if nx < 0 || nx >= width || ny < 0 || ny >= height {
                result[i] = (Point { x: nx, y: ny }, Some(CellState::Obstacle)); // Treat out-of-bounds as boundary
            } else {
                let idx = (ny as usize) * self.state.map.width + (nx as usize);
                result[i] = (Point { x: nx, y: ny }, Some(self.state.map.cells[idx]));
            }
        }
        result
    }

    /// Returns the state of the cell directly in front (-Y direction, up).
    pub fn sense_front(&self) -> (Point, Option<CellState>) {
        let next_pos = Point {
            x: self.state.pose.position.x,
            y: self.state.pose.position.y - 1,
        };
        let width = self.state.map.width as i32;
        let height = self.state.map.height as i32;
        if next_pos.x < 0 || next_pos.x >= width || next_pos.y < 0 || next_pos.y >= height {
            (next_pos, Some(CellState::Obstacle))
        } else {
            let idx = (next_pos.y as usize) * self.state.map.width + (next_pos.x as usize);
            (next_pos, Some(self.state.map.cells[idx]))
        }
    }

    /// PHASE 1: Move in a straight line until a wall is seen directly in front (now -Y direction).
    pub fn execute_initial_wall_find(&mut self, all_robots: &[RobotNode], global_map: &GridMap) {
        let (next_pos, cell) = self.sense_front();
        match cell {
            Some(CellState::Obstacle) => {
                if self.state.phase != RobotPhase::BoundaryScouting {
                    println!("Robot {} sees obstacle in front at ({}, {}), stopping and starting boundary scouting.", self.state.id, next_pos.x, next_pos.y);
                    self.state.phase = RobotPhase::BoundaryScouting;
                    // Immediately begin boundary scouting
                    self.execute_boundary_scouting_leg(all_robots, global_map);
                }
                // If already in BoundaryScouting, just return
                return;
            }
            _ => {
                // Move forward (-Y)
                println!("Robot {} moves from ({}, {}) to ({}, {})", self.state.id, self.state.pose.position.x, self.state.pose.position.y, next_pos.x, next_pos.y);
                self.state.pose.position = next_pos;
                // Update local map (mark as empty)
                let idx = (next_pos.y as usize) * self.state.map.width + (next_pos.x as usize);
                self.state.map.cells[idx] = CellState::Empty;
            }
        }
    }

    /// Returns true if two positions are within Manhattan distance <= 2.
    pub fn within_comm_range(a: &Point, b: &Point) -> bool {
        (a.x - b.x).abs() + (a.y - b.y).abs() <= 2
    }

    /// Print the robot's current map for inspection.
    pub fn print_map(&self) {
        println!("Robot {}'s map:", self.state.id);
        for y in 0..self.state.map.height {
            for x in 0..self.state.map.width {
                let idx = y * self.state.map.width + x;
                let ch = match self.state.map.cells[idx] {
                    CellState::Obstacle => '#',
                    CellState::Empty => '.',
                    CellState::Unexplored => ' ',
                };
                if self.state.pose.position.x == x as i32 && self.state.pose.position.y == y as i32 {
                    print!("R");
                } else {
                    print!("{}", ch);
                }
            }
            println!("");
        }
    }

    /// PHASE 2: Perform one leg of the iterative boundary scouting.
    pub fn execute_boundary_scouting_leg(&mut self, all_robots: &[RobotNode], global_map: &GridMap) {
        // Initialize state if first tick in this phase
        if self.state.boundary_scout.is_none() {
            let tracing_direction = if self.state.id == 0 { -1 } else { 1 }; // 0: left, 1: right
            self.state.boundary_scout = Some(BoundaryScoutState {
                tracing_direction,
                steps_taken: 0, // This will now track total steps in the phase, not per leg
                steps_taken_this_scouting_mission: 0, // New variable for current leg
                returning: false,
                path: vec![self.state.pose.position],
                first_move: true,
                initial_scouting_direction: None, // Will be set on first move
            });
            println!("Robot {} begins boundary scouting, tracing_direction {}", self.state.id, tracing_direction);
        } else {
            // If boundary_scout is already Some, it means we are starting a new leg after reconvening
            // or continuing an existing leg.
            // We need to ensure first_move is reset for the new leg if it's a new leg,
            // but not if it's just a continuation.
            // The logic for resetting steps_taken_this_scouting_mission and path
            // is now handled when `returning` becomes false after a full return.
            // The tracing_direction should persist.
            if let Some(scout) = self.state.boundary_scout.as_mut() {
                // If we just finished returning, and are starting a new leg, first_move should be true
                // This is handled by the `returning` logic setting `returning = false` and clearing path.
                // If `returning` is false, and `steps_taken_this_scouting_mission` is 0, it's a new leg.
                if !scout.returning && scout.steps_taken_this_scouting_mission == 0 {
                    scout.first_move = true;
                }
            }
        }

        let scout_n = self.state.scout_depth_n;
        // Split mutable and immutable borrows
        let (tracing_direction, returning, steps_taken_this_scouting_mission, path_len, first_move);
        {
            let scout = self.state.boundary_scout.as_ref().unwrap();
            tracing_direction = scout.tracing_direction;
            returning = scout.returning;
            steps_taken_this_scouting_mission = scout.steps_taken_this_scouting_mission;
            path_len = scout.path.len();
            first_move = scout.first_move;
        }
        if returning {
            if path_len > 1 {
                if let Some(scout) = self.state.boundary_scout.as_mut() {
                    scout.path.pop(); // Remove current position
                    let prev = *scout.path.last().unwrap();
                    println!("Robot {} returns to ({}, {}). Path length remaining: {}", self.state.id, prev.x, prev.y, scout.path.len());
                    self.state.pose.position = prev;
                }
            } else {
                // Robot has returned to the start of the leg
                println!("Robot {} completed return scan. Doubling scout_depth_n ({} -> {}) and starting next leg.", self.state.id, scout_n, scout_n * 2);
                self.state.scout_depth_n *= 2;
                if let Some(scout) = self.state.boundary_scout.as_mut() {
                    scout.steps_taken_this_scouting_mission = 0;
                    scout.returning = false;
                    scout.path.clear(); // Clear path for the new leg
                    scout.path.push(self.state.pose.position); // Add current position as start of new path
                    scout.first_move = true; // Reset first_move for new scouting leg
                }
            }
            return;
        }
        if steps_taken_this_scouting_mission < scout_n {
            let next = if first_move {
                // For first move, either record initial direction or reuse stored one
                let current_pos = self.state.pose.position;
                
                // Check if we have stored initial direction
                let stored_direction = self.state.boundary_scout.as_ref()
                    .and_then(|scout| scout.initial_scouting_direction);
                
                if let Some(initial_dir) = stored_direction {
                    // Reuse stored direction
                    let next_pos = Point {
                        x: current_pos.x + initial_dir.x,
                        y: current_pos.y + initial_dir.y,
                    };
                    
                    // Validate move
                    if next_pos.x >= 0 && next_pos.y >= 0 && 
                       next_pos.x < global_map.width as i32 && next_pos.y < global_map.height as i32 {
                        let idx = (next_pos.y as usize) * global_map.width + (next_pos.x as usize);
                        if global_map.cells[idx] != CellState::Obstacle {
                            Some(next_pos)
                        } else {
                            // Fallback to normal wall following
                            self.wall_follow_step_first_move(global_map, tracing_direction)
                        }
                    } else {
                        // Fallback to normal wall following  
                        self.wall_follow_step_first_move(global_map, tracing_direction)
                    }
                } else {
                    // First time, calculate and store direction
                    let next_pos = self.wall_follow_step_first_move(global_map, tracing_direction);
                    if let Some(pos) = next_pos {
                        let direction = Point {
                            x: pos.x - current_pos.x,
                            y: pos.y - current_pos.y,
                        };
                        // Store the direction
                        if let Some(scout) = self.state.boundary_scout.as_mut() {
                            scout.initial_scouting_direction = Some(direction);
                        }
                    }
                    next_pos
                }
            } else {
                self.wall_follow_step(global_map, tracing_direction)
            };
            if let Some(next_pos) = next {
                println!("Robot {} wall-follows to ({}, {}) [step {}/{}]", self.state.id, next_pos.x, next_pos.y, steps_taken_this_scouting_mission + 1, scout_n);
                let prev_pos = self.state.pose.position;
                self.update_orientation(prev_pos, next_pos); // Update orientation before move
                self.state.pose.position = next_pos;
                if let Some(scout) = self.state.boundary_scout.as_mut() {
                    scout.path.push(next_pos);
                    scout.steps_taken += 1; // Increment total steps in phase
                    scout.steps_taken_this_scouting_mission += 1; // Always increment steps for current leg
                    scout.first_move = false; // After the first move, it's no longer the first move
                }

                // Check for rendezvous during active scouting leg
                let partner = all_robots.iter().find(|r| r.state.id == self.state.partner_id).unwrap();
                // Only consider rendezvous if at least one step has been taken in this leg
                // and the partner is not the current position (to avoid self-rendezvous issues)
                if !first_move && Self::within_comm_range(&self.state.pose.position, &partner.state.pose.position) && self.state.pose.position != partner.state.pose.position {
                    println!("Robot {} rendezvous with partner {} during scouting leg. Transitioning to BOUNDARY_ANALYSIS and doubling n.", self.state.id, partner.state.id);
                    self.state.phase = RobotPhase::BoundaryAnalysis;
                    self.state.scout_depth_n *= 2;
                    self.state.boundary_scout = None; // Reset state for the next phase
                    return; // Exit function as phase has changed
                }
            } else {
                println!("Robot {} cannot wall-follow, stays at ({}, {})", self.state.id, self.state.pose.position.x, self.state.pose.position.y);
            }
        } else {
            if let Some(scout) = self.state.boundary_scout.as_mut() {
                scout.returning = true;
                scout.steps_taken_this_scouting_mission = 0; // Reset for next leg
            }
            println!("Robot {} finished {} steps, returning.", self.state.id, scout_n);
        }
    }


    /// Wall-following step for the very first move after hitting the wall.
    /// Robot 0: prioritize left, Robot 1: prioritize right.
    pub fn wall_follow_step_first_move(&self, global_map: &GridMap, tracing_direction: i8) -> Option<Point> {
        let (current_dx, current_dy) = self.get_direction_vector();
        let current_pos = self.state.pose.position;
        let dirs = if tracing_direction == -1 {
            // Robot 0: prioritize left, then forward, then right, then back
            vec![
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (current_dx, current_dy),   // Forward
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (-current_dx, -current_dy), // Back
            ]
        } else {
            // Robot 1: prioritize right, then forward, then left, then back
            vec![
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (current_dx, current_dy),   // Forward
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (-current_dx, -current_dy), // Back
            ]
        };
        for (rdx, rdy) in dirs {
            let nx = current_pos.x + rdx;
            let ny = current_pos.y + rdy;
            if nx < 0 || ny < 0 || nx >= global_map.width as i32 || ny >= global_map.height as i32 {
                continue;
            }
            let idx = (ny as usize) * global_map.width + (nx as usize);
            if global_map.cells[idx] != CellState::Obstacle {
                return Some(Point { x: nx, y: ny });
            }
        }
        None
    }

    /// Translates the robot's orientation_rad into a (dx, dy) movement vector.
    pub fn get_direction_vector(&self) -> (i32, i32) {
        // Normalize angle to be between -PI and PI
        let angle = self.state.pose.orientation_rad.rem_euclid(2.0 * PI);
        let angle_deg = angle.to_degrees().round() as i32;

        // Using rounded degrees to avoid floating point comparison issues
        // North (-Y): -90 or 270
        // South (+Y): 90
        // East (+X): 0
        // West (-X): 180 or -180

        if angle_deg == 0 { // East
            (1, 0)
        } else if angle_deg == 90 { // South
            (0, 1)
        } else if angle_deg == 180 || angle_deg == -180 { // West
            (-1, 0)
        } else if angle_deg == 270 || angle_deg == -90 { // North
            (0, -1)
        } else {
            // Default to North if orientation is not one of the cardinal directions
            println!("Warning: Robot {} has non-cardinal orientation: {}. Defaulting to North.", self.state.id, self.state.pose.orientation_rad.to_degrees());
            (0, -1)
        }
    }

    /// Updates the robot's orientation based on its previous position and new position.
    pub fn update_orientation(&mut self, prev_pos: Point, next_pos: Point) {
        let dx = next_pos.x - prev_pos.x;
        let dy = next_pos.y - prev_pos.y;
        self.state.pose.orientation_rad = match (dx, dy) {
            (1, 0) => 0.0,    // East
            (-1, 0) => PI,   // West
            (0, 1) => PI / 2.0, // South
            (0, -1) => -PI / 2.0, // North
            _ => self.state.pose.orientation_rad, // No change or invalid move
        };
    }

    /// Wall-following step: try to move forward, else turn (left/right) to follow wall.
    /// Returns the next position if a move is possible.
    pub fn wall_follow_step(&self, global_map: &GridMap, tracing_direction: i8) -> Option<Point> {
        // Facing: 0=East (+X), PI/2=South (+Y), PI=West (-X), -PI/2=North (-Y)
        let (current_dx, current_dy) = self.get_direction_vector();
        let current_pos = self.state.pose.position;

        // Define relative directions based on current orientation
        // (dx, dy) represents changes in x and y coordinates relative to the current position
        // Order of checks depends on tracing_direction (left-hand or right-hand rule)
        let relative_dirs: Vec<(i32, i32)>;

        if tracing_direction == -1 { // Robot 0: Left-hand rule (keep wall on left, so try right first)
            relative_dirs = vec![
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (current_dx, current_dy),   // Forward
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (-current_dx, -current_dy), // Back (180 degree turn)
            ];
        } else { // Robot 1: Right-hand rule (keep wall on right, so try left first)
            relative_dirs = vec![
                (current_dy, -current_dx),  // Relative Left (CCW rotation: 90° left from current)
                (current_dx, current_dy),   // Forward
                (-current_dy, current_dx),  // Relative Right (CW rotation: 90° right from current)
                (-current_dx, -current_dy), // Back (180 degree turn)
            ];
        }

        for (rdx, rdy) in relative_dirs {
            let nx = current_pos.x + rdx;
            let ny = current_pos.y + rdy;

            if nx < 0 || ny < 0 || nx >= global_map.width as i32 || ny >= global_map.height as i32 {
                continue; // Out of bounds, consider it an obstacle
            }

            let idx = (ny as usize) * global_map.width + (nx as usize);
            if global_map.cells[idx] != CellState::Obstacle {
                return Some(Point { x: nx, y: ny });
            }
        }
        None
    }

    /// PHASE 3: Analyze a completed loop to determine if it's an island.
    pub fn analyze_completed_loop(&mut self) {
        // TODO: Implement
    }

    /// PHASE 4: Perform a coordinated scan across the middle of a known area.
    pub fn execute_central_island_scan(&mut self) {
        // TODO: Implement
    }

    /// PHASE 5: Perform one leg of the coordinated inward sweep.
    pub fn execute_interior_sweep_leg(&mut self) {
        // TODO: Implement
    }

    /// Merges a map received from a partner robot into this robot's own map.
    pub fn merge_map(&mut self, partner_map: &GridMap) {
        // Merge known cells from partner_map into self.map
        for (i, &cell) in partner_map.cells.iter().enumerate() {
            if cell != CellState::Unexplored {
                self.state.map.cells[i] = cell;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // Helper function to create a simple grid map for testing
    fn create_test_map(cells: &[&str]) -> GridMap {
        let height = cells.len();
        let width = cells[0].len();
        let mut map_cells = vec![CellState::Unexplored; width * height];
        for (y, row) in cells.iter().enumerate() {
            for (x, char) in row.chars().enumerate() {
                let idx = y * width + x;
                map_cells[idx] = match char {
                    '#' => CellState::Obstacle,
                    '.' => CellState::Empty,
                    ' ' => CellState::Unexplored,
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
                tracing_direction: -1,
                steps_taken: 0,
                returning: false,
                path: vec![Point { x: 1, y: 1 }],
                first_move: true,
            }),
        };
        let mut robot0 = RobotNode { state: robot0_state };

        // Step 1: Robot 0 (left-hand rule), at (1,1) facing North. Wall to the left (West).
        // Prioritize: Left (East), Forward (North), Right (West), Back (South).
        // From (1,1) facing North: 
        // Left (East): (2,1) - Empty. Should move to (2,1), facing East.
        let next_pos = robot0.wall_follow_step(&map, -1).unwrap();
        assert_eq!(next_pos, Point { x: 2, y: 1 });
        robot0.update_orientation(Point { x: 1, y: 1 }, next_pos);
        robot0.state.pose.position = next_pos;
        assert_eq!(robot0.state.pose.orientation_rad.to_degrees().round() as i32, 0); // Facing East

        // Step 2: Robot 0 at (2,1) facing East. Wall to the left (North).
        // Prioritize: Left (North), Forward (East), Right (South), Back (West).
        // From (2,1) facing East:
        // Left (North): (2,0) - Obstacle (#)
        // Forward (East): (3,1) - Out of bounds
        // Right (South): (2,2) - Obstacle (#)
        // Back (West): (1,1) - Empty (.). Should move to (1,1), facing West.
        let next_pos = robot0.wall_follow_step(&map, -1).unwrap();
        assert_eq!(next_pos, Point { x: 1, y: 1 });
        robot0.update_orientation(Point { x: 2, y: 1 }, next_pos);
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
                tracing_direction: 1,
                steps_taken: 0,
                returning: false,
                path: vec![Point { x: 1, y: 1 }],
                first_move: true,
            }),
        };
        let mut robot1 = RobotNode { state: robot1_state };

        // Step 1: Robot 1 (right-hand rule), at (1,1) facing North. Wall to the right (East).
        // Prioritize: Right (West), Forward (North), Left (East), Back (South).
        // From (1,1) facing North:
        // Right (West): (0,1) - Empty (.). Should move to (0,1), facing West.
        let next_pos = robot1.wall_follow_step(&map, 1).unwrap();
        assert_eq!(next_pos, Point { x: 0, y: 1 });
        robot1.update_orientation(Point { x: 1, y: 1 }, next_pos);
        robot1.state.pose.position = next_pos;
        assert_eq!(robot1.state.pose.orientation_rad.to_degrees().round() as i32, 180); // Facing West

        // Step 2: Robot 1 at (0,1) facing West. Wall to the right (South).
        // Prioritize: Right (South), Forward (West), Left (North), Back (East).
        // From (0,1) facing West:
        // Right (South): (0,2) - Obstacle (#)
        // Forward (West): (-1,1) - Out of bounds
        // Left (North): (0,0) - Obstacle (#)
        // Back (East): (1,1) - Empty (.). Should move to (1,1), facing East.
        let next_pos = robot1.wall_follow_step(&map, 1).unwrap();
        assert_eq!(next_pos, Point { x: 1, y: 1 });
        robot1.update_orientation(Point { x: 0, y: 1 }, next_pos);
        robot1.state.pose.position = next_pos;
        assert_eq!(robot1.state.pose.orientation_rad.to_degrees().round() as i32, 0); // Facing East
    }
} 