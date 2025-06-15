use crate::types::*;

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
            RobotPhase::InitialWallFind => self.execute_initial_wall_find(all_robots),
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
    pub fn execute_initial_wall_find(&mut self, _all_robots: &[RobotNode]) {
        let (next_pos, cell) = self.sense_front();
        match cell {
            Some(CellState::Obstacle) => {
                if self.state.phase != RobotPhase::BoundaryScouting {
                    println!("Robot {} sees obstacle in front at ({}, {}), stopping.", self.state.id, next_pos.x, next_pos.y);
                    self.state.phase = RobotPhase::BoundaryScouting;
                }
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
                steps_taken: 0,
                returning: false,
                path: vec![self.state.pose.position],
            });
            println!("Robot {} begins boundary scouting, tracing_direction {}", self.state.id, tracing_direction);
        }
        let scout_n = self.state.scout_depth_n;
        // Split mutable and immutable borrows
        let (tracing_direction, returning, steps_taken, path_len);
        {
            let scout = self.state.boundary_scout.as_ref().unwrap();
            tracing_direction = scout.tracing_direction;
            returning = scout.returning;
            steps_taken = scout.steps_taken;
            path_len = scout.path.len();
        }
        if returning {
            if path_len > 1 {
                if let Some(scout) = self.state.boundary_scout.as_mut() {
                    scout.path.pop(); // Remove current position
                    let prev = *scout.path.last().unwrap();
                    println!("Robot {} returns to ({}, {})", self.state.id, prev.x, prev.y);
                    self.state.pose.position = prev;
                }
            }
            // Check for rendezvous (in comm range)
            let partner = all_robots.iter().find(|r| r.state.id == self.state.partner_id).unwrap();
            if Self::within_comm_range(&self.state.pose.position, &partner.state.pose.position) {
                println!("Robot {} rendezvous with partner {}. Doubling n.", self.state.id, partner.state.id);
                self.state.scout_depth_n *= 2;
                self.state.boundary_scout = None; // Start new leg next tick
            }
            return;
        }
        if steps_taken < scout_n {
            let next = self.wall_follow_step(global_map, tracing_direction);
            if let Some(next_pos) = next {
                println!("Robot {} wall-follows to ({}, {})", self.state.id, next_pos.x, next_pos.y);
                self.state.pose.position = next_pos;
                if let Some(scout) = self.state.boundary_scout.as_mut() {
                    scout.path.push(next_pos);
                    scout.steps_taken += 1;
                }
            } else {
                println!("Robot {} cannot wall-follow, stays at ({}, {})", self.state.id, self.state.pose.position.x, self.state.pose.position.y);
            }
        } else {
            if let Some(scout) = self.state.boundary_scout.as_mut() {
                scout.returning = true;
            }
            println!("Robot {} finished {} steps, returning.", self.state.id, scout_n);
        }
    }

    /// Wall-following step: try to move forward, else turn (left/right) to follow wall.
    /// Returns the next position if a move is possible.
    pub fn wall_follow_step(&self, global_map: &GridMap, tracing_direction: i8) -> Option<Point> {
        // Facing up (-Y), tracing_direction: -1=left, +1=right
        // Try: left, forward, right, back (relative to current orientation)
        let dirs = [
            (-1, 0), // left
            (0, -1), // forward
            (1, 0),  // right
            (0, 1),  // back
        ];
        let order = if tracing_direction == -1 {
            [0, 1, 2, 3] // left wall-follow: left, forward, right, back
        } else {
            [2, 1, 0, 3] // right wall-follow: right, forward, left, back
        };
        let pos = self.state.pose.position;
        for &i in &order {
            let (dx, dy) = dirs[i];
            let nx = pos.x + dx;
            let ny = pos.y + dy;
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