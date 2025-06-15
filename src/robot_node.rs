use crate::types::*;

/// Main robot logic node, encapsulating state and behavior.
#[derive(Debug, Clone)]
pub struct RobotNode {
    pub state: RobotState,
    // Add communication handles or other fields as needed
}

impl RobotNode {
    /// The main decision-making loop, called on each simulation tick.
    /// Accepts a reference to all robots for coordination and the global map for sensing.
    pub fn tick(&mut self, all_robots: &[RobotNode], global_map: &GridMap) {
        // Communication: merge maps if within range of any other robot
        for other in all_robots {
            if other.state.id != self.state.id && Self::within_comm_range(&self.state.pose.position, &other.state.pose.position) {
                self.merge_map(&other.state.map);
            }
        }
        match self.state.phase {
            RobotPhase::InitialWallFind => self.execute_initial_wall_find(all_robots),
            // TODO: Add other phases
            _ => {}
        }
        // After moving, update local map with new surroundings
        self.update_local_map(global_map);
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

    /// Returns the state of the cell directly in front (+Y direction).
    pub fn sense_front(&self) -> (Point, Option<CellState>) {
        let next_pos = Point {
            x: self.state.pose.position.x,
            y: self.state.pose.position.y + 1,
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

    /// PHASE 1: Move in a straight line until a wall is seen directly in front.
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
                // Move forward (+Y)
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
    pub fn execute_boundary_scouting_leg(&mut self) {
        // TODO: Implement
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