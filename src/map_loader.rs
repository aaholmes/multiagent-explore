use std::fs::File;
use std::io::{self, BufRead, BufReader};
use crate::types::{GridMap, CellState};

/// Loads an ASCII grid map from a file.
///
/// # Format
/// - Each line is a row in the grid.
/// - '#' = Obstacle, '.' = Empty, ' ' or '?' = Unexplored
/// - All lines must have the same length.
///
/// Returns a GridMap or an io::Error.
pub fn load_map_from_file(path: &str) -> io::Result<GridMap> {
    let file = File::open(path)?;
    let reader = BufReader::new(file);
    let lines: Vec<String> = reader.lines().collect::<Result<_, _>>()?;
    if lines.is_empty() {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "Map file is empty"));
    }
    let width = lines[0].len();
    let height = lines.len();
    if !lines.iter().all(|l| l.len() == width) {
        return Err(io::Error::new(io::ErrorKind::InvalidData, "Inconsistent line lengths in map file"));
    }
    let mut cells = Vec::with_capacity(width * height);
    for line in &lines {
        for ch in line.chars() {
            let cell = match ch {
                '#' => CellState::Obstacle,
                '.' => CellState::Empty,
                ' ' | '?' => CellState::Unexplored,
                _ => return Err(io::Error::new(io::ErrorKind::InvalidData, format!("Invalid map character: {}", ch))),
            };
            cells.push(cell);
        }
    }
    Ok(GridMap { width, height, cells })
} 