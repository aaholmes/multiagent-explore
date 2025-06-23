use multiagent_explore::types::*;

#[test]
fn test_point_equality() {
    let p1 = Point { x: 1, y: 2 };
    let p2 = Point { x: 1, y: 2 };
    assert_eq!(p1, p2);
}

#[test]
fn test_cell_state_enum() {
    let cell = CellState::Unexplored;
    assert_eq!(cell, CellState::Unexplored);
}

#[test]
fn test_robot_phase_enum() {
    let phase = RobotPhase::InitialWallFind;
    assert_eq!(phase, RobotPhase::InitialWallFind);
}

#[test]
fn test_grid_map_creation() {
    let map = GridMap {
        width: 10,
        height: 10,
        cells: vec![CellState::Unexplored; 100],
    };
    assert_eq!(map.cells.len(), 100);
}

#[test]
fn test_map_loader_ascii_grid() {
    use multiagent_explore::map_loader::load_map_from_file;
    use std::fs::File;
    use std::io::Write;
    use std::env;
    let map_str = "#.#\n. .\n###";
    let tmp_dir = env::temp_dir();
    let tmp_path = tmp_dir.join("test_map.txt");
    let mut file = File::create(&tmp_path).unwrap();
    file.write_all(map_str.as_bytes()).unwrap();
    let map = load_map_from_file(tmp_path.to_str().unwrap()).unwrap();
    assert_eq!(map.width, 3);
    assert_eq!(map.height, 3);
    assert_eq!(map.cells[0], CellState::Obstacle);
    assert_eq!(map.cells[1], CellState::Empty);
    assert_eq!(map.cells[2], CellState::Obstacle);
    assert_eq!(map.cells[3], CellState::Empty);
    assert_eq!(map.cells[4], CellState::Unexplored);
    assert_eq!(map.cells[5], CellState::Empty);
    assert_eq!(map.cells[6], CellState::Obstacle);
    assert_eq!(map.cells[7], CellState::Obstacle);
    assert_eq!(map.cells[8], CellState::Obstacle);
} 