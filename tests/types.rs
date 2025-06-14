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