use vestra_physics::systems::constraints::bending::BendingConstraint;
use vestra_physics::engine::state::PhysicsState;
use glam::Vec4;

#[test]
fn test_bending_constraint_flatness() {
    // Two triangles forming a flat square/quad:
    // 0---1
    // | / |
    // 2---3
    // Tri1: 0, 2, 1
    // Tri2: 1, 2, 3
    // Shared edge: 1-2. Bending pair: 0-3.
    let positions = vec![
        0.0, 1.0, 0.0, // 0
        1.0, 1.0, 0.0, // 1
        0.0, 0.0, 0.0, // 2
        1.0, 0.0, 0.0, // 3
    ];
    let indices = vec![
        0, 2, 1,
        1, 2, 3
    ];
    let uvs = vec![0.0; 8];

    let mut state = PhysicsState::new(&positions, &indices, &uvs);
    let constraint = BendingConstraint::new(&state, 1.0);

    // Fold it: Move 3 out of plane (z = 1.0)
    state.positions[3] = Vec4::new(1.0, 0.0, 1.0, 0.0);

    let omega = 1.0;
    let dt = 0.016;

    constraint.solve(&mut state, omega, dt);

    // Check if 3 moved back towards z=0 (restoring flatness)
    assert!(state.positions[3].z < 1.0, "Bending constraint should restore flatness");
}
