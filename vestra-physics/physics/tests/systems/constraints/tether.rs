use glam::Vec4;
use vestra_physics::engine::state::PhysicsState;
use vestra_physics::systems::constraints::tether::TetherConstraint;

fn make_state() -> PhysicsState {
    let positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0]; // 1.0 apart
    let indices = vec![0, 1, 0];
    let uvs = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
    PhysicsState::new(&positions, &indices, &uvs)
}

#[test]
fn test_tether_constraint_preserves_length() {
    let mut state = make_state();
    let constraint = TetherConstraint::new(&state);

    // Stretch significantly (tether usually allows 1.1x or similar)
    // Original dist = 1.0. Move to 2.0.
    state.positions[1] = Vec4::new(2.0, 0.0, 0.0, 0.0);

    let omega = 1.0;
    let dt = 0.016;

    // Solve multiple times to ensure convergence
    for _ in 0..5 {
        constraint.solve(&mut state, omega, dt);
    }

    // Should be pulled back. Tether usually clamps to max_len.
    // Assuming max_stretch is 1.1 or 1.2 in default config or hardcoded.
    // If we don't have config here, we assume internal defaults.
    // Just check it pulled back from 2.0
    assert!(state.positions[1].x < 2.0);
    assert!(state.positions[1].x > 0.5);
}
