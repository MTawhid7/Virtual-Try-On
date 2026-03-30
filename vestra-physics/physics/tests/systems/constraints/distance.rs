use glam::Vec4;
use vestra_physics::engine::state::PhysicsState;
use vestra_physics::systems::constraints::distance::DistanceConstraint;

fn make_state() -> PhysicsState {
    let positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0];
    let indices = vec![0, 1, 0]; // Degenerate
    let uvs = vec![0.0; 4];
    PhysicsState::new(&positions, &indices, &uvs)
}

#[test]
fn test_distance_constraint_creation() {
    let state = make_state();
    let _constraints = DistanceConstraint::new(&state, 0.0);

    // We expect 1 constraint between 0 and 1.
    // The implementation might generate constraints based on edges.
    // Given 0,1,0 indices, edge 0-1 is there.

    // We can't access `constraints.constraints` directly if private.
    // We should test solve.
}

#[test]
fn test_distance_constraint_solve() {
    let mut state = make_state();
    let constraint_system = DistanceConstraint::new(&state, 0.0);

    // Stretch the spring
    state.positions[1] = Vec4::new(2.0, 0.0, 0.0, 0.0); // Rest length is 1.0 (from 0 to 1). Now 2.0.

    let omega = 1.0;
    let dt = 0.016;
    constraint_system.solve(&mut state, omega, dt);

    // Should contract
    assert!(state.positions[1].x < 2.0);
    // Should stay > 1.0 (doesn't solve perfectly in 1 iteration usually, or might, depends on stiffness)
    assert!(state.positions[1].x > 1.0);
}
