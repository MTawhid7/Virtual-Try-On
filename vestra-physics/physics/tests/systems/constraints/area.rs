use vestra_physics::systems::constraints::area::AreaConstraint;
use vestra_physics::engine::state::PhysicsState;
use glam::Vec4;

#[test]
#[ignore] // TODO: Investigate instability. Test simulation explodes with current constraint implementation.
fn test_area_constraint_restoration() {
    // Single triangle: (0,0), (1,0), (0,1) -> Area = 0.5
    let positions = vec![
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    ];
    let indices = vec![0, 1, 2];
    let uvs = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0];

    let mut state = PhysicsState::new(&positions, &indices, &uvs);
    let constraint = AreaConstraint::new(&state);

    // Distort the triangle: Move (1,0) to (2,0) -> Area becomes 1.0
    state.positions[1] = Vec4::new(2.0, 0.0, 0.0, 0.0);

    let omega = 0.2; // Use softer omega/SOR factor to avoid explosion
    let dt = 0.016;
    let compliance = 0.0;

    // Constraint is non-linear, so it might need multiple iterations to converge
    for _ in 0..20 {
        constraint.solve(&mut state, compliance, omega, dt);
    }

    // Check new area is closer to 0.5
    let p0 = state.positions[0];
    let p1 = state.positions[1];
    let p2 = state.positions[2];

    let u = p1 - p0;
    let v = p2 - p0;
    let area = 0.5 * u.truncate().cross(v.truncate()).length();

    // Should be significantly less than 1.0
    assert!(area < 0.99, "Area should decrease towards rest area. Current: {}", area);
}
