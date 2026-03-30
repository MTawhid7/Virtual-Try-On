use vestra_physics::engine::config::PhysicsConfig;
use vestra_physics::engine::state::PhysicsState;
use vestra_physics::systems::forces::Aerodynamics;
use glam::Vec4;

#[test]
fn test_aerodynamics_wind_force() {
    let positions = vec![0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0];
    let indices = vec![0, 1, 2]; // One triangle
    let uvs = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0];

    let mut state = PhysicsState::new(&positions, &indices, &uvs);

    // Ensure normals are computed for wind to work
    // Normals for (0,1,0), (1,1,0), (0,0,0) -> Normal should be Z (0,0,1)
    state.normals[0] = Vec4::new(0.0, 0.0, 1.0, 0.0);
    state.normals[1] = Vec4::new(0.0, 0.0, 1.0, 0.0);
    state.normals[2] = Vec4::new(0.0, 0.0, 1.0, 0.0);

    let mut a = Aerodynamics::new(state.count);

    let mut config = PhysicsConfig::default();
    config.wind = Vec4::new(0.0, 0.0, -5.0, 0.0).truncate(); // Wind blowing against the normal
    config.drag_coeff = 1.0; // Needs drag to feel wind
    config.lift_coeff = 1.0;

    let dt = 0.016;
    let forces = a.apply(&state, &config, dt);

    // Should produce force in -Z direction
    // Only vertices 0, 1, 2 are active.
    assert!(forces[0].z < 0.0);
    assert!(forces[1].z < 0.0);
    assert!(forces[2].z < 0.0);
}
