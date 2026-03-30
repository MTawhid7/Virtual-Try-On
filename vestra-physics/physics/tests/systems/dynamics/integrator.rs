use vestra_physics::systems::dynamics::Integrator;
use vestra_physics::engine::state::PhysicsState;
use vestra_physics::engine::config::PhysicsConfig;
use glam::Vec3;

#[test]
fn test_integrator_gravity() {
    let positions = vec![0.0, 0.0, 0.0];
    let indices = vec![0, 0, 0]; // unused
    let uvs = vec![0.0; 2];

    let mut state = PhysicsState::new(&positions, &indices, &uvs);
    let mut config = PhysicsConfig::default();
    config.gravity = Vec3::new(0.0, -9.8, 0.0);
    config.damping = 1.0; // No damping loss

    let dt = 1.0; // Large dt for noticeable effect
    let forces = vec![Vec3::ZERO]; // No aero

    // Initial state: pos=0, prev=0. V=0.

    // Integrate 1 step
    // a = -9.8
    // pos_new = pos + (pos - prev)*damping + a * dt^2
    // pos_new = 0 + 0 + (-9.8) * 1.0 = -9.8
    Integrator::integrate(&mut state, &config, &forces, dt);

    assert_eq!(state.positions[0].y, -9.8);
    // Prev position becomes old pos (0.0)
    assert_eq!(state.prev_positions[0].y, 0.0);
}
