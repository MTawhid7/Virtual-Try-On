use vestra_physics::collision::CollisionResolver;
use vestra_physics::collision::MeshCollider;
use vestra_physics::engine::config::PhysicsConfig;
use vestra_physics::engine::state::PhysicsState;
use glam::Vec4;

#[test]
fn test_narrow_phase_integration() {
    let positions = vec![0.0, 0.5, 0.0];
    let indices = vec![0, 0, 0];
    let uvs = vec![0.0, 0.0];

    // Smaller triangle at Y=0
    let collider_pos = vec![-1.0, 0.0, -1.0, 1.0, 0.0, -1.0, 0.0, 0.0, 1.0];
    // Normal Up (Counter-Clockwise winding needed, or matching geometric normal)
    let collider_normals = vec![0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0];
    let collider_indices = vec![0, 2, 1];

    let mut state = PhysicsState::new(&positions, &indices, &uvs);
    let mut collider = MeshCollider::new(collider_pos, collider_normals, collider_indices, 0, 0.01);
    let mut resolver = CollisionResolver::new(1);

    // Initial state: penetrating floor by small amount.
    state.prev_positions[0] = Vec4::new(0.0, 0.01, 0.0, 0.0);
    state.positions[0] = Vec4::new(0.0, -0.005, 0.0, 0.0); // Inside by 5mm

    let config = PhysicsConfig::default();
    let dt = 0.016;

    // Debug loop
    println!("Initial Pos: {}", state.positions[0].y);

    resolver.broad_phase(&state, &mut collider);

    // Iterative solving
    for i in 0..4 {
        resolver.narrow_phase(&mut state, &collider, &config, dt);
        resolver.resolve_contacts(&mut state, &config, dt);
        println!("Step {}: Pos: {}", i, state.positions[0].y);
    }

    assert!(
        state.positions[0].y > -0.001,
        "Particle should be pushed out. Final Pos: {}",
        state.positions[0].y
    );
}
