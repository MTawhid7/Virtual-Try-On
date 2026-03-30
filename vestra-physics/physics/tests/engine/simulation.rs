use vestra_physics::engine::simulation::Simulation;
// use glam::Vec4;

#[test]
fn test_simulation_initialization() {
    let garment_pos = vec![
        0.0, 1.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 0.0, 0.0,
    ];
    let garment_indices = vec![0, 1, 2];
    let garment_uvs = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0];

    // Collider (empty for this test)
    let collider_pos = vec![];
    let collider_normals = vec![];
    let collider_indices = vec![];

    let sim = Simulation::new(
        garment_pos,
        garment_indices,
        garment_uvs,
        collider_pos,
        collider_normals,
        collider_indices,
        1, // smoothing
        0.01, // inflation
        1.0 // scale factor
    );

    assert_eq!(sim.state.count, 3);
    assert_eq!(sim.state.positions.len(), 3);
}

#[test]
fn test_simulation_step() {
    let garment_pos = vec![
        0.0, 1.0, 0.0,
        1.0, 1.0, 0.0,
        0.0, 0.0, 0.0,
    ];
    let garment_indices = vec![0, 1, 2];
    let garment_uvs = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0];

    let collider_pos = vec![0.0, -1.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0, 1.0];
    let collider_normals = vec![0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0];
    let collider_indices = vec![0, 1, 2];

    let mut sim = Simulation::new(
        garment_pos,
        garment_indices,
        garment_uvs,
        collider_pos,
        collider_normals,
        collider_indices,
        1,
        0.01,
        1.0
    );

    // Initial position of p0
    let initial_y = sim.state.positions[0].y;

    // Step simulation
    sim.step(0.016);

    // Should fall due to gravity
    assert!(sim.state.positions[0].y < initial_y, "Particle should fall due to gravity");
}
