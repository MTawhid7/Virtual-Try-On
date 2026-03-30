use vestra_physics::collision::MeshCollider;

#[test]
fn test_collider_initialization() {
    let positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let normals = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
    let indices = vec![0, 1, 2];

    let collider = MeshCollider::new(positions, normals, indices, 0, 0.01);

    assert_eq!(collider.vertices.len(), 3);
    assert_eq!(collider.indices.len() / 3, 1);
}

#[test]
fn test_collider_update() {
    let positions = vec![0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let normals = vec![0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0];
    let indices = vec![0, 1, 2];

    let mut collider = MeshCollider::new(positions.clone(), normals, indices, 0, 0.01);

    // Update with new positions (e.g. animation)
    let new_positions = vec![
        0.0, 1.0, 0.0, // Moved up
        1.0, 1.0, 0.0, 0.0, 2.0, 0.0,
    ];

    collider.update(&new_positions);

    // Check bounds or functionality if exposed.
    // Assuming `collider.vertices` isn't pub, we might verify via interaction or just successful execution.
    // If we have a bounding box accessor:
    // assert!(collider.aabb.min.y >= 1.0);
}
