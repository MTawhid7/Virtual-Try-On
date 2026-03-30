use vestra_physics::collision::preprocessing::process_mesh;
use glam::Vec3;

#[test]
fn test_mesh_conversion() {
    let raw_vertices = vec![
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    ];
    let indices = vec![0, 1, 2];

    let processed = process_mesh(&raw_vertices, &indices, 0, 0.0);

    assert_eq!(processed.vertices.len(), 3);
    assert_eq!(processed.vertices[0], Vec3::new(0.0, 0.0, 0.0));
    assert_eq!(processed.vertices[1], Vec3::new(1.0, 0.0, 0.0));
    assert_eq!(processed.vertices[2], Vec3::new(0.0, 1.0, 0.0));
}

#[test]
fn test_face_normals() {
    // Triangle in XY plane (0,0), (1,0), (0,1). Normal should be +Z.
    let raw_vertices = vec![
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    ];
    let indices = vec![0, 1, 2];

    let processed = process_mesh(&raw_vertices, &indices, 0, 0.0);

    for n in processed.normals {
        assert_eq!(n, Vec3::Z);
    }
}

#[test]
fn test_smoothing() {
    // 3 vertices in a line (0,0), (1,1), (2,0)
    // Smoothing should pull the middle one down towards the average of neighbors.
    let raw_vertices = vec![
        0.0, 0.0, 0.0,
        1.0, 1.0, 0.0, // Peak
        2.0, 0.0, 0.0,
    ];
    // Connectivity: 0-1, 1-2. (indices are triangles, so let's make a degenerate triangle 0,1,2 so they are connected)
    let indices = vec![0, 1, 2];

    // Process with 1 iteration of smoothing
    let processed = process_mesh(&raw_vertices, &indices, 1, 0.0);

    // Vertex 1: Neighbors 0 (0,0) and 2 (2,0). Avg neighbor pos = (1,0).
    // Original = (1,1). New = lerp((1,1), (1,0), 0.5) = (1, 0.5).
    // Y should be lower.
    assert!(processed.vertices[1].y < 1.0);
    assert!(processed.vertices[1].y > 0.0);
}

#[test]
fn test_inflation() {
    let raw_vertices = vec![
        0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    ];
    let indices = vec![0, 1, 2];
    let inflation = 0.1;

    let processed = process_mesh(&raw_vertices, &indices, 0, inflation);

    // Normals are +Z. Vertices should move by +0.1 in Z.
    assert!((processed.vertices[0].z - 0.1).abs() < 1e-6);
}
