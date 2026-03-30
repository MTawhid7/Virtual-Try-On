use vestra_physics::utils::normals::compute_vertex_normals;
use glam::Vec4;

#[test]
fn test_simple_triangle() {
    let positions = vec![
        Vec4::new(0.0, 0.0, 0.0, 0.0),
        Vec4::new(1.0, 0.0, 0.0, 0.0),
        Vec4::new(0.0, 1.0, 0.0, 0.0),
    ];
    let indices = vec![0, 1, 2];
    let mut normals = vec![Vec4::ZERO; 3];

    compute_vertex_normals(&positions, &indices, &mut normals);

    // For a triangle in the XY plane, normal should point in +Z
    for n in &normals {
        assert!((n.z - 1.0).abs() < 1e-6);
    }
}

#[test]
fn test_degenerate_triangle() {
    let positions = vec![
        Vec4::new(0.0, 0.0, 0.0, 0.0),
        Vec4::new(0.0, 0.0, 0.0, 0.0), // Same pos, degenerate
        Vec4::new(0.0, 1.0, 0.0, 0.0),
    ];
    let indices = vec![0, 1, 2];
    let mut normals = vec![Vec4::ZERO; 3];

    compute_vertex_normals(&positions, &indices, &mut normals);

    // Should fallback to up vector (0,1,0) or at least not NaN
    for n in &normals {
        assert!(!n.is_nan());
        // Implementation falls back to (0,1,0,0)
        assert_eq!(*n, Vec4::new(0.0, 1.0, 0.0, 0.0));
    }
}
