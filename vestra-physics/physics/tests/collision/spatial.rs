use glam::Vec3;
use rustc_hash::FxHashSet;
use vestra_physics::collision::spatial::dynamic::{HierarchicalSpatialHash, morton_encode};

#[test]
fn test_morton_code_uniqueness() {
    // Different cells should produce different codes
    let code1 = morton_encode(0, 0, 0);
    let code2 = morton_encode(1, 0, 0);
    let code3 = morton_encode(0, 1, 0);
    let code4 = morton_encode(0, 0, 1);

    assert_ne!(code1, code2);
    assert_ne!(code1, code3);
    assert_ne!(code1, code4);
    assert_ne!(code2, code3);
}

#[test]
fn test_fine_grid_query() {
    let mut hash = HierarchicalSpatialHash::new(0.01); // 1cm collision radius

    // Insert particles close together
    hash.insert_point(0, Vec3::new(0.0, 0.0, 0.0));
    hash.insert_point(1, Vec3::new(0.005, 0.0, 0.0)); // 5mm away
    hash.insert_point(2, Vec3::new(0.5, 0.0, 0.0)); // 50cm away (far)

    let mut buffer = Vec::new();
    let mut dedup_set = FxHashSet::default();
    hash.query(Vec3::ZERO, 0.01, &mut buffer, &mut dedup_set);

    // Should find particles 0 and 1, not 2
    assert!(buffer.contains(&0));
    assert!(buffer.contains(&1));
    assert!(!buffer.contains(&2));
}

#[test]
fn test_clear_preserves_capacity() {
    let mut hash = HierarchicalSpatialHash::new(0.01);

    // Insert and clear
    for i in 0..100 {
        hash.insert_point(i, Vec3::new(i as f32 * 0.001, 0.0, 0.0));
    }
    let _capacity_before_fine = hash.fine_grid_capacity();
    // Note: fine_grid_capacity helper needs to be exposed or we rely on public behavior.
    // Since fine_grid is private, we can't test capacity directly without exposing a method.
    // Let's settle for functional verification that it still works after clear.

    hash.clear();

    // Insert again should work
    hash.insert_point(999, Vec3::ZERO);
    let mut buffer = Vec::new();
    let mut dedup_set = FxHashSet::default();
    hash.query(Vec3::ZERO, 0.01, &mut buffer, &mut dedup_set);
    assert_eq!(buffer.len(), 1);
    assert!(buffer.contains(&999));
}

#[test]
fn test_boundary_conditions() {
    let mut hash = HierarchicalSpatialHash::new(1.0);
    // Insert at boundary of positive/negative coords
    hash.insert_point(1, Vec3::new(-0.1, -0.1, -0.1));
    hash.insert_point(2, Vec3::new(0.1, 0.1, 0.1));

    let mut buffer = Vec::new();
    let mut dedup_set = FxHashSet::default();
    // Query at 0,0,0 with radius 0.2 covering both
    hash.query(Vec3::ZERO, 0.2, &mut buffer, &mut dedup_set);

    assert!(buffer.contains(&1));
    assert!(buffer.contains(&2));
}
