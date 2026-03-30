use vestra_physics::collision::exclusion::TopologyExclusion;


#[test]
fn test_immediate_neighbors_excluded() {
    // Simple triangle: 0-1-2
    let indices = vec![0u32, 1, 2];
    let exclusion = TopologyExclusion::new(&indices, 3, 1);

    // All three should exclude each other (1-ring)
    assert!(exclusion.should_exclude(0, 1));
    assert!(exclusion.should_exclude(0, 2));
    assert!(exclusion.should_exclude(1, 0));
    assert!(exclusion.should_exclude(1, 2));
    assert!(exclusion.should_exclude(2, 0));
    assert!(exclusion.should_exclude(2, 1));
}

#[test]
fn test_two_ring_exclusion() {
    // Two triangles sharing edge 1-2: (0,1,2) and (1,2,3)
    // Topology: 0 -- 1 -- 3
    //               |
    //               2
    let indices = vec![0u32, 1, 2, 1, 2, 3];
    let exclusion = TopologyExclusion::new(&indices, 4, 2);

    // 0 and 3 are 2-ring neighbors (0->1->3 or 0->2->3)
    assert!(exclusion.should_exclude(0, 3));
    assert!(exclusion.should_exclude(3, 0));
}

#[test]
fn test_distant_particles_not_excluded() {
    // Two separate triangles: (0,1,2) and (3,4,5)
    let indices = vec![0u32, 1, 2, 3, 4, 5];
    let exclusion = TopologyExclusion::new(&indices, 6, 2);

    // Particles from different triangles should NOT be excluded
    assert!(!exclusion.should_exclude(0, 3));
    assert!(!exclusion.should_exclude(0, 4));
    assert!(!exclusion.should_exclude(1, 5));
}

#[test]
fn test_self_always_excluded() {
    let indices = vec![0u32, 1, 2];
    let exclusion = TopologyExclusion::new(&indices, 3, 1);

    // Particle should exclude itself
    assert!(exclusion.should_exclude(0, 0));
    assert!(exclusion.should_exclude(1, 1));
    assert!(exclusion.should_exclude(2, 2));
}

#[test]
fn test_disconnected_graph() {
    // Two disconnected points 0, 1 and a triangle 2-3-4
    // Indices only cover the triangle
    let indices = vec![2u32, 3, 4];
    // Particle count is 5 (0..4)
    let exclusion = TopologyExclusion::new(&indices, 5, 1);

    // 0 and 1 have no neighbors
    assert!(exclusion.should_exclude(0, 0)); // Self check
    assert!(!exclusion.should_exclude(0, 1)); // No link
    assert!(!exclusion.should_exclude(0, 2));

    // Triangle still works
    assert!(exclusion.should_exclude(2, 3));
}

#[test]
fn test_high_ring_depth() {
    // Chain: 0-1-2-3-4
    // Indices: (0,1,0), (1,2,1), ... (degenerate triangles for edges)
    // Real topology needs triangles, let's just make a strip:
    // 0-1-2
    // 2-3-4 (sharing 2? no, let's make a strip of triangles)
    // (0,1,2), (1,2,3), (2,3,4)
    // 0 connects to 1,2
    // 1 connects to 0,2,3
    // 2 connects to 0,1,3,4
    // 3 connects to 1,2,4
    // 4 connects to 2,3

    let indices = vec![
        0, 1, 2,
        1, 2, 3,
        2, 3, 4
    ];
    let exclusion = TopologyExclusion::new(&indices, 5, 3); // Depth 3

    // 0 to 3: distance is 2 hops (0->1->3). Should be excluded.
    assert!(exclusion.should_exclude(0, 3));

    // 0 to 4: distance is 3 hops (0->2->4 or 0->1->3->4). Should be excluded.
    assert!(exclusion.should_exclude(0, 4));
}
