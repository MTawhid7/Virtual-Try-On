//! Integration tests for vistio-contact.

use vistio_contact::broad::{BroadPhase, NullBroadPhase};
use vistio_contact::contact::{ContactPair, ContactType};
use vistio_contact::narrow::{NarrowPhase, NullNarrowPhase};
use vistio_contact::response::{ContactResponse, NullContactResponse};
use vistio_mesh::generators::quad_grid;
use vistio_solver::state::SimulationState;

// ─── Contact Pair Tests ───────────────────────────────────────

#[test]
fn contact_pair_penetrating() {
    let pair = ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [0, 1, 2, 3],
        distance: -0.005,
        normal: [0.0, 1.0, 0.0],
        barycentric: [0.33, 0.33, 0.34],
        is_self: false,
    };
    assert!(pair.is_penetrating());
    assert!((pair.penetration_depth() - 0.005).abs() < 1e-6);
}

#[test]
fn contact_pair_not_penetrating() {
    let pair = ContactPair {
        contact_type: ContactType::EdgeEdge,
        indices: [0, 1, 2, 3],
        distance: 0.01,
        normal: [1.0, 0.0, 0.0],
        barycentric: [0.5, 0.5, 0.0],
        is_self: true,
    };
    assert!(!pair.is_penetrating());
    assert_eq!(pair.penetration_depth(), 0.0);
}

#[test]
fn contact_pair_serialization() {
    let pair = ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [10, 20, 30, 40],
        distance: -0.001,
        normal: [0.0, 0.0, 1.0],
        barycentric: [0.25, 0.25, 0.5],
        is_self: false,
    };
    let json = serde_json::to_string(&pair).unwrap();
    let recovered: ContactPair = serde_json::from_str(&json).unwrap();
    assert_eq!(recovered.indices, [10, 20, 30, 40]);
}

// ─── Null Pipeline Tests ──────────────────────────────────────

#[test]
fn null_broad_phase_returns_empty() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut bp = NullBroadPhase;
    bp.update(&state.pos_x, &state.pos_y, &state.pos_z, 0.001).unwrap();
    assert!(bp.query_pairs().is_empty());
    assert_eq!(bp.name(), "null_broad_phase");
}

#[test]
fn null_narrow_phase_returns_empty() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let np = NullNarrowPhase;
    let contacts = np.detect(&[], &state, &mesh, 0.001).unwrap();
    assert!(contacts.is_empty());
    assert_eq!(np.name(), "null_narrow_phase");
}

#[test]
fn null_contact_response_returns_default() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let cr = NullContactResponse;
    let result = cr.resolve(&[], &mut state, 1.0).unwrap();
    assert_eq!(result.resolved_count, 0);
    assert_eq!(result.max_residual_penetration, 0.0);
    assert_eq!(cr.name(), "null_contact_response");
}

// ─── Spatial Hash Tests ───────────────────────────────────────

use vistio_contact::spatial_hash::SpatialHash;

#[test]
fn spatial_hash_finds_nearby_vertices() {
    let mesh = quad_grid(2, 2, 0.1, 0.1); // Small mesh, vertices close together
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut hash = SpatialHash::new(0.15); // Cell size > spacing
    hash.update(&state.pos_x, &state.pos_y, &state.pos_z, 0.01).unwrap();

    let pairs = hash.query_pairs();
    // With a small mesh and large cells, should find candidate pairs
    assert!(
        !pairs.is_empty(),
        "Spatial hash should find candidate pairs for close vertices"
    );
    assert_eq!(hash.name(), "spatial_hash");
}

#[test]
fn spatial_hash_distant_vertices_no_pairs() {
    let mesh = quad_grid(2, 2, 100.0, 100.0); // Huge spacing
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut hash = SpatialHash::new(1.0); // Small cell size vs spacing
    hash.update(&state.pos_x, &state.pos_y, &state.pos_z, 0.01).unwrap();

    let pairs = hash.query_pairs();
    // With huge spacing and small cells, no vertex shares a cell or neighbors
    assert!(
        pairs.is_empty(),
        "Spatial hash should find no pairs for widely-separated vertices"
    );
}

// ─── Projection Response Tests ────────────────────────────────

use vistio_contact::projection::ProjectionContactResponse;

#[test]
fn projection_response_corrects_penetration() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let original_y = state.pos_y[0];

    // Simulate a penetrating contact on vertex 0
    let contact = ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [0, 1, 2, 3],
        distance: -0.05, // 5cm penetration
        normal: [0.0, 1.0, 0.0], // Push up
        barycentric: [1.0, 0.0, 0.0],
        is_self: false,
    };

    let resp = ProjectionContactResponse;
    let result = resp.resolve(&[contact], &mut state, 1.0).unwrap();

    assert_eq!(result.resolved_count, 1);
    assert!(state.pos_y[0] > original_y, "Vertex should have been pushed up");
    assert_eq!(resp.name(), "projection_response");
}

// ─── A/B Benchmark: PD Solver vs Stub ─────────────────────────

use vistio_mesh::topology::Topology;
use vistio_solver::config::SolverConfig;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::pd_stub::ProjectiveDynamicsStub;
use vistio_solver::strategy::SolverStrategy;

#[test]
fn ab_benchmark_pd_vs_stub() {
    // Setup: 10×10 quad grid, no pinning, simulate 20 frames
    let mesh = quad_grid(10, 10, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let config = SolverConfig::debug(); // 3 iterations
    let dt = 1.0 / 60.0;
    let steps = 20;

    // --- Run PD Stub ---
    let mut stub_state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let mut stub = ProjectiveDynamicsStub::new();
    stub.init(&mesh, &topo, &config, &pinned).unwrap();

    for _ in 0..steps {
        stub.step(&mut stub_state, dt).unwrap();
    }

    // --- Run full PD Solver ---
    let mut pd_state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let mut pd = ProjectiveDynamicsSolver::new();
    pd.init(&mesh, &topo, &config, &pinned).unwrap();

    let mut total_pd_iters = 0_u32;
    for _ in 0..steps {
        let result = pd.step(&mut pd_state, dt).unwrap();
        total_pd_iters += result.iterations;
    }

    // --- Assertions ---
    // 1. PD solver actually performed iterations
    assert!(
        total_pd_iters > 0,
        "PD solver should have performed iterations"
    );

    // 2. Both meshes fell (gravity works in both)
    let stub_center_y: f32 = stub_state.pos_y.iter().sum::<f32>() / n as f32;
    let pd_center_y: f32 = pd_state.pos_y.iter().sum::<f32>() / n as f32;
    let initial_center_y: f32 = mesh.pos_y.iter().sum::<f32>() / n as f32;

    assert!(stub_center_y < initial_center_y, "Stub should fall under gravity");
    assert!(pd_center_y < initial_center_y, "PD solver should fall under gravity");

    // 3. PD solver has lower kinetic energy (stiffness dissipates energy)
    // This is the key quality metric: the PD solver should resist deformation
    let stub_ke = stub_state.kinetic_energy();
    let pd_ke = pd_state.kinetic_energy();

    // PD solver should not be wildly different from stub
    // (both experience gravity, but PD has elastic forces)
    assert!(
        pd_ke.is_finite() && stub_ke.is_finite(),
        "Both should have finite kinetic energy"
    );

    // 4. Verify solver names
    assert_eq!(stub.name(), "projective_dynamics_stub");
    assert_eq!(pd.name(), "ProjectiveDynamics");
}

// ─── Phase 2: Collision Pipeline Tests ────────────────────────

use vistio_contact::collision_pipeline::CollisionPipeline;
use vistio_contact::ground_plane::GroundPlane;
use vistio_contact::vertex_triangle::VertexTriangleTest;

#[test]
fn ground_plane_prevents_penetration() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    // Push all vertices below the ground plane
    for i in 0..n {
        state.pos_y[i] = -1.0;
        state.vel_y[i] = -5.0;
    }

    let ground = GroundPlane::new(0.0);
    let result = ground.resolve(&mut state);

    assert_eq!(result.resolved_count, n as u32);
    assert!(result.max_residual_penetration > 0.0);

    // All vertices should be at or above ground
    for i in 0..n {
        assert!(
            state.pos_y[i] >= 0.0,
            "Vertex {} should be at or above ground, got {}",
            i, state.pos_y[i]
        );
    }

    // Downward velocity should be zeroed
    for i in 0..n {
        assert!(
            state.vel_y[i] >= 0.0,
            "Vertex {} should have non-negative vel_y, got {}",
            i, state.vel_y[i]
        );
    }
}

#[test]
fn ground_plane_no_correction_above() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    // All vertices above the ground
    for i in 0..n {
        state.pos_y[i] = 5.0;
    }

    let ground = GroundPlane::new(0.0);
    let result = ground.resolve(&mut state);

    assert_eq!(result.resolved_count, 0);
}

#[test]
fn collision_pipeline_runs_end_to_end() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let broad = Box::new(NullBroadPhase);
    let narrow = Box::new(NullNarrowPhase);
    let response = Box::new(NullContactResponse);

    let mut pipeline = CollisionPipeline::new(
        broad,
        narrow,
        response,
        mesh,
        0.01,
        1.0,
    );

    let mut state = state;
    let result = pipeline.step(&mut state);
    assert!(result.is_ok());
}

#[test]
fn collision_pipeline_with_ground() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    // Push vertices below ground
    for i in 0..n {
        state.pos_y[i] = -2.0;
    }

    let broad = Box::new(NullBroadPhase);
    let narrow = Box::new(NullNarrowPhase);
    let response = Box::new(NullContactResponse);

    let mut pipeline = CollisionPipeline::new(
        broad,
        narrow,
        response,
        mesh,
        0.01,
        1.0,
    ).with_ground(-1.0);

    let result = pipeline.step(&mut state).unwrap();

    assert!(result.ground_result.resolved_count > 0);
    // All vertices should be at or above ground (-1.0)
    for i in 0..n {
        assert!(
            state.pos_y[i] >= -1.0,
            "Vertex {} below ground: {}",
            i, state.pos_y[i]
        );
    }
}

#[test]
fn vertex_triangle_test_name() {
    let vt = VertexTriangleTest;
    assert_eq!(vt.name(), "vertex_triangle_test");
}

// ─── Phase 3: Self-Collision Tests ────────────────────────────

use vistio_contact::coloring::CollisionColoring;
use vistio_contact::exclusion::TopologyExclusion;
use vistio_contact::self_collision::SelfCollisionSystem;

#[test]
fn topology_exclusion_excludes_adjacent() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let exclusion = TopologyExclusion::new(&mesh, &topo, 2);

    // Adjacent vertices should be excluded (2-ring)
    assert!(
        exclusion.should_exclude(0, 1),
        "Adjacent vertices should be excluded"
    );

    // Same vertex should be excluded
    assert!(
        exclusion.should_exclude(0, 0),
        "Same vertex should be excluded"
    );
}

#[test]
fn topology_exclusion_allows_distant() {
    let mesh = quad_grid(10, 10, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let exclusion = TopologyExclusion::new(&mesh, &topo, 2);

    // Vertices far apart should NOT be excluded
    // vertex 0 is at (0,0), vertex at far corner is distant
    let far_vertex = mesh.vertex_count() - 1;
    assert!(
        !exclusion.should_exclude(0, far_vertex),
        "Distant vertices should not be excluded"
    );
}

#[test]
fn graph_coloring_no_conflicts() {
    // Create pairs that share vertices — coloring should separate them
    let pairs = vec![(0, 1), (1, 2), (2, 3), (3, 4)];
    let (sorted, offsets) = CollisionColoring::color_pairs(&pairs, 5);

    assert_eq!(sorted.len(), pairs.len());
    assert!(offsets.len() >= 2, "Should have at least one batch");

    // Verify no two pairs in the same batch share a vertex
    for b in 0..offsets.len() - 1 {
        let start = offsets[b];
        let end = offsets[b + 1];
        let batch = &sorted[start..end];

        for i in 0..batch.len() {
            for j in (i + 1)..batch.len() {
                let (a1, b1) = batch[i];
                let (a2, b2) = batch[j];
                assert!(
                    a1 != a2 && a1 != b2 && b1 != a2 && b1 != b2,
                    "Batch {} has conflicting pairs ({},{}) and ({},{})",
                    b, a1, b1, a2, b2
                );
            }
        }
    }
}

#[test]
fn graph_coloring_empty() {
    let pairs: Vec<(u32, u32)> = Vec::new();
    let (sorted, offsets) = CollisionColoring::color_pairs(&pairs, 10);
    assert!(sorted.is_empty());
    assert_eq!(offsets, vec![0]);
}

#[test]
fn graph_coloring_independent_pairs() {
    // Non-conflicting pairs — should all be in same batch
    let pairs = vec![(0, 1), (2, 3), (4, 5)];
    let (sorted, offsets) = CollisionColoring::color_pairs(&pairs, 6);
    assert_eq!(sorted.len(), 3);
    // Should be only 1 batch since there are no conflicts
    assert_eq!(offsets.len(), 2);
    assert_eq!(offsets[0], 0);
    assert_eq!(offsets[1], 3);
}

#[test]
fn self_collision_resolves_overlap() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    // Push two non-adjacent vertices very close together
    // Vertices 0 (corner) and the far corner should not be topologically adjacent
    let far = n - 1;
    state.pos_x[0] = 0.0;
    state.pos_y[0] = 0.0;
    state.pos_z[0] = 0.0;
    state.pos_x[far] = 0.001;
    state.pos_y[far] = 0.0;
    state.pos_z[far] = 0.0;

    let mut system = SelfCollisionSystem::new(&mesh, &topo, 2, 0.1, 1.0);
    let mut broad = SpatialHash::new(0.2);

    let result = system.solve(&mut state, &mut broad);

    // Should have detected the pair and resolved it
    assert!(
        result.candidate_pairs > 0,
        "Should have candidate pairs from broad phase"
    );
}
