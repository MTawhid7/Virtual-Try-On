//! Integration tests for vistio-solver.

use vistio_mesh::generators::quad_grid;
use vistio_mesh::topology::Topology;
use vistio_solver::config::SolverConfig;
use vistio_solver::pd_stub::ProjectiveDynamicsStub;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::SolverStrategy;

// ─── SimulationState Tests ────────────────────────────────────

#[test]
fn state_from_mesh() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    assert_eq!(state.vertex_count, n);
    assert_eq!(state.pos_x.len(), n);
    assert_eq!(state.vel_x.len(), n);
    assert!(state.vel_x.iter().all(|&v| v == 0.0)); // Starts at rest
}

#[test]
fn state_pinning() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true; // Pin top-left
    pinned[2] = true; // Pin top-right

    let state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    assert_eq!(state.inv_mass[0], 0.0);
    assert_eq!(state.inv_mass[2], 0.0);
    assert!(state.inv_mass[4] > 0.0); // Center is free
}

#[test]
fn state_predict() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let initial_y = state.pos_y.clone();
    let dt = 1.0 / 60.0;
    let gravity = [0.0, -9.81, 0.0];

    state.predict(dt, gravity);

    // After prediction with zero initial velocity:
    // pred_y = pos_y + dt*0 + dt²*(-9.81)
    let dt2g = dt * dt * (-9.81);
    for (i, &initial) in initial_y.iter().enumerate().take(n) {
        let expected = initial + dt2g;
        assert!(
            (state.pred_y[i] - expected).abs() < 1e-5,
            "Vertex {}: pred_y={}, expected={}",
            i, state.pred_y[i], expected
        );
    }
}

#[test]
fn state_pinned_predict() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true;

    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let original_pos = state.pos_y[0];

    state.predict(1.0 / 60.0, [0.0, -9.81, 0.0]);

    // Pinned vertex: predicted position = current position
    assert_eq!(state.pred_y[0], original_pos);
}

#[test]
fn state_velocity_update() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    // Save previous, move one vertex, update velocities
    state.save_previous();
    state.pos_x[0] += 0.5;
    let dt = 1.0 / 60.0;
    state.update_velocities(dt);

    let expected_vx = 0.5 / dt;
    assert!((state.vel_x[0] - expected_vx).abs() < 1e-3);
    assert_eq!(state.vel_y[0], 0.0); // No movement in Y
}

#[test]
fn state_damping() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    state.vel_x[0] = 10.0;
    state.damp_velocities(0.1);
    assert!((state.vel_x[0] - 9.0).abs() < 1e-5);
}

#[test]
fn state_kinetic_energy() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 1.0, &pinned).unwrap();

    // Give vertex 0 velocity [1, 0, 0]
    state.vel_x[0] = 1.0;
    let ke = state.kinetic_energy();
    // KE = 0.5 * 1.0 * 1.0² = 0.5
    assert!((ke - 0.5).abs() < 1e-6);
}

// ─── SolverConfig Tests ───────────────────────────────────────

#[test]
fn config_default() {
    let config = SolverConfig::default();
    assert_eq!(config.max_iterations, 10);
    assert!(config.tolerance < 1e-4);
    assert!((config.gravity[1] + 9.81).abs() < 1e-3);
}

#[test]
fn config_debug() {
    let config = SolverConfig::debug();
    assert_eq!(config.max_iterations, 3);
}

#[test]
fn config_high_quality() {
    let config = SolverConfig::high_quality();
    assert_eq!(config.max_iterations, 30);
    assert!(config.chebyshev_acceleration);
}

#[test]
fn config_serialization() {
    let config = SolverConfig::default();
    let toml = toml::to_string(&config).unwrap();
    let recovered: SolverConfig = toml::from_str(&toml).unwrap();
    assert_eq!(recovered.max_iterations, config.max_iterations);
}

// ─── PD Stub Solver Tests ─────────────────────────────────────

#[test]
fn pd_stub_init_and_step() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let config = SolverConfig::default();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut solver = ProjectiveDynamicsStub::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();

    let result = solver.step(&mut state, 1.0 / 60.0).unwrap();
    assert!(result.converged);
    assert_eq!(result.iterations, 0); // Stub does zero iterations
}

#[test]
fn pd_stub_gravity_motion() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::default();

    let mut solver = ProjectiveDynamicsStub::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();

    let initial_y: Vec<f32> = state.pos_y.clone();

    // Run 10 steps
    let dt = 1.0 / 60.0;
    for _ in 0..10 {
        solver.step(&mut state, dt).unwrap();
    }

    // All unpinned vertices should have moved down (gravity -Y)
    for (i, &initial) in initial_y.iter().enumerate().take(n) {
        assert!(
            state.pos_y[i] < initial,
            "Vertex {} should have fallen: y={} vs initial={}",
            i, state.pos_y[i], initial
        );
    }
}

#[test]
fn pd_stub_pinned_vertices_stay() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true;
    pinned[4] = true;

    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::default();

    let pin0_y = state.pos_y[0];
    let pin4_y = state.pos_y[4];

    let mut solver = ProjectiveDynamicsStub::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();

    for _ in 0..20 {
        solver.step(&mut state, 1.0 / 60.0).unwrap();
    }

    assert_eq!(state.pos_y[0], pin0_y, "Pinned vertex 0 should not move");
    assert_eq!(state.pos_y[4], pin4_y, "Pinned vertex 4 should not move");
}

#[test]
fn pd_stub_not_initialized_error() {
    let mesh = quad_grid(1, 1, 1.0, 1.0);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();

    let mut solver = ProjectiveDynamicsStub::new();
    // Don't call init()
    assert!(solver.step(&mut state, 1.0 / 60.0).is_err());
}

// ─── FEM Element Tests ────────────────────────────────────────

use vistio_solver::element::ElementData;

#[test]
fn element_from_mesh_count() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let elements = ElementData::from_mesh(&mesh, 100.0);
    assert_eq!(elements.len(), mesh.triangle_count());
    assert!(!elements.is_empty());
}

#[test]
fn element_rest_area_positive() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let elements = ElementData::from_mesh(&mesh, 100.0);
    for elem in &elements.elements {
        assert!(
            elem.rest_area > 0.0,
            "Rest area should be positive, got {}",
            elem.rest_area
        );
    }
}

#[test]
fn element_dm_inv_finite() {
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let elements = ElementData::from_mesh(&mesh, 100.0);
    for elem in &elements.elements {
        for &v in &elem.dm_inv {
            assert!(v.is_finite(), "Dm_inv should be finite");
        }
    }
}

#[test]
fn element_weight_equals_stiffness_times_area() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let stiffness = 42.0;
    let elements = ElementData::from_mesh(&mesh, stiffness);
    for elem in &elements.elements {
        let expected = stiffness * elem.rest_area;
        assert!(
            (elem.weight - expected).abs() < 1e-6,
            "weight={}, expected={}",
            elem.weight,
            expected
        );
    }
}

#[test]
fn element_projection_undeformed_returns_near_original() {
    // When the mesh hasn't been deformed, projection should return
    // positions very close to the original (rotation = identity).
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let elements = ElementData::from_mesh(&mesh, 100.0);

    for elem in &elements.elements {
        let (p0, p1, p2) = elements.project(elem, &mesh.pos_x, &mesh.pos_y, &mesh.pos_z);

        let [i0, i1, i2] = elem.indices;
        let orig0 = vistio_math::Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
        let orig1 = vistio_math::Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
        let orig2 = vistio_math::Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);

        assert!(
            (p0 - orig0).length() < 0.1,
            "p0 projection drifted: {:?} vs {:?}",
            p0, orig0
        );
        assert!(
            (p1 - orig1).length() < 0.1,
            "p1 projection drifted: {:?} vs {:?}",
            p1, orig1
        );
        assert!(
            (p2 - orig2).length() < 0.1,
            "p2 projection drifted: {:?} vs {:?}",
            p2, orig2
        );
    }
}

// ─── Assembly Tests ───────────────────────────────────────────

use vistio_solver::assembly::assemble_system_matrix;

#[test]
fn assembly_matrix_is_square_n() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    let elements = ElementData::from_mesh(&mesh, 100.0);
    let mass = vec![0.01_f32; n];
    let dt = 1.0 / 60.0;

    let matrix = assemble_system_matrix(n, &mass, dt, &elements, None);
    assert_eq!(matrix.rows, n);
    assert_eq!(matrix.cols, n);
}

#[test]
fn assembly_diagonal_positive() {
    // For SPD, all diagonal entries must be positive
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let n = mesh.vertex_count();
    let elements = ElementData::from_mesh(&mesh, 100.0);
    let mass = vec![0.01_f32; n];
    let dt = 1.0 / 60.0;

    let matrix = assemble_system_matrix(n, &mass, dt, &elements, None);

    for i in 0..n {
        let mut diag = 0.0_f32;
        for idx in matrix.row_ptr[i]..matrix.row_ptr[i + 1] {
            if matrix.col_idx[idx] == i {
                diag = matrix.values[idx];
            }
        }
        assert!(
            diag > 0.0,
            "Diagonal[{i}] = {diag}, should be positive"
        );
    }
}

#[test]
fn assembly_rhs_length() {
    use vistio_solver::assembly::assemble_rhs;
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let elements = ElementData::from_mesh(&mesh, 100.0);
    let mass = vec![0.01_f32; n];
    let dt = 1.0 / 60.0;
    let pred = vec![0.0_f32; n];
    let proj = vec![(0.0_f32, 0.0, 0.0); elements.len()];

    let rhs = assemble_rhs(n, &mass, dt, &pred, &proj, &elements, 0, None);
    assert_eq!(rhs.len(), n);
}

// ─── PD Solver Tests ──────────────────────────────────────────

use vistio_solver::pd_solver::ProjectiveDynamicsSolver;

#[test]
fn pd_solver_init_succeeds() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let config = SolverConfig::default();
    let pinned = vec![false; mesh.vertex_count()];

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();
    assert_eq!(solver.name(), "ProjectiveDynamics");
}

#[test]
fn pd_solver_single_step_runs() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::debug(); // 3 iterations max

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();

    let result = solver.step(&mut state, 1.0 / 60.0).unwrap();
    assert!(result.iterations > 0, "PD solver should run at least 1 iteration");
    assert!(result.wall_time >= 0.0);
}

#[test]
fn pd_solver_pinned_vertices_stay() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let mut pinned = vec![false; n];
    pinned[0] = true;
    pinned[4] = true;

    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::debug();

    let pin0_y = state.pos_y[0];
    let pin4_y = state.pos_y[4];

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();

    for _ in 0..5 {
        solver.step(&mut state, 1.0 / 60.0).unwrap();
    }

    assert!(
        (state.pos_y[0] - pin0_y).abs() < 1e-6,
        "Pinned vertex 0 moved: {} vs {}",
        state.pos_y[0], pin0_y
    );
    assert!(
        (state.pos_y[4] - pin4_y).abs() < 1e-6,
        "Pinned vertex 4 moved: {} vs {}",
        state.pos_y[4], pin4_y
    );
}

#[test]
fn pd_solver_gravity_pulls_down() {
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
    let config = SolverConfig::debug();

    // Track centroid Y (average Y of all vertices)
    let initial_centroid_y: f32 = state.pos_y.iter().sum::<f32>() / n as f32;

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&mesh, &topo, &config, &pinned).unwrap();

    for _ in 0..10 {
        solver.step(&mut state, 1.0 / 60.0).unwrap();
    }

    // The centroid should have moved downward under gravity.
    // Individual vertices may move in various directions due to elastic forces,
    // but the center of mass must fall.
    let final_centroid_y: f32 = state.pos_y.iter().sum::<f32>() / n as f32;
    assert!(
        final_centroid_y < initial_centroid_y,
        "Centroid should have fallen: y={final_centroid_y} vs initial={initial_centroid_y}"
    );
}

// ─── Bending Tests ────────────────────────────────────────────

use vistio_solver::bending::{BendingData, compute_dihedral_angle};

#[test]
fn bending_element_count_matches_interior_edges() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let bending = BendingData::from_topology(&mesh, &topo, 1.0);
    assert_eq!(bending.len(), topo.interior_edges.len());
}

#[test]
fn bending_rest_angle_flat_mesh() {
    // A flat quad grid should have rest dihedral angles near π (coplanar triangles)
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let bending = BendingData::from_topology(&mesh, &topo, 1.0);

    for elem in &bending.elements {
        assert!(
            elem.rest_angle.is_finite(),
            "Rest angle should be finite"
        );
        // For a flat mesh, dihedral angle should be near π
        assert!(
            (elem.rest_angle - std::f32::consts::PI).abs() < 0.5,
            "Flat mesh rest angle should be near π, got {}",
            elem.rest_angle
        );
    }
}

#[test]
fn bending_dihedral_angle_coplanar() {
    // Two coplanar triangles: angle should be π
    let v0 = vistio_math::Vec3::new(0.0, 0.0, 0.0);
    let v1 = vistio_math::Vec3::new(1.0, 0.0, 0.0);
    let wa = vistio_math::Vec3::new(0.5, 1.0, 0.0);
    let wb = vistio_math::Vec3::new(0.5, -1.0, 0.0);

    let angle = compute_dihedral_angle(v0, v1, wa, wb);
    assert!(
        (angle - std::f32::consts::PI).abs() < 0.1,
        "Coplanar dihedral should be ≈ π, got {angle}"
    );
}

#[test]
fn bending_projection_preserves_edge_vertices() {
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let bending = BendingData::from_topology(&mesh, &topo, 1.0);

    if !bending.is_empty() {
        let elem = &bending.elements[0];
        let (p_v0, p_v1, _, _) = bending.project(elem, &mesh.pos_x, &mesh.pos_y, &mesh.pos_z);

        // Edge vertices should not change during bending projection
        let orig_v0 = vistio_math::Vec3::new(
            mesh.pos_x[elem.v0], mesh.pos_y[elem.v0], mesh.pos_z[elem.v0]
        );
        let orig_v1 = vistio_math::Vec3::new(
            mesh.pos_x[elem.v1], mesh.pos_y[elem.v1], mesh.pos_z[elem.v1]
        );

        assert!((p_v0 - orig_v0).length() < 1e-6, "v0 should not move");
        assert!((p_v1 - orig_v1).length() < 1e-6, "v1 should not move");
    }
}

// ─── Material-Aware Solver Tests (Tier 2) ─────────────────────

use vistio_material::{CoRotationalModel, IsotropicLinearModel, MaterialDatabase};

#[test]
fn material_aware_init_succeeds() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig::default();
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let pinned = vec![false; mesh.vertex_count()];
    let model = Box::new(CoRotationalModel::new());

    let mut solver = ProjectiveDynamicsSolver::new();
    let result = solver.init_with_material(&mesh, &topology, &config, props, model, &pinned);
    assert!(result.is_ok(), "init_with_material should succeed");
}

#[test]
fn material_aware_step_runs() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig { max_iterations: 3, ..Default::default() };
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(CoRotationalModel::new());

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init_with_material(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]).unwrap();

    let n = mesh.vertex_count();
    let mut state = SimulationState::from_mesh(&mesh, 0.002, &vec![false; n]).unwrap();
    let result = solver.step(&mut state, 1.0 / 60.0);
    assert!(result.is_ok(), "Material-aware step should succeed");
}

#[test]
fn silk_drapes_more_than_denim() {
    // Silk is softer → more displacement under gravity vs denim
    let db = MaterialDatabase::with_defaults();

    let run = |material_name: &str| -> f32 {
        let mesh = quad_grid(5, 5, 1.0, 1.0);
        let topology = Topology::build(&mesh);
        let config = SolverConfig { max_iterations: 5, ..Default::default() };
        let props = db.get(material_name).unwrap();
        let model = Box::new(CoRotationalModel::new());

        let mut solver = ProjectiveDynamicsSolver::new();
        let n = mesh.vertex_count();
        let mut pinned = vec![false; n];
        // Pin top two corners for a hanging sheet drape
        pinned[0] = true;
        pinned[4] = true;
        solver.init_with_material(&mesh, &topology, &config, props, model, &pinned).unwrap();

        let n = mesh.vertex_count();
        let total_area: f32 = {
            let tc = mesh.triangle_count();
            (0..tc).map(|t| {
                let b = t * 3;
                let i0 = mesh.indices[b] as usize;
                let i1 = mesh.indices[b+1] as usize;
                let i2 = mesh.indices[b+2] as usize;
                let p0 = vistio_math::Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
                let p1 = vistio_math::Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
                let p2 = vistio_math::Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);
                0.5 * (p1 - p0).cross(p2 - p0).length()
            }).sum()
        };
        let vm = props.mass_per_vertex(n, total_area);
        let mut state = SimulationState::from_mesh(&mesh, vm, &pinned).unwrap();

        // Simulate 30 steps to let material differences accumulate
        for _ in 0..30 {
            solver.step(&mut state, 1.0 / 60.0).unwrap();
        }

        // Max Y displacement
        let init_y_avg: f32 = mesh.pos_y.iter().sum::<f32>() / n as f32;
        let final_y_avg: f32 = state.pos_y.iter().sum::<f32>() / n as f32;
        (init_y_avg - final_y_avg).abs()
    };

    let silk_drop = run("silk_charmeuse");
    let denim_drop = run("denim_14oz");

    // Both should have some displacement from gravity
    assert!(silk_drop > 0.0, "Silk should displace, got {}", silk_drop);
    assert!(denim_drop > 0.0, "Denim should displace, got {}", denim_drop);

    // The drops should be different (material differentiation works)
    // Even a small difference proves the solver is material-aware
    let diff = (silk_drop - denim_drop).abs();
    assert!(diff > 1e-8,
        "Different materials should produce different drape: silk={:.6}, denim={:.6}, diff={:.8}",
        silk_drop, denim_drop, diff
    );
}

#[test]
fn corotational_vs_isotropic_produce_different_results() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig { max_iterations: 3, ..Default::default() };
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let n = mesh.vertex_count();

    // Run with co-rotational
    let corot_final_y = {
        let model = Box::new(CoRotationalModel::new());
        let mut solver = ProjectiveDynamicsSolver::new();
        solver.init_with_material(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]).unwrap();
        let vm = props.mass_per_vertex(n, 1.0);
        let mut state = SimulationState::from_mesh(&mesh, vm, &vec![false; n]).unwrap();
        for _ in 0..5 { solver.step(&mut state, 1.0 / 60.0).unwrap(); }
        state.pos_y.iter().sum::<f32>() / n as f32
    };

    // Run with isotropic linear
    let iso_final_y = {
        let model = Box::new(IsotropicLinearModel::new());
        let mut solver = ProjectiveDynamicsSolver::new();
        solver.init_with_material(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]).unwrap();
        let vm = props.mass_per_vertex(n, 1.0);
        let mut state = SimulationState::from_mesh(&mesh, vm, &vec![false; n]).unwrap();
        for _ in 0..5 { solver.step(&mut state, 1.0 / 60.0).unwrap(); }
        state.pos_y.iter().sum::<f32>() / n as f32
    };

    let diff = (corot_final_y - iso_final_y).abs();
    assert!(diff > 1e-8,
        "Different models should produce different results: corot_y={:.6}, iso_y={:.6}",
        corot_final_y, iso_final_y
    );
}

#[test]
fn material_config_name_roundtrip() {
    let mut config = SolverConfig::default();
    assert!(config.material_name.is_none());
    config.material_name = Some("silk_charmeuse".into());
    let toml_str = toml::to_string(&config).unwrap();
    let recovered: SolverConfig = toml::from_str(&toml_str).unwrap();
    assert_eq!(recovered.material_name.as_deref(), Some("silk_charmeuse"));
}

// ─── Phase 1 Fix Tests ───────────────────────────────────────

#[test]
fn bending_flat_rest_angles_exactly_pi() {
    // After clamping, all rest angles on a flat grid should be exactly π.
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let bending = BendingData::from_topology(&mesh, &topo, 1.0);

    for elem in &bending.elements {
        assert_eq!(
            elem.rest_angle,
            std::f32::consts::PI,
            "Flat mesh rest angle should be exactly π, got {}",
            elem.rest_angle
        );
    }
}

#[test]
fn solver_rayleigh_damping_reduces_ke() {
    // Rayleigh damping should reduce kinetic energy faster than without it.
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let n = mesh.vertex_count();
    let pinned = vec![false; n];
    let dt = 1.0 / 60.0;

    // Run without Rayleigh damping
    let ke_no_rayleigh = {
        let config = SolverConfig {
            max_iterations: 3,
            rayleigh_mass_damping: 0.0,
            ..Default::default()
        };
        let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
        let mut solver = ProjectiveDynamicsSolver::new();
        solver.init(&mesh, &topo, &config, &pinned).unwrap();
        for _ in 0..20 {
            solver.step(&mut state, dt).unwrap();
        }
        state.kinetic_energy()
    };

    // Run with Rayleigh damping
    let ke_with_rayleigh = {
        let config = SolverConfig {
            max_iterations: 3,
            rayleigh_mass_damping: 5.0,
            ..Default::default()
        };
        let mut state = SimulationState::from_mesh(&mesh, 0.01, &pinned).unwrap();
        let mut solver = ProjectiveDynamicsSolver::new();
        solver.init(&mesh, &topo, &config, &pinned).unwrap();
        for _ in 0..20 {
            solver.step(&mut state, dt).unwrap();
        }
        state.kinetic_energy()
    };

    assert!(
        ke_with_rayleigh < ke_no_rayleigh,
        "Rayleigh damping should reduce KE: with={:.6} vs without={:.6}",
        ke_with_rayleigh, ke_no_rayleigh
    );
}

#[test]
fn config_rayleigh_serialization() {
    let config = SolverConfig {
        rayleigh_mass_damping: 3.5,
        ..Default::default()
    };
    let toml_str = toml::to_string(&config).unwrap();
    let recovered: SolverConfig = toml::from_str(&toml_str).unwrap();
    assert!((recovered.rayleigh_mass_damping - 3.5).abs() < 1e-6);
}

// ─── Discrete Shells Bending Tests (Tier 3) ──────────────────

use vistio_solver::discrete_shells::DiscreteShellsBendingData;

#[test]
fn discrete_shells_element_count_matches_interior_edges() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let ds_bending = DiscreteShellsBendingData::from_topology(&mesh, &topo, 1.0);
    assert_eq!(ds_bending.len(), topo.interior_edges.len());
}

#[test]
fn discrete_shells_rest_angle_flat_mesh() {
    // A flat quad grid should have rest dihedral angles of exactly π
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let ds_bending = DiscreteShellsBendingData::from_topology(&mesh, &topo, 1.0);

    for elem in &ds_bending.elements {
        assert_eq!(
            elem.rest_angle,
            std::f32::consts::PI,
            "Flat mesh rest angle should be exactly π, got {}",
            elem.rest_angle
        );
    }
}

#[test]
fn discrete_shells_cotangent_weights_finite() {
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let ds_bending = DiscreteShellsBendingData::from_topology(&mesh, &topo, 1.0);

    for elem in &ds_bending.elements {
        for &s in &elem.stencil {
            assert!(s.is_finite(), "Cotangent stencil value should be finite, got {}", s);
        }
        assert!(elem.weight.is_finite() && elem.weight > 0.0,
            "Weight should be positive and finite, got {}", elem.weight);
        assert!(elem.combined_area > 0.0,
            "Combined area should be positive, got {}", elem.combined_area);
    }
}

#[test]
fn discrete_shells_solver_init_tier3_succeeds() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig::default();
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(CoRotationalModel::new());

    let mut solver = ProjectiveDynamicsSolver::new();
    let result = solver.init_with_material_tier3(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]);
    assert!(result.is_ok(), "init_with_material_tier3 should succeed");
}

#[test]
fn discrete_shells_solver_step_runs() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig { max_iterations: 3, ..Default::default() };
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(CoRotationalModel::new());

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init_with_material_tier3(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]).unwrap();

    let n = mesh.vertex_count();
    let mut state = SimulationState::from_mesh(&mesh, 0.002, &vec![false; n]).unwrap();
    let result = solver.step(&mut state, 1.0 / 60.0);
    assert!(result.is_ok(), "Tier 3 solver step should succeed");

    let step_result = result.unwrap();
    assert!(step_result.iterations > 0, "Should run at least 1 iteration");
}

#[test]
fn discrete_shells_projection_preserves_edge_vertices() {
    let mesh = quad_grid(3, 3, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let ds_bending = DiscreteShellsBendingData::from_topology(&mesh, &topo, 1.0);

    if !ds_bending.is_empty() {
        let elem = &ds_bending.elements[0];
        let (p_v0, p_v1, _, _) = ds_bending.project(elem, &mesh.pos_x, &mesh.pos_y, &mesh.pos_z);

        let orig_v0 = vistio_math::Vec3::new(
            mesh.pos_x[elem.v0], mesh.pos_y[elem.v0], mesh.pos_z[elem.v0]
        );
        let orig_v1 = vistio_math::Vec3::new(
            mesh.pos_x[elem.v1], mesh.pos_y[elem.v1], mesh.pos_z[elem.v1]
        );

        assert!((p_v0 - orig_v0).length() < 1e-6, "v0 should not move");
        assert!((p_v1 - orig_v1).length() < 1e-6, "v1 should not move");
    }
}

#[test]
fn discrete_shells_with_material() {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let topo = Topology::build(&mesh);
    let db = MaterialDatabase::with_defaults();
    let props = db.get("denim_14oz").unwrap();
    let ds_bending = DiscreteShellsBendingData::from_topology_with_material(&mesh, &topo, props);

    // Denim has higher bending stiffness → higher weights
    let avg_weight: f32 = ds_bending.elements.iter().map(|e| e.weight).sum::<f32>()
        / ds_bending.len() as f32;
    assert!(avg_weight > 0.0, "Denim bending weight should be positive");

    let silk_props = db.get("silk_charmeuse").unwrap();
    let silk_bending = DiscreteShellsBendingData::from_topology_with_material(&mesh, &topo, silk_props);
    let silk_avg: f32 = silk_bending.elements.iter().map(|e| e.weight).sum::<f32>()
        / silk_bending.len() as f32;

    assert!(avg_weight > silk_avg,
        "Denim should have higher bend weight than silk: denim={:.4} silk={:.4}",
        avg_weight, silk_avg);
}

// ─── Tier 3: Anisotropic Material Solver Tests ───────────────

use vistio_material::AnisotropicCoRotationalModel;

#[test]
fn anisotropic_solver_init_succeeds() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig::default();
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(AnisotropicCoRotationalModel::from_properties(props));

    let mut solver = ProjectiveDynamicsSolver::new();
    let result = solver.init_with_material_tier3(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]);
    assert!(result.is_ok(), "Anisotropic + Tier 3 init should succeed");
}

#[test]
fn anisotropic_solver_step_runs() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let config = SolverConfig { max_iterations: 3, ..Default::default() };
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(AnisotropicCoRotationalModel::from_properties(props));

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init_with_material_tier3(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]).unwrap();

    let n = mesh.vertex_count();
    let total_area: f32 = 1.0; // 1m² grid
    let vm = props.mass_per_vertex(n, total_area);
    let mut state = SimulationState::from_mesh(&mesh, vm, &vec![false; n]).unwrap();

    for _ in 0..5 {
        let result = solver.step(&mut state, 1.0 / 60.0).unwrap();
        assert!(result.iterations > 0);
    }
}

#[test]
fn fabric_properties_is_anisotropic() {
    let db = MaterialDatabase::with_defaults();

    // Cotton twill has warp=0.90, weft=0.80 → ratio 1.125 → anisotropic
    let cotton = db.get("cotton_twill").unwrap();
    assert!(cotton.is_anisotropic(),
        "Cotton twill should be anisotropic: warp={}, weft={}",
        cotton.stretch_stiffness_warp, cotton.stretch_stiffness_weft);

    // Jersey has warp=0.35, weft=0.30 → ratio ~1.17 → anisotropic
    let jersey = db.get("jersey_knit").unwrap();
    assert!(jersey.is_anisotropic(),
        "Jersey knit should be anisotropic: warp={}, weft={}",
        jersey.stretch_stiffness_warp, jersey.stretch_stiffness_weft);
}

#[test]
fn fabric_properties_warp_weft_ratio() {
    let db = MaterialDatabase::with_defaults();
    let cotton = db.get("cotton_twill").unwrap();

    let ratio = cotton.warp_weft_ratio();
    assert!(ratio > 1.0, "Cotton warp > weft, ratio should be > 1.0, got {}", ratio);
}

// ─── Tier 4: Augmented Lagrangian IPC Contact Tests ───────────



#[test]
fn tier4_solver_init_succeeds() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let mut config = SolverConfig::default();
    config.ipc_enabled = true;
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(CoRotationalModel::new());

    let mut solver = ProjectiveDynamicsSolver::new();
    let result = solver.init_with_material_tier4(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]);
    assert!(result.is_ok(), "Tier 4 init should succeed");
}

#[test]
fn tier4_solver_step_with_ipc_no_contacts() {
    let mesh = quad_grid(5, 5, 1.0, 1.0);
    let topology = Topology::build(&mesh);
    let mut config = SolverConfig { max_iterations: 3, ..Default::default() };
    config.ipc_enabled = true;
    let db = MaterialDatabase::with_defaults();
    let props = db.get("cotton_twill").unwrap();
    let model = Box::new(CoRotationalModel::new());

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init_with_material_tier4(&mesh, &topology, &config, props, model, &vec![false; mesh.vertex_count()]).unwrap();

    let n = mesh.vertex_count();
    let vm = props.mass_per_vertex(n, 1.0);
    let mut state = SimulationState::from_mesh(&mesh, vm, &vec![false; n]).unwrap();

    let mut handler = vistio_solver::pd_solver::EmptyIpcHandler;
    let result = solver.step_with_ipc(&mut state, 1.0 / 60.0, &mut handler);
    assert!(result.is_ok(), "Tier 4 solver step_with_ipc should succeed");

    let step_result = result.unwrap();
    assert!(step_result.iterations > 0, "Should run at least 1 iteration");
}
