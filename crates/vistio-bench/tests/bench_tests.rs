//! Integration tests for vistio-bench.

use vistio_bench::metrics::BenchmarkMetrics;
use vistio_bench::runner::BenchmarkRunner;
use vistio_bench::scenarios::{Scenario, ScenarioKind};
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::SolverStrategy;
use vistio_contact::{SpatialHash, VertexTriangleTest, ProjectionContactResponse};


// ─── Scenario Tests ───────────────────────────────────────────

#[test]
fn hanging_sheet_setup() {
    let s = Scenario::hanging_sheet();
    assert_eq!(s.kind, ScenarioKind::HangingSheet);
    assert_eq!(s.garment.vertex_count(), 441);    // 21×21
    assert_eq!(s.garment.triangle_count(), 800);   // 20×20×2
    // Top row should be pinned (21 vertices)
    let pinned_count = s.pinned.iter().filter(|&&p| p).count();
    assert_eq!(pinned_count, 21);
}

#[test]
fn sphere_drape_setup() {
    let s = Scenario::sphere_drape();
    assert_eq!(s.kind, ScenarioKind::SphereDrape);
    assert!(s.body.is_some());
    let pinned_count = s.pinned.iter().filter(|&&p| p).count();
    assert_eq!(pinned_count, 0); // Free fall
}

#[test]
fn all_scenarios() {
    assert_eq!(ScenarioKind::all().len(), 6);
}

// ─── Runner Tests ─────────────────────────────────────────────

#[test]
fn run_hanging_sheet() {
    let mut scenario = Scenario::hanging_sheet();
    scenario.timesteps = 5; // Very short for testing
    let mut solver = ProjectiveDynamicsSolver::new();
    let metrics = BenchmarkRunner::run(&scenario, &mut solver).unwrap();

    assert_eq!(metrics.scenario, "hanging_sheet");
    assert_eq!(metrics.timesteps, 5);
    assert!(metrics.total_wall_time > 0.0);
    assert!(metrics.max_displacement > 0.0); // Gravity should cause displacement
}

#[test]
fn run_all_scenarios() {
    // Use minimal timesteps for speed
    let mut solver = ProjectiveDynamicsSolver::new();
    let kinds = ScenarioKind::all();
    for &kind in kinds {
        let mut scenario = Scenario::from_kind(kind);
        scenario.timesteps = 3;
        let metrics = BenchmarkRunner::run(&scenario, &mut solver).unwrap();
        assert_eq!(metrics.scenario, kind.name());
        assert!(metrics.total_wall_time >= 0.0);
    }
}

// ─── Metrics Tests ────────────────────────────────────────────

#[test]
fn metrics_csv_output() {
    let metrics = BenchmarkMetrics {
        scenario: "test".into(),
        total_wall_time: 1.5,
        timesteps: 100,
        avg_step_time: 0.015,
        min_step_time: 0.01,
        max_step_time: 0.02,
        final_kinetic_energy: 1e-5,
        max_displacement: 0.5,
        avg_iterations: 10.0,
        vertex_count: 441,
        triangle_count: 800,
        drape_coefficient: 0.0,
    };

    let csv_row = metrics.to_csv_row();
    assert!(csv_row.contains("test"));
    assert!(csv_row.contains("441"));
    assert!(csv_row.contains("800"));
}

#[test]
fn metrics_csv_multi() {
    let m1 = BenchmarkMetrics {
        scenario: "a".into(),
        total_wall_time: 1.0,
        timesteps: 10,
        avg_step_time: 0.1,
        min_step_time: 0.05,
        max_step_time: 0.15,
        final_kinetic_energy: 0.0,
        max_displacement: 0.0,
        avg_iterations: 0.0,
        vertex_count: 4,
        triangle_count: 2,
        drape_coefficient: 0.0,
    };
    let csv = BenchmarkMetrics::to_csv(&[m1]);
    let lines: Vec<&str> = csv.lines().collect();
    assert_eq!(lines.len(), 2); // Header + 1 data row
    assert!(lines[0].contains("scenario"));
}

#[test]
fn metrics_json_round_trip() {
    let metrics = BenchmarkMetrics {
        scenario: "test".into(),
        total_wall_time: 1.0,
        timesteps: 10,
        avg_step_time: 0.1,
        min_step_time: 0.05,
        max_step_time: 0.15,
        final_kinetic_energy: 1e-3,
        max_displacement: 0.25,
        avg_iterations: 5.0,
        vertex_count: 100,
        triangle_count: 180,
        drape_coefficient: 0.0,
    };
    let json = serde_json::to_string(&metrics).unwrap();
    let recovered: BenchmarkMetrics = serde_json::from_str(&json).unwrap();
    assert_eq!(recovered.timesteps, 10);
}

// ─── Verification Tests (Non-Penetration) ─────────────────────

#[test]
fn verify_sphere_drape_no_penetration() {
    let mut scenario = Scenario::sphere_drape();
    // Run for a shorter time to keep the test fast but long enough to hit the sphere
    scenario.timesteps = 60; // 1 second

    let mut solver = ProjectiveDynamicsSolver::new();
    let metrics = BenchmarkRunner::run(&scenario, &mut solver).unwrap();

    // We can't easily assert on every single frame's state from the runner output,
    // but we can ensure the solver completed successfully and didn't explode.
    assert!(metrics.total_wall_time > 0.0);

    // Re-run manually to check coordinates frame-by-frame
    let mut scenario = Scenario::sphere_drape();
    scenario.timesteps = 40;
    let mut solver = ProjectiveDynamicsSolver::new();

    let mut state = vistio_solver::state::SimulationState::from_mesh(
        &scenario.garment,
        scenario.vertex_mass,
        &scenario.pinned
    ).unwrap();

    let topo = vistio_mesh::Topology::build(&scenario.garment);
    solver.init(&scenario.garment, &topo, &scenario.config, &scenario.pinned).unwrap();

    // Estimate kappa the same way the runner/viewer do
    let kappa = vistio_contact::barrier::estimate_initial_kappa(
        scenario.config.barrier_kappa,
        scenario.vertex_mass,
        scenario.config.gravity[1].abs(),
        scenario.config.barrier_d_hat,
    );

    let mut pipeline = vistio_contact::collision_pipeline::CollisionPipeline::new(
        Box::new(SpatialHash::new(0.02)),
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        scenario.config.barrier_d_hat,
        scenario.config.barrier_kappa,
    ).with_ipc(true).with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3)
     .with_ground(-0.3);



    struct TestIpcHandler<'a> {
        pipeline: &'a mut vistio_contact::collision_pipeline::CollisionPipeline,
        d_hat: f32,
        kappa: f32,
        mesh: &'a vistio_mesh::TriangleMesh,
    }
    impl<'a> vistio_solver::pd_solver::IpcCollisionHandler for TestIpcHandler<'a> {
        fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> vistio_solver::pd_solver::IpcBarrierForces {
            self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa)
        }
        fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32]) -> f32 {
            self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z)
        }
    }

    // Set ground height for ground enforcement
    state.ground_height = Some(-0.3);

    for _ in 0..scenario.timesteps {
        let mut handler = TestIpcHandler {
            pipeline: &mut pipeline,
            d_hat: scenario.config.barrier_d_hat,
            kappa,
            mesh: &scenario.garment,
        };

        let _ = solver.step_with_ipc(&mut state, scenario.dt, &mut handler).unwrap();
        // IPC barriers handle all contact — no post-solve projection needed.

        // Verify no vertex is inside the sphere of radius 0.3
        for i in 0..state.vertex_count {
            let dist = (state.pos_x[i].powi(2) + state.pos_y[i].powi(2) + state.pos_z[i].powi(2)).sqrt();
            let threshold = 0.3_f32 - 0.02; // Allow 2cm tolerance for IPC barrier equilibrium
            assert!(
                dist >= threshold,
                "Penetration detected! Vertex {} is at distance {} from origin, inside the sphere of radius {}",
                i, dist, 0.3
            );
        }
    }
}

#[test]
fn verify_cusick_drape_no_penetration() {
    let mut scenario = Scenario::cusick_drape();
    scenario.timesteps = 60; // 1 second

    let mut solver = ProjectiveDynamicsSolver::new();
    let mut state = vistio_solver::state::SimulationState::from_mesh(
        &scenario.garment,
        scenario.vertex_mass,
        &scenario.pinned
    ).unwrap();

    let topo = vistio_mesh::Topology::build(&scenario.garment);
    solver.init(&scenario.garment, &topo, &scenario.config, &scenario.pinned).unwrap();

    let mut pipeline = vistio_contact::collision_pipeline::CollisionPipeline::new(
        Box::new(SpatialHash::new(0.02)),
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        scenario.config.barrier_d_hat,
        scenario.config.barrier_kappa,
    ).with_ipc(true).with_cylinder(0.0, 0.0, 0.2, 0.18)
     .with_ground(-0.3);

    // Estimate kappa the same way the runner/viewer do
    let kappa = vistio_contact::barrier::estimate_initial_kappa(
        scenario.config.barrier_kappa,
        scenario.vertex_mass,
        scenario.config.gravity[1].abs(),
        scenario.config.barrier_d_hat,
    );

    struct TestIpcHandler<'a> {
        pipeline: &'a mut vistio_contact::collision_pipeline::CollisionPipeline,
        d_hat: f32,
        kappa: f32,
        mesh: &'a vistio_mesh::TriangleMesh,
    }
    impl<'a> vistio_solver::pd_solver::IpcCollisionHandler for TestIpcHandler<'a> {
        fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> vistio_solver::pd_solver::IpcBarrierForces {
            self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa)
        }
        fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32]) -> f32 {
            self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z)
        }
    }

    // Set ground height for ground enforcement
    state.ground_height = Some(-0.3);

    for _ in 0..scenario.timesteps {
        let mut handler = TestIpcHandler {
            pipeline: &mut pipeline,
            d_hat: scenario.config.barrier_d_hat,
            kappa,
            mesh: &scenario.garment,
        };

        let _ = solver.step_with_ipc(&mut state, scenario.dt, &mut handler).unwrap();
        // IPC barriers handle all contact — no post-solve projection needed.

        // Verify no vertex penetrates the cylinder (radius 0.18, top 0.2)
        for i in 0..state.vertex_count {
            if state.pos_y[i] < 0.2 {
                let dist = (state.pos_x[i].powi(2) + state.pos_z[i].powi(2)).sqrt();
                let threshold = 0.18_f32 - 0.02; // Allow 2cm tolerance for IPC barrier equilibrium
                assert!(
                    dist >= threshold,
                    "Penetration detected! Vertex {} is at distance {} from central axis, inside the cylinder of radius {}",
                    i, dist, 0.18
                );
            }
        }
    }
}
