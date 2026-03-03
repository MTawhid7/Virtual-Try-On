use vistio_bench::scenarios::Scenario;
use vistio_contact::CollisionPipeline;
use vistio_contact::VertexTriangleTest;
use vistio_contact::ProjectionContactResponse;
use vistio_contact::BvhBroadPhase;
use vistio_material::CoRotationalModel;
use vistio_mesh::topology::Topology;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;

struct TestIpcHandler<'a> {
    pipeline: &'a mut CollisionPipeline,
    d_hat: f32,
    kappa: f32,
    mesh: &'a vistio_mesh::TriangleMesh,
    min_toi_seen: &'a mut f32,
    max_viol_seen: &'a mut f32,
}

impl<'a> vistio_solver::pd_solver::IpcCollisionHandler for TestIpcHandler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> vistio_solver::pd_solver::IpcBarrierForces {
        let forces = self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa);
        *self.max_viol_seen = (*self.max_viol_seen).max(forces.max_violation);

        let mut min_gy = 0.0_f32;
        let mut max_gy = 0.0_f32;
        for &g in &forces.grad_y {
            min_gy = min_gy.min(g);
            max_gy = max_gy.max(g);
        }
        println!("  detect_contacts: max_viol={:.6}, act={}, grad_y bounds [{:.6}, {:.6}]",
                 forces.max_violation, forces.active_contacts, min_gy, max_gy);

        forces
    }

    fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32]) -> f32 {
        let toi = self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z);
        *self.min_toi_seen = toi.min(*self.min_toi_seen);
        println!("  compute_ccd_step: min_toi={:.8}", toi);
        toi
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!(">>> INITIALIZING CUSICK DRAPE DEBUGGER <<<");

    let mut scenario = Scenario::cusick_drape();
    scenario.config.ipc_enabled = true;
    scenario.config.barrier_d_hat = 1e-4; // 1cm activation radius
    scenario.config.barrier_kappa = 1000.0;
    scenario.config.al_max_iterations = 2; // Keep logs small

    let topology = Topology::build(&scenario.garment);
    let mut solver = ProjectiveDynamicsSolver::new();

    // Standard initialization without materials for focused debugging
    solver.init_with_material_tier4(
        &scenario.garment,
        &topology,
        &scenario.config,
        &vistio_material::database::MaterialDatabase::with_defaults().get("cotton_twill").unwrap(),
        Box::new(CoRotationalModel::new()),
        &scenario.pinned,
    )?;

    let mut state = SimulationState::from_mesh(
        &scenario.garment,
        scenario.vertex_mass,
        &scenario.pinned,
    )?;

    let broad_phase = Box::new(BvhBroadPhase::new(&scenario.garment));
    let mut pipeline = CollisionPipeline::new(
        broad_phase,
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        scenario.config.barrier_d_hat * 1.5,
        0.0,
    )
    .with_self_collision(&topology, 1)
    .with_ipc(true)
    .with_cylinder(0.0, 0.0, 0.3, 0.09) // Viewer matched cylinder
    .with_ground(-0.3);

    println!("Cylinder bounds: top=0.3, radius=0.09");
    println!("Simulation starting. Tracking a center vertex...");

    let test_vertex = 0; // The absolute center vertex of the circle

    for step in 1..=10 {
        // 1. Run solver inner logging
        let mut min_toi_taken = 1.0_f32;
        let mut max_violation_seen = 0.0_f32;

        let mut handler = TestIpcHandler {
            pipeline: &mut pipeline,
            d_hat: scenario.config.barrier_d_hat,
            kappa: scenario.config.barrier_kappa,
            mesh: &scenario.garment,
            min_toi_seen: &mut min_toi_taken,
            max_viol_seen: &mut max_violation_seen,
        };

        // Snapshot before solver
        let y_before = state.pos_y[test_vertex];

        solver.step_with_ipc(&mut state, scenario.dt, &mut handler)?;

        // 2. Run post-step collision
        pipeline.step(&mut state)?;

        let y_after = state.pos_y[test_vertex];
        let r_after = (state.pos_x[test_vertex].powi(2) + state.pos_z[test_vertex].powi(2)).sqrt();

        println!("\n[Step {}]", step);
        println!("Center Vertex Y: {:.4} -> {:.4}", y_before, y_after);
        println!("Center Vertex R from origin: {:.4}", r_after);
        println!("Solver CCD TOI taken: {:.6}", min_toi_taken);
        println!("Solver Max Violation: {:.6}", max_violation_seen);

        if y_after < 0.3 && r_after < 0.09 {
            println!(">>> WARNING: PENETRATION DETECTED! Vertex is inside the cylinder! <<<");

            // Reconstruct what forces the IPC solver saw!
            let forces = pipeline.detect_ipc_contacts(
                &state.pos_x, &state.pos_y, &state.pos_z,
                scenario.config.barrier_d_hat,
                scenario.config.barrier_kappa,
            );

            let force_y = forces.grad_y[test_vertex];
            println!("IPC AL Barrier Force on Vertex (Y): {:.6}", force_y);

            if force_y == 0.0 {
                println!("DIAGNOSIS CONFIRMED: IPC completely ignored the cylinder! The vertex fell right through.");
                break;
            }
        }
    }

    Ok(())
}
