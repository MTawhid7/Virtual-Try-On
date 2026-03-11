use vistio_bench::scenarios::Scenario;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::SolverStrategy;
use vistio_contact::collision_pipeline::CollisionPipeline;
use vistio_contact::{VertexTriangleTest, ProjectionContactResponse};
use vistio_mesh::topology::Topology;

fn main() {
    let scenario = Scenario::sphere_drape();
    let broad_phase: Box<dyn vistio_contact::broad::BroadPhase + Send + Sync> =
        Box::new(vistio_contact::bvh::BvhBroadPhase::new(&scenario.garment));
    let mut pipeline = CollisionPipeline::new(
        broad_phase,
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        scenario.config.barrier_d_hat * 1.5,
        0.0,
    )
    .with_ground(-0.3)
    .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3)
    .with_ipc(true);

    let topology = Topology::build(&scenario.garment);
    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned).unwrap();

    let mut state = SimulationState::from_mesh_with_masses(
        &scenario.garment,
        solver.lumped_masses(),
        &scenario.pinned,
    ).unwrap();
    state.ground_height = Some(-0.3);
    let initial_y: Vec<f32> = state.pos_y.clone();

    let kappa = vistio_contact::barrier::estimate_initial_kappa(
        scenario.config.barrier_kappa,
        scenario.vertex_mass,
        scenario.config.gravity[1].abs(),
        scenario.config.barrier_d_hat,
    );

    struct InspectIpcHandler<'a> {
        pipeline: &'a mut CollisionPipeline,
        d_hat: f32,
        kappa: f32,
        mesh: &'a vistio_mesh::TriangleMesh,
    }
    impl<'a> vistio_solver::pd_solver::IpcCollisionHandler for InspectIpcHandler<'a> {
        fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> vistio_solver::pd_solver::IpcBarrierForces {
            self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa)
        }
        fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32], padding: f32) -> f32 {
            self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z, padding)
        }
    }

    for step_idx in 0..50 {
        let mut handler = InspectIpcHandler {
            pipeline: &mut pipeline,
            d_hat: scenario.config.barrier_d_hat,
            kappa,
            mesh: &scenario.garment,
        };
        let _ = solver.step_with_ipc(&mut state, scenario.dt, &mut handler).unwrap();

        let max_d = (0..state.vertex_count).map(|i| {
            let dx = state.pos_x[i] - scenario.garment.pos_x[i];
            let dy = state.pos_y[i] - initial_y[i];
            let dz = state.pos_z[i] - scenario.garment.pos_z[i];
            (dx*dx + dy*dy + dz*dz).sqrt()
        }).fold(0.0f32, f32::max);

        let max_lam = state.al_lambda_y.iter().map(|&x| x.abs()).fold(0.0f32, f32::max);
        let max_vel = state.vel_y.iter().map(|&x| x.abs()).fold(0.0f32, f32::max);

        println!("Step {}: max_d={:.2}, max_lam={:.2}, max_vy={:.2}", step_idx, max_d, max_lam, max_vel);
    }
}
