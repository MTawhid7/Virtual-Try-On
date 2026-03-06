use vistio_bench::scenarios::Scenario;
use vistio_solver::pd_solver::{ProjectiveDynamicsSolver, IpcCollisionHandler, IpcBarrierForces};
use vistio_solver::SolverStrategy;
use vistio_solver::state::SimulationState;
use vistio_contact::CollisionPipeline;
use vistio_contact::{VertexTriangleTest, ProjectionContactResponse};
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

struct Handler<'a> {
    pipeline: &'a mut CollisionPipeline,
    mesh: &'a vistio_mesh::TriangleMesh,
    config: &'a vistio_solver::SolverConfig,
}

impl<'a> IpcCollisionHandler for Handler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> IpcBarrierForces {
        let kappa = vistio_contact::barrier::estimate_initial_kappa(
            self.config.barrier_kappa,
            0.002,
            self.config.gravity[1].abs(),
            self.config.barrier_d_hat
        );
        let forces = self.pipeline.detect_ipc_contacts(px, py, pz, self.config.barrier_d_hat, kappa);
        let mut max_grad = 0.0_f32;
        for i in 0..forces.grad_x.len() {
            max_grad = max_grad.max(forces.grad_x[i].abs());
            max_grad = max_grad.max(forces.grad_y[i].abs());
            max_grad = max_grad.max(forces.grad_z[i].abs());
        }
        println!("    [IPC] active={}, max_viol={:.6}, max_grad={:.4}, kappa={:.1}",
            forces.active_contacts, forces.max_violation, max_grad, kappa);
        forces
    }
    fn compute_ccd_step(&mut self, px0: &[f32], py0: &[f32], pz0: &[f32], px1: &[f32], py1: &[f32], pz1: &[f32]) -> f32 {
        self.pipeline.compute_ccd_step(&self.mesh.indices, px0, py0, pz0, px1, py1, pz1)
    }
}

fn main() -> VistioResult<()> {
    let scenario = Scenario::self_fold();
    let n = scenario.garment.vertex_count();
    let topology = Topology::build(&scenario.garment);

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned)?;

    let mut state = SimulationState::from_mesh(&scenario.garment, scenario.vertex_mass, &scenario.pinned)?;

    let mut pipeline = CollisionPipeline::new(
        Box::new(vistio_contact::BvhBroadPhase::new(&scenario.garment)),
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        scenario.config.cloth_thickness,
        0.0,
    ).with_self_collision(&topology, 1).with_ipc(true);

    pipeline = pipeline
        .with_ground(-0.3)
        .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3);

    println!("Starting diagnostic simulation (400 steps)...");
    for step in 0..400 {
        let mut handler = Handler {
            pipeline: &mut pipeline,
            mesh: &scenario.garment,
            config: &scenario.config,
        };

        let result = solver.step_with_ipc(&mut state, scenario.dt, &mut handler)?;

        let mut max_y = -1e10_f32;
        let mut min_y = 1e10_f32;
        let mut has_nan = false;
        for i in 0..n {
            if state.pos_y[i].is_nan() { has_nan = true; }
            max_y = max_y.max(state.pos_y[i]);
            min_y = min_y.min(state.pos_y[i]);
        }
        println!("Step {}: iters={}, max_y={:.4}, min_y={:.4}, nan={}",
            step, result.iterations, max_y, min_y, has_nan);

        if has_nan { break; }
    }

    Ok(())
}
