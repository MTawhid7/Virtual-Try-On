use std::fs::File;
use std::io::Write;
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
    pub last_max_grad: f32,
    pub last_max_viol: f32,
    pub last_kappa: f32,
}

impl<'a> IpcCollisionHandler for Handler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> IpcBarrierForces {
        let kappa = vistio_contact::barrier::estimate_initial_kappa(
            self.config.barrier_kappa,
            0.002, // dummy vertex mass
            self.config.gravity[1].abs(),
            self.config.barrier_d_hat
        );
        let forces = self.pipeline.detect_ipc_contacts(px, py, pz, self.config.barrier_d_hat, kappa);
        self.last_max_viol = forces.max_violation;
        self.last_kappa = kappa;

        let mut max_g = 0.0_f32;
        for i in 0..forces.grad_x.len() {
            max_g = max_g.max(forces.grad_x[i].abs());
            max_g = max_g.max(forces.grad_y[i].abs());
            max_g = max_g.max(forces.grad_z[i].abs());
        }
        self.last_max_grad = max_g;
        forces
    }
    fn compute_ccd_step(&mut self, px0: &[f32], py0: &[f32], pz0: &[f32], px1: &[f32], py1: &[f32], pz1: &[f32], padding: f32) -> f32 {
        self.pipeline.compute_ccd_step(&self.mesh.indices, px0, py0, pz0, px1, py1, pz1, padding)
    }
}

fn main() -> VistioResult<()> {
    let scenario = Scenario::sphere_drape();
    let n = scenario.garment.vertex_count();
    let topology = Topology::build(&scenario.garment);

    let mut solver = ProjectiveDynamicsSolver::new();
    solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned)?;

    let mut state = SimulationState::from_mesh(&scenario.garment, scenario.vertex_mass, &scenario.pinned)?;

    let config = scenario.config.clone();
    let mut pipeline = CollisionPipeline::new(
        Box::new(vistio_contact::BvhBroadPhase::new(&scenario.garment)),
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        config.cloth_thickness,
        0.0,
    ).with_self_collision(&topology, 1).with_ipc(true);

    pipeline = pipeline
        .with_ground(-0.3)
        .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3);

    let mut csv_file = File::create("debug/diagnostics.csv").expect("Could not create debug/diagnostics.csv");
    writeln!(csv_file, "frame,iters,max_y,min_y,ke,max_speed,max_viol,max_grad,nan").unwrap();

    println!("Starting extended diagnostic simulation (355 steps)...");
    for step in 0..355 {
        let mut handler = Handler {
            pipeline: &mut pipeline,
            mesh: &scenario.garment,
            config: &scenario.config,
            last_max_grad: 0.0,
            last_max_viol: 0.0,
            last_kappa: 0.0,
        };

        let result = solver.step_with_ipc(&mut state, scenario.dt, &mut handler)?;

        let mut max_y = -1e10_f32;
        let mut min_y = 1e10_f32;
        let mut max_speed = 0.0_f32;
        let mut has_nan = false;

        for i in 0..n {
            if state.pos_y[i].is_nan() || state.vel_x[i].is_nan() { has_nan = true; }
            max_y = max_y.max(state.pos_y[i]);
            min_y = min_y.min(state.pos_y[i]);
            let speed = (state.vel_x[i]*state.vel_x[i] + state.vel_y[i]*state.vel_y[i] + state.vel_z[i]*state.vel_z[i]).sqrt();
            max_speed = max_speed.max(speed);
        }

        let ke = state.kinetic_energy();

        writeln!(csv_file, "{},{},{:.6},{:.6},{:.8},{:.6},{:.8},{:.8},{}",
                 step, result.iterations, max_y, min_y, ke, max_speed,
                 handler.last_max_viol, handler.last_max_grad, has_nan).unwrap();

        if step % 50 == 0 || has_nan {
            println!("Step {}: iters={}, ke={:.6}, max_y={:.4}, max_speed={:.4}, max_viol={:.6}, nan={}",
                step, result.iterations, ke, max_y, max_speed, handler.last_max_viol, has_nan);
        }

        if has_nan {
            println!("Explosion detected at frame {}", step);
            break;
        }
    }

    Ok(())
}
