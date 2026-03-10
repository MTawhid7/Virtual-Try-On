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
    pub last_max_hess: f32,
    pub last_max_viol: f32,
    pub last_active: usize,
    pub last_ccd_toi: f32,
    pub vertex_mass: f32,
}

impl<'a> IpcCollisionHandler for Handler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> IpcBarrierForces {
        let kappa = vistio_contact::barrier::estimate_initial_kappa(
            self.config.barrier_kappa,
            self.vertex_mass,
            self.config.gravity[1].abs(),
            self.config.barrier_d_hat
        );
        let forces = self.pipeline.detect_ipc_contacts(px, py, pz, self.config.barrier_d_hat, kappa);
        self.last_max_viol = forces.max_violation;
        self.last_active = forces.active_contacts;

        let mut max_g = 0.0_f32;
        let mut max_h = 0.0_f32;
        for i in 0..forces.grad_x.len() {
            max_g = max_g.max(forces.grad_x[i].abs().max(forces.grad_y[i].abs()).max(forces.grad_z[i].abs()));
            max_h = max_h.max(forces.hessian_diag[i].abs());
        }
        self.last_max_grad = max_g;
        self.last_max_hess = max_h;
        forces
    }
    fn compute_ccd_step(&mut self, px0: &[f32], py0: &[f32], pz0: &[f32], px1: &[f32], py1: &[f32], pz1: &[f32], padding: f32) -> f32 {
        let toi = self.pipeline.compute_ccd_step(&self.mesh.indices, px0, py0, pz0, px1, py1, pz1, padding);
        self.last_ccd_toi = self.last_ccd_toi.min(toi);
        toi
    }
    fn set_d_hat(&mut self, _d_hat: f32) {}
}

fn main() -> VistioResult<()> {
    let scenario = Scenario::cusick_drape();
    let n = scenario.garment.vertex_count();
    let topology = Topology::build(&scenario.garment);

    let mut solver = ProjectiveDynamicsSolver::new();

    let vertex_mass = if let Some(ref properties) = scenario.material {
        let model = Box::new(vistio_material::CoRotationalModel::new());
        solver.init_with_material(
            &scenario.garment,
            &topology,
            &scenario.config,
            properties,
            model,
            &scenario.pinned,
        )?;

        // Derive mass
        let mut total_area = 0.0_f32;
        let mesh = &scenario.garment;
        for t in 0..mesh.triangle_count() {
            let idx_base = t * 3;
            let i0 = mesh.indices[idx_base] as usize;
            let i1 = mesh.indices[idx_base + 1] as usize;
            let i2 = mesh.indices[idx_base + 2] as usize;
            let p0 = vistio_math::Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
            let p1 = vistio_math::Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
            let p2 = vistio_math::Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);
            total_area += 0.5 * (p1 - p0).cross(p2 - p0).length();
        }
        properties.mass_per_vertex(scenario.garment.vertex_count(), total_area)
    } else {
        solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned)?;
        scenario.vertex_mass
    };

    let mut state = SimulationState::from_mesh_with_masses(
        &scenario.garment,
        solver.lumped_masses(),
        &scenario.pinned,
    )?;

    let mut config = scenario.config.clone(); config.barrier_d_hat = 1e-5; config.self_collision_d_hat = 1e-5; config.cloth_thickness = 0.0005; config.bending_weight = 1e-4;
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
        .with_cylinder(0.0, 0.0, 0.3, 0.09);

    std::fs::create_dir_all("debug").ok();
    let mut csv_file = File::create("debug/cusick_deep_metrics.csv").expect("Could not create debug/cusick_deep_metrics.csv");
    writeln!(csv_file, "frame,iters,max_y,min_y,ke,max_speed,max_viol,active_contacts,max_grad,max_hess,min_ccd_toi,nan_detected").unwrap();

    let _initial_y = state.pos_y.clone();

    println!("Starting DEEP diagnostic simulation (400 steps)...");
    for step in 0..400 {
        let mut handler = Handler {
            pipeline: &mut pipeline,
            mesh: &scenario.garment,
            config: &scenario.config,
            last_max_grad: 0.0,
            last_max_hess: 0.0,
            last_max_viol: 0.0,
            last_active: 0,
            last_ccd_toi: 1.0,
            vertex_mass,
        };

        // One step solver
        let result = solver.step_with_ipc(&mut state, scenario.dt, &mut handler)?;

        let mut max_y = -1e10_f32;
        let mut min_y = 1e10_f32;
        let mut max_speed = 0.0_f32;
        let mut has_nan = false;

        for i in 0..n {
            if state.pos_y[i].is_nan() || state.vel_x[i].is_nan() { has_nan = true; }
            if !state.pos_y[i].is_nan() {
                max_y = max_y.max(state.pos_y[i]);
                min_y = min_y.min(state.pos_y[i]);
            }
            if !state.vel_x[i].is_nan() {
                let speed = (state.vel_x[i]*state.vel_x[i] + state.vel_y[i]*state.vel_y[i] + state.vel_z[i]*state.vel_z[i]).sqrt();
                max_speed = max_speed.max(speed);
            }
        }

        let ke = if has_nan { 0.0 } else { state.kinetic_energy() };

        writeln!(csv_file, "{},{},{:.6},{:.6},{:.8},{:.6},{:.8},{},{:.8},{:.8},{:.8},{}",
                 step, result.iterations, max_y, min_y, ke, max_speed,
                 handler.last_max_viol, handler.last_active, handler.last_max_grad, handler.last_max_hess,
                 handler.last_ccd_toi, if has_nan { 1 } else { 0 }).unwrap();

        if step % 10 == 0 || has_nan {
            println!("Step {}: iters={}, ke={:.6}, max_speed={:.4}, max_hess={:.4}, CCD={:.4}, nan={}",
                step, result.iterations, ke, max_speed, handler.last_max_hess, handler.last_ccd_toi, has_nan);
        }

        if has_nan {
            println!("!!! EXPLOSION DETECTED at frame {} !!!", step);
            break;
        }
    }

    Ok(())
}
