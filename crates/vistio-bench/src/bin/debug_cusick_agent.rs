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
    pub vertex_mass: f32,
    pub step_barrier_forces: Vec<f32>,
    pub step_penetrations: Vec<f32>,
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

        let mut max_grad = 0.0_f32;
        for i in 0..forces.grad_y.len() {
            let g = (forces.grad_x[i].powi(2) + forces.grad_y[i].powi(2) + forces.grad_z[i].powi(2)).sqrt();
            max_grad = max_grad.max(g);
        }
        self.step_barrier_forces.push(max_grad);
        self.step_penetrations.push(forces.max_violation);

        forces
    }

    fn compute_ccd_step(&mut self, px0: &[f32], py0: &[f32], pz0: &[f32], px1: &[f32], py1: &[f32], pz1: &[f32], padding: f32, _alphas: &mut [f32]) -> f32 {
        self.pipeline.compute_ccd_step(&self.mesh.indices, px0, py0, pz0, px1, py1, pz1, padding, _alphas)
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

    let config = scenario.config.clone();
    let mut pipeline = CollisionPipeline::new(
        Box::new(vistio_contact::BvhBroadPhase::new(&scenario.garment)),
        Box::new(VertexTriangleTest),
        Box::new(ProjectionContactResponse),
        scenario.garment.clone(),
        config.cloth_thickness,
        0.0,
    ).with_ipc(true)
    .with_self_collision(&topology, 1)
    .with_ground(-0.3)
    .with_cylinder(0.0, 0.0, 0.3, 0.09); // From lib.rs viewer setup

    std::fs::create_dir_all("debug").ok();
    let mut csv_file = File::create("debug/ai_cusick_metrics.csv").expect("Could not create stats file");
    writeln!(csv_file, "frame,iters,ke,max_speed,min_dist_top,min_dist_side,max_bar_force,al_violation").unwrap();

    println!("Starting AI diagnostic simulation (200 steps)...");

    for step in 0..300 {
        let mut handler = Handler {
            pipeline: &mut pipeline,
            mesh: &scenario.garment,
            config: &scenario.config,
            vertex_mass,
            step_barrier_forces: Vec::new(),
            step_penetrations: Vec::new(),
        };

        let result = solver.step_with_ipc(&mut state, scenario.dt, &mut handler)?;

        let mut has_nan = false;
        let mut max_speed = 0.0_f32;
        let mut min_dist_top = 1e10_f32;
        let mut min_dist_side = 1e10_f32;

        for i in 0..n {
            if state.pos_y[i].is_nan() { has_nan = true; break; }

            let speed = (state.vel_x[i].powi(2) + state.vel_y[i].powi(2) + state.vel_z[i].powi(2)).sqrt();
            max_speed = max_speed.max(speed);

            // Cylinder bounds: x=0, z=0, r=0.09, top_y=0.3
            let dx = state.pos_x[i];
            let dz = state.pos_z[i];
            let r = (dx * dx + dz * dz).sqrt();
            let dy = state.pos_y[i] - 0.3; // Dist above top

            // If strictly within the radius cylinder footprint
            if r <= 0.09 {
                min_dist_top = min_dist_top.min(dy);
            }
            // If strictly below top plane but outside radius
            if state.pos_y[i] <= 0.3 {
                min_dist_side = min_dist_side.min(r - 0.09);
            }
        }

        let ke = if has_nan { 0.0 } else { state.kinetic_energy() };
        let max_bar_force = handler.step_barrier_forces.into_iter().fold(0.0_f32, f32::max);
        let max_viol = handler.step_penetrations.into_iter().fold(0.0_f32, f32::max);

        writeln!(csv_file, "{},{},{:.6},{:.6},{:.6},{:.6},{:.4},{:.8}",
                 step, result.iterations, ke, max_speed, min_dist_top, min_dist_side, max_bar_force, max_viol).unwrap();

        if step % 20 == 0 || has_nan || step == 200 {
            println!("Frame {:3} | Iters {:2} | KE {:.5} | MaxSpd {:.4} | D_Top {:7.4} | D_Side {:7.4} | BarF {:.2}",
                step, result.iterations, ke, max_speed, min_dist_top, min_dist_side, max_bar_force);
        }

        if has_nan {
            println!("!!! EXPLOSION DETECTED at frame {} !!!", step);
            break;
        }
    }

    println!("Done. Output written to debug/ai_cusick_metrics.csv");
    Ok(())
}
