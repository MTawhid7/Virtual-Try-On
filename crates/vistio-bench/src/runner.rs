//! Benchmark runner — executes scenarios with a solver and collects metrics.

use std::time::Instant;

use vistio_contact::CollisionPipeline;
use vistio_material::CoRotationalModel;
use vistio_mesh::topology::Topology;
use vistio_solver::pd_solver::ProjectiveDynamicsSolver;
use vistio_solver::state::SimulationState;
use vistio_solver::strategy::{SolverStrategy, StepResult};
use vistio_types::VistioResult;
use vistio_contact::{SpatialHash, VertexTriangleTest, ProjectionContactResponse};

use crate::metrics::BenchmarkMetrics;
use crate::scenarios::Scenario;
use vistio_solver::pd_solver::{IpcCollisionHandler, IpcBarrierForces};

struct RunnerIpcHandler<'a> {
    pipeline: &'a mut CollisionPipeline,
    d_hat: f32,
    kappa: f32,
    mesh: &'a vistio_mesh::TriangleMesh,
}

impl<'a> IpcCollisionHandler for RunnerIpcHandler<'a> {
    fn detect_contacts(&mut self, px: &[f32], py: &[f32], pz: &[f32]) -> IpcBarrierForces {
        self.pipeline.detect_ipc_contacts(px, py, pz, self.d_hat, self.kappa)
    }

    fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32]) -> f32 {
        self.pipeline.compute_ccd_step(&self.mesh.indices, prev_x, prev_y, prev_z, new_x, new_y, new_z)
    }

    fn set_d_hat(&mut self, d_hat: f32) {
        self.d_hat = d_hat;
    }
}


/// Runs benchmark scenarios and collects metrics.
pub struct BenchmarkRunner;

impl BenchmarkRunner {
    /// Run a single scenario with the given solver.
    ///
    /// If the scenario has material properties set, uses `init_with_material()`
    /// for material-aware simulation. Otherwise falls back to standard `init()`.
    pub fn run(
        scenario: &Scenario,
        solver: &mut ProjectiveDynamicsSolver,
    ) -> VistioResult<BenchmarkMetrics> {
        use crate::scenarios::ScenarioKind;

        let is_ipc = scenario.config.ipc_enabled;

        let broad_phase: Box<dyn vistio_contact::broad::BroadPhase + Send + Sync> = if is_ipc {
            Box::new(vistio_contact::bvh::BvhBroadPhase::new(&scenario.garment))
        } else {
            Box::new(SpatialHash::new(0.05))
        };

        let thickness = if is_ipc { scenario.config.barrier_d_hat * 1.5 } else { 0.0 };

        let mut pipeline = CollisionPipeline::new(
            broad_phase,
            Box::new(VertexTriangleTest),
            Box::new(ProjectionContactResponse),
            scenario.garment.clone(),
            thickness, // thickness (0.0 disables Tier 1 self-collision)
            0.0,       // stiffness
        );

        if is_ipc {
            pipeline = pipeline.with_self_collision(&Topology::build(&scenario.garment), 1);
        }

        match scenario.kind {
            ScenarioKind::SphereDrape => {
                pipeline = pipeline
                    .with_ground(-0.3)
                    .with_sphere(vistio_math::Vec3::new(0.0, 0.0, 0.0), 0.3);
            },
            ScenarioKind::CusickDrape => {
                pipeline = pipeline
                    .with_ground(-0.3)
                    .with_cylinder(0.0, 0.0, 0.3, 0.09);
            },
            ScenarioKind::CantileverBending => {
                pipeline = pipeline
                    .with_box(-0.1, 0.1, 0.0, 0.499, -0.2, 0.0);
            },
            _ => {
                pipeline = pipeline.with_ground(-0.3);
            }
        }

        Self::run_with_collision(scenario, solver, Some(pipeline))
    }

    /// Run a scenario with an optional collision pipeline.
    ///
    /// When a `CollisionPipeline` is provided, it is called after each solver
    /// step to detect and resolve contacts before the next timestep.
    pub fn run_with_collision(
        scenario: &Scenario,
        solver: &mut ProjectiveDynamicsSolver,
        mut collision: Option<CollisionPipeline>,
    ) -> VistioResult<BenchmarkMetrics> {
        let topology = Topology::build(&scenario.garment);

        // Choose init path based on whether a material is specified
        let _vertex_mass = if let Some(ref properties) = scenario.material {
            // Tier 3 path: when material is anisotropic, use Discrete Shells + anisotropic model
            // Tier 2 path: when material is isotropic, use dihedral + co-rotational model
            if properties.is_anisotropic() {
                let model = Box::new(vistio_material::AnisotropicCoRotationalModel::from_properties(properties));
                solver.init_with_material_tier3(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                )?;
            } else {
                let model = Box::new(CoRotationalModel::new());
                solver.init_with_material(
                    &scenario.garment,
                    &topology,
                    &scenario.config,
                    properties,
                    model,
                    &scenario.pinned,
                )?;
            }
            // Derive mass from material
            let total_area = compute_mesh_area(&scenario.garment);
            properties.mass_per_vertex(scenario.garment.vertex_count(), total_area)
        } else {
            // Tier 1 path: standard initialization
            solver.init(&scenario.garment, &topology, &scenario.config, &scenario.pinned)?;
            scenario.vertex_mass
        };

        // Initialize simulation state with solver's lumped masses for consistency
        let mut state = SimulationState::from_mesh_with_masses(
            &scenario.garment,
            solver.lumped_masses(),
            &scenario.pinned,
        )?;

        // Wire ground height into the solver's internal constraint enforcement
        if let Some(ref pipeline) = collision {
            if let Some(ref ground) = pipeline.ground {
                state.ground_height = Some(ground.height);
            }
        }

        // Save initial positions for displacement tracking
        let initial_y: Vec<f32> = state.pos_y.clone();

        let mut step_times: Vec<f64> = Vec::with_capacity(scenario.timesteps as usize);
        let mut total_iterations: u32 = 0;

        let total_start = Instant::now();

        // Run timesteps
        for _step in 0..scenario.timesteps {
            if scenario.kind == crate::scenarios::ScenarioKind::UniaxialStretch {
                // Stretch from 0.50m width up to 0.75m width (+50% strain) over the total timesteps
                let total_stretch = 0.25;
                let stretch_amount = total_stretch / scenario.timesteps as f32;

                for i in 0..state.vertex_count {
                    // Update only rightmost pinned vertices
                    if state.pos_x[i] > 0.249 {
                        state.pos_x[i] += stretch_amount;
                        state.vel_x[i] = stretch_amount / scenario.dt;
                        // It is statically pinned via A matrix, so position update physically pulls it.
                    }
                }
            }

            let is_ipc = scenario.config.ipc_enabled;
            if let Some(mut pipeline) = collision.take() {
                if is_ipc {
                    // Update pipeline to know IPC is enabled (skip projection self-col)
                    pipeline = pipeline.with_ipc(true);
                    let d_hat = scenario.config.barrier_d_hat;
                    let kappa = vistio_contact::barrier::estimate_initial_kappa(
                        scenario.config.barrier_kappa,
                        scenario.vertex_mass,
                        scenario.config.gravity[1].abs(),
                        d_hat,
                    );

                    let mut handler = RunnerIpcHandler {
                        pipeline: &mut pipeline,
                        d_hat,
                        kappa,
                        mesh: &scenario.garment,
                    };

                    let result = solver.step_with_ipc(&mut state, scenario.dt, &mut handler)?;
                    step_times.push(result.wall_time);
                    total_iterations += result.iterations;

                    // IPC barriers handle all contact — no post-solve projection needed.
                } else {
                    let result: StepResult = solver.step(&mut state, scenario.dt)?;
                    step_times.push(result.wall_time);
                    total_iterations += result.iterations;

                    // Collision step (after solver resolves elastic forces)
                    let _ = pipeline.step(&mut state)?;
                }
                collision = Some(pipeline);
            } else if is_ipc {
                let mut empty_handler = vistio_solver::pd_solver::EmptyIpcHandler;
                let result = solver.step_with_ipc(&mut state, scenario.dt, &mut empty_handler)?;
                step_times.push(result.wall_time);
                total_iterations += result.iterations;
            } else {
                let result: StepResult = solver.step(&mut state, scenario.dt)?;
                step_times.push(result.wall_time);
                total_iterations += result.iterations;
            }
        }

        let total_wall_time = total_start.elapsed().as_secs_f64();

        // Compute final metrics
        let final_ke = state.kinetic_energy();

        let max_displacement = (0..state.vertex_count)
            .map(|i| {
                let dx = state.pos_x[i] - scenario.garment.pos_x[i];
                let dy = state.pos_y[i] - initial_y[i];
                let dz = state.pos_z[i] - scenario.garment.pos_z[i];
                (dx * dx + dy * dy + dz * dz).sqrt()
            })
            .fold(0.0f32, f32::max);



        let avg_step = if step_times.is_empty() {
            0.0
        } else {
            step_times.iter().sum::<f64>() / step_times.len() as f64
        };
        let min_step = step_times.iter().copied().fold(f64::MAX, f64::min);
        let max_step = step_times.iter().copied().fold(0.0, f64::max);
        let avg_iter = if scenario.timesteps > 0 {
            total_iterations as f32 / scenario.timesteps as f32
        } else {
            0.0
        };

        let mut drape_coefficient = 0.0;
        if scenario.kind == crate::scenarios::ScenarioKind::CusickDrape {
            let mut max_r_at_theta = vec![0.09_f32; 360];
            for i in 0..state.vertex_count {
                let x = state.pos_x[i];
                let z = state.pos_z[i];
                let r = (x * x + z * z).sqrt();
                let theta = z.atan2(x);
                let mut deg = (theta.to_degrees().round() as i32) % 360;
                if deg < 0 { deg += 360; }
                if r > max_r_at_theta[deg as usize] {
                    max_r_at_theta[deg as usize] = r;
                }
            }
            let mut shadow_area = 0.0;
            for r in max_r_at_theta {
                shadow_area += std::f32::consts::PI * r * r / 360.0;
            }
            let area_pedestal = std::f32::consts::PI * 0.09 * 0.09;
            let area_flat = std::f32::consts::PI * 0.15 * 0.15;
            drape_coefficient = 100.0 * (shadow_area - area_pedestal) / (area_flat - area_pedestal);
        }

        Ok(BenchmarkMetrics {
            scenario: scenario.kind.name().to_string(),
            total_wall_time,
            timesteps: scenario.timesteps,
            avg_step_time: avg_step,
            min_step_time: min_step,
            max_step_time: max_step,
            final_kinetic_energy: final_ke,
            max_displacement,
            avg_iterations: avg_iter,
            vertex_count: scenario.garment.vertex_count(),
            triangle_count: scenario.garment.triangle_count(),
            drape_coefficient,
        })
    }

    /// Run all scenarios and return metrics for each.
    pub fn run_all(
        solver: &mut ProjectiveDynamicsSolver,
    ) -> VistioResult<Vec<BenchmarkMetrics>> {
        use crate::scenarios::ScenarioKind;
        let mut results = Vec::new();
        for &kind in ScenarioKind::all() {
            let scenario = Scenario::from_kind(kind);
            let metrics = Self::run(&scenario, solver)?;
            results.push(metrics);
        }
        Ok(results)
    }
}

/// Compute the total surface area of a triangle mesh.
fn compute_mesh_area(mesh: &vistio_mesh::TriangleMesh) -> f32 {
    use vistio_math::Vec3;
    let tri_count = mesh.triangle_count();
    let mut total = 0.0_f32;
    for t in 0..tri_count {
        let idx_base = t * 3;
        let i0 = mesh.indices[idx_base] as usize;
        let i1 = mesh.indices[idx_base + 1] as usize;
        let i2 = mesh.indices[idx_base + 2] as usize;
        let p0 = Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
        let p1 = Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
        let p2 = Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);
        let e1 = p1 - p0;
        let e2 = p2 - p0;
        total += 0.5 * e1.cross(e2).length();
    }
    total
}
