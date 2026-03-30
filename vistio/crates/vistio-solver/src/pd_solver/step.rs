//! Standard PD solver step (Tier 1–3, no IPC).
//!
//! Contains the `SolverStrategy::step()` implementation for the standard
//! local-global PD loop without Augmented Lagrangian contact.

use std::time::Instant;

use vistio_types::VistioResult;

use crate::assembly::assemble_rhs;
use crate::state::SimulationState;
use crate::strategy::{SolverStrategy, StepResult};

use super::ProjectiveDynamicsSolver;

impl SolverStrategy for ProjectiveDynamicsSolver {
    fn init(
        &mut self,
        mesh: &vistio_mesh::TriangleMesh,
        topology: &vistio_mesh::topology::Topology,
        config: &crate::config::SolverConfig,
        pinned: &[bool],
    ) -> VistioResult<()> {
        self.n = mesh.vertex_count();
        self.config = config.clone();

        // Build FEM elements with stiffness from config (normalized to physical range)
        let elements = crate::element::ElementData::from_mesh(mesh, config.stretch_weight * 500.0);

        // Default density 200 g/m² for the base config
        let density = 200.0;
        self.mass = super::mass::compute_lumped_masses(self.n, &elements, density, pinned);

        // Build bending elements from topology (normalized to physical range)
        let bending = crate::bending::BendingData::from_topology(mesh, topology, config.bending_weight * 1.0);
        self.bending = Some(bending);
        self.ds_bending = None;

        // Assemble the constant system matrix (includes bending stiffness)
        let dt = 1.0 / 60.0;
        let bending_model = self.bending.as_ref().map(crate::assembly::BendingModel::Dihedral);
        let system_matrix = crate::assembly::assemble_system_matrix(self.n, &self.mass, dt, &elements, bending_model);

        // Prefactor via sparse Cholesky
        use vistio_math::sparse::SparseSolver;
        self.solver.factorize(&system_matrix).map_err(|e| {
            vistio_types::VistioError::InvalidConfig(format!("Cholesky factorization failed: {e}"))
        })?;

        self.elements = Some(elements);
        self.material_model = None;
        self.initialized = true;
        Ok(())
    }

    fn step(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
    ) -> VistioResult<StepResult> {
        if !self.initialized {
            return Err(vistio_types::VistioError::InvalidConfig(
                "Solver not initialized. Call init() first.".into(),
            ));
        }

        let start = Instant::now();
        let elements = self.elements.as_ref().unwrap();
        let n = self.n;

        // 1. Save previous positions
        state.save_previous();

        // 2. Predict: q_pred = pos + dt*vel + dt²*gravity
        state.predict(dt, self.config.gravity);

        // 3. Initialize current positions from predictions (initial guess)
        state.pos_x.copy_from_slice(&state.pred_x);
        state.pos_y.copy_from_slice(&state.pred_y);
        state.pos_z.copy_from_slice(&state.pred_z);

        // Buffers for the global solve
        let mut sol_x = vec![0.0_f32; n];
        let mut sol_y = vec![0.0_f32; n];
        let mut sol_z = vec![0.0_f32; n];

        let mut iterations = 0_u32;
        let mut final_residual = f64::MAX;

        // 4. Local-Global iteration loop
        for iter in 0..self.config.max_iterations {
            // === LOCAL STEP: MEMBRANE ===
            let mut proj_x = Vec::with_capacity(elements.len());
            let mut proj_y = Vec::with_capacity(elements.len());
            let mut proj_z = Vec::with_capacity(elements.len());

            for elem in &elements.elements {
                let (p0, p1, p2) = if let Some(ref model) = self.material_model {
                    elements.project_with_model(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                        model.as_ref(),
                    )
                } else {
                    elements.project(
                        elem,
                        &state.pos_x,
                        &state.pos_y,
                        &state.pos_z,
                    )
                };
                proj_x.push((p0.x, p1.x, p2.x));
                proj_y.push((p0.y, p1.y, p2.y));
                proj_z.push((p0.z, p1.z, p2.z));
            }

            // === LOCAL STEP: BENDING ===
            // Compute bending projection targets for the RHS assembly.
            // Both dihedral (Tier 1-2) and Discrete Shells (Tier 3) bending are
            // fully integrated into the local-global loop via the RHS.
            let (bend_targets_x, bend_targets_y, bend_targets_z) =
                if let Some(ref ds_bend) = self.ds_bending {
                    // Discrete Shells path
                    let mut btx = Vec::with_capacity(ds_bend.len());
                    let mut bty = Vec::with_capacity(ds_bend.len());
                    let mut btz = Vec::with_capacity(ds_bend.len());

                    for elem in &ds_bend.elements {
                        let (p_v0, p_v1, p_wa, p_wb) = ds_bend.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        );
                        btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                        bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                        btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                    }

                    (Some(btx), Some(bty), Some(btz))
                } else if let Some(ref bending) = self.bending {
                    // Dihedral path — compute projections for RHS integration
                    let mut btx = Vec::with_capacity(bending.len());
                    let mut bty = Vec::with_capacity(bending.len());
                    let mut btz = Vec::with_capacity(bending.len());

                    for elem in &bending.elements {
                        let (p_v0, p_v1, p_wa, p_wb) = bending.project(
                            elem,
                            &state.pos_x,
                            &state.pos_y,
                            &state.pos_z,
                        );
                        btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                        bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                        btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
                    }

                    (Some(btx), Some(bty), Some(btz))
                } else {
                    (None, None, None)
                };

            // === GLOBAL STEP ===
            // Build bending RHS references
            let bending_rhs_x = self.build_bending_rhs(
                bend_targets_x.as_deref(),
            );
            let bending_rhs_y = self.build_bending_rhs(
                bend_targets_y.as_deref(),
            );
            let bending_rhs_z = self.build_bending_rhs(
                bend_targets_z.as_deref(),
            );

            let rhs_x = assemble_rhs(
                n, &self.mass, dt, &state.pred_x, &proj_x, elements, 0, bending_rhs_x,
            );
            let rhs_y = assemble_rhs(
                n, &self.mass, dt, &state.pred_y, &proj_y, elements, 1, bending_rhs_y,
            );
            let rhs_z = assemble_rhs(
                n, &self.mass, dt, &state.pred_z, &proj_z, elements, 2, bending_rhs_z,
            );

            // Solve A * q = rhs (three backsubstitutions)
            use vistio_math::sparse::SparseSolver;
            self.solver.solve(&rhs_x, &mut sol_x).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("X solve failed: {e}"))
            })?;
            self.solver.solve(&rhs_y, &mut sol_y).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("Y solve failed: {e}"))
            })?;
            self.solver.solve(&rhs_z, &mut sol_z).map_err(|e| {
                vistio_types::VistioError::InvalidConfig(format!("Z solve failed: {e}"))
            })?;

            // Compute convergence: ||q_new - q_old||² / ||q_old||²
            let mut diff_sq = 0.0_f64;
            let mut norm_sq = 0.0_f64;
            for i in 0..n {
                let dx = (sol_x[i] - state.pos_x[i]) as f64;
                let dy = (sol_y[i] - state.pos_y[i]) as f64;
                let dz = (sol_z[i] - state.pos_z[i]) as f64;
                diff_sq += dx * dx + dy * dy + dz * dz;

                let ox = state.pos_x[i] as f64;
                let oy = state.pos_y[i] as f64;
                let oz = state.pos_z[i] as f64;
                norm_sq += ox * ox + oy * oy + oz * oz;
            }

            final_residual = if norm_sq > 1e-12 {
                (diff_sq / norm_sq).sqrt()
            } else {
                diff_sq.sqrt()
            };

            // Update positions
            state.pos_x.copy_from_slice(&sol_x);
            state.pos_y.copy_from_slice(&sol_y);
            state.pos_z.copy_from_slice(&sol_z);

            // Enforce pinning constraints
            for i in 0..n {
                if state.inv_mass[i] == 0.0 {
                    state.pos_x[i] = state.prev_x[i];
                    state.pos_y[i] = state.prev_y[i];
                    state.pos_z[i] = state.prev_z[i];
                }
            }

            // Enforce ground plane constraint
            state.enforce_ground();

            iterations = iter + 1;

            if final_residual < self.config.tolerance {
                break;
            }
        }

        // 5. Update velocities from position difference
        state.update_velocities(dt);

        // 6. Enforce ground velocity constraints
        state.enforce_ground_velocities();

        // 7. Apply basic damping
        state.damp_velocities(self.config.damping);

        // 8. Rayleigh mass-proportional damping: v *= 1 / (1 + α_M * dt)
        if self.config.rayleigh_mass_damping > 0.0 {
            let factor = 1.0 / (1.0 + self.config.rayleigh_mass_damping * dt);
            for i in 0..n {
                if state.inv_mass[i] > 0.0 {
                    state.vel_x[i] *= factor;
                    state.vel_y[i] *= factor;
                    state.vel_z[i] *= factor;
                }
            }
        }

        let wall_time = start.elapsed().as_secs_f64();

        Ok(StepResult {
            iterations,
            final_residual,
            converged: final_residual < self.config.tolerance,
            wall_time,
        })
    }

    fn name(&self) -> &str {
        "ProjectiveDynamics"
    }
}
