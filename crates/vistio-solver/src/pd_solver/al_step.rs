//! Augmented Lagrangian solver step with IPC barrier contact.
//!
//! Contains `step_with_ipc()` — the Tier 4 AL outer loop wrapping the
//! standard PD local-global inner loop with barrier force integration.

use std::time::Instant;

use vistio_types::VistioResult;

use vistio_math::sparse::SparseSolver;

use crate::assembly::{assemble_rhs, assemble_barrier_rhs};
use crate::state::SimulationState;
use crate::strategy::StepResult;

use super::ipc::{IpcCollisionHandler, IpcBarrierForces};
use super::velocity_filter::apply_contact_velocity_filter;
use super::ProjectiveDynamicsSolver;

impl ProjectiveDynamicsSolver {
    /// Advance one timestep with IPC barrier contact using Augmented Lagrangian.
    ///
    /// This is the Tier 4 solver step. It wraps the standard PD local-global
    /// loop inside an outer AL loop that enforces contact constraints:
    ///
    /// ```text
    /// for al_iter in 0..al_max_iterations:
    ///     barrier_forces = detect_and_compute_forces(positions)
    ///     for pd_iter in 0..max_iterations:
    ///         local_step(positions)
    ///         rhs = assemble_rhs(pred, projections, bending)
    ///         rhs += -(mu * barrier_grad + lambda)   // IPC forces
    ///         positions = solve(A, rhs)
    ///     lambda += mu * constraint_violation
    ///     if ||constraint_violation|| < tolerance: break
    ///     else: mu *= growth_factor
    /// ```
    pub fn step_with_ipc<H: IpcCollisionHandler>(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
        handler: &mut H,
    ) -> VistioResult<StepResult>
    {
        if !self.initialized {
            return Err(vistio_types::VistioError::InvalidConfig(
                "Solver not initialized. Call init() first.".into(),
            ));
        }

        let start = Instant::now();
        let elements = self.elements.as_ref().unwrap();
        let n = self.n;

        // Calculate effective padding.
        let d_hat_eff = if self.config.compliant_contact {
            self.config.barrier_d_hat * self.config.compliant_d_hat_scale
        } else {
            self.config.barrier_d_hat
        };

        // Projective Dynamics requires a CONSTANT system matrix. This matrix is pre-factorized
        // during `init()` using a hardcoded `dt = 1.0 / 60.0`. If we change `sub_dt` dynamically,
        // the RHS inertia scalar `m / sub_dt^2` will no longer match the matrix diagonal,
        // causing instant instability and violent explosions.
        // Therefore, we MUST use a constant `num_substeps`.
        let num_substeps = 1;
        let sub_dt = dt / num_substeps as f32;

        let mut total_iterations_sum = 0;
        let mut final_residual_last = 0.0;
        let mut converged_all = true;

        for _substep in 0..num_substeps {
            // 1. Save previous positions
            state.save_previous();

            // 2. Predict
            state.predict(sub_dt, self.config.gravity);

            if state.pred_y.iter().any(|&v| !v.is_finite()) {
                #[cfg(debug_assertions)]
                eprintln!("TRAP: NaN/Inf in state.pred_y right after predict! (substep {})", _substep);
                return Ok(StepResult { iterations: total_iterations_sum, converged: false, final_residual: 0.0, wall_time: 0.0 });
            }

            // 3. Initialize positions from previous safe state
            // DO NOT copy from pred, because pred might be penetrating colliders!
            // PD Iterative solver will pull pos towards pred safely using CCD.
            // state.pos_x/y/z are already set to prev at the start of the timestep.
            // (Actually, wait, the `state.save_previous()` leaves `state.pos` at `prev`! So we do nothing.)

            // Warm-start: only clear AL multipliers for vertices NOT in contact.
            // Vertices still in contact keep their multipliers for faster convergence.
            // On the first substep we clear everything since contact state is stale.
            if _substep == 0 {
                state.al_lambda_x.fill(0.0);
                state.al_lambda_y.fill(0.0);
                state.al_lambda_z.fill(0.0);
            }

            let mut sol_x = vec![0.0_f32; n];
            let mut sol_y = vec![0.0_f32; n];
            let mut sol_z = vec![0.0_f32; n];

            handler.set_d_hat(d_hat_eff);

            let mut mu = self.config.al_mu_initial;
            let mut total_iterations = 0_u32;
            let mut final_residual = f64::MAX;
            let mut al_converged = false;

            let mut updated_forces = IpcBarrierForces::empty(n);
            let mut ever_in_contact = vec![false; n];
            let mut final_contact_nx = vec![0.0_f32; n];
            let mut final_contact_ny = vec![0.0_f32; n];
            let mut final_contact_nz = vec![0.0_f32; n];

            // OUTER LOOP: Augmented Lagrangian
            let mut previous_violation = f32::MAX;
            for _al_iter in 0..self.config.al_max_iterations {

                // Detect contacts at current positions
                let barrier_forces = handler.detect_contacts(
                    &state.pos_x, &state.pos_y, &state.pos_z,
                );

                if barrier_forces.grad_y.iter().any(|&v| !v.is_finite()) {
                    #[cfg(debug_assertions)]
                    eprintln!("TRAP: NaN/Inf in barrier_forces.grad_y detected!");
                    return Ok(StepResult { iterations: total_iterations, converged: false, final_residual: 0.0, wall_time: 0.0 });
                }

                // Track contact state
                for i in 0..n {
                    if barrier_forces.in_contact[i] {
                        ever_in_contact[i] = true;
                        final_contact_nx[i] = barrier_forces.contact_nx[i];
                        final_contact_ny[i] = barrier_forces.contact_ny[i];
                        final_contact_nz[i] = barrier_forces.contact_nz[i];
                    }
                }

                // Clear lambda for vertices that left the contact zone
                for i in 0..n {
                    if !barrier_forces.in_contact[i] {
                        state.al_lambda_x[i] = 0.0;
                        state.al_lambda_y[i] = 0.0;
                        state.al_lambda_z[i] = 0.0;
                    }
                }

                let (iters, residual) = self.run_pd_inner_loop(
                    state, elements, n, sub_dt,
                    &barrier_forces, mu,
                    &mut sol_x, &mut sol_y, &mut sol_z,
                    handler, _al_iter, d_hat_eff,
                )?;
                total_iterations += iters;
                final_residual = residual;

                // Re-detect contacts after PD converged
                updated_forces = handler.detect_contacts(
                    &state.pos_x, &state.pos_y, &state.pos_z,
                );

                for i in 0..n {
                    if updated_forces.in_contact[i] {
                        ever_in_contact[i] = true;
                        final_contact_nx[i] = updated_forces.contact_nx[i];
                        final_contact_ny[i] = updated_forces.contact_ny[i];
                        final_contact_nz[i] = updated_forces.contact_nz[i];
                    }
                }

                // Check AL convergence
                if updated_forces.max_violation < self.config.al_tolerance {
                    al_converged = true;
                    break;
                }

                // Break if plateaued
                if (previous_violation - updated_forces.max_violation).abs() < 1e-6
                    && updated_forces.max_violation < d_hat_eff {
                    break;
                }
                previous_violation = updated_forces.max_violation;

                // Update AL multipliers
                for i in 0..n {
                    if updated_forces.in_contact[i] {
                        state.al_lambda_x[i] += mu * updated_forces.grad_x[i];
                        state.al_lambda_y[i] += mu * updated_forces.grad_y[i];
                        state.al_lambda_z[i] += mu * updated_forces.grad_z[i];
                    }
                }

                mu *= self.config.al_mu_growth;
                if mu > 1e6 { mu = 1e6; }

                #[cfg(debug_assertions)]
                if _al_iter == self.config.al_max_iterations - 1 {
                    eprintln!("  [AL Outer] reached max iters ({}), violation={:.6}", _al_iter + 1, updated_forces.max_violation);
                }
            } // outer loop

            // Velocity update for this substep
            state.update_velocities(sub_dt);

            // Apply consolidated velocity filter
            apply_contact_velocity_filter(
                state,
                &self.config,
                &ever_in_contact,
                &final_contact_nx,
                &final_contact_ny,
                &final_contact_nz,
                updated_forces.active_contacts,
                sub_dt,
            );

            if state.vel_y.iter().any(|&v| !v.is_finite()) {
                #[cfg(debug_assertions)]
                eprintln!("TRAP: NaN/Inf in state.vel_y after velocity filter! (substep {})", _substep);
                return Ok(StepResult { iterations: total_iterations_sum, converged: false, final_residual: 0.0, wall_time: 0.0 });
            }

            total_iterations_sum += total_iterations;
            final_residual_last = final_residual;
            if !al_converged && final_residual >= self.config.tolerance {
                converged_all = false;
            }
        } // substep loop

        let wall_time = start.elapsed().as_secs_f64();

        Ok(StepResult {
            iterations: total_iterations_sum,
            final_residual: final_residual_last,
            converged: converged_all,
            wall_time,
        })
    }

    /// Run the PD inner local-global loop for a single AL iteration.
    ///
    /// Returns (iterations_run, final_residual).
    #[allow(clippy::too_many_arguments)]
    fn run_pd_inner_loop<H: IpcCollisionHandler>(
        &self,
        state: &mut SimulationState,
        elements: &crate::element::ElementData,
        n: usize,
        sub_dt: f32,
        barrier_forces: &IpcBarrierForces,
        mu: f32,
        sol_x: &mut [f32],
        sol_y: &mut [f32],
        sol_z: &mut [f32],
        handler: &mut H,
        _al_iter: u32,
        d_hat_eff: f32,
    ) -> VistioResult<(u32, f64)>
    {
        let mut total_iterations = 0_u32;
        let mut final_residual = f64::MAX;

        for _iter in 0..self.config.max_iterations {
            // Local step: project each element
            let mut proj_x = Vec::with_capacity(elements.len());
            let mut proj_y = Vec::with_capacity(elements.len());
            let mut proj_z = Vec::with_capacity(elements.len());

            for elem in &elements.elements {
                let (p0, p1, p2) = if let Some(ref model) = self.material_model {
                    elements.project_with_model(
                        elem, &state.pos_x, &state.pos_y, &state.pos_z, model.as_ref(),
                    )
                } else {
                    elements.project(
                        elem, &state.pos_x, &state.pos_y, &state.pos_z,
                    )
                };
                proj_x.push((p0.x, p1.x, p2.x));
                proj_y.push((p0.y, p1.y, p2.y));
                proj_z.push((p0.z, p1.z, p2.z));
            }

            if proj_y.iter().any(|&(y0, y1, y2)| !y0.is_finite() || !y1.is_finite() || !y2.is_finite()) {
                #[cfg(debug_assertions)]
                eprintln!("TRAP: NaN/Inf in membrane projection targets! (iter {}, al_iter {})", _iter, _al_iter);
                return Ok((total_iterations, 0.0));
            }

            // Bending projections
            let (bend_targets_x, bend_targets_y, bend_targets_z) =
                self.compute_bending_projections(state);

            if let Some(ref bt_y) = bend_targets_y {
                if bt_y.iter().any(|&(y0, y1, y2, y3)| !y0.is_finite() || !y1.is_finite() || !y2.is_finite() || !y3.is_finite()) {
                    #[cfg(debug_assertions)]
                    eprintln!("TRAP: NaN/Inf in bending projection targets! (iter {}, al_iter {})", _iter, _al_iter);
                    return Ok((total_iterations, 0.0));
                }
            }

            let bending_rhs_x = self.build_bending_rhs(bend_targets_x.as_deref());
            let bending_rhs_y = self.build_bending_rhs(bend_targets_y.as_deref());
            let bending_rhs_z = self.build_bending_rhs(bend_targets_z.as_deref());

            // Global step: assemble RHS
            let mut rhs_x = assemble_rhs(
                n, &self.mass, sub_dt, &state.pred_x, &proj_x, elements, 0, bending_rhs_x,
            );
            let mut rhs_y = assemble_rhs(
                n, &self.mass, sub_dt, &state.pred_y, &proj_y, elements, 1, bending_rhs_y,
            );
            let mut rhs_z = assemble_rhs(
                n, &self.mass, sub_dt, &state.pred_z, &proj_z, elements, 2, bending_rhs_z,
            );

            if rhs_y.iter().any(|&v| !v.is_finite()) {
                #[cfg(debug_assertions)]
                eprintln!("TRAP: NaN/Inf in rhs_y BEFORE barrier RHS! (iter {}, al_iter {})", _iter, _al_iter);
                return Ok((total_iterations, 0.0));
            }

            // Add barrier forces to RHS
            assemble_barrier_rhs(&mut rhs_x, &barrier_forces.grad_x, &state.al_lambda_x, mu);
            assemble_barrier_rhs(&mut rhs_y, &barrier_forces.grad_y, &state.al_lambda_y, mu);
            assemble_barrier_rhs(&mut rhs_z, &barrier_forces.grad_z, &state.al_lambda_z, mu);

            if rhs_y.iter().any(|&v| v.is_nan()) {
                #[cfg(debug_assertions)]
                eprintln!("TRAP: NaN in rhs_y after barrier RHS! (iter {}, al_iter {})", _iter, _al_iter);
                return Ok((total_iterations, 0.0));
            }

            // Solve linear system
            self.solver.solve(&rhs_x, sol_x).map_err(|e| vistio_types::VistioError::InvalidConfig(format!("X solve failed: {e}")))?;
            self.solver.solve(&rhs_y, sol_y).map_err(|e| vistio_types::VistioError::InvalidConfig(format!("Y solve failed: {e}")))?;
            self.solver.solve(&rhs_z, sol_z).map_err(|e| vistio_types::VistioError::InvalidConfig(format!("Z solve failed: {e}")))?;

            // Compute convergence residual
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

            if sol_y.iter().any(|&v| !v.is_finite()) {
                #[cfg(debug_assertions)]
                eprintln!("TRAP: NaN/Inf in linear solver output! (iter {}, al_iter {})", _iter, _al_iter);
                return Ok((total_iterations, 0.0));
            }

            // CCD line search
            // Padding forces CCD to stop vertices at the halfway point of the barrier zone (rather than the exact object skin).
            let ccd_padding = d_hat_eff.sqrt() * 0.5;
            let mut max_alpha = handler.compute_ccd_step(
                &state.pos_x, &state.pos_y, &state.pos_z,
                sol_x, sol_y, sol_z,
                ccd_padding,
            );
            if max_alpha < 1.0 {
                max_alpha *= 0.8;
            }
            max_alpha = max_alpha.min(1.0);

            // Armijo backtracking
            let armijo_c = 1e-4_f32;
            let mut alpha = max_alpha;
            for _bt in 0..4 {
                if alpha < 0.1 { break; }
                let mut grad_dot_d = 0.0_f64;
                for i in 0..n {
                    if state.inv_mass[i] > 0.0 {
                        let dx = sol_x[i] - state.pos_x[i];
                        let dy = sol_y[i] - state.pos_y[i];
                        let dz = sol_z[i] - state.pos_z[i];
                        grad_dot_d += (barrier_forces.grad_x[i] * dx
                            + barrier_forces.grad_y[i] * dy
                            + barrier_forces.grad_z[i] * dz) as f64;
                    }
                }
                if grad_dot_d * (mu as f64) > (armijo_c as f64) * diff_sq * 100.0 {
                    alpha *= 0.5;
                } else {
                    break;
                }
            }

            // Apply position update with displacement capping
            for i in 0..n {
                if state.inv_mass[i] > 0.0 {
                    let mut dx = alpha * (sol_x[i] - state.pos_x[i]);
                    let mut dy = alpha * (sol_y[i] - state.pos_y[i]);
                    let mut dz = alpha * (sol_z[i] - state.pos_z[i]);

                // Cap maximum displacement to prevent infinity explosions
                let disp_len_sq = dx*dx + dy*dy + dz*dz;
                if disp_len_sq > 100.0 {
                    #[cfg(debug_assertions)]
                    eprintln!("TRAP: Massive displacement! dx={}, dy={}, dz={}, sol_y={}, prev_y={}, b_y_val={}, diag={}, m={}, H={}",
                        dx, dy, dz, sol_y[i], state.pos_y[i], barrier_forces.grad_y[i],
                        (state.mass[i] / (sub_dt * sub_dt)) + mu * barrier_forces.hessian_diag[i],
                        state.mass[i], barrier_forces.hessian_diag[i]
                    );

                    let scale = 1.0 / disp_len_sq.sqrt();
                    dx *= scale;
                    dy *= scale;
                    dz *= scale;
                }

                    state.pos_x[i] += dx;
                    state.pos_y[i] += dy;
                    state.pos_z[i] += dz;
                }
            }

            total_iterations += 1;

            if final_residual < self.config.tolerance {
                break;
            }

            #[cfg(debug_assertions)]
            if _iter == self.config.max_iterations - 1 {
                eprintln!("    [PD Inner] reached max iters ({}), residual={:.6}", _iter + 1, final_residual);
            }
        }

        Ok((total_iterations, final_residual))
    }

    /// Compute bending projection targets for all bending elements.
    #[allow(clippy::type_complexity)]
    fn compute_bending_projections(
        &self,
        state: &SimulationState,
    ) -> (
        Option<Vec<(f32, f32, f32, f32)>>,
        Option<Vec<(f32, f32, f32, f32)>>,
        Option<Vec<(f32, f32, f32, f32)>>,
    ) {
        if let Some(ref ds_bend) = self.ds_bending {
            let mut btx = Vec::with_capacity(ds_bend.len());
            let mut bty = Vec::with_capacity(ds_bend.len());
            let mut btz = Vec::with_capacity(ds_bend.len());
            for elem in &ds_bend.elements {
                let (p_v0, p_v1, p_wa, p_wb) = ds_bend.project(
                    elem, &state.pos_x, &state.pos_y, &state.pos_z,
                );
                btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
            }
            (Some(btx), Some(bty), Some(btz))
        } else if let Some(ref bending) = self.bending {
            let mut btx = Vec::with_capacity(bending.len());
            let mut bty = Vec::with_capacity(bending.len());
            let mut btz = Vec::with_capacity(bending.len());
            for elem in &bending.elements {
                let (p_v0, p_v1, p_wa, p_wb) = bending.project(
                    elem, &state.pos_x, &state.pos_y, &state.pos_z,
                );
                btx.push((p_v0.x, p_v1.x, p_wa.x, p_wb.x));
                bty.push((p_v0.y, p_v1.y, p_wa.y, p_wb.y));
                btz.push((p_v0.z, p_v1.z, p_wa.z, p_wb.z));
            }
            (Some(btx), Some(bty), Some(btz))
        } else {
            (None, None, None)
        }
    }
}

