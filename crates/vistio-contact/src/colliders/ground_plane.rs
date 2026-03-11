//! Ground plane collision.
//!
//! A simple collision object that prevents vertices from penetrating
//! a horizontal ground plane at `y = height`.

use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Ground plane collision at a fixed Y height.
///
/// Generates contacts for any vertex below `y = height` and applies
/// direct position correction to push them above the plane.
pub struct GroundPlane {
    /// Height of the ground plane (Y coordinate).
    pub height: f32,
}

impl GroundPlane {
    /// Creates a new ground plane at the given height.
    pub fn new(height: f32) -> Self {
        Self { height }
    }

    /// Resolve ground plane contacts by projecting vertices above the plane.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_penetration = 0.0_f32;
        let mut total_force = 0.0_f32;

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 {
                continue;
            }
            let depth = self.height - state.pos_y[i];
            if depth > 0.0 {
                state.pos_y[i] = self.height;

                // Also update prev_y so the velocity computation
                // (pos - prev) / dt doesn't produce spurious downward
                // velocity next step — this prevents jelly oscillation.
                state.prev_y[i] = self.height;

                // Apply simple kinetic friction to tangential velocities
                state.vel_x[i] *= 0.5;
                state.vel_z[i] *= 0.5;

                // Perfectly inelastic impulse against the floor:
                // If the vertex is moving downwards (into the floor), stop it.
                if state.vel_y[i] < 0.0 {
                    state.vel_y[i] = 0.0;
                }

                resolved += 1;
                max_penetration = max_penetration.max(depth);
                total_force += depth;
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_penetration,
            total_force_magnitude: total_force,
        }
    }

    /// Compute IPC barrier gradients for vertices near the ground.
    pub fn detect_ipc_contacts(
        &self,
        pos_y: &[f32],
        d_hat: f32,
        kappa: f32,
        grad_y: &mut [f32],
    ) -> (usize, f32) {
        let mut active = 0;
        let mut max_violation = 0.0_f32;

        for i in 0..pos_y.len() {
            let d_surface = pos_y[i] - self.height;

            if d_surface <= 0.0 {
                // Inside the ground -> violation!
                max_violation = max_violation.max(-d_surface);

                // Evaluate at 1e-12 to generate max repulsive force.
                // Do NOT use penetration depth, as it evaluates to 0 if depth > d_hat.
                let dist_sq = 1e-12_f32;
                active += 1;
                let barrier_grad = crate::barrier::scaled_barrier_gradient(dist_sq, d_hat, kappa);

                let penalty = 1.0 * kappa * d_surface;
                let mut factor = barrier_grad * 2.0 * 1e-6 + penalty;
                factor = factor.clamp(-10.0, 10.0);
                grad_y[i] += factor;
            } else {
                let dist_sq = d_surface * d_surface;
                if dist_sq < d_hat {
                    active += 1;
                    let barrier_grad = crate::barrier::scaled_barrier_gradient(dist_sq, d_hat, kappa);

                    let mut factor = barrier_grad * 2.0 * d_surface;
                    factor = factor.clamp(-500.0, 50.0);
                    grad_y[i] += factor;

                    // Do NOT report barrier-zone proximity as violation for AL loop
                    // as it causes infinite penalty growth.
                }
            }
        }
        (active, max_violation)
    }

    /// Compute maximum safe step size to prevent vertices from passing through the ground.
    pub fn compute_ccd_step(
        &self,
        prev_y: &[f32],
        new_y: &[f32],
        padding: f32,
    ) -> f32 {
        let mut min_toi: f32 = 1.0;

        for i in 0..prev_y.len() {
            let py0 = prev_y[i];
            let py1 = new_y[i];

            let eff_height = self.height + padding;

            if py0 <= eff_height {
                // Already inside padded zone. If moving closer to ground, stop.
                if py1 < py0 {
                    min_toi = 0.0;
                }
                continue;
            }

            if py0 > eff_height && py1 < eff_height {
                let vy = py1 - py0;
                let t = (eff_height - py0) / vy;

                if (0.0..=1.0).contains(&t) {
                    min_toi = min_toi.min(t * 0.9);
                }
            }
        }

        min_toi.max(1e-6)
    }
}
