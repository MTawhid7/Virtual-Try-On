//! Analytical sphere collision.
//!
//! A simple collision object that prevents vertices from penetrating
//! a sphere of a given radius and center.

use vistio_math::Vec3;
use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Analytical sphere collision.
///
/// Generates contacts for any vertex inside the sphere and applies
/// direct position correction to push them to the surface.
pub struct SphereCollider {
    /// Center of the sphere.
    pub center: Vec3,
    /// Radius of the sphere.
    pub radius: f32,
}

impl SphereCollider {
    /// Creates a new sphere collider.
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }

    /// Resolve sphere contacts by projecting penetrating vertices to the surface.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_penetration = 0.0_f32;
        let mut total_force = 0.0_f32;

        let r2 = self.radius * self.radius;

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 {
                continue;
            }

            let dx = state.pos_x[i] - self.center.x;
            let dy = state.pos_y[i] - self.center.y;
            let dz = state.pos_z[i] - self.center.z;

            let dist2 = dx * dx + dy * dy + dz * dz;

            if dist2 < r2 && dist2 > 1e-12 {
                let dist = dist2.sqrt();
                let depth = self.radius - dist;

                let nx = dx / dist;
                let ny = dy / dist;
                let nz = dz / dist;

                let dx_corr = nx * depth;
                let dy_corr = ny * depth;
                let dz_corr = nz * depth;

                // Push to surface
                state.pos_x[i] += dx_corr;
                state.pos_y[i] += dy_corr;
                state.pos_z[i] += dz_corr;

                // Remove velocity component along the normal and apply kinetic friction
                let v_dot_n = state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz;
                if v_dot_n < 0.0 {
                    let v_tangent_x = state.vel_x[i] - nx * v_dot_n;
                    let v_tangent_y = state.vel_y[i] - ny * v_dot_n;
                    let v_tangent_z = state.vel_z[i] - nz * v_dot_n;

                    let friction = 0.5;
                    state.vel_x[i] = v_tangent_x * friction;
                    state.vel_y[i] = v_tangent_y * friction;
                    state.vel_z[i] = v_tangent_z * friction;
                }

                resolved += 1;
                max_penetration = max_penetration.max(depth);
                total_force += depth;
            } else if dist2 <= 1e-12 {
                // Exactly at center, push up
                state.pos_y[i] += self.radius;
                // Purely inelastic velocity damping
                state.vel_x[i] = 0.0;
                state.vel_y[i] = 0.0;
                state.vel_z[i] = 0.0;

                resolved += 1;
                max_penetration = max_penetration.max(self.radius);
                total_force += self.radius;
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_penetration,
            total_force_magnitude: total_force,
        }
    }

    /// Compute IPC barrier gradients for vertices near the sphere.
    pub fn detect_ipc_contacts(
        &self,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        d_hat: f32,
        kappa: f32,
        grad_x: &mut [f32],
        grad_y: &mut [f32],
        grad_z: &mut [f32],
    ) -> (usize, f32) {
        let mut active = 0;
        let mut max_violation = 0.0_f32;

        #[cfg(debug_assertions)]
        {
            let min_d = (0..pos_x.len()).map(|i| {
                let ddx = pos_x[i] - self.center.x;
                let ddy = pos_y[i] - self.center.y;
                let ddz = pos_z[i] - self.center.z;
                (ddx*ddx + ddy*ddy + ddz*ddz).sqrt() - self.radius
            }).fold(f32::INFINITY, f32::min);
            if min_d < 0.1 {
                eprintln!("    SPHERE: d_hat={:.6} kappa={:.1} min_d_surface={:.6} min_d²={:.8} d_surface<√d_hat={}",
                    d_hat, kappa, min_d, min_d*min_d, min_d < d_hat.sqrt());
            }
        }

        for i in 0..pos_x.len() {
            let dx = pos_x[i] - self.center.x;
            let dy = pos_y[i] - self.center.y;
            let dz = pos_z[i] - self.center.z;

            let r = (dx * dx + dy * dy + dz * dz).sqrt();
            let d_surface = r - self.radius;

            if d_surface <= 0.0 {
                // Inside the sphere -> violation!
                max_violation = max_violation.max(-d_surface);

                // Evaluate barrier at an infinitesimally small distance (1e-6) so it
                // generates maximum repulsive force. Do NOT evaluate at penetration depth,
                // because if depth > d_hat, the barrier function returns 0!
                let dist_sq = 1e-12_f32;
                active += 1;
                let barrier_grad = crate::barrier::scaled_barrier_gradient(dist_sq, d_hat, kappa);

                // Note: d_surface is NEGATIVE here. A negative gradient means energy decreases
                // as distance INCREASES (which pushes the vertex outward).
                let penalty = 1.0 * kappa * d_surface;

                if r > 1e-6 {
                    // chain rule: dE/dx = (barrier_grad * 2d + penalty) * (dx/r)
                    let mut factor = barrier_grad * 2.0 * 1e-6 + penalty;

                    // CRITICAL: The Augmented Lagrangian solver integrates this explicitly!
                    // If the gradient exceeds the explicit mass-inertia limit, the solver will
                    // explode geometrically. We MUST mathematically cap the maximum explicit gradient.
                    factor = factor.max(-10.0).min(10.0);

                    grad_x[i] += factor * (dx / r);
                    grad_y[i] += factor * (dy / r);
                    grad_z[i] += factor * (dz / r);
                } else {
                    let mut factor = barrier_grad * 2.0 * 1e-6 + penalty;
                    factor = factor.max(-10.0).min(10.0);
                    grad_y[i] += factor;
                }
            } else {
                let dist_sq = d_surface * d_surface;
                if dist_sq < d_hat {
                    active += 1;
                    let barrier_grad = crate::barrier::scaled_barrier_gradient(dist_sq, d_hat, kappa);

                    let mut factor = barrier_grad * 2.0 * d_surface;
                    factor = factor.max(-500.0).min(50.0);

                    grad_x[i] += factor * (dx / r);
                    grad_y[i] += factor * (dy / r);
                    grad_z[i] += factor * (dz / r);

                    // Do NOT report proximity as a constraint violation.
                    // Doing so causes mu to grow to infinity during normal resting contact.
                }
            }
        }
        (active, max_violation)
    }

    /// Compute maximum safe step size to prevent vertices from entering the sphere.
    pub fn compute_ccd_step(
        &self,
        prev_x: &[f32], prev_y: &[f32], prev_z: &[f32],
        new_x: &[f32], new_y: &[f32], new_z: &[f32],
        padding: f32,
    ) -> f32 {
        let mut min_toi: f32 = 1.0;
        let r_eff = self.radius + padding;
        let r2 = r_eff * r_eff;

        for i in 0..prev_x.len() {
            let px0 = prev_x[i] - self.center.x;
            let py0 = prev_y[i] - self.center.y;
            let pz0 = prev_z[i] - self.center.z;

            let px1 = new_x[i] - self.center.x;
            let py1 = new_y[i] - self.center.y;
            let pz1 = new_z[i] - self.center.z;

            let vx = px1 - px0;
            let vy = py1 - py0;
            let vz = pz1 - pz0;

            // Check if vertex STARTS inside the padded sphere
            let start_dist_sq = px0 * px0 + py0 * py0 + pz0 * pz0;
            if start_dist_sq < r2 {
                let dot_pv = px0 * vx + py0 * vy + pz0 * vz;
                if dot_pv < 0.0 {
                    // Moving deeper into the barrier! Stop immediately.
                    min_toi = 0.0;
                }
                // If dot_pv >= 0, moving outwards, skip CCD to let it exit.
                continue;
            }

            // (px0 + t*vx)^2 + (py0 + t*vy)^2 + (pz0 + t*vz)^2 = r^2
            let a = vx * vx + vy * vy + vz * vz;
            let b = 2.0 * (px0 * vx + py0 * vy + pz0 * vz);
            let c = px0 * px0 + py0 * py0 + pz0 * pz0 - r2;

            if a > 1e-8 {
                let disc = b * b - 4.0 * a * c;
                if disc >= 0.0 {
                     let t1 = (-b - disc.sqrt()) / (2.0 * a);

                    // We ONLY care about the entry time (t1).
                    // If t1 < 0, the vertex started inside the barrier (c < 0) and is exiting at t2.
                    // We must NEVER clip exits (t2), otherwise IPC forces will trap vertices!
                    if t1 > 0.0 && t1 < 1.0 {
                        min_toi = min_toi.min(t1 * 0.9);
                    }
                }
            }
        }
        min_toi.max(1e-6)
    }
}
