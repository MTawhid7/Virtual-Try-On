//! Analytical vertical cylinder collision.
//!
//! A simple collision object that prevents vertices from falling down through
//! the top face or passing inside the cylinder's walls.
//! Suitable for the Cusick Drape pedestal.

use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Analytical vertical cylinder collider.
pub struct CylinderCollider {
    /// X coordinate of the center.
    pub center_x: f32,
    /// Z coordinate of the center.
    pub center_z: f32,
    /// The top surface height (Y axis).
    pub top_y: f32,
    /// Radius of the cylinder.
    pub radius: f32,
}

impl CylinderCollider {
    /// Creates a new cylinder collider.
    pub fn new(center_x: f32, center_z: f32, top_y: f32, radius: f32) -> Self {
        Self { center_x, center_z, top_y, radius }
    }

    /// Resolve cylinder contacts by projecting penetrating vertices to the top or side surface.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_penetration = 0.0_f32;
        let mut total_force = 0.0_f32;

        let r2 = self.radius * self.radius;
        let friction = 0.7; // Moderate friction for pedestal

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 {
                continue;
            }

            // Only consider vertices strictly below the top face.
            if state.pos_y[i] > self.top_y {
                continue;
            }

            let dx = state.pos_x[i] - self.center_x;
            let dz = state.pos_z[i] - self.center_z;
            let dist2 = dx * dx + dz * dz;

            // Note: Since this cylinder extends to y = -infinity, anything inside r2 is a hit.
            if dist2 < r2 {
                let dist = dist2.sqrt();
                let depth_side = self.radius - dist;
                let depth_top = self.top_y - state.pos_y[i];

                // CCD heuristic: if it was above the top surface in the previous frame,
                // it MUST have tunnelled vertically. Push it UP.
                let mut push_up = depth_top < depth_side;

                if state.prev_y[i] >= self.top_y {
                    push_up = true;
                }

                // If it hits dead center, we also must push up to avoid NaN normal
                if dist < 1e-12 {
                    push_up = true;
                }

                if push_up {
                    // It penetrated the top face. Push UP.
                    state.pos_y[i] = self.top_y;

                    if state.vel_y[i] < 0.0 {
                        state.vel_y[i] = 0.0;
                    }

                    // Friction resists horizontal motion. Very high friction for pedestal
                    // to prevent it sliding off like ice.
                    state.vel_x[i] *= 0.1;
                    state.vel_z[i] *= 0.1;

                    resolved += 1;
                    max_penetration = max_penetration.max(depth_top);
                    total_force += depth_top;
                } else {
                    // It penetrated the side more than the top, push OUT.
                    if dist > 1e-12 {
                        let nx = dx / dist;
                        let nz = dz / dist;
                        state.pos_x[i] += nx * depth_side;
                        state.pos_z[i] += nz * depth_side;

                        // Inelastic collision for normal velocity
                        let v_dot_n = state.vel_x[i] * nx + state.vel_z[i] * nz;
                        if v_dot_n < 0.0 {
                            state.vel_x[i] -= v_dot_n * nx;
                            state.vel_z[i] -= v_dot_n * nz;
                        }

                        // Friction resists vertical sliding
                        state.vel_y[i] *= friction;
                    }

                    resolved += 1;
                    max_penetration = max_penetration.max(depth_side);
                    total_force += depth_side;
                }
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_penetration,
            total_force_magnitude: total_force,
        }
    }

    /// Compute IPC barrier gradients for vertices near the cylinder.
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

        for i in 0..pos_x.len() {
            let dx = pos_x[i] - self.center_x;
            let dy = pos_y[i] - self.top_y;
            let dz = pos_z[i] - self.center_z;

            let r = (dx * dx + dz * dz).sqrt();
            let d_side = r - self.radius; // < 0 if inside cylinder bounds
            let d_top = dy; // < 0 if below top surface

            // The vertex is interacting with the cylinder if it's below the top
            // and within the radius, OR near the boundary.
            let mut dd_dx = 0.0;
            let mut dd_dy = 0.0;
            let mut dd_dz = 0.0;

            let dist_sq = if d_top <= 0.0 && d_side <= 0.0 {
                // Inside the cylinder -> violation!
                // Distance is the shortest path out. It's theoretically negative, but our
                // barrier only works on positive d. A true violation is handled by the solver
                // growth, but we clamp to a tiny d to yield massive forces pointing out.
                let inv = d_top.max(d_side);
                max_violation = max_violation.max(-inv);

                if d_top > d_side {
                    dd_dy = 1.0;
                } else if r > 1e-6 {
                    dd_dx = dx / r;
                    dd_dz = dz / r;
                }
                1e-12 // Massive force
            } else if d_top > 0.0 && d_side <= 0.0 {
                // Directly above the top surface
                dd_dy = 1.0;
                d_top * d_top
            } else if d_top <= 0.0 && d_side > 0.0 {
                // Directly outside the side wall
                if r > 1e-6 {
                    dd_dx = dx / r;
                    dd_dz = dz / r;
                }
                d_side * d_side
            } else {
                // Above and outside (near the rim)
                if r > 1e-6 {
                    let d = (d_side * d_side + d_top * d_top).sqrt();
                    dd_dx = (d_side / d) * (dx / r);
                    dd_dy = d_top / d;
                    dd_dz = (d_side / d) * (dz / r);
                }
                d_side * d_side + d_top * d_top
            };

            if dist_sq < d_hat && dist_sq > 0.0 {
                active += 1;
                let barrier_grad = crate::barrier::scaled_barrier_gradient(dist_sq, d_hat, kappa);

                // Chain rule: dE/dx = dE/d(d²) * d(d²)/dx = barrier_grad * 2 * d * dd_dx
                let d = dist_sq.sqrt();
                let factor = barrier_grad * 2.0 * d;

                grad_x[i] += factor * dd_dx;
                grad_y[i] += factor * dd_dy;
                grad_z[i] += factor * dd_dz;
            }
        }

        (active, max_violation)
    }

    /// Compute maximum safe step size for vertices to prevent passing through the cylinder.
    pub fn compute_ccd_step(
        &self,
        prev_x: &[f32], prev_y: &[f32], prev_z: &[f32],
        new_x: &[f32], new_y: &[f32], new_z: &[f32],
    ) -> f32 {
        let mut min_toi: f32 = 1.0;
        let r2 = self.radius * self.radius;

        for i in 0..prev_x.len() {
            let px0 = prev_x[i] - self.center_x;
            let py0 = prev_y[i] - self.top_y;
            let pz0 = prev_z[i] - self.center_z;

            let px1 = new_x[i] - self.center_x;
            let py1 = new_y[i] - self.top_y;
            let pz1 = new_z[i] - self.center_z;

            let vx = px1 - px0;
            let vy = py1 - py0;
            let vz = pz1 - pz0;

            // 1. Check infinite cylinder wall intersection
            // (px0 + t*vx)^2 + (pz0 + t*vz)^2 = r^2
            let a = vx * vx + vz * vz;
            let b = 2.0 * (px0 * vx + pz0 * vz);
            let c = px0 * px0 + pz0 * pz0 - r2;

            if a > 1e-8 {
                let disc = b * b - 4.0 * a * c;
                if disc >= 0.0 {
                    // Check smallest positive root
                    let t_wall = (-b - disc.sqrt()) / (2.0 * a);
                    if (0.0..=1.0).contains(&t_wall) {
                        // Is the hit below the top surface?
                        let y_hit = py0 + t_wall * vy;
                        if y_hit <= 0.0 {
                            min_toi = min_toi.min(t_wall * 0.9); // 0.9 safety margin
                        }
                    }
                }
            }

            // 2. Check top face intersection
            // py0 + t*vy = 0
            if vy < -1e-8 && py0 > 0.0 {
                let t_top = -py0 / vy;
                if (0.0..=1.0).contains(&t_top) {
                    // Is the hit inside the cylinder radius?
                    let x_hit = px0 + t_top * vx;
                    let z_hit = pz0 + t_top * vz;
                    if x_hit * x_hit + z_hit * z_hit <= r2 {
                        min_toi = min_toi.min(t_top * 0.9);
                    }
                }
            }
        }

        min_toi.max(1e-6)
    }
}
