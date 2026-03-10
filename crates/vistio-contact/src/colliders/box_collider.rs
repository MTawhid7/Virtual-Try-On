//! Analytical Axis-Aligned Bounding Box (AABB) collision.
//!
//! A semi-infinite or finite box collider that prevents vertices from
//! moving inside its volume. Suitable for Cantilever Bending ledges.

use vistio_solver::state::SimulationState;

use crate::response::ContactResult;

/// Analytical AABB collider.
pub struct BoxCollider {
    pub min_x: f32,
    pub max_x: f32,
    pub min_y: f32,
    pub max_y: f32,
    pub min_z: f32,
    pub max_z: f32,
}

impl BoxCollider {
    /// Creates a new box collider with definite bounds.
    pub fn new(min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) -> Self {
        Self { min_x, max_x, min_y, max_y, min_z, max_z }
    }

    /// Resolve box contacts by projecting penetrating vertices to the nearest face.
    pub fn resolve(&self, state: &mut SimulationState) -> ContactResult {
        let mut resolved = 0u32;
        let mut max_pen = 0.0_f32;
        let mut tf = 0.0_f32;

        for i in 0..state.vertex_count {
            if state.inv_mass[i] == 0.0 { continue; }

            let x = state.pos_x[i];
            let y = state.pos_y[i];
            let z = state.pos_z[i];

            if x >= self.min_x && x <= self.max_x &&
               y >= self.min_y && y <= self.max_y &&
               z >= self.min_z && z <= self.max_z {

                // Inside the box. Find shortest penetration depth to a face.
                let d_min_x = x - self.min_x;
                let d_max_x = self.max_x - x;
                let d_min_y = y - self.min_y;
                let d_max_y = self.max_y - y;
                let d_min_z = z - self.min_z;
                let d_max_z = self.max_z - z;

                let mut min_d = d_min_x.min(d_max_x).min(d_min_y).min(d_max_y).min(d_min_z).min(d_max_z);

                let friction = 0.7; // Friction factor

                // CCD heuristic: if it was above the top surface in the previous frame,
                // it MUST have tunnelled vertically. Push it UP.
                if state.prev_y[i] >= self.max_y {
                    min_d = d_max_y;
                }

                if min_d == d_min_x {
                    // Push out left (X-)
                    state.pos_x[i] = self.min_x;
                    if state.vel_x[i] > 0.0 { state.vel_x[i] = 0.0; }
                    state.vel_y[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_max_x {
                    // Push out right (X+)
                    state.pos_x[i] = self.max_x;
                    if state.vel_x[i] < 0.0 { state.vel_x[i] = 0.0; }
                    state.vel_y[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_max_y {
                    // Push out top (Y+)
                    state.pos_y[i] = self.max_y;
                    if state.vel_y[i] < 0.0 { state.vel_y[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_min_y {
                    // Push out bottom (Y-)
                    state.pos_y[i] = self.min_y;
                    if state.vel_y[i] > 0.0 { state.vel_y[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_z[i] *= friction;
                } else if min_d == d_max_z {
                    // Push out front (Z+)
                    state.pos_z[i] = self.max_z;
                    if state.vel_z[i] < 0.0 { state.vel_z[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_y[i] *= friction;
                } else if min_d == d_min_z {
                    // Push out back (Z-)
                    state.pos_z[i] = self.min_z;
                    if state.vel_z[i] > 0.0 { state.vel_z[i] = 0.0; }
                    state.vel_x[i] *= friction; state.vel_y[i] *= friction;
                }

                resolved += 1;
                max_pen = max_pen.max(min_d);
                tf += min_d;
            }
        }

        ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_pen,
            total_force_magnitude: tf,
        }
    }

    /// Compute IPC barrier gradients for vertices near the box.
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

        let cx = (self.min_x + self.max_x) * 0.5;
        let cy = (self.min_y + self.max_y) * 0.5;
        let cz = (self.min_z + self.max_z) * 0.5;

        let ex = (self.max_x - self.min_x) * 0.5;
        let ey = (self.max_y - self.min_y) * 0.5;
        let ez = (self.max_z - self.min_z) * 0.5;

        for i in 0..pos_x.len() {
            let dx = pos_x[i] - cx;
            let dy = pos_y[i] - cy;
            let dz = pos_z[i] - cz;

            let abs_x = dx.abs();
            let abs_y = dy.abs();
            let abs_z = dz.abs();

            let qx = abs_x - ex;
            let qy = abs_y - ey;
            let qz = abs_z - ez;

            let max_q = qx.max(qy).max(qz);

            let mut dd_dx = 0.0;
            let mut dd_dy = 0.0;
            let mut dd_dz = 0.0;

            let dist_sq = if max_q <= 0.0 {
                // Inside the box -> violation!
                max_violation = max_violation.max(-max_q);

                if max_q == qx {
                    dd_dx = dx.signum();
                } else if max_q == qy {
                    dd_dy = dy.signum();
                } else {
                    dd_dz = dz.signum();
                }
                1e-12 // Clamp for massive repulsive force
            } else {
                // Outside the box
                let mut d_out = 0.0_f32;
                if qx > 0.0 {
                    d_out += qx * qx;
                    dd_dx = qx * dx.signum();
                }
                if qy > 0.0 {
                    d_out += qy * qy;
                    dd_dy = qy * dy.signum();
                }
                if qz > 0.0 {
                    d_out += qz * qz;
                    dd_dz = qz * dz.signum();
                }

                let d = d_out.sqrt();
                if d > 1e-6 {
                    dd_dx /= d;
                    dd_dy /= d;
                    dd_dz /= d;
                }
                d_out
            };

            if dist_sq < d_hat && dist_sq > 0.0 {
                active += 1;
                let barrier_grad = crate::barrier::scaled_barrier_gradient(dist_sq, d_hat, kappa);

                let d = dist_sq.sqrt();
                let factor = barrier_grad * 2.0 * d;

                grad_x[i] += factor * dd_dx;
                grad_y[i] += factor * dd_dy;
                grad_z[i] += factor * dd_dz;
            }
        }

        (active, max_violation)
    }

    /// Compute maximum safe step size to prevent vertices from passing into the box.
    pub fn compute_ccd_step(
        &self,
        prev_x: &[f32], prev_y: &[f32], prev_z: &[f32],
        new_x: &[f32], new_y: &[f32], new_z: &[f32],
        padding: f32,
    ) -> f32 {
        let mut min_toi: f32 = 1.0;

        let min_x_eff = self.min_x - padding;
        let max_x_eff = self.max_x + padding;
        let min_y_eff = self.min_y - padding;
        let max_y_eff = self.max_y + padding;
        let min_z_eff = self.min_z - padding;
        let max_z_eff = self.max_z + padding;

        for i in 0..prev_x.len() {
            let px0 = prev_x[i];
            let py0 = prev_y[i];
            let pz0 = prev_z[i];

            let px1 = new_x[i];
            let py1 = new_y[i];
            let pz1 = new_z[i];

            let vx = px1 - px0;
            let vy = py1 - py0;
            let vz = pz1 - pz0;

            if px0 >= min_x_eff && px0 <= max_x_eff &&
               py0 >= min_y_eff && py0 <= max_y_eff &&
               pz0 >= min_z_eff && pz0 <= max_z_eff {
                let cx = (self.min_x + self.max_x) * 0.5;
                let cy = (self.min_y + self.max_y) * 0.5;
                let cz = (self.min_z + self.max_z) * 0.5;
                let dot_pv = (px0 - cx) * vx + (py0 - cy) * vy + (pz0 - cz) * vz;
                if dot_pv < 0.0 {
                    min_toi = 0.0;
                }
                continue;
            }

            let mut t_min = 0.0_f32;
            let mut t_max = 1.0_f32;

            let check_axis = |p0: f32, v: f32, min_b: f32, max_b: f32, t_min: &mut f32, t_max: &mut f32| -> bool {
                if v.abs() < 1e-8 {
                    p0 >= min_b && p0 <= max_b
                } else {
                    let mut t1 = (min_b - p0) / v;
                    let mut t2 = (max_b - p0) / v;
                    if t1 > t2 {
                        std::mem::swap(&mut t1, &mut t2);
                    }
                    *t_min = t_min.max(t1);
                    *t_max = t_max.min(t2);
                    *t_min <= *t_max && *t_max >= 0.0
                }
            };

            if check_axis(px0, vx, min_x_eff, max_x_eff, &mut t_min, &mut t_max) &&
               check_axis(py0, vy, min_y_eff, max_y_eff, &mut t_min, &mut t_max) &&
               check_axis(pz0, vz, min_z_eff, max_z_eff, &mut t_min, &mut t_max)

                && (0.0..=1.0).contains(&t_min) {
                    // It entered the box during this frame!
                    min_toi = min_toi.min(t_min * 0.9);
                }
        }

        min_toi.max(1e-6)
    }
}
