//! IPC contact detection for all collider types.
//!
//! Computes barrier energy gradients, contact normals, and Hessian
//! diagonals for the Augmented Lagrangian solver.

use super::CollisionPipeline;

impl CollisionPipeline {
    /// Detect IPC contacts for the Tier 4 Augmented Lagrangian solver.
    ///
    /// This bypasses standard `step()` resolution and purely builds
    /// barrier energy gradients for the solver's RHS.
    /// Also computes per-vertex geometric contact normals for the velocity filter.
    pub fn detect_ipc_contacts(
        &mut self,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        d_hat: f32,
        kappa: f32,
    ) -> vistio_solver::pd_solver::IpcBarrierForces {
        let n_vertices = pos_x.len();
        let mut grad_x = vec![0.0_f32; n_vertices];
        let mut grad_y = vec![0.0_f32; n_vertices];
        let mut grad_z = vec![0.0_f32; n_vertices];
        let mut contact_nx = vec![0.0_f32; n_vertices];
        let mut contact_ny = vec![0.0_f32; n_vertices];
        let mut contact_nz = vec![0.0_f32; n_vertices];
        let mut in_contact = vec![false; n_vertices];
        let mut hessian_diag = vec![0.0_f32; n_vertices];
        let mut active_contacts = 0;
        let mut max_violation = 0.0_f32;

        // 1. Self Collision
        self.detect_self_collision_ipc(
            pos_x, pos_y, pos_z, d_hat, kappa,
            &mut grad_x, &mut grad_y, &mut grad_z,
            &mut contact_nx, &mut contact_ny, &mut contact_nz,
            &mut in_contact, &mut active_contacts, &mut max_violation,
            n_vertices,
        );

        // 2. Ground Plane
        self.detect_ground_ipc(
            pos_y, d_hat, kappa,
            &mut grad_y, &mut contact_ny, &mut in_contact,
            &mut hessian_diag, &mut active_contacts, &mut max_violation,
            n_vertices,
        );

        // 3. Sphere Collider
        self.detect_sphere_ipc(
            pos_x, pos_y, pos_z, d_hat, kappa,
            &mut grad_x, &mut grad_y, &mut grad_z,
            &mut contact_nx, &mut contact_ny, &mut contact_nz,
            &mut in_contact, &mut hessian_diag,
            &mut active_contacts, &mut max_violation,
            n_vertices,
        );

        // 4. Cylinder Collider
        self.detect_cylinder_ipc(
            pos_x, pos_y, pos_z, d_hat, kappa,
            &mut grad_x, &mut grad_y, &mut grad_z,
            &mut contact_nx, &mut contact_ny, &mut contact_nz,
            &mut in_contact, &mut hessian_diag,
            &mut active_contacts, &mut max_violation,
            n_vertices,
        );

        // 5. Box Collider
        self.detect_box_ipc(
            pos_x, pos_y, pos_z, d_hat, kappa,
            &mut grad_x, &mut grad_y, &mut grad_z,
            &mut contact_nx, &mut contact_ny, &mut contact_nz,
            &mut in_contact, &mut hessian_diag,
            &mut active_contacts, &mut max_violation,
            n_vertices,
        );

        // Normalize accumulated contact normals
        for i in 0..n_vertices {
            if in_contact[i] {
                let len_sq = contact_nx[i] * contact_nx[i]
                    + contact_ny[i] * contact_ny[i]
                    + contact_nz[i] * contact_nz[i];
                if len_sq > 1e-12 {
                    let inv_len = 1.0 / len_sq.sqrt();
                    contact_nx[i] *= inv_len;
                    contact_ny[i] *= inv_len;
                    contact_nz[i] *= inv_len;
                }
            }
        }

        vistio_solver::pd_solver::IpcBarrierForces {
            grad_x,
            grad_y,
            grad_z,
            max_violation,
            active_contacts,
            contact_nx,
            contact_ny,
            contact_nz,
            in_contact,
            hessian_diag,
        }
    }

    // ─── Per-collider IPC detection helpers ──────────────────

    #[allow(clippy::too_many_arguments)]
    fn detect_self_collision_ipc(
        &mut self,
        pos_x: &[f32], pos_y: &[f32], pos_z: &[f32],
        d_hat: f32, kappa: f32,
        grad_x: &mut [f32], grad_y: &mut [f32], grad_z: &mut [f32],
        contact_nx: &mut [f32], contact_ny: &mut [f32], contact_nz: &mut [f32],
        in_contact: &mut [bool], active_contacts: &mut usize, max_violation: &mut f32,
        n_vertices: usize,
    ) {
        if let Some(ref mut self_col) = self.self_collision {
            let ipc_set = self_col.detect_ipc_contacts(
                self.broad.as_mut(), pos_x, pos_y, pos_z,
                self.thickness, d_hat, kappa,
            );

            let (gx, gy, gz) = ipc_set.compute_barrier_gradient(pos_x, pos_y, pos_z, n_vertices);

            for i in 0..n_vertices {
                grad_x[i] += gx[i];
                grad_y[i] += gy[i];
                grad_z[i] += gz[i];
                let g_sq = gx[i] * gx[i] + gy[i] * gy[i] + gz[i] * gz[i];
                if g_sq > 1e-12 {
                    let g_len = g_sq.sqrt();
                    contact_nx[i] += gx[i] / g_len;
                    contact_ny[i] += gy[i] / g_len;
                    contact_nz[i] += gz[i] / g_len;
                    in_contact[i] = true;
                }
            }

            *max_violation = max_violation.max(ipc_set.max_constraint_violation(pos_x, pos_y, pos_z));
            *active_contacts += ipc_set.len();
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn detect_ground_ipc(
        &self,
        pos_y: &[f32], d_hat: f32, kappa: f32,
        grad_y: &mut [f32], contact_ny: &mut [f32], in_contact: &mut [bool],
        hessian_diag: &mut [f32], active_contacts: &mut usize, max_violation: &mut f32,
        n_vertices: usize,
    ) {
        if let Some(ref ground) = self.ground {
            let (active, violation) = ground.detect_ipc_contacts(pos_y, d_hat, kappa, grad_y);
            *active_contacts += active;
            *max_violation = max_violation.max(violation);

            for i in 0..n_vertices {
                let d_surface = pos_y[i] - ground.height;
                if d_surface * d_surface < d_hat || d_surface <= 0.0 {
                    contact_ny[i] += 1.0;
                    in_contact[i] = true;
                    if d_surface <= 0.0 {
                        let dist_sq = 1e-12_f32;
                        let h = crate::barrier::barrier_hessian(dist_sq, d_hat);
                        hessian_diag[i] += (kappa * h * 4.0 * dist_sq).abs() + 1e6 * kappa;
                    } else {
                        let dist_sq = d_surface * d_surface;
                        let h = crate::barrier::barrier_hessian(dist_sq, d_hat);
                        hessian_diag[i] += (kappa * h * 4.0 * dist_sq).abs();
                    }
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn detect_sphere_ipc(
        &self,
        pos_x: &[f32], pos_y: &[f32], pos_z: &[f32],
        d_hat: f32, kappa: f32,
        grad_x: &mut [f32], grad_y: &mut [f32], grad_z: &mut [f32],
        contact_nx: &mut [f32], contact_ny: &mut [f32], contact_nz: &mut [f32],
        in_contact: &mut [bool], hessian_diag: &mut [f32],
        active_contacts: &mut usize, max_violation: &mut f32,
        n_vertices: usize,
    ) {
        if let Some(ref sphere) = self.sphere {
            let (active, violation) = sphere.detect_ipc_contacts(
                pos_x, pos_y, pos_z, d_hat, kappa, grad_x, grad_y, grad_z,
            );
            *active_contacts += active;
            *max_violation = max_violation.max(violation);

            for i in 0..n_vertices {
                let dx = pos_x[i] - sphere.center.x;
                let dy = pos_y[i] - sphere.center.y;
                let dz = pos_z[i] - sphere.center.z;
                let r = (dx * dx + dy * dy + dz * dz).sqrt();
                let d_surface = r - sphere.radius;
                if d_surface * d_surface < d_hat || d_surface <= 0.0 {
                    if r > 1e-6 {
                        contact_nx[i] += dx / r;
                        contact_ny[i] += dy / r;
                        contact_nz[i] += dz / r;
                    }
                    in_contact[i] = true;
                    if d_surface <= 0.0 {
                        let dist_sq = 1e-12_f32;
                        let h = crate::barrier::barrier_hessian(dist_sq, d_hat);
                        hessian_diag[i] += (kappa * h * 4.0 * dist_sq).abs() + 1e6 * kappa;
                    } else {
                        let dist_sq = d_surface * d_surface;
                        let h = crate::barrier::barrier_hessian(dist_sq, d_hat);
                        hessian_diag[i] += (kappa * h * 4.0 * dist_sq).abs();
                    }
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn detect_cylinder_ipc(
        &self,
        pos_x: &[f32], pos_y: &[f32], pos_z: &[f32],
        d_hat: f32, kappa: f32,
        grad_x: &mut [f32], grad_y: &mut [f32], grad_z: &mut [f32],
        contact_nx: &mut [f32], contact_ny: &mut [f32], contact_nz: &mut [f32],
        in_contact: &mut [bool], hessian_diag: &mut [f32],
        active_contacts: &mut usize, max_violation: &mut f32,
        n_vertices: usize,
    ) {
        if let Some(ref cylinder) = self.cylinder {
            let (active, violation) = cylinder.detect_ipc_contacts(
                pos_x, pos_y, pos_z, d_hat, kappa, grad_x, grad_y, grad_z,
            );
            *active_contacts += active;
            *max_violation = max_violation.max(violation);

            for i in 0..n_vertices {
                let dx = pos_x[i] - cylinder.center_x;
                let dy = pos_y[i] - cylinder.top_y;
                let dz = pos_z[i] - cylinder.center_z;

                let r = (dx * dx + dz * dz).sqrt();
                let d_side = r - cylinder.radius;
                let d_top = dy;

                let is_near = if d_top <= 0.0 && d_side <= 0.0 {
                    true
                } else if d_top > 0.0 && d_side <= 0.0 {
                    d_top * d_top < d_hat
                } else if d_top <= 0.0 && d_side > 0.0 {
                    d_side * d_side < d_hat
                } else {
                    d_side * d_side + d_top * d_top < d_hat
                };

                if is_near {
                    let dist_sq = if d_top <= 0.0 && d_side <= 0.0 {
                        1e-12
                    } else if d_top > 0.0 && d_side <= 0.0 {
                        contact_ny[i] += 1.0;
                        d_top * d_top
                    } else if d_top <= 0.0 && d_side > 0.0 {
                        if r > 1e-6 {
                            contact_nx[i] += dx / r;
                            contact_nz[i] += dz / r;
                        }
                        d_side * d_side
                    } else {
                        if r > 1e-6 {
                            let d = (d_side * d_side + d_top * d_top).sqrt();
                            contact_nx[i] += (d_side / d) * (dx / r);
                            contact_ny[i] += d_top / d;
                            contact_nz[i] += (d_side / d) * (dz / r);
                        }
                        d_side * d_side + d_top * d_top
                    };

                    in_contact[i] = true;
                    let d_eff = dist_sq.sqrt().max(1e-6);
                    let safe_dist_sq = d_eff * d_eff;
                    let h = crate::barrier::barrier_hessian(safe_dist_sq, d_hat);
                    hessian_diag[i] += (kappa * h * 4.0 * safe_dist_sq).abs();
                }
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn detect_box_ipc(
        &self,
        pos_x: &[f32], pos_y: &[f32], pos_z: &[f32],
        d_hat: f32, kappa: f32,
        grad_x: &mut [f32], grad_y: &mut [f32], grad_z: &mut [f32],
        contact_nx: &mut [f32], contact_ny: &mut [f32], contact_nz: &mut [f32],
        in_contact: &mut [bool], hessian_diag: &mut [f32],
        active_contacts: &mut usize, max_violation: &mut f32,
        n_vertices: usize,
    ) {
        if let Some(ref box_col) = self.box_collider {
            let (active, violation) = box_col.detect_ipc_contacts(
                pos_x, pos_y, pos_z, d_hat, kappa, grad_x, grad_y, grad_z,
            );
            *active_contacts += active;
            *max_violation = max_violation.max(violation);

            let cx = (box_col.min_x + box_col.max_x) * 0.5;
            let cy = (box_col.min_y + box_col.max_y) * 0.5;
            let cz = (box_col.min_z + box_col.max_z) * 0.5;

            let ex = (box_col.max_x - box_col.min_x) * 0.5;
            let ey = (box_col.max_y - box_col.min_y) * 0.5;
            let ez = (box_col.max_z - box_col.min_z) * 0.5;

            for i in 0..n_vertices {
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

                let dist_sq = if max_q <= 0.0 {
                    if max_q == qx { contact_nx[i] += dx.signum(); }
                    else if max_q == qy { contact_ny[i] += dy.signum(); }
                    else { contact_nz[i] += dz.signum(); }
                    1e-12
                } else {
                    let mut d_out = 0.0_f32;
                    let mut nx = 0.0;
                    let mut ny = 0.0;
                    let mut nz = 0.0;
                    if qx > 0.0 { d_out += qx * qx; nx = qx * dx.signum(); }
                    if qy > 0.0 { d_out += qy * qy; ny = qy * dy.signum(); }
                    if qz > 0.0 { d_out += qz * qz; nz = qz * dz.signum(); }

                    if d_out < d_hat {
                        let d = d_out.sqrt();
                        if d > 1e-6 {
                            contact_nx[i] += nx / d;
                            contact_ny[i] += ny / d;
                            contact_nz[i] += nz / d;
                        }
                    }
                    d_out
                };

                if dist_sq < d_hat {
                    in_contact[i] = true;
                    let d_eff = dist_sq.sqrt().max(1e-6);
                    let safe_dist_sq = d_eff * d_eff;
                    let h = crate::barrier::barrier_hessian(safe_dist_sq, d_hat);
                    hessian_diag[i] += (kappa * h * 4.0 * safe_dist_sq).abs();
                }
            }
        }
    }
}
