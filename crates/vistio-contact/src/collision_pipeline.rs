//! Unified collision pipeline that orchestrates broad → narrow → response.
//!
//! The pipeline is handed to the PD solver and called each timestep
//! (or each PD iteration) to detect and resolve contacts.

use vistio_mesh::TriangleMesh;
use vistio_solver::state::SimulationState;
use vistio_types::VistioResult;

use crate::broad::BroadPhase;
use crate::ground_plane::GroundPlane;
use crate::narrow::NarrowPhase;
use crate::response::{ContactResponse, ContactResult};
use crate::sphere::SphereCollider;
use crate::cylinder::CylinderCollider;
use crate::box_collider::BoxCollider;
use crate::self_collision::{SelfCollisionSystem, SelfCollisionResult};

/// Unified collision pipeline: broad → narrow → response.
///
/// Orchestrates the three collision phases and optional ground plane.
/// Handed to the PD solver via `set_collision_pipeline()`.
pub struct CollisionPipeline {
    /// Broad phase acceleration structure.
    pub broad: Box<dyn BroadPhase + Send + Sync>,
    /// Narrow phase exact testing.
    pub narrow: Box<dyn NarrowPhase + Send + Sync>,
    /// Contact response (position correction or barrier).
    pub response: Box<dyn ContactResponse + Send + Sync>,
    /// Collision thickness / separation margin.
    pub thickness: f32,
    /// Response stiffness.
    pub stiffness: f32,
    /// Optional ground plane.
    pub ground: Option<GroundPlane>,
    /// Optional analytical sphere collider.
    pub sphere: Option<SphereCollider>,
    pub cylinder: Option<CylinderCollider>,
    pub box_collider: Option<BoxCollider>,
    /// Optional self-collision system.
    pub self_collision: Option<SelfCollisionSystem>,
    pub ipc_enabled: bool,
    /// Reference mesh (for triangle queries in narrow phase).
    mesh: TriangleMesh,
}

impl CollisionPipeline {
    /// Create a new collision pipeline.
    pub fn new(
        broad: Box<dyn BroadPhase + Send + Sync>,
        narrow: Box<dyn NarrowPhase + Send + Sync>,
        response: Box<dyn ContactResponse + Send + Sync>,
        mesh: TriangleMesh,
        thickness: f32,
        stiffness: f32,
    ) -> Self {
        Self {
            broad,
            narrow,
            response,
            thickness,
            stiffness,
            ground: None,
            sphere: None,
            cylinder: None,
            box_collider: None,
            self_collision: None,
            ipc_enabled: false,
            mesh,
        }
    }

    /// Enable or disable IPC mode (prevents projection-based self-collision).
    pub fn with_ipc(mut self, enabled: bool) -> Self {
        self.ipc_enabled = enabled;
        self
    }

    /// Add an optional ground plane.
    pub fn with_ground(mut self, height: f32) -> Self {
        self.ground = Some(GroundPlane::new(height));
        self
    }

    /// Add an optional analytical sphere collider.
        pub fn with_cylinder(mut self, center_x: f32, center_z: f32, top_y: f32, radius: f32) -> Self {
        self.cylinder = Some(CylinderCollider::new(center_x, center_z, top_y, radius));
        self
    }

    pub fn with_box(mut self, min_x: f32, max_x: f32, min_y: f32, max_y: f32, min_z: f32, max_z: f32) -> Self {
        self.box_collider = Some(BoxCollider::new(min_x, max_x, min_y, max_y, min_z, max_z));
        self
    }

    pub fn with_sphere(mut self, center: vistio_math::Vec3, radius: f32) -> Self {
        self.sphere = Some(SphereCollider::new(center, radius));
        self
    }

    /// Add the self-collision module.
    pub fn with_self_collision(
        mut self,
        topology: &vistio_mesh::topology::Topology,
        exclusion_depth: usize,
    ) -> Self {
        self.self_collision = Some(SelfCollisionSystem::new(
            &self.mesh,
            topology,
            exclusion_depth,
            self.thickness,
            self.stiffness,
        ));
        self
    }

    /// Run the full collision pipeline: broad → narrow → response + ground.
    pub fn step(&mut self, state: &mut SimulationState) -> VistioResult<CollisionStepResult> {
        let mut candidates = Vec::new();
        let mut contacts = Vec::new();
        let mut mesh_result = ContactResult { resolved_count: 0, max_residual_penetration: 0.0, total_force_magnitude: 0.0 };

        if self.thickness > 0.0 && self.stiffness > 0.0 {
            // 1. Broad phase: update acceleration structure and query pairs
            self.broad.update(&state.pos_x, &state.pos_y, &state.pos_z, self.thickness)?;
            candidates = self.broad.query_pairs();

            // 2. Narrow phase: exact proximity tests
            contacts = self.narrow.detect(&candidates, state, &self.mesh, self.thickness)?;

            // 3. Contact response: resolve penetrations
            // If IPC is enabled, we don't want standard generic contact response to move vertices
            if !self.ipc_enabled {
                mesh_result = self.response.resolve(&contacts, state, self.stiffness)?;
            }
        }

        // 4. Ground plane (if present) — hard constraint, must run first
        let ground_result = if let Some(ref ground) = self.ground {
            ground.resolve(state)
        } else {
            ContactResult {
                resolved_count: 0,
                max_residual_penetration: 0.0,
                total_force_magnitude: 0.0,
            }
        };

        // 5. Self collision resolution (if enabled)
        let self_collision_result = if !self.ipc_enabled {
            if let Some(ref mut self_col) = self.self_collision {
                Some(self_col.solve(state, self.broad.as_mut()))
            } else {
                None
            }
        } else {
            None
        };

        // 6. Ground plane again (post self-collision enforcement)
        if let Some(ref ground) = self.ground {
            ground.resolve(state);
        }

        // 7. Sphere collider (if present)
        let sphere_result = if let Some(ref sphere) = self.sphere {
            sphere.resolve(state)
        } else {
            ContactResult {
                resolved_count: 0,
                max_residual_penetration: 0.0,
                total_force_magnitude: 0.0,
            }
        };

                let cylinder_result = if let Some(ref cylinder) = self.cylinder {
            cylinder.resolve(state)
        } else {
            ContactResult { resolved_count: 0, max_residual_penetration: 0.0, total_force_magnitude: 0.0 }
        };

        let box_result = if let Some(ref box_col) = self.box_collider {
            box_col.resolve(state)
        } else {
            ContactResult { resolved_count: 0, max_residual_penetration: 0.0, total_force_magnitude: 0.0 }
        };

        let candidate_count = candidates.len() as u32;

        Ok(CollisionStepResult {
            candidate_pairs: candidate_count,
            contacts_detected: contacts.len() as u32,
            mesh_result,
            ground_result,
            sphere_result,
            cylinder_result,
            box_result,
            self_collision_result,
        })
    }

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
        let mut active_contacts = 0;
        let mut max_violation = 0.0_f32;

        // 1. Self Collision
        if let Some(ref mut self_col) = self.self_collision {
            let ipc_set = self_col.detect_ipc_contacts(
                self.broad.as_mut(),
                pos_x,
                pos_y,
                pos_z,
                self.thickness,
                d_hat,
                kappa,
            );

            let (gx, gy, gz) = ipc_set.compute_barrier_gradient(pos_x, pos_y, pos_z, n_vertices);

            for i in 0..n_vertices {
                grad_x[i] += gx[i];
                grad_y[i] += gy[i];
                grad_z[i] += gz[i];
                // For self-collision, the barrier gradient direction IS the contact normal
                let g_sq = gx[i] * gx[i] + gy[i] * gy[i] + gz[i] * gz[i];
                if g_sq > 1e-12 {
                    let g_len = g_sq.sqrt();
                    contact_nx[i] += gx[i] / g_len;
                    contact_ny[i] += gy[i] / g_len;
                    contact_nz[i] += gz[i] / g_len;
                    in_contact[i] = true;
                }
            }

            max_violation = max_violation.max(ipc_set.max_constraint_violation(pos_x, pos_y, pos_z));
            active_contacts += ipc_set.len();
        }

        // 2. Ground Plane — normal is always (0, 1, 0)
        if let Some(ref ground) = self.ground {
            let (active, violation) = ground.detect_ipc_contacts(pos_y, d_hat, kappa, &mut grad_y);
            active_contacts += active;
            max_violation = max_violation.max(violation);

            // Mark vertices near ground as in-contact with upward normal
            for i in 0..n_vertices {
                let d_surface = pos_y[i] - ground.height;
                if d_surface * d_surface < d_hat || d_surface <= 0.0 {
                    contact_ny[i] += 1.0; // ground normal is +Y
                    in_contact[i] = true;
                }
            }
        }

        // 3. Sphere Collider — normal is radial outward from sphere center
        if let Some(ref sphere) = self.sphere {
            let (active, violation) = sphere.detect_ipc_contacts(pos_x, pos_y, pos_z, d_hat, kappa, &mut grad_x, &mut grad_y, &mut grad_z);
            active_contacts += active;
            max_violation = max_violation.max(violation);

            // Compute per-vertex sphere normals for vertices in contact
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
                }
            }
        }

        // 4. Cylinder Collider
        if let Some(ref cylinder) = self.cylinder {
            let (active, violation) = cylinder.detect_ipc_contacts(pos_x, pos_y, pos_z, d_hat, kappa, &mut grad_x, &mut grad_y, &mut grad_z);
            active_contacts += active;
            max_violation = max_violation.max(violation);
        }

        // 5. Box Collider
        if let Some(ref box_col) = self.box_collider {
            let (active, violation) = box_col.detect_ipc_contacts(pos_x, pos_y, pos_z, d_hat, kappa, &mut grad_x, &mut grad_y, &mut grad_z);
            active_contacts += active;
            max_violation = max_violation.max(violation);
        }

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
        }
    }

    /// Compute the maximum safe step fraction to prevent any vertex from passing
    /// through a collision boundary explicitly modelled by IPC.
    pub fn compute_ccd_step(
        &self,
        mesh_indices: &[u32],
        prev_x: &[f32], prev_y: &[f32], prev_z: &[f32],
        new_x: &[f32], new_y: &[f32], new_z: &[f32],
    ) -> f32 {
        let mut min_toi = 1.0_f32;

        // 1. Self Collision
        if let Some(_) = self.self_collision {
            let tri_pairs = self.broad.query_triangle_pairs();
            let tri_pairs_usize: Vec<(usize, usize)> = tri_pairs.into_iter().map(|p| (p.ta as usize, p.tb as usize)).collect();
            let toi = crate::ccd::compute_max_step_size(
                prev_x, prev_y, prev_z,
                new_x, new_y, new_z,
                mesh_indices, &tri_pairs_usize
            );
            min_toi = min_toi.min(toi);
        }

        // 2. Ground Plane
        if let Some(ref ground) = self.ground {
            let toi = ground.compute_ccd_step(prev_y, new_y);
            min_toi = min_toi.min(toi);
        }

        // 3. Sphere Collider
        if let Some(ref sphere) = self.sphere {
            let toi = sphere.compute_ccd_step(prev_x, prev_y, prev_z, new_x, new_y, new_z);
            min_toi = min_toi.min(toi);
        }

        // 4. Cylinder Collider
        if let Some(ref cylinder) = self.cylinder {
            let toi = cylinder.compute_ccd_step(prev_x, prev_y, prev_z, new_x, new_y, new_z);
            min_toi = min_toi.min(toi);
        }

        // 5. Box Collider
        if let Some(ref box_col) = self.box_collider {
            let toi = box_col.compute_ccd_step(prev_x, prev_y, prev_z, new_x, new_y, new_z);
            min_toi = min_toi.min(toi);
        }

        min_toi.max(1e-6) // Do not allow step to be less than 1e-6
    }
}

/// Result of a full collision pipeline step.
#[derive(Debug, Clone)]
pub struct CollisionStepResult {
    /// Number of broad-phase candidate pairs.
    pub candidate_pairs: u32,
    /// Number of narrow-phase contacts detected.
    pub contacts_detected: u32,
    /// Mesh collision resolution result.
    pub mesh_result: ContactResult,
    /// Ground plane resolution result.
    pub ground_result: ContactResult,
    /// Sphere collision resolution result.
    pub sphere_result: ContactResult,
    pub cylinder_result: ContactResult,
    pub box_result: ContactResult,
    /// Result from self collision resolution.
    pub self_collision_result: Option<SelfCollisionResult>,
}
