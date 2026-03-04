//! IPC contact response — computes barrier energy, gradient, and Hessian
//! for all active contacts.
//!
//! This module bridges the distance primitives and barrier functions into
//! a form consumable by the PD solver's Augmented Lagrangian loop.
//!
//! Instead of directly modifying positions (like `ProjectionContactResponse`),
//! this module computes gradient forces that are added to the solver's RHS.

use vistio_math::Vec3;

use crate::barrier;
use crate::distance_primitives::{
    self, DistanceType, point_triangle_distance_squared, point_triangle_distance_gradient,
    edge_edge_distance_squared, edge_edge_distance_gradient,
};

/// Type of IPC contact primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IpcContactType {
    /// Vertex p against triangle (a, b, c).
    VertexTriangle,
    /// Edge (a, b) against edge (c, d).
    EdgeEdge,
}

/// A single IPC contact with precomputed distance and barrier data.
#[derive(Debug, Clone)]
pub struct IpcContact {
    /// Contact type.
    pub contact_type: IpcContactType,
    /// Vertex indices involved: [p, a, b, c] for VT, [a0, a1, b0, b1] for EE.
    pub indices: [usize; 4],
    /// Current squared distance between the primitives.
    pub distance_sq: f32,
    /// Distance classification (face, edge, vertex for VT; interior for EE).
    pub distance_type: DistanceType,
    /// Whether this is a self-collision contact.
    pub is_self: bool,
}

/// A set of active IPC contacts with computed barrier forces.
#[derive(Debug, Clone)]
pub struct IpcContactSet {
    /// Active contacts (those with d < d_hat).
    pub contacts: Vec<IpcContact>,
    /// Barrier activation threshold.
    pub d_hat: f32,
    /// Barrier stiffness scaling.
    pub kappa: f32,
    /// Fabric thickness for C-IPC (Phase 3.1).
    /// Self-collision contacts use thickness-aware barriers;
    /// body collider contacts use standard barriers.
    pub thickness: f32,
}

impl IpcContactSet {
    /// Create a new, empty contact set with the given parameters.
    pub fn new(d_hat: f32, kappa: f32) -> Self {
        Self {
            contacts: Vec::new(),
            d_hat,
            kappa,
            thickness: 0.0,
        }
    }

    /// Create a new, empty contact set with thickness-aware barriers.
    pub fn new_with_thickness(d_hat: f32, kappa: f32, thickness: f32) -> Self {
        Self {
            contacts: Vec::new(),
            d_hat,
            kappa,
            thickness,
        }
    }

    /// Detect vertex-triangle contacts from candidate pairs and add active ones.
    ///
    /// Detect barrier interaction between a vertex and a triangle,
    /// generating continuous proxy contacts with distance evaluation.
    #[allow(clippy::too_many_arguments)]
    pub fn detect_vertex_triangle(
        &mut self,
        vertex_idx: usize,
        tri_v0: usize,
        tri_v1: usize,
        tri_v2: usize,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        is_self: bool,
    ) {
        let p = Vec3::new(pos_x[vertex_idx], pos_y[vertex_idx], pos_z[vertex_idx]);
        let a = Vec3::new(pos_x[tri_v0], pos_y[tri_v0], pos_z[tri_v0]);
        let b = Vec3::new(pos_x[tri_v1], pos_y[tri_v1], pos_z[tri_v1]);
        let c = Vec3::new(pos_x[tri_v2], pos_y[tri_v2], pos_z[tri_v2]);

        let (d_sq, dtype) = point_triangle_distance_squared(p, a, b, c);

        if d_sq < self.d_hat && d_sq > 0.0 {
            self.contacts.push(IpcContact {
                contact_type: IpcContactType::VertexTriangle,
                indices: [vertex_idx, tri_v0, tri_v1, tri_v2],
                distance_sq: d_sq,
                distance_type: dtype,
                is_self,
            });
        }
    }

    /// Detect edge-edge contacts and add active ones.
    ///
    /// Detect barrier interaction between two edges,
    /// generating continuous proxy contacts with distance evaluation.
    #[allow(clippy::too_many_arguments)]
    pub fn detect_edge_edge(
        &mut self,
        ea0: usize,
        ea1: usize,
        eb0: usize,
        eb1: usize,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        is_self: bool,
    ) {
        let a0 = Vec3::new(pos_x[ea0], pos_y[ea0], pos_z[ea0]);
        let a1 = Vec3::new(pos_x[ea1], pos_y[ea1], pos_z[ea1]);
        let b0 = Vec3::new(pos_x[eb0], pos_y[eb0], pos_z[eb0]);
        let b1 = Vec3::new(pos_x[eb1], pos_y[eb1], pos_z[eb1]);

        let (d_sq, dtype) = edge_edge_distance_squared(a0, a1, b0, b1);

        if d_sq < self.d_hat && d_sq > 0.0 {
            self.contacts.push(IpcContact {
                contact_type: IpcContactType::EdgeEdge,
                indices: [ea0, ea1, eb0, eb1],
                distance_sq: d_sq,
                distance_type: dtype,
                is_self,
            });
        }
    }

    /// Clear all contacts (call before re-detection).
    pub fn clear(&mut self) {
        self.contacts.clear();
    }

    /// Compute total barrier energy across all active contacts.
    pub fn total_barrier_energy(
        &self,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> f32 {
        let mut total = 0.0;
        for contact in &self.contacts {
            let d_sq = self.recompute_distance(contact, pos_x, pos_y, pos_z);
            total += barrier::scaled_barrier_energy(d_sq, self.d_hat, self.kappa);
        }
        total
    }

    /// Compute barrier gradient contributions for all active contacts.
    ///
    /// Returns accumulated force vectors per vertex (grad_x, grad_y, grad_z).
    /// These should be **subtracted** from the solver RHS (negative gradient = force).
    pub fn compute_barrier_gradient(
        &self,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        n_vertices: usize,
    ) -> (Vec<f32>, Vec<f32>, Vec<f32>) {
        let mut grad_x = vec![0.0_f32; n_vertices];
        let mut grad_y = vec![0.0_f32; n_vertices];
        let mut grad_z = vec![0.0_f32; n_vertices];

        for contact in &self.contacts {
            let d_sq = self.recompute_distance(contact, pos_x, pos_y, pos_z);

            // dE/d(d²): barrier gradient w.r.t. squared distance
            // C-IPC: self-collision contacts use thickness-aware barrier (Phase 3.1)
            let barrier_grad = if contact.is_self && self.thickness > 0.0 {
                barrier::scaled_barrier_gradient_with_thickness(d_sq, self.d_hat, self.kappa, self.thickness)
            } else {
                barrier::scaled_barrier_gradient(d_sq, self.d_hat, self.kappa)
            };

            // d(d²)/d(vertex positions): distance gradient w.r.t. vertex positions
            let dist_grads = self.compute_distance_gradient(contact, pos_x, pos_y, pos_z);

            // Chain rule: dE/d(pos) = dE/d(d²) · d(d²)/d(pos)
            match contact.contact_type {
                IpcContactType::VertexTriangle => {
                    let idx = contact.indices;
                    // [p, a, b, c]
                    for (k, &vi) in idx.iter().enumerate() {
                        grad_x[vi] += barrier_grad * dist_grads[k].x;
                        grad_y[vi] += barrier_grad * dist_grads[k].y;
                        grad_z[vi] += barrier_grad * dist_grads[k].z;
                    }
                }
                IpcContactType::EdgeEdge => {
                    let idx = contact.indices;
                    // [a0, a1, b0, b1]
                    for (k, &vi) in idx.iter().enumerate() {
                        grad_x[vi] += barrier_grad * dist_grads[k].x;
                        grad_y[vi] += barrier_grad * dist_grads[k].y;
                        grad_z[vi] += barrier_grad * dist_grads[k].z;
                    }
                }
            }
        }

        (grad_x, grad_y, grad_z)
    }

    /// Compute the maximum constraint violation (minimum distance / d_hat ratio).
    pub fn max_constraint_violation(
        &self,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> f32 {
        let mut max_violation = 0.0_f32;
        for contact in &self.contacts {
            let d_sq = self.recompute_distance(contact, pos_x, pos_y, pos_z);
            // Violation is absolute penetration depth.
            // For point-triangle and edge-edge distances, unsigned distance squared means
            // penetration only happens if d_sq <= 0.0.
            if d_sq <= 1e-12 {
                // If we reach exactly zero, the primitives intersect.
                // We'll return a small violation to keep the AL loop trying to separate them.
                max_violation = max_violation.max(1e-3);
            }
        }
        max_violation
    }

    /// Number of active contacts.
    pub fn len(&self) -> usize {
        self.contacts.len()
    }

    /// Whether there are no active contacts.
    pub fn is_empty(&self) -> bool {
        self.contacts.is_empty()
    }

    // ─── Internal helpers ────────────────────────────────────

    fn recompute_distance(
        &self,
        contact: &IpcContact,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> f32 {
        match contact.contact_type {
            IpcContactType::VertexTriangle => {
                let [pi, ai, bi, ci] = contact.indices;
                let p = Vec3::new(pos_x[pi], pos_y[pi], pos_z[pi]);
                let a = Vec3::new(pos_x[ai], pos_y[ai], pos_z[ai]);
                let b = Vec3::new(pos_x[bi], pos_y[bi], pos_z[bi]);
                let c = Vec3::new(pos_x[ci], pos_y[ci], pos_z[ci]);
                let (d, _) = distance_primitives::point_triangle_distance_squared(p, a, b, c);
                d
            }
            IpcContactType::EdgeEdge => {
                let [a0i, a1i, b0i, b1i] = contact.indices;
                let a0 = Vec3::new(pos_x[a0i], pos_y[a0i], pos_z[a0i]);
                let a1 = Vec3::new(pos_x[a1i], pos_y[a1i], pos_z[a1i]);
                let b0 = Vec3::new(pos_x[b0i], pos_y[b0i], pos_z[b0i]);
                let b1 = Vec3::new(pos_x[b1i], pos_y[b1i], pos_z[b1i]);
                let (d, _) = distance_primitives::edge_edge_distance_squared(a0, a1, b0, b1);
                d
            }
        }
    }

    fn compute_distance_gradient(
        &self,
        contact: &IpcContact,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> [Vec3; 4] {
        match contact.contact_type {
            IpcContactType::VertexTriangle => {
                let [pi, ai, bi, ci] = contact.indices;
                let p = Vec3::new(pos_x[pi], pos_y[pi], pos_z[pi]);
                let a = Vec3::new(pos_x[ai], pos_y[ai], pos_z[ai]);
                let b = Vec3::new(pos_x[bi], pos_y[bi], pos_z[bi]);
                let c = Vec3::new(pos_x[ci], pos_y[ci], pos_z[ci]);
                point_triangle_distance_gradient(p, a, b, c)
            }
            IpcContactType::EdgeEdge => {
                let [a0i, a1i, b0i, b1i] = contact.indices;
                let a0 = Vec3::new(pos_x[a0i], pos_y[a0i], pos_z[a0i]);
                let a1 = Vec3::new(pos_x[a1i], pos_y[a1i], pos_z[a1i]);
                let b0 = Vec3::new(pos_x[b0i], pos_y[b0i], pos_z[b0i]);
                let b1 = Vec3::new(pos_x[b1i], pos_y[b1i], pos_z[b1i]);
                edge_edge_distance_gradient(a0, a1, b0, b1)
            }
        }
    }
}
