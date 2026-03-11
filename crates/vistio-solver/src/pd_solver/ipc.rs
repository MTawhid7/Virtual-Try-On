//! IPC (Incremental Potential Contact) types and traits.
//!
//! Defines the callback interface between the solver and the collision system:
//! - `IpcCollisionHandler` trait — detects contacts and computes CCD step
//! - `EmptyIpcHandler` — no-op implementation for non-IPC scenarios
//! - `IpcBarrierForces` — barrier gradient data passed from contact detection to solver

// ─── IPC Collision Handler Trait ──────────────────────────────

pub trait IpcCollisionHandler {
    fn detect_contacts(&mut self, pos_x: &[f32], pos_y: &[f32], pos_z: &[f32]) -> IpcBarrierForces;
    #[allow(clippy::too_many_arguments)]
    fn compute_ccd_step(&mut self, prev_x: &[f32], prev_y: &[f32], prev_z: &[f32], new_x: &[f32], new_y: &[f32], new_z: &[f32], padding: f32) -> f32;
    /// Set the effective d_hat (barrier activation zone) for subsequent calls.
    /// Used by the solver to implement compliant contact (Phase 3.2).
    fn set_d_hat(&mut self, _d_hat: f32) {}
}

pub struct EmptyIpcHandler;
impl IpcCollisionHandler for EmptyIpcHandler {
    fn detect_contacts(&mut self, px: &[f32], _py: &[f32], _pz: &[f32]) -> IpcBarrierForces {
        IpcBarrierForces::empty(px.len())
    }
    fn compute_ccd_step(&mut self, _px: &[f32], _py: &[f32], _pz: &[f32], _nx: &[f32], _ny: &[f32], _nz: &[f32], _padding: f32) -> f32 {
        1.0
    }
}

// ─── IPC Barrier Forces ──────────────────────────────────────

/// Barrier force data passed from the contact detection system into the solver.
///
/// Computed by the caller using `IpcContactSet::compute_barrier_gradient()`.
/// Barrier forces computed by the contact detection system
/// and passed back into the solver's Augmented Lagrangian loop.
#[derive(Debug, Clone)]
pub struct IpcBarrierForces {
    /// Gradient of the barrier energy w.r.t vertex x-coordinates
    pub grad_x: Vec<f32>,
    /// Gradient of the barrier energy w.r.t vertex y-coordinates
    pub grad_y: Vec<f32>,
    /// Gradient of the barrier energy w.r.t vertex z-coordinates
    pub grad_z: Vec<f32>,
    /// Maximum constraint violation across all active contacts
    pub max_violation: f32,
    /// Number of active IPC contacts pushing on the system
    pub active_contacts: usize,

    // ─── Per-vertex contact information for velocity filter ───

    /// Per-vertex contact normal (x component). Normalized.
    /// Zero for vertices not in contact.
    pub contact_nx: Vec<f32>,
    /// Per-vertex contact normal (y component). Normalized.
    pub contact_ny: Vec<f32>,
    /// Per-vertex contact normal (z component). Normalized.
    pub contact_nz: Vec<f32>,
    /// Whether each vertex is currently in contact with any collider.
    pub in_contact: Vec<bool>,

    // ─── Barrier Hessian diagonal (Phase 2) ──────────────────

    /// Per-vertex barrier Hessian diagonal: κ · ∂²b/∂d² · (∂d/∂x)².
    /// Used as a Jacobi preconditioner to compensate for the system matrix
    /// not including barrier stiffness.
    pub hessian_diag: Vec<f32>,
}

impl IpcBarrierForces {
    pub fn empty(n_vertices: usize) -> Self {
        Self {
            grad_x: vec![0.0; n_vertices],
            grad_y: vec![0.0; n_vertices],
            grad_z: vec![0.0; n_vertices],
            max_violation: 0.0,
            active_contacts: 0,
            contact_nx: vec![0.0; n_vertices],
            contact_ny: vec![0.0; n_vertices],
            contact_nz: vec![0.0; n_vertices],
            in_contact: vec![false; n_vertices],
            hessian_diag: vec![0.0; n_vertices],
        }
    }
}
