//! PD system matrix assembly for Projective Dynamics.
//!
//! Builds the constant system matrix `A = (M/h²) + Σ wᵢ Sᵢᵀ Sᵢ + Σ wᵇⱼ Lⱼᵀ Lⱼ`
//! where:
//! - `M/h²` is the mass matrix scaled by 1/dt²
//! - `Sᵢ` is the selection-gradient matrix for triangle i (membrane)
//! - `wᵢ` is the stiffness weight for triangle i
//! - `Lⱼ` is the Laplacian operator for bending edge j (cotangent-weighted or uniform)
//! - `wᵇⱼ` is the bending stiffness weight for edge j
//!
//! The matrix is N×N (one per coordinate axis), solved three times
//! per global step (for X, Y, Z) using the same cached factorization.
//!
//! Also assembles the RHS vector for the PD global step.

use vistio_math::sparse::CsrMatrix;
use crate::bending::BendingData;
use crate::discrete_shells::DiscreteShellsBendingData;
use crate::element::ElementData;

/// Selects which bending model to use for system matrix and RHS assembly.
pub enum BendingModel<'a> {
    /// Legacy Tier 1-2 dihedral angle springs with uniform Laplacian stencil.
    Dihedral(&'a BendingData),
    /// Tier 3+ Discrete Shells with cotangent-weighted Laplacian stencil.
    DiscreteShells(&'a DiscreteShellsBendingData),
}

/// Assemble the PD system matrix A = (M/h²) + Σ wᵢ Sᵢᵀ Sᵢ + Σ wᵇⱼ Lⱼᵀ Lⱼ.
///
/// This matrix is constant as long as:
/// - Topology doesn't change (no remeshing)
/// - Masses don't change
/// - Timestep dt doesn't change
///
/// Returns an N×N CSR matrix.
///
/// # Arguments
/// * `n` — Number of vertices
/// * `mass` — Per-vertex mass (length N)
/// * `dt` — Timestep
/// * `elements` — Precomputed element data (membrane stiffness)
/// * `bending` — Optional bending model (dihedral or Discrete Shells)
pub fn assemble_system_matrix(
    n: usize,
    mass: &[f32],
    dt: f32,
    elements: &ElementData,
    bending: Option<BendingModel<'_>>,
) -> CsrMatrix {
    let inv_dt2 = 1.0 / (dt * dt);

    // Accumulate via triplets (row, col, value)
    let bending_count = match &bending {
        Some(BendingModel::Dihedral(b)) => b.len(),
        Some(BendingModel::DiscreteShells(b)) => b.len(),
        None => 0,
    };
    let mut triplets: Vec<(usize, usize, f32)> =
        Vec::with_capacity(n + elements.len() * 9 + bending_count * 16);

    // Mass term: M/h² → diagonal entries
    for (i, &m) in mass.iter().enumerate().take(n) {
        triplets.push((i, i, m * inv_dt2));
    }

    // Membrane stiffness term: Σ wᵢ Sᵢᵀ Sᵢ
    for elem in &elements.elements {
        let [i0, i1, i2] = elem.indices;
        let w = elem.weight;

        let a = elem.dm_inv[0];
        let b = elem.dm_inv[1];
        let c = elem.dm_inv[2];
        let d = elem.dm_inv[3];

        let g = [
            [-(a + b), -(c + d)],
            [a, c],
            [b, d],
        ];
        let idx = [i0, i1, i2];

        for a in 0..3 {
            for b in 0..3 {
                let dot = g[a][0] * g[b][0] + g[a][1] * g[b][1];
                if dot.abs() > 1e-12 {
                    triplets.push((idx[a], idx[b], w * dot));
                }
            }
        }
    }

    // Bending stiffness term: Σ wᵇⱼ Lⱼᵀ Lⱼ
    match bending {
        Some(BendingModel::Dihedral(bend)) => {
            // Uniform Laplacian stencil: [-1, -1, 1, 1]
            for elem in &bend.elements {
                let idx = [elem.v0, elem.v1, elem.wing_a, elem.wing_b];
                let w = elem.weight;
                let stencil = [-1.0_f32, -1.0, 1.0, 1.0];

                for a in 0..4 {
                    for b in 0..4 {
                        let val = w * stencil[a] * stencil[b];
                        if val.abs() > 1e-12 {
                            triplets.push((idx[a], idx[b], val));
                        }
                    }
                }
            }
        }
        Some(BendingModel::DiscreteShells(bend)) => {
            // Cotangent-weighted Laplacian stencil per element
            for elem in &bend.elements {
                let idx = [elem.v0, elem.v1, elem.wing_a, elem.wing_b];
                let w = elem.weight;
                let s = &elem.stencil;

                for a in 0..4 {
                    for b in 0..4 {
                        let val = w * s[a] * s[b];
                        if val.abs() > 1e-12 {
                            triplets.push((idx[a], idx[b], val));
                        }
                    }
                }
            }
        }
        None => {}
    }

    CsrMatrix::from_triplets(n, n, &triplets)
}

/// Assemble the RHS vector for the PD global step.
///
/// RHS = (M/h²) · q_pred + Σ wᵢ Sᵢᵀ pᵢ + Σ wᵇⱼ Lⱼᵀ pᵇⱼ
///
/// where:
/// - `q_pred` is the predicted position (from inertia)
/// - `pᵢ` is the local projection target for element i (membrane)
/// - `pᵇⱼ` is the local projection target for bending element j
/// - `Sᵢᵀ` scatters the triangle-local projection back to global
/// - `Lⱼᵀ` scatters the bending projection back to global
///
/// # Arguments
/// * `n` — Number of vertices
/// * `mass` — Per-vertex mass
/// * `dt` — Timestep
/// * `pred` — Predicted positions (one coordinate axis)
/// * `proj_targets` — Per-element projection target for each vertex
/// * `elements` — Precomputed element data
/// * `coord` — Which coordinate axis (0=X, 1=Y, 2=Z)
/// * `bending` — Optional bending model with targets
#[allow(clippy::too_many_arguments)]
pub fn assemble_rhs(
    n: usize,
    mass: &[f32],
    dt: f32,
    pred: &[f32],
    proj_targets: &[(f32, f32, f32)],
    elements: &ElementData,
    coord: usize,
    bending: Option<BendingRhs<'_>>,
) -> Vec<f32> {
    let inv_dt2 = 1.0 / (dt * dt);
    let mut rhs = vec![0.0_f32; n];

    // Mass/inertia term: (M/h²) · q_pred
    for i in 0..n {
        rhs[i] += mass[i] * inv_dt2 * pred[i];
    }

    // Membrane stiffness term: Σ wᵢ Sᵢᵀ pᵢ
    for (e, elem) in elements.elements.iter().enumerate() {
        let [i0, i1, i2] = elem.indices;
        let w = elem.weight;

        let a = elem.dm_inv[0];
        let b = elem.dm_inv[1];
        let c = elem.dm_inv[2];
        let d = elem.dm_inv[3];

        let g = [
            [-(a + b), -(c + d)],
            [a, c],
            [b, d],
        ];
        let idx = [i0, i1, i2];

        let target = [proj_targets[e].0, proj_targets[e].1, proj_targets[e].2];

        let te1 = target[1] - target[0];
        let te2 = target[2] - target[0];

        let f_target_0 = te1 * a + te2 * b;
        let f_target_1 = te1 * c + te2 * d;

        let _ = coord;
        for a_idx in 0..3 {
            let contrib = g[a_idx][0] * f_target_0 + g[a_idx][1] * f_target_1;
            rhs[idx[a_idx]] += w * contrib;
        }
    }

    // Bending stiffness term: Σ wᵇⱼ Lⱼᵀ pᵇⱼ
    match bending {
        Some(BendingRhs::Dihedral { data, targets }) => {
            let stencil = [-1.0_f32, -1.0, 1.0, 1.0];
            for (e, elem) in data.elements.iter().enumerate() {
                let idx = [elem.v0, elem.v1, elem.wing_a, elem.wing_b];
                let w = elem.weight;
                let target = [targets[e].0, targets[e].1, targets[e].2, targets[e].3];

                let l_dot_target: f32 = (0..4).map(|k| stencil[k] * target[k]).sum();
                for i in 0..4 {
                    rhs[idx[i]] += w * stencil[i] * l_dot_target;
                }
            }
        }
        Some(BendingRhs::DiscreteShells { data, targets }) => {
            for (e, elem) in data.elements.iter().enumerate() {
                let idx = [elem.v0, elem.v1, elem.wing_a, elem.wing_b];
                let w = elem.weight;
                let s = &elem.stencil;
                let target = [targets[e].0, targets[e].1, targets[e].2, targets[e].3];

                // Lᵀ · (L · target) using per-element cotangent stencil
                let l_dot_target: f32 = (0..4).map(|k| s[k] * target[k]).sum();
                for i in 0..4 {
                    rhs[idx[i]] += w * s[i] * l_dot_target;
                }
            }
        }
        None => {}
    }

    rhs
}

/// Bending RHS assembly data — pairs a bending model with its projection targets.
pub enum BendingRhs<'a> {
    /// Dihedral model with uniform stencil.
    Dihedral {
        data: &'a BendingData,
        targets: &'a [(f32, f32, f32, f32)],
    },
    /// Discrete Shells model with cotangent-weighted stencil.
    DiscreteShells {
        data: &'a DiscreteShellsBendingData,
        targets: &'a [(f32, f32, f32, f32)],
    },
}

/// Accumulate IPC barrier gradient and Augmented Lagrangian multiplier
/// contributions into an existing RHS vector.
///
/// RHS += -(μ · barrier_grad + λ) for each vertex, where:
/// - `barrier_grad` is the barrier energy gradient w.r.t. positions
///   (already scaled by κ in `detect_ipc_contacts`)
/// - `lambda` is the Lagrange multiplier estimate
/// - `mu` is the current AL penalty parameter
///
/// The negative sign converts energy gradient into force (F = -∇E).
///
/// # Arguments
/// * `rhs` — RHS vector to modify in place (length N)
/// * `barrier_grad` — per-vertex barrier gradient (length N), already κ-scaled
/// * `lambda` — per-vertex Lagrange multiplier (length N)
/// * `mu` — current AL penalty parameter
pub fn assemble_barrier_rhs(
    rhs: &mut [f32],
    barrier_grad: &[f32],
    mu: f32,
) {
    for i in 0..rhs.len().min(barrier_grad.len()) {
        // Force = -(μ · ∇barrier)
        rhs[i] -= mu * barrier_grad[i];
    }
}
