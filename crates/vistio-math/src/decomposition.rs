//! Matrix decompositions for constitutive models.
//!
//! Provides polar decomposition (F = R·S) needed by co-rotational FEM,
//! and SVD for robust handling of degenerate/inverted elements.
//!
//! ## Tension-Field Theory
//!
//! The [`clamp_stretch`] function implements tension-field theory by clamping
//! compressive singular values to 1.0. This allows the constitutive model to
//! zero out compressive stress while preserving tensile behavior — essential
//! for realistic cloth that buckles rather than resisting compression.

use crate::mat3x2::Mat3x2;
use glam::Vec3;

/// Result of a 3×2 polar decomposition: F = R · S
///
/// - `R` is a 3×2 rotation matrix (orthonormal columns)
/// - `S` is a 2×2 symmetric positive semi-definite stretch tensor
#[derive(Debug, Clone, Copy)]
pub struct PolarDecomposition {
    /// Rotation part (3×2, orthonormal columns).
    pub rotation: Mat3x2,
    /// Stretch part (2×2 symmetric, stored as [s00, s01, s01, s11]).
    pub stretch: [f32; 4],
    /// Principal stretches (singular values σ₁ ≥ σ₂ ≥ 0).
    /// σ < 1.0 means compression, σ > 1.0 means extension.
    pub singular_values: (f32, f32),
    /// Eigenvectors of the stretch tensor (columns of V in S = V diag(σ) V^T).
    /// Stored as (v0, v1) where v0 corresponds to σ₁.
    pub eigenvectors: (glam::Vec2, glam::Vec2),
}

/// Clamp singular values to implement **tension-field theory** compatible
/// with Projective Dynamics' constant system matrix.
///
/// The effect on the energy E = ||F - R·S_target||²:
/// - **Tensile** (σ > 1.0): S_target has σ_target = 1.0, so the element is pulled
///   back to its rest length. Normal stretch resistance is preserved.
/// - **Compressive** (σ < 1.0): S_target blends toward σ_current with a small
///   offset toward 1.0. This gives near-zero compression resistance (allowing
///   draping) while avoiding the full PD drag artifact that occurs when
///   target == current exactly.
///
/// In standard PD, the stiffness K is pre-factored into the constant LHS matrix.
/// - If target = s_current exactly: K*x_current on RHS, creating massive drag
/// - If target = 1.0 (ARAP): full compression penalty, preventing draping
/// - Blend (0.1 toward 1.0): ~10% of ARAP compression force, enough to
///   avoid drag while allowing gravity to dominate
///
/// Returns the clamped 2×2 stretch tensor [s00, s01, s01, s11].
pub fn clamp_stretch(polar: &PolarDecomposition) -> [f32; 4] {
    let (s0, s1) = polar.singular_values;
    let (v0, v1) = polar.eigenvectors;

    // Tension-field blend factor for compression.
    // 0.0 = pure tension-field (target=s, full PD drag)
    // 1.0 = pure ARAP (target=1.0, full compression penalty)
    // ANY value < 1.0 creates viscous drag proportional to (1-BLEND)*K.
    // To allow the cloth edges to fall, we MUST eliminate viscous drag 
    // by using 1.0, and rely on weak membrane stiffness to allow draping.
    const COMPRESSION_BLEND: f32 = 1.0;

    let cs0 = if s0 >= 1.0 {
        1.0  // Tensile: clamp to rest length (standard stretch resistance)
    } else {
        // Compressive: blend toward rest length to avoid PD drag
        // target = s0 + BLEND * (1.0 - s0) = lerp(s0, 1.0, BLEND)
        s0 + COMPRESSION_BLEND * (1.0 - s0)
    };

    let cs1 = if s1 >= 1.0 {
        1.0
    } else {
        s1 + COMPRESSION_BLEND * (1.0 - s1)
    };

    // Reconstruct S_target = V * diag(cs0, cs1) * V^T
    let s00 = v0.x * v0.x * cs0 + v1.x * v1.x * cs1;
    let s01 = v0.x * v0.y * cs0 + v1.x * v1.y * cs1;
    let s11 = v0.y * v0.y * cs0 + v1.y * v1.y * cs1;

    [s00, s01, s01, s11]
}

/// Compute the polar decomposition of a 3×2 deformation gradient.
///
/// Uses the SVD-based approach for robustness:
/// 1. Compute C = F^T F (2×2 symmetric)
/// 2. Eigendecompose C to get S = sqrt(C)
/// 3. R = F · S^{-1}
///
/// For degenerate triangles (det(S) ≈ 0), falls back to a safe default.
pub fn polar_decomposition_3x2(f: &Mat3x2) -> PolarDecomposition {
    let c = f.ftf(); // C = F^T F is a 2x2 symmetric matrix [a, b, b, d]
    let a = c[0];
    let b = c[1];
    let d = c[3];

    // Eigenvalues of 2x2 symmetric matrix: λ = (a+d)/2 ± sqrt(((a-d)/2)^2 + b^2)
    let half_trace = 0.5 * (a + d);
    let half_diff = 0.5 * (a - d);
    let disc = (half_diff * half_diff + b * b).sqrt();

    if !disc.is_finite() {
        return PolarDecomposition {
            rotation: Mat3x2::IDENTITY,
            stretch: [0.0, 0.0, 0.0, 0.0],
            singular_values: (0.0, 0.0),
            eigenvectors: (glam::Vec2::X, glam::Vec2::Y),
        };
    }

    let lambda0 = (half_trace + disc).max(0.0);
    let lambda1 = (half_trace - disc).max(0.0);

    let s0 = lambda0.sqrt();
    let s1 = lambda1.sqrt();

    let eps = vistio_types::constants::DEGENERATE_AREA_THRESHOLD;

    if s0 < eps {
        // Fully degenerate — return identity-like decomposition
        let mut r = Mat3x2::IDENTITY;
        // If F isn't completely zero, we can extract an orientation
        if f.col0.length_squared() > 1e-10 && f.col1.length_squared() > 1e-10 {
            let u = f.col0.normalize();
            let mut v = f.col1 - u * f.col1.dot(u);
            if v.length_squared() > 1e-10 {
                v = v.normalize();
                r.col0 = u;
                r.col1 = v;
            }
        }

        return PolarDecomposition {
            rotation: r,
            stretch: [0.0, 0.0, 0.0, 0.0],
            singular_values: (0.0, 0.0),
            eigenvectors: (glam::Vec2::X, glam::Vec2::Y),
        };
    }

    // Eigenvectors of the 2x2 symmetric matrix
    let (v0, v1) = if b.abs() > eps {
        let v0 = glam::Vec2::new(lambda0 - d, b).normalize();
        let v1 = glam::Vec2::new(-v0.y, v0.x);
        (v0, v1)
    } else if a >= d {
        (glam::Vec2::X, glam::Vec2::Y)
    } else {
        (glam::Vec2::Y, glam::Vec2::X)
    };

    // S = V * diag(s0, s1) * V^T (2x2)
    let s00 = v0.x * v0.x * s0 + v1.x * v1.x * s1;
    let s01 = v0.x * v0.y * s0 + v1.x * v1.y * s1;
    let s11 = v0.y * v0.y * s0 + v1.y * v1.y * s1;

    // S_inv = V * diag(1/s0, 1/s1) * V^T
    let inv_s0 = if s0 > eps { 1.0 / s0 } else { 0.0 };
    let inv_s1 = if s1 > eps { 1.0 / s1 } else { 0.0 };

    let si00 = v0.x * v0.x * inv_s0 + v1.x * v1.x * inv_s1;
    let si01 = v0.x * v0.y * inv_s0 + v1.x * v1.y * inv_s1;
    let si11 = v0.y * v0.y * inv_s0 + v1.y * v1.y * inv_s1;

    // R = F * S^{-1}
    let rotation = f.mul_mat2([si00, si01, si01, si11]);

    PolarDecomposition {
        rotation,
        stretch: [s00, s01, s01, s11],
        singular_values: (s0, s1),
        eigenvectors: (v0, v1),
    }
}

/// Compute the deformation gradient F for a triangle.
///
/// Given three 3D vertex positions (p0, p1, p2) and the precomputed
/// inverse of the rest-state edge matrix (Dm_inv, 2×2), returns
/// the 3×2 deformation gradient.
///
/// F = Ds * Dm_inv, where Ds = [p1-p0, p2-p0] (3×2).
pub fn deformation_gradient(
    p0: Vec3,
    p1: Vec3,
    p2: Vec3,
    dm_inv: [f32; 4], // 2x2 column-major [a, b, c, d]
) -> Mat3x2 {
    let ds = Mat3x2::from_cols(p1 - p0, p2 - p0);
    ds.mul_mat2(dm_inv)
}
