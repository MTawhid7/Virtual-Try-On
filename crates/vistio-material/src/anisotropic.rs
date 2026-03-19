//! Anisotropic co-rotational constitutive model.
//!
//! Combines the co-rotational framework (polar decomposition + tension-field
//! theory) with **direction-dependent stiffness** along warp and weft axes.
//!
//! ## Pipeline
//!
//! 1. Compute deformation gradient F
//! 2. Polar decomposition: F = R · S
//! 3. Eigendecompose S to get principal stretches (σ₁, σ₂) and directions
//! 4. For each principal direction, compute alignment with warp/weft axes
//! 5. Apply direction-weighted tension-field clamping:
//!    - Tensile (σ > 1.0): restore toward 1.0, weighted by aligned stiffness
//!    - Compressive (σ ≤ 1.0): zero force (tension-field theory)
//! 6. Reconstruct target F = R · S_target
//!
//! ## Anisotropy
//!
//! The warp/weft stiffness ratio controls how the fabric responds differently
//! when stretched along the warp vs weft directions. For example:
//! - **Denim**: warp stiffness >> weft stiffness (rigid along warp threads)
//! - **Jersey knit**: warp ≈ weft, both low (stretchy in all directions)
//! - **Silk charmeuse**: low stiffness overall, slight warp bias

use vistio_math::decomposition::polar_decomposition_3x2;
use vistio_math::mat3x2::Mat3x2;

use crate::traits::{ConstitutiveModel, ProjectedGradient};

/// Anisotropic co-rotational model with tension-field theory.
///
/// This is the primary material model for Tier 3+, replacing both the
/// isotropic `CoRotationalModel` and the `OrthotropicLinearModel`.
///
/// It correctly handles:
/// - Large rotations (via co-rotational decomposition)
/// - Compression resistance zeroing (via tension-field theory)
/// - Direction-dependent stretch stiffness (via warp/weft weighting)
pub struct AnisotropicCoRotationalModel {
    /// Stiffness along the warp direction (u-axis in material space).
    /// Higher values = stiffer along warp threads.
    pub warp_stiffness: f32,
    /// Stiffness along the weft direction (v-axis in material space).
    /// Higher values = stiffer along weft threads.
    pub weft_stiffness: f32,
}

impl AnisotropicCoRotationalModel {
    /// Create a new anisotropic model with explicit warp/weft stiffness.
    pub fn new(warp_stiffness: f32, weft_stiffness: f32) -> Self {
        Self {
            warp_stiffness,
            weft_stiffness,
        }
    }

    /// Create from FabricProperties.
    pub fn from_properties(properties: &crate::FabricProperties) -> Self {
        Self {
            warp_stiffness: properties.stretch_stiffness_warp,
            weft_stiffness: properties.stretch_stiffness_weft,
        }
    }

    /// Create an isotropic version (warp = weft).
    ///
    /// When both stiffnesses are equal, this model behaves identically
    /// to `CoRotationalModel` with tension-field theory.
    pub fn isotropic(stiffness: f32) -> Self {
        Self {
            warp_stiffness: stiffness,
            weft_stiffness: stiffness,
        }
    }
}

impl ConstitutiveModel for AnisotropicCoRotationalModel {
    fn project(
        &self,
        deformation_gradient: &Mat3x2,
        rest_area: f32,
        stiffness: f32,
    ) -> ProjectedGradient {
        let polar = polar_decomposition_3x2(deformation_gradient);
        let (s0, s1) = polar.singular_values;

        // Clamp stiffness explicitly to [0, 1] instead of normalizing.
        // Normalization washes out the true stiffness differences if max_k != 1.0!
        let k_warp_norm = self.warp_stiffness.clamp(0.0, 1.0);
        let k_weft_norm = self.weft_stiffness.clamp(0.0, 1.0);

        // The principal stretch directions (eigenvectors of S) are in the 2D
        // material space. The first eigenvector (v0) corresponds to the larger
        // singular value (s0), and the second (v1) to the smaller (s1).
        //
        // We assume warp = u-axis = [1, 0] and weft = v-axis = [0, 1] in
        // the reference (material) space. The alignment of each eigenvector
        // with these axes determines how much warp vs weft stiffness applies.
        let (v0, _v1) = polar.eigenvectors;

        // Compute alignment of first principal direction with warp axis.
        // cos²(θ) where θ is angle between eigenvector and warp direction [1, 0].
        let cos2 = v0.x * v0.x;
        let sin2 = v0.y * v0.y;

        // Effective stiffness for each principal direction:
        // Direction aligned with warp → use warp stiffness
        // Direction aligned with weft → use weft stiffness
        let k0 = k_warp_norm * cos2 + k_weft_norm * sin2;
        let k1 = k_warp_norm * sin2 + k_weft_norm * cos2;

        // Tension-field theory + anisotropic restoration:
        // - Tensile (σ > 1.0): target = 1.0 + (σ - 1.0) * (1 - k)
        //   With k=1 (full stiffness): target = 1.0 (fully restore)
        //   With k=0 (no stiffness): target = σ (no restoration)
        // - Compressive (σ ≤ 1.0): completely disable tension-field logic 
        //   (use 1.0 pure ARAP) because returning s_current in a constant-matrix
        //   PD solver equates to massive viscous drag.
        const COMPRESSION_BLEND: f32 = 1.0;

        let s0_target = if s0 > 1.0 {
            1.0 + (s0 - 1.0) * (1.0 - k0)
        } else {
            s0 + COMPRESSION_BLEND * (1.0 - s0)
        };

        let s1_target = if s1 > 1.0 {
            1.0 + (s1 - 1.0) * (1.0 - k1)
        } else {
            s1 + COMPRESSION_BLEND * (1.0 - s1)
        };

        // Reconstruct target stretch: S_target = V · diag(s0_t, s1_t) · Vᵀ
        let (ev0, ev1) = polar.eigenvectors;
        let t00 = ev0.x * ev0.x * s0_target + ev1.x * ev1.x * s1_target;
        let t01 = ev0.x * ev0.y * s0_target + ev1.x * ev1.y * s1_target;
        let t11 = ev0.y * ev0.y * s0_target + ev1.y * ev1.y * s1_target;

        let target_s = [t00, t01, t01, t11];
        let target_f = polar.rotation.mul_mat2(target_s);

        // Energy: weighted Frobenius norm of (F - F_target)
        let diff = *deformation_gradient - target_f;
        let energy = 0.5 * stiffness * rest_area * diff.frobenius_norm_sq();

        ProjectedGradient { target_f, energy }
    }

    fn name(&self) -> &str {
        "anisotropic_corotational"
    }
}
