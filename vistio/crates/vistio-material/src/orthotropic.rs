//! Orthotropic linear constitutive model.
//!
//! Implements direction-dependent stiffness for woven fabrics where
//! warp and weft directions have different elastic properties.
//!
//! This model extends the isotropic linear model by applying different
//! stiffness multipliers along the warp and weft material axes.

use vistio_math::mat3x2::Mat3x2;
use vistio_math::decomposition::polar_decomposition_3x2;

use crate::traits::{ConstitutiveModel, ProjectedGradient};

/// Orthotropic linear constitutive model.
///
/// Applies direction-dependent stiffness along warp and weft axes.
///
/// # Material Axes
/// - **Warp direction** (u-axis): Default [1, 0] in material space
/// - **Weft direction** (v-axis): Default [0, 1] in material space
///
/// The stiffness is modulated per-axis:
/// - `F_target = R · diag(s_warp, s_weft)` where `s_warp` and `s_weft`
///   are clamped singular values weighted by their respective stiffnesses.
pub struct OrthotropicLinearModel {
    /// Stiffness along warp direction (higher = stiffer).
    pub warp_stiffness: f32,
    /// Stiffness along weft direction (higher = stiffer).
    pub weft_stiffness: f32,
}

impl OrthotropicLinearModel {
    /// Create a new orthotropic model with given warp/weft stiffness.
    pub fn new(warp_stiffness: f32, weft_stiffness: f32) -> Self {
        Self {
            warp_stiffness,
            weft_stiffness,
        }
    }

    /// Create from material properties.
    pub fn from_properties(properties: &crate::FabricProperties) -> Self {
        Self {
            warp_stiffness: properties.stretch_stiffness_warp,
            weft_stiffness: properties.stretch_stiffness_weft,
        }
    }
}

impl ConstitutiveModel for OrthotropicLinearModel {
    fn project(&self, deformation_gradient: &Mat3x2, rest_area: f32, stiffness: f32) -> ProjectedGradient {
        let polar = polar_decomposition_3x2(deformation_gradient);
        let (s0, s1) = polar.singular_values;

        // Apply direction-dependent restoring force.
        let max_k = self.warp_stiffness.max(self.weft_stiffness).max(1e-6);
        let k_warp = self.warp_stiffness / max_k;
        let k_weft = self.weft_stiffness / max_k;

        // Target singular values: stronger restoration for stiffer directions.
        let s0_target = s0 + (1.0 - s0) * k_warp;
        let s1_target = s1 + (1.0 - s1) * k_weft;

        // Reconstruct target stretch from eigenvectors
        let (v0, v1) = polar.eigenvectors;
        let t00 = v0.x * v0.x * s0_target + v1.x * v1.x * s1_target;
        let t01 = v0.x * v0.y * s0_target + v1.x * v1.y * s1_target;
        let t11 = v0.y * v0.y * s0_target + v1.y * v1.y * s1_target;

        let target_s = [t00, t01, t01, t11];
        let target_f = polar.rotation.mul_mat2(target_s);

        // Energy: weighted Frobenius norm of (F - F_target)
        let diff = *deformation_gradient - target_f;
        let frob_sq = diff.frobenius_norm_sq();
        let energy = 0.5 * stiffness * rest_area * frob_sq;

        ProjectedGradient { target_f, energy }
    }

    fn name(&self) -> &str {
        "orthotropic_linear"
    }
}
