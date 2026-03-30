//! Co-rotational constitutive model for Projective Dynamics.
//!
//! Performs polar decomposition F = R·S on the deformation gradient,
//! then applies **tension-field theory**: the target deformation gradient
//! becomes R · S_target, where each principal stretch σ_target = min(σ, 1.0).
//!
//! - **Tensile** (σ > 1.0): target = 1.0, preserving stretch resistance.
//! - **Compressive** (σ < 1.0): target = σ, zeroing out compressive stress.
//!
//! This is essential for realistic cloth: real fabric has near-zero
//! compressive resistance and buckles microscopically instead of
//! resisting in-plane compression. Without clamping, the ARAP energy
//! forces macroscopic accordion-buckling (the "crumpling" artifact).
//!
//! Energy: E = w/2 · ||F - R·S_target||²_F

use vistio_math::decomposition::{clamp_stretch, polar_decomposition_3x2};
use vistio_math::mat3x2::Mat3x2;

use crate::traits::{ConstitutiveModel, ProjectedGradient};

/// Co-rotational FEM constitutive model with tension-field strain limiting.
///
/// The production default for cloth simulation. Extracts the rotation
/// from the deformation gradient via polar decomposition, clamps
/// compressive stretches to 1.0, and returns R·S_clamped as the target.
///
/// # Why tension-field theory?
///
/// The standard ARAP (As-Rigid-As-Possible) projection targets pure
/// rotation R, which penalizes compression equally to stretching.
/// Tension-field theory modifies the target to R·S_clamped, allowing
/// the fabric to compress freely (σ < 1.0 → σ = 1.0) while still
/// penalizing stretching (σ > 1.0 retained).
pub struct CoRotationalModel;

impl CoRotationalModel {
    /// Creates a new co-rotational model.
    pub fn new() -> Self {
        Self
    }
}

impl Default for CoRotationalModel {
    fn default() -> Self {
        Self::new()
    }
}

impl ConstitutiveModel for CoRotationalModel {
    fn project(
        &self,
        deformation_gradient: &Mat3x2,
        rest_area: f32,
        stiffness: f32,
    ) -> ProjectedGradient {
        let polar = polar_decomposition_3x2(deformation_gradient);

        // Tension-field theory: clamp compressive singular values to 1.0
        // This converts the target from pure R (ARAP) to R·S_clamped,
        // which allows free compression while penalizing stretching.
        let clamped_s = clamp_stretch(&polar);
        let target = polar.rotation.mul_mat2(clamped_s);

        let diff = *deformation_gradient - target;
        let energy = 0.5 * stiffness * rest_area * diff.frobenius_norm_sq();

        ProjectedGradient {
            target_f: target,
            energy,
        }
    }

    fn name(&self) -> &str {
        "co_rotational"
    }
}
