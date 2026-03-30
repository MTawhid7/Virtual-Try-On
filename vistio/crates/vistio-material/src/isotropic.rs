//! Isotropic linear elastic constitutive model (St. Venant-Kirchhoff).
//!
//! The simplest membrane model: measures strain as the deviation of the
//! deformation gradient from identity and projects by clamping the
//! singular values. Breaks under large rotations (folds produce
//! non-physical energy spikes), but useful as a baseline for comparison.
//!
//! Energy: E = w/2 · ||F - I||²_F  (deviation from rest)

use vistio_math::mat3x2::Mat3x2;

use crate::traits::{ConstitutiveModel, ProjectedGradient};

/// Isotropic linear elastic (StVK) constitutive model.
///
/// Projects the deformation gradient toward the identity (rest state).
/// This is the simplest possible model — the element always "wants"
/// to return to its undeformed configuration, regardless of rotation.
///
/// # Limitations
///
/// Linear elasticity measures strain in the world frame. When an element
/// rotates (e.g., a fold), the rotation appears as strain, producing
/// artificial energy. Use `CoRotationalModel` for production simulations
/// with folds and drapes.
pub struct IsotropicLinearModel;

impl IsotropicLinearModel {
    /// Creates a new isotropic linear model.
    pub fn new() -> Self {
        Self
    }
}

impl Default for IsotropicLinearModel {
    fn default() -> Self {
        Self::new()
    }
}

impl ConstitutiveModel for IsotropicLinearModel {
    fn project(
        &self,
        deformation_gradient: &Mat3x2,
        rest_area: f32,
        stiffness: f32,
    ) -> ProjectedGradient {
        // SVK energy: E = w/2 · ||F - I||²_F
        let diff = *deformation_gradient - Mat3x2::IDENTITY;
        let energy = 0.5 * stiffness * rest_area * diff.frobenius_norm_sq();

        // Target is always identity (undeformed rest state).
        ProjectedGradient {
            target_f: Mat3x2::IDENTITY,
            energy,
        }
    }

    fn name(&self) -> &str {
        "isotropic_linear"
    }
}
