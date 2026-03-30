//! Constitutive model trait — the core material abstraction.
//!
//! Every material model implements this trait, enabling the solver
//! to swap material strategies without changing its local step logic.

use vistio_math::mat3x2::Mat3x2;
use vistio_math::Vec3;

/// Result of a local step projection for one triangle element.
///
/// The solver uses this to update vertex positions during
/// the Projective Dynamics local step.
#[derive(Debug, Clone, Copy)]
pub struct Projection {
    /// Target positions for the three triangle vertices
    /// after projecting to the constitutive model's constraint set.
    pub target: [Vec3; 3],

    /// Elastic energy of this element at the current deformation.
    /// Used for energy tracking and convergence monitoring.
    pub energy: f32,
}

/// Trait for constitutive models (material behavior).
///
/// Implementations define how a triangle element responds to deformation.
/// The solver calls `project()` during the PD local step to compute
/// each element's target configuration.
///
/// # Strategy Pattern
///
/// This trait enables runtime swapping of material models:
/// - `LinearElastic` — Simple, fast, breaks under large rotations
/// - `CoRotational` — Handles large rotations (folds), production default
/// - `Anisotropic` — Warp/weft directional stiffness for woven fabrics
pub trait ConstitutiveModel: Send + Sync {
    /// Compute the local projection for a triangle element.
    ///
    /// # Arguments
    /// - `deformation_gradient` — The 3×2 deformation gradient F = Ds · Dm_inv
    /// - `rest_area` — Area of the triangle in its rest configuration
    /// - `stiffness` — Material stiffness weight for this element
    ///
    /// # Returns
    /// The projected deformation gradient (closest valid configuration)
    /// and the elastic energy at the current deformation.
    fn project(
        &self,
        deformation_gradient: &Mat3x2,
        rest_area: f32,
        stiffness: f32,
    ) -> ProjectedGradient;

    /// Returns the name of this constitutive model.
    fn name(&self) -> &str;
}

/// Result of projecting a deformation gradient to the model's constraint set.
#[derive(Debug, Clone, Copy)]
pub struct ProjectedGradient {
    /// The projected (target) deformation gradient.
    pub target_f: Mat3x2,
    /// Elastic energy at the current deformation.
    pub energy: f32,
}
