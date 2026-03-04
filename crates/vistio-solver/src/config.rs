//! Solver configuration.
//!
//! Parameters that control solver behavior: iteration counts,
//! convergence tolerance, material stiffness weights.

use serde::{Deserialize, Serialize};

/// Configuration for the simulation solver.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolverConfig {
    /// Maximum solver iterations per timestep.
    pub max_iterations: u32,

    /// Convergence tolerance (residual norm threshold).
    /// Solver stops early if residual < tolerance.
    pub tolerance: f64,

    /// Gravity vector [gx, gy, gz] in m/s².
    pub gravity: [f32; 3],

    /// Velocity damping factor (0.0 = no damping, 1.0 = full damping).
    pub damping: f32,

    /// Stretch stiffness weight (scales constitutive model energy).
    pub stretch_weight: f32,

    /// Bending stiffness weight.
    pub bending_weight: f32,

    /// Contact stiffness weight.
    pub contact_weight: f32,

    /// Whether to use Chebyshev acceleration.
    pub chebyshev_acceleration: bool,

    /// Chebyshev spectral radius estimate (0.0–1.0).
    pub spectral_radius: f32,

    /// Rayleigh mass-proportional damping coefficient (α_M).
    /// Controls velocity-proportional damping that dissipates high-frequency
    /// oscillations at boundary vertices (corner twist, edge rattle).
    /// 0.0 = no Rayleigh damping, 2.0–5.0 = typical for cloth.
    pub rayleigh_mass_damping: f32,

    /// Optional material name (from MaterialDatabase). When set, the solver
    /// uses material-aware initialization with the corresponding FabricProperties.
    pub material_name: Option<String>,

    // ─── IPC + Augmented Lagrangian (Tier 4) ─────────────────

    /// Barrier activation threshold d̂ (squared distance units).
    /// Contacts with d² < d_hat activate the barrier.
    /// Default: 1e-3 (~3cm activation zone for garment-scale simulation).
    pub barrier_d_hat: f32,

    /// Separate barrier threshold for self-collision contacts.
    /// Typically equal to or larger than `barrier_d_hat` to ensure
    /// visible layer separation when cloth folds onto itself.
    /// Default: 1e-3.
    pub self_collision_d_hat: f32,

    /// Physical fabric thickness (meters) for C-IPC thickness-aware contact.
    /// Enforces minimum separation between cloth mid-surfaces equal to this value.
    /// Derived from KES-FB3 compression measurements.
    /// Default: 0.001 (1mm, typical for cotton/jersey).
    pub cloth_thickness: f32,

    /// Initial barrier stiffness scaling κ.
    /// Set to 0.0 to use adaptive estimation based on mesh/material properties.
    /// Default: 0.0 (adaptive).
    pub barrier_kappa: f32,

    /// Whether IPC contact is enabled (Tier 4).
    pub ipc_enabled: bool,

    /// Whether CCD step validation is enabled.
    pub ccd_enabled: bool,

    /// Maximum Augmented Lagrangian outer iterations.
    /// Default: 5.
    pub al_max_iterations: u32,

    /// Initial AL penalty parameter μ₀.
    /// Default: 1e3.
    pub al_mu_initial: f32,

    /// AL penalty growth factor β (μ → β·μ when constraints not satisfied).
    /// Default: 2.0.
    pub al_mu_growth: f32,

    /// AL constraint violation tolerance ε_AL.
    /// Default: 1e-4.
    pub al_tolerance: f32,

    /// Coulomb friction coefficient μ for contact surfaces.
    /// Controls how much tangential velocity is removed during contact.
    /// 0.0 = frictionless, 0.5 = typical cloth, 1.0 = very rough.
    /// Default: 0.4.
    pub friction_coefficient: f32,

    /// Contact-aware velocity damping factor.
    /// Applied only to vertices currently in contact with a collider.
    /// Closer to 1.0 = less damping, closer to 0.0 = more damping.
    /// Default: 0.05 (strong damping for inelastic cloth contact).
    pub contact_damping: f32,

    // ─── Compliant Contact (Phase 3) ─────────────────────────

    /// Enable compliant contact model (wider, softer barrier zone).
    /// When true, d_hat is scaled by `compliant_d_hat_scale` for a more
    /// gradual deceleration instead of hard stopping.
    pub compliant_contact: bool,

    /// Scale factor applied to d_hat when compliant contact is enabled.
    /// Default: 2.0 (doubles the barrier activation zone).
    pub compliant_d_hat_scale: f32,

    /// Enable velocity-dependent adaptive contact damping.
    /// Fast-approaching vertices get less damping (let barrier decelerate naturally),
    /// near-rest vertices get more damping (accelerate settling).
    pub adaptive_contact_damping: bool,

    /// Maximum contact damping factor (used at near-zero velocity).
    pub contact_damping_max: f32,

    /// Velocity threshold for adaptive damping (m/s).
    /// Below this velocity, damping increases toward `contact_damping_max`.
    pub contact_velocity_threshold: f32,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: vistio_types::constants::DEFAULT_PD_ITERATIONS,
            tolerance: 1e-6,
            gravity: [0.0, -vistio_types::constants::GRAVITY, 0.0],
            damping: 0.01,
            stretch_weight: 1.0,
            bending_weight: 0.5,
            contact_weight: 1.0,
            chebyshev_acceleration: false,
            spectral_radius: 0.5,
            rayleigh_mass_damping: 2.0,
            material_name: None,
            // IPC defaults — disabled unless explicitly enabled
            barrier_d_hat: 1e-3,  // ~3cm activation zone (squared distance)
            self_collision_d_hat: 1e-3, // same activation for self-collision
            cloth_thickness: 0.001, // 1mm physical fabric thickness
            barrier_kappa: 0.0,   // 0 = adaptive estimation
            ipc_enabled: false,
            ccd_enabled: true,
            al_max_iterations: 5,
            al_mu_initial: 1.0,
            al_mu_growth: 10.0,
            al_tolerance: 1e-3,
            friction_coefficient: 0.4,
            contact_damping: 0.3,
            // Phase 3: Compliant contact defaults
            compliant_contact: true,
            compliant_d_hat_scale: 2.0,
            adaptive_contact_damping: true,
            contact_damping_max: 0.5,
            contact_velocity_threshold: 0.1,
        }
    }
}

impl SolverConfig {
    /// Creates a config for debugging (fewer iterations, looser tolerance).
    pub fn debug() -> Self {
        Self {
            max_iterations: 3,
            tolerance: 1e-3,
            ..Default::default()
        }
    }

    /// Creates a high-quality config (more iterations, tighter tolerance).
    pub fn high_quality() -> Self {
        Self {
            max_iterations: 30,
            tolerance: 1e-8,
            chebyshev_acceleration: true,
            ..Default::default()
        }
    }
}
