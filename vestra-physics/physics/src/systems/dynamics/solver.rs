// physics/src/systems/dynamics/solver.rs

use crate::collision::CollisionResolver;
use crate::engine::config::PhysicsConfig;
use crate::engine::state::PhysicsState;
use crate::systems::constraints::{
    AreaConstraint, BendingConstraint, DistanceConstraint, TetherConstraint,
};
use crate::utils::profiler::{ProfileCategory, Profiler};

/// The XPBD (Extended Position Based Dynamics) Solver.
/// Manages and solves all internal constraints of the cloth system.
///
/// OPTIMIZATION: Uses SIMD-vectorized constraints for faster processing.
pub struct Solver {
    pub distance_constraint: DistanceConstraint,
    pub bending_constraint: BendingConstraint,
    pub tether_constraint: TetherConstraint,
    pub area_constraint: AreaConstraint,
}

impl Solver {
    pub fn new(state: &PhysicsState, scale_factor: f32, distance_compliance: f32) -> Self {
        let distance_constraint = DistanceConstraint::new(state, distance_compliance);
        let base_compliance = 1.0;
        let tuned_compliance = base_compliance * (scale_factor * scale_factor);
        let bending_constraint = BendingConstraint::new(state, tuned_compliance);
        let tether_constraint = TetherConstraint::new(state);
        let area_constraint = AreaConstraint::new(state);

        Self {
            distance_constraint,
            bending_constraint,
            tether_constraint,
            area_constraint,
        }
    }

    /// Main simulation loop iteration.
    /// Uses "Sub-stepping" with Chebyshev acceleration (Omega) for faster convergence.
    /// - Iterates `config.solver_iterations` times.
    /// - Adjusts `omega` dynamically for stability.
    /// - Resolves constraints and collisions in order.
    ///
    /// OPTIMIZATION: All constraints use SIMD vectorization for 4-wide parallel processing.
    /// PROFILING: Each constraint type is measured individually.
    pub fn solve(
        &self,
        state: &mut PhysicsState,
        resolver: &CollisionResolver,
        config: &PhysicsConfig,
        dt: f32,
    ) {
        let mut omega = 1.0;
        let rho = config.spectral_radius;

        for i in 0..config.solver_iterations {
            if i == 0 {
                omega = 1.0;
            } else if i == 1 {
                omega = 2.0 / (2.0 - rho * rho);
            } else {
                omega = 4.0 / (4.0 - rho * rho * omega);
            }

            // Accelerate Internal Constraints (SIMD-vectorized)
            Profiler::start(ProfileCategory::DistanceConstraint);
            self.distance_constraint.solve(state, omega, dt);
            Profiler::end(ProfileCategory::DistanceConstraint);

            Profiler::start(ProfileCategory::BendingConstraint);
            self.bending_constraint.solve(state, omega, dt);
            Profiler::end(ProfileCategory::BendingConstraint);

            Profiler::start(ProfileCategory::TetherConstraint);
            self.tether_constraint.solve(state, omega, dt);
            Profiler::end(ProfileCategory::TetherConstraint);

            Profiler::start(ProfileCategory::AreaConstraint);
            self.area_constraint
                .solve(state, config.area_compliance, omega, dt);
            Profiler::end(ProfileCategory::AreaConstraint);

            // FIX: Do NOT accelerate Collisions
            Profiler::start(ProfileCategory::CollisionResolve);
            resolver.resolve_contacts(state, config, dt);
            Profiler::end(ProfileCategory::CollisionResolve);
        }
    }
}
