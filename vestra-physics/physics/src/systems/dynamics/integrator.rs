// physics/src/systems/dynamics/integrator.rs

use glam::Vec3;
use crate::engine::state::PhysicsState;
use crate::engine::config::PhysicsConfig;

/// Performs Verlet Integration to update particle positions.
/// This is an "explicit" integration step, but combined with XPBD (which happens after),
/// it forms a semi-implicit solver that is stable for cloth simulation.
pub struct Integrator;

impl Integrator {
    /// Updates position based on velocity and external forces (Gravity + Aerodynamics).
    /// P(new) = P(curr) + V * dt + 0.5 * A * dt^2
    ///
    /// OPTIMIZATION: Uses 4x loop unrolling for instruction-level parallelism.
    pub fn integrate(
        state: &mut PhysicsState,
        config: &PhysicsConfig,
        external_forces: &[Vec3],
        dt: f32
    ) {
        let dt_sq = dt * dt;

        // Process particles with 4x unrolling
        let count = state.count;
        let chunks = count / 4;
        let remainder = count % 4;

        for chunk in 0..chunks {
            let base = chunk * 4;
            Self::integrate_single(state, config, external_forces, dt_sq, base);
            Self::integrate_single(state, config, external_forces, dt_sq, base + 1);
            Self::integrate_single(state, config, external_forces, dt_sq, base + 2);
            Self::integrate_single(state, config, external_forces, dt_sq, base + 3);
        }

        // Handle remainder
        for i in (chunks * 4)..(chunks * 4 + remainder) {
            Self::integrate_single(state, config, external_forces, dt_sq, i);
        }
    }

    /// Integrates a single particle.
    #[inline(always)]
    fn integrate_single(
        state: &mut PhysicsState,
        config: &PhysicsConfig,
        external_forces: &[Vec3],
        dt_sq: f32,
        i: usize
    ) {
        if state.inv_mass[i] == 0.0 { return; }

        let pos = state.positions[i];
        let prev = state.prev_positions[i];

        // F = ma => a = F * inv_mass
        // Gravity is constant acceleration.
        // Aerodynamics is a Force, so we multiply by inv_mass.
        let f_aero = external_forces[i];
        let acceleration = config.gravity + (f_aero * state.inv_mass[i]);

        // Verlet integration with damping
        // velocity = (pos - prev) * damping
        // next_pos = pos + velocity + acceleration * dt^2
        let velocity_term = (pos - prev) * config.damping;
        let acceleration_term = glam::Vec4::from((acceleration * dt_sq, 0.0));
        let next_pos = pos + velocity_term + acceleration_term;

        state.prev_positions[i] = pos;
        state.positions[i] = next_pos;
    }
}