//! Post-solve velocity filtering for IPC contacts.
//!
//! After the AL loop converges, this module applies:
//! - Contact normal velocity removal (inelastic contact)
//! - Coulomb friction on tangential velocity
//! - Adaptive contact damping
//! - Rayleigh mass-proportional damping

use crate::config::SolverConfig;
use crate::state::SimulationState;

/// Apply the full post-solve velocity filter for vertices in contact.
///
/// This consolidates all velocity modifications into a single pass:
/// 1. Remove normal velocity (inelastic collision)
/// 2. Apply Coulomb friction to tangential velocity
/// 3. Apply contact-specific damping
pub(crate) fn apply_contact_velocity_filter(
    state: &mut SimulationState,
    config: &SolverConfig,
    ever_in_contact: &[bool],
    contact_nx: &[f32],
    contact_ny: &[f32],
    contact_nz: &[f32],
    active_contacts: usize,
    sub_dt: f32,
) {
    let n = state.vertex_count;
    let mu_friction = config.friction_coefficient;
    let contact_damp = config.contact_damping;

    for i in 0..n {
        if state.inv_mass[i] == 0.0 { continue; }
        if !ever_in_contact[i] { continue; }

        let nx = contact_nx[i];
        let ny = contact_ny[i];
        let nz = contact_nz[i];

        let n_len_sq = nx * nx + ny * ny + nz * nz;
        if n_len_sq < 0.5 { continue; }

        // 1. Remove normal velocity (perfectly inelastic contact)
        let v_dot_n = state.vel_x[i] * nx + state.vel_y[i] * ny + state.vel_z[i] * nz;
        state.vel_x[i] -= v_dot_n * nx;
        state.vel_y[i] -= v_dot_n * ny;
        state.vel_z[i] -= v_dot_n * nz;

        // 2. Coulomb friction on tangential velocity
        let v_n_mag = v_dot_n.abs();
        let vt_x = state.vel_x[i];
        let vt_y = state.vel_y[i];
        let vt_z = state.vel_z[i];
        let vt_mag = (vt_x * vt_x + vt_y * vt_y + vt_z * vt_z).sqrt();

        if vt_mag > 1e-8 {
            let friction_impulse = mu_friction * v_n_mag;
            if friction_impulse >= vt_mag {
                state.vel_x[i] = 0.0;
                state.vel_y[i] = 0.0;
                state.vel_z[i] = 0.0;
            } else {
                let scale = 1.0 - friction_impulse / vt_mag;
                state.vel_x[i] = vt_x * scale;
                state.vel_y[i] = vt_y * scale;
                state.vel_z[i] = vt_z * scale;
            }
        }

        // 3. Contact-specific damping
        if config.adaptive_contact_damping {
            let base_damp = contact_damp;
            let max_damp = config.contact_damping_max;
            let v_thresh = config.contact_velocity_threshold;
            let v_sq = state.vel_x[i] * state.vel_x[i] + state.vel_y[i] * state.vel_y[i] + state.vel_z[i] * state.vel_z[i];
            let v_mag = v_sq.sqrt();
            let t = (1.0 - v_mag / v_thresh.max(1e-6)).clamp(0.0, 1.0);
            let adaptive_damp = base_damp + t * (max_damp - base_damp);
            state.vel_x[i] *= 1.0 - adaptive_damp;
            state.vel_y[i] *= 1.0 - adaptive_damp;
            state.vel_z[i] *= 1.0 - adaptive_damp;
        } else {
            state.vel_x[i] *= 1.0 - contact_damp;
            state.vel_y[i] *= 1.0 - contact_damp;
            state.vel_z[i] *= 1.0 - contact_damp;
        }
    }

    // 4. Global contact damping (only when contacts are active)
    if active_contacts > 0 {
        state.damp_velocities(0.05);
    }

    // 5. Ground velocity enforcement + global damping
    state.enforce_ground_velocities();
    state.damp_velocities(config.damping);

    // 6. Rayleigh mass-proportional damping
    if config.rayleigh_mass_damping > 0.0 {
        let factor = 1.0 / (1.0 + config.rayleigh_mass_damping * sub_dt);
        for i in 0..n {
            if state.inv_mass[i] > 0.0 {
                state.vel_x[i] *= factor;
                state.vel_y[i] *= factor;
                state.vel_z[i] *= factor;
            }
        }
    }
}
