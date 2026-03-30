//! Position-projection contact response.
//!
//! The simplest contact resolution strategy: directly project penetrating
//! vertices to the contact surface along the contact normal.
//! Suitable for Tier 1 where robustness is prioritized over accuracy.

use vistio_solver::SimulationState;
use vistio_types::VistioResult;

use crate::contact::ContactPair;
use crate::response::{ContactResponse, ContactResult};

/// Position-projection contact response.
///
/// For each penetrating contact, moves the vertex along the contact normal
/// by the penetration depth, scaled by a stiffness factor.
pub struct ProjectionContactResponse;

impl ContactResponse for ProjectionContactResponse {
    fn resolve(
        &self,
        contacts: &[ContactPair],
        state: &mut SimulationState,
        stiffness: f32,
    ) -> VistioResult<ContactResult> {
        let mut resolved = 0_u32;
        let mut max_pen = 0.0_f32;
        let mut total_force = 0.0_f32;

        for contact in contacts {
            let pen = contact.penetration_depth();
            if pen <= 0.0 {
                continue;
            }

            max_pen = max_pen.max(pen);

            // Move vertex along contact normal by stiffness * penetration
            let correction = pen * stiffness;
            let v = contact.indices[0] as usize;

            if v < state.vertex_count && state.inv_mass[v] > 0.0 {
                let dx = contact.normal[0] * correction;
                let dy = contact.normal[1] * correction;
                let dz = contact.normal[2] * correction;

                state.pos_x[v] += dx;
                state.pos_y[v] += dy;
                state.pos_z[v] += dz;

                // Inelastic collision impulses:
                // Isolate the velocity along the collision normal
                let vn = state.vel_x[v] * contact.normal[0]
                       + state.vel_y[v] * contact.normal[1]
                       + state.vel_z[v] * contact.normal[2];

                // If moving into the plane (vn < 0), zero out that approach velocity
                if vn < 0.0 {
                    state.vel_x[v] -= vn * contact.normal[0];
                    state.vel_y[v] -= vn * contact.normal[1];
                    state.vel_z[v] -= vn * contact.normal[2];
                }

                total_force += correction;
                resolved += 1;
            }
        }

        Ok(ContactResult {
            resolved_count: resolved,
            max_residual_penetration: max_pen,
            total_force_magnitude: total_force,
        })
    }

    fn name(&self) -> &str {
        "projection_response"
    }
}
