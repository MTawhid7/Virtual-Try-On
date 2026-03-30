//! Contact response trait.
//!
//! Applies position corrections or barrier forces to resolve
//! detected contacts, preventing cloth penetration.

use vistio_solver::SimulationState;
use vistio_types::VistioResult;

use crate::contact::ContactPair;

/// Result of contact response.
#[derive(Debug, Clone, Default)]
pub struct ContactResult {
    /// Number of contacts resolved.
    pub resolved_count: u32,
    /// Maximum penetration depth remaining after resolution.
    pub max_residual_penetration: f32,
    /// Total contact force magnitude applied.
    pub total_force_magnitude: f32,
}

/// Trait for contact response strategies.
///
/// # Implementations
/// - `ProjectionResponse` — Direct position projection (Tier 1)
/// - `IpcBarrier` — Incremental Potential Contact barriers (Tier 3)
pub trait ContactResponse: Send {
    /// Resolve detected contacts by modifying vertex positions.
    ///
    /// Called after each solver iteration or at the end of the timestep.
    fn resolve(
        &self,
        contacts: &[ContactPair],
        state: &mut SimulationState,
        stiffness: f32,
    ) -> VistioResult<ContactResult>;

    /// Returns the response strategy name.
    fn name(&self) -> &str;
}

/// No-op contact response for collision-free scenarios.
pub struct NullContactResponse;

impl ContactResponse for NullContactResponse {
    fn resolve(
        &self,
        _contacts: &[ContactPair],
        _state: &mut SimulationState,
        _stiffness: f32,
    ) -> VistioResult<ContactResult> {
        Ok(ContactResult::default())
    }

    fn name(&self) -> &str {
        "null_contact_response"
    }
}
