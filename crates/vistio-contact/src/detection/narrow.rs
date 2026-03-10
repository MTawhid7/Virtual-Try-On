//! Narrow phase collision detection trait.
//!
//! Refines broad phase candidates into exact contact data
//! (distance, normal, barycentric coordinates).

use vistio_solver::SimulationState;
use vistio_mesh::TriangleMesh;
use vistio_types::VistioResult;

use crate::broad::CandidatePair;
use crate::contact::ContactPair;

/// Trait for narrow phase collision detection.
///
/// Takes candidate pairs from the broad phase and performs
/// exact geometric proximity tests.
///
/// # Implementations
/// - `VertexTriangleTest` — Point-triangle distance (Tier 1)
/// - `EdgeEdgeTest` — Edge-edge CCD for tunneling prevention (Tier 2)
pub trait NarrowPhase: Send {
    /// Test each candidate pair for actual proximity/penetration.
    ///
    /// Returns only pairs with distance < `thickness`.
    fn detect(
        &self,
        candidates: &[CandidatePair],
        state: &SimulationState,
        mesh: &TriangleMesh,
        thickness: f32,
    ) -> VistioResult<Vec<ContactPair>>;

    /// Returns the narrow phase strategy name.
    fn name(&self) -> &str;
}

/// No-op narrow phase for benchmarks without collision.
pub struct NullNarrowPhase;

impl NarrowPhase for NullNarrowPhase {
    fn detect(
        &self,
        _candidates: &[CandidatePair],
        _state: &SimulationState,
        _mesh: &TriangleMesh,
        _thickness: f32,
    ) -> VistioResult<Vec<ContactPair>> {
        Ok(Vec::new())
    }

    fn name(&self) -> &str {
        "null_narrow_phase"
    }
}
