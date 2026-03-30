//! Solver strategy trait — the core abstraction for time integration.
//!
//! Every solver implements this trait, enabling the simulation pipeline
//! to swap between PD, implicit, or XPBD solvers at runtime.

use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_types::VistioResult;

use crate::config::SolverConfig;
use crate::state::SimulationState;

/// Result of a solver step.
#[derive(Debug, Clone)]
pub struct StepResult {
    /// Number of iterations actually performed.
    pub iterations: u32,
    /// Final residual norm.
    pub final_residual: f64,
    /// Whether the solver converged to tolerance.
    pub converged: bool,
    /// Wall-clock time for this step (seconds).
    pub wall_time: f64,
}

/// Trait for time integration solvers.
///
/// The simulation pipeline calls these methods in order:
///
/// ```text
/// solver.init(mesh, topology, config)?;
/// loop {
///     solver.step(state, dt)?;
/// }
/// ```
///
/// # Implementations
///
/// - [`ProjectiveDynamicsStub`](crate::pd_stub::ProjectiveDynamicsStub) — Tier-0 stub (no-op local step)
/// - `ProjectiveDynamics` — Full PD solver (Tier 1)
/// - `ImplicitEuler` — Newton-Raphson implicit solver (Tier 4)
pub trait SolverStrategy: Send {
    /// Initialize the solver with mesh topology and configuration.
    ///
    /// Called once (or after topology change, e.g. remeshing).
    /// The solver pre-computes rest-state data (edge lengths, element
    /// rest shapes, system matrix sparsity pattern).
    fn init(
        &mut self,
        mesh: &TriangleMesh,
        topology: &Topology,
        config: &SolverConfig,
        pinned: &[bool],
    ) -> VistioResult<()>;

    /// Advance the simulation by one timestep.
    ///
    /// Modifies `state.pos_{x,y,z}` in place. The caller is responsible
    /// for calling `state.predict()` before and `state.update_velocities()`
    /// after this method.
    fn step(
        &mut self,
        state: &mut SimulationState,
        dt: f32,
    ) -> VistioResult<StepResult>;

    /// Returns the solver's name.
    fn name(&self) -> &str;
}
