

//! # vistio-solver
//!
//! Time integration, solver strategies, and simulation state management.
//!
//! ## Key Types
//!
//! - [`SimulationState`] — SoA buffers for positions, velocities, masses
//! - [`SolverStrategy`] — Pluggable solver trait (PD, implicit, XPBD)
//! - [`SolverConfig`] — Solver-specific configuration
//! - [`ProjectiveDynamicsStub`] — Tier-0 stub solver (no-op local step)
//! - [`element::ElementData`] — FEM element precomputation (Tier 1)

pub mod constraints;
pub mod config;
pub mod pd_solver;
pub mod pd_stub;
pub mod state;
pub mod strategy;

// Backward-compatible re-exports from the constraints subdirectory.
// All existing `crate::element`, `crate::assembly`, etc. paths continue to work.
pub use constraints::assembly;
pub use constraints::bending;
pub use constraints::discrete_shells;
pub use constraints::element;

pub use config::SolverConfig;
pub use pd_solver::ProjectiveDynamicsSolver;
pub use pd_stub::ProjectiveDynamicsStub;
pub use state::SimulationState;
pub use strategy::SolverStrategy;
