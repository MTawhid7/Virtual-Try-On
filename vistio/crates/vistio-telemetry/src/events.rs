//! Simulation event types.
//!
//! Structured events emitted by the simulation engine at various points
//! in each timestep. Events are lightweight value types that carry
//! just enough data to be useful for monitoring and debugging.

use serde::{Deserialize, Serialize};

/// A simulation event emitted by the engine.
///
/// Events are tagged with a timestep index and carry domain-specific data.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationEvent {
    /// Timestep number (0-indexed).
    pub timestep: u32,
    /// Event payload.
    pub kind: EventKind,
}

/// Event payload variants.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum EventKind {
    /// Timestep started.
    TimestepBegin {
        /// Target simulation time for this step (seconds).
        sim_time: f64,
    },

    /// Timestep completed.
    TimestepEnd {
        /// Wall-clock time for the entire timestep (seconds).
        wall_time: f64,
    },

    /// Solver iteration completed.
    SolverIteration {
        /// Iteration number within the timestep.
        iteration: u32,
        /// Residual norm (convergence metric).
        residual: f64,
    },

    /// Contact detection completed.
    ContactDetection {
        /// Number of active contact pairs.
        contact_count: u32,
        /// Maximum penetration depth (meters).
        max_penetration: f32,
    },

    /// Energy snapshot at current state.
    Energy {
        /// Kinetic energy (0.5 * m * v^2).
        kinetic: f64,
        /// Potential energy (m * g * h).
        potential: f64,
        /// Elastic strain energy.
        elastic: f64,
    },

    /// Solver convergence report for the timestep.
    Convergence {
        /// Total iterations used.
        iterations: u32,
        /// Final residual.
        final_residual: f64,
        /// Whether the solver converged within tolerance.
        converged: bool,
    },

    /// Custom event for extensibility.
    Custom {
        /// Arbitrary label.
        label: String,
        /// JSON-encoded payload.
        payload: String,
    },
}

impl SimulationEvent {
    /// Creates a new event for the given timestep.
    pub fn new(timestep: u32, kind: EventKind) -> Self {
        Self { timestep, kind }
    }
}
