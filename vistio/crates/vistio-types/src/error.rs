//! Error types for the Vistio engine.
//!
//! All crates return `VistioResult<T>` from fallible operations.

use thiserror::Error;

/// Unified error type for the Vistio engine.
#[derive(Debug, Error)]
pub enum VistioError {
    /// Mesh data is malformed or inconsistent.
    #[error("Invalid mesh: {0}")]
    InvalidMesh(String),

    /// Material parameter is out of valid range.
    #[error("Invalid material parameter: {0}")]
    InvalidMaterial(String),

    /// Configuration value is invalid.
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),

    /// I/O operation failed.
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// Serialization/deserialization failure.
    #[error("Serialization error: {0}")]
    Serialization(String),

    /// GPU backend error.
    #[error("GPU error: {0}")]
    Gpu(String),

    /// Solver failed to converge.
    #[error("Solver did not converge after {iterations} iterations (residual: {residual:.2e})")]
    SolverDivergence {
        iterations: u32,
        residual: f64,
    },

    /// A simulation invariant was violated (e.g., penetration detected).
    #[error("Invariant violation: {0}")]
    InvariantViolation(String),
}

/// Convenience alias for `Result<T, VistioError>`.
pub type VistioResult<T> = Result<T, VistioError>;
