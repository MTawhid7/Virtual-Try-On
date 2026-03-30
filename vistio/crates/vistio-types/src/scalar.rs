//! Scalar type alias for the simulation.
//!
//! Using `f32` for GPU compatibility (most GPU compute shaders operate on f32).
//! This alias makes it easy to experiment with `f64` precision if needed.

/// The floating-point type used throughout the simulation.
///
/// Set to `f32` for GPU compatibility. Change to `f64` for
/// double-precision CPU-only mode (useful for validation).
pub type Scalar = f32;
