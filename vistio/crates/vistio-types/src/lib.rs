//! # vistio-types
//!
//! Shared types, identifiers, error types, and physical constants
//! for the Vistio garment simulation engine.
//!
//! This crate has zero domain logic — it defines the vocabulary
//! that all other Vistio crates share.

pub mod constants;
pub mod error;
pub mod ids;
pub mod scalar;

pub use error::{VistioError, VistioResult};
pub use ids::{MaterialId, ParticleId, TriangleId};
pub use scalar::Scalar;
