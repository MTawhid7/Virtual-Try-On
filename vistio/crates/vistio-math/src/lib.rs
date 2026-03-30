//! # vistio-math
//!
//! Linear algebra primitives for the Vistio simulation engine.
//!
//! Provides:
//! - Re-exports of `glam` types (`Vec3`, `Mat3`, etc.)
//! - 3×2 matrix type for deformation gradients
//! - Polar decomposition (SVD-based, robust for degenerate triangles)
//! - Sparse matrix representation (CSR) and Cholesky solver interface

pub mod decomposition;
pub mod faer_solver;
pub mod mat3x2;
pub mod sparse;

// Re-export glam types as the canonical math types for Vistio.
pub use glam::{Mat3, Mat4, Quat, Vec2, Vec3, Vec4};
