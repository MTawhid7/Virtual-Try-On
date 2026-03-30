// physics/src/utils/simd/mod.rs

//! SIMD utilities for vectorized physics computations.
//! Uses WASM SIMD128 intrinsics for 4-wide parallel operations.
//!
//! This module is conditionally compiled - when targeting WASM with SIMD,
//! it uses real SIMD intrinsics. Otherwise, it falls back to scalar emulation.
//!
//! NOTE: Some helper methods (min, lane, select, etc.) are currently unused but
//! are retained as a complete API for future optimizations (e.g., SoA layouts,
//! temporal coherence, more advanced SIMD patterns).

mod f32x4;
mod vec3x4;

pub use f32x4::F32x4;
pub use vec3x4::Vec3x4;
