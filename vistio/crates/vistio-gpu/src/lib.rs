//! # vistio-gpu
//!
//! GPU compute abstraction layer for the Vistio simulation engine.
//!
//! Provides a [`GpuBackend`] trait with two implementations:
//! - [`CpuFallback`] — Reference CPU implementation (always available)
//! - `WgpuBackend` — GPU compute via wgpu (coming in Tier 1)
//!
//! The abstraction enables running the same solver code on GPU or CPU
//! without changing the simulation pipeline.

pub mod backend;
pub mod buffers;

pub use backend::{CpuFallback, GpuBackend};
pub use buffers::ComputeBuffer;
