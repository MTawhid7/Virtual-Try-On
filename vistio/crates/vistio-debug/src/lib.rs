//! # vistio-debug
//!
//! Inspection hooks and state snapshots for debugging simulation issues.
//! Supports serializing full simulation state to binary for replay,
//! and pluggable visualization sinks (Rerun, file export).

pub mod hooks;
pub mod snapshot;
