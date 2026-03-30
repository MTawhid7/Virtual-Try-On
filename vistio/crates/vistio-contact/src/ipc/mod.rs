//! IPC (Incremental Potential Contact) — barriers, CCD, and distance primitives.

pub mod barrier;
pub mod ipc_response;
pub mod ccd;
pub mod distance_primitives;

pub use ipc_response::IpcContactSet;
