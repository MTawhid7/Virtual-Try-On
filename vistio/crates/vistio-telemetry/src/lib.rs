//! # vistio-telemetry
//!
//! Event bus for simulation telemetry. Emits structured events
//! (timing, energy, contacts, convergence) that can be consumed
//! by pluggable sinks (log files, Rerun, WebSocket, etc.).

pub mod bus;
pub mod events;
pub mod sinks;

pub use bus::EventBus;
pub use events::SimulationEvent;
