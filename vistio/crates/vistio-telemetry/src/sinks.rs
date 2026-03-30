//! Pluggable event sinks.
//!
//! Sinks consume events from the bus and process them
//! (log to file, send to Rerun, stream over WebSocket, etc.).

use crate::events::SimulationEvent;

/// Trait for event consumers.
///
/// Implement this to create custom telemetry outputs.
pub trait EventSink: Send {
    /// Process a single event.
    fn handle(&mut self, event: &SimulationEvent);

    /// Called when the simulation ends. Flush buffers, close files, etc.
    fn finalize(&mut self) {}

    /// Returns a human-readable name for this sink.
    fn name(&self) -> &str;
}

/// A simple sink that logs events to a `Vec` for testing and inspection.
pub struct VecSink {
    /// Collected events.
    pub events: Vec<SimulationEvent>,
}

impl VecSink {
    /// Creates an empty vec sink.
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }
}

impl Default for VecSink {
    fn default() -> Self {
        Self::new()
    }
}

impl EventSink for VecSink {
    fn handle(&mut self, event: &SimulationEvent) {
        self.events.push(event.clone());
    }

    fn name(&self) -> &str {
        "vec_sink"
    }
}

/// A sink that logs events using the `tracing` crate.
pub struct TracingSink {
    /// Minimum log level for events.
    _level: tracing::Level,
}

impl TracingSink {
    /// Creates a new tracing sink at the given log level.
    pub fn new(level: tracing::Level) -> Self {
        Self { _level: level }
    }
}

impl EventSink for TracingSink {
    fn handle(&mut self, event: &SimulationEvent) {
        tracing::info!(
            timestep = event.timestep,
            event = ?event.kind,
            "simulation_event"
        );
    }

    fn name(&self) -> &str {
        "tracing_sink"
    }
}
