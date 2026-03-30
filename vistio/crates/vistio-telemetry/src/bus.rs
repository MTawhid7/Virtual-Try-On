//! Event bus — broadcast-style event dispatch with pluggable sinks.
//!
//! The bus uses `std::sync::mpsc` for thread-safe, lock-free event delivery.
//! Sinks are registered once at initialization and receive events asynchronously.

use std::sync::mpsc;

use crate::events::SimulationEvent;
use crate::sinks::EventSink;

/// Broadcast event bus for simulation telemetry.
///
/// The producer side (`emit`) sends events to all registered sinks.
/// Each sink runs on the consumer side and processes events independently.
pub struct EventBus {
    /// Channel sender — cloned once per bus instance.
    sender: mpsc::Sender<SimulationEvent>,
    /// Channel receiver — owned by the bus for dispatching to sinks.
    receiver: mpsc::Receiver<SimulationEvent>,
    /// Registered sinks.
    sinks: Vec<Box<dyn EventSink>>,
    /// Whether the bus is active. Disabled bus is a no-op.
    enabled: bool,
}

impl EventBus {
    /// Creates a new event bus with no sinks.
    pub fn new() -> Self {
        let (sender, receiver) = mpsc::channel();
        Self {
            sender,
            receiver,
            sinks: Vec::new(),
            enabled: true,
        }
    }

    /// Registers a sink to receive events.
    pub fn add_sink(&mut self, sink: Box<dyn EventSink>) {
        self.sinks.push(sink);
    }

    /// Enables or disables the bus. Disabled bus drops events silently.
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }

    /// Returns true if the bus is active.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Emit an event. If the bus is disabled, this is a no-op.
    pub fn emit(&self, event: SimulationEvent) {
        if !self.enabled {
            return;
        }
        // Send to channel — ignore error if receiver is somehow dropped
        let _ = self.sender.send(event);
    }

    /// Flush all pending events to registered sinks.
    ///
    /// Call this at the end of each timestep or at shutdown
    /// to ensure all events are processed.
    pub fn flush(&mut self) {
        while let Ok(event) = self.receiver.try_recv() {
            for sink in &mut self.sinks {
                sink.handle(&event);
            }
        }
    }

    /// Returns the number of registered sinks.
    pub fn sink_count(&self) -> usize {
        self.sinks.len()
    }
}

impl Default for EventBus {
    fn default() -> Self {
        Self::new()
    }
}
