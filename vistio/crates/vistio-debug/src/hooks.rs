//! Inspection hooks for live debugging.
//!
//! Hooks are injected into the solver loop and called at specific points
//! (pre-solve, post-iteration, post-solve) to capture or modify state.

use vistio_telemetry::events::SimulationEvent;

/// Trait for simulation inspection hooks.
///
/// Implement this to inject debugging/monitoring logic into the solver
/// loop without modifying the solver itself. Hooks are called at defined
/// points and can capture metrics, snapshots, or even modify state
/// (for debugging only — production hooks should be read-only).
///
/// # Lifecycle
///
/// ```text
/// for each timestep:
///   hook.on_timestep_begin(...)
///   for each solver iteration:
///     hook.on_iteration(...)
///   hook.on_timestep_end(...)
/// hook.on_simulation_end()
/// ```
pub trait InspectionHook: Send {
    /// Called at the beginning of each timestep.
    fn on_timestep_begin(&mut self, timestep: u32, sim_time: f64) {
        let _ = (timestep, sim_time);
    }

    /// Called after each solver iteration.
    fn on_iteration(&mut self, timestep: u32, iteration: u32, residual: f64) {
        let _ = (timestep, iteration, residual);
    }

    /// Called at the end of each timestep.
    fn on_timestep_end(&mut self, timestep: u32, wall_time: f64) {
        let _ = (timestep, wall_time);
    }

    /// Called when the simulation completes.
    fn on_simulation_end(&mut self) {}

    /// Returns the hook's name for logging.
    fn name(&self) -> &str;
}

/// Hook that bridges to the telemetry event bus.
///
/// Translates solver lifecycle calls into telemetry events
/// and emits them through the provided bus sender.
pub struct TelemetryHook {
    events: Vec<SimulationEvent>,
}

impl TelemetryHook {
    /// Creates a new telemetry hook.
    pub fn new() -> Self {
        Self { events: Vec::new() }
    }

    /// Drains collected events for dispatch.
    pub fn drain_events(&mut self) -> Vec<SimulationEvent> {
        std::mem::take(&mut self.events)
    }
}

impl Default for TelemetryHook {
    fn default() -> Self {
        Self::new()
    }
}

impl InspectionHook for TelemetryHook {
    fn on_timestep_begin(&mut self, timestep: u32, sim_time: f64) {
        use vistio_telemetry::events::EventKind;
        self.events.push(SimulationEvent::new(
            timestep,
            EventKind::TimestepBegin { sim_time },
        ));
    }

    fn on_iteration(&mut self, timestep: u32, iteration: u32, residual: f64) {
        use vistio_telemetry::events::EventKind;
        self.events.push(SimulationEvent::new(
            timestep,
            EventKind::SolverIteration { iteration, residual },
        ));
    }

    fn on_timestep_end(&mut self, timestep: u32, wall_time: f64) {
        use vistio_telemetry::events::EventKind;
        self.events.push(SimulationEvent::new(
            timestep,
            EventKind::TimestepEnd { wall_time },
        ));
    }

    fn name(&self) -> &str {
        "telemetry_hook"
    }
}
