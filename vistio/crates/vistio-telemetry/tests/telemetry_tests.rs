//! Integration tests for vistio-telemetry.

use vistio_telemetry::bus::EventBus;
use vistio_telemetry::events::{EventKind, SimulationEvent};
use vistio_telemetry::sinks::VecSink;

#[test]
fn emit_and_flush() {
    let mut bus = EventBus::new();
    let sink = VecSink::new();
    bus.add_sink(Box::new(sink));

    bus.emit(SimulationEvent::new(0, EventKind::TimestepBegin { sim_time: 0.0 }));
    bus.emit(SimulationEvent::new(0, EventKind::TimestepEnd { wall_time: 0.001 }));

    bus.flush();
    // After flush, events should have been dispatched to the sink.
    // We can't inspect the sink directly because it's behind Box<dyn>,
    // but we verify no panics occurred.
}

#[test]
fn disabled_bus_drops_events() {
    let mut bus = EventBus::new();
    bus.set_enabled(false);
    bus.emit(SimulationEvent::new(0, EventKind::TimestepBegin { sim_time: 0.0 }));
    // Should not panic or accumulate
    bus.flush();
}

#[test]
fn multiple_sinks() {
    let mut bus = EventBus::new();
    bus.add_sink(Box::new(VecSink::new()));
    bus.add_sink(Box::new(VecSink::new()));
    assert_eq!(bus.sink_count(), 2);
}

#[test]
fn event_serialization() {
    let event = SimulationEvent::new(
        5,
        EventKind::Energy {
            kinetic: 1.0,
            potential: 2.0,
            elastic: 0.5,
        },
    );
    let json = serde_json::to_string(&event).unwrap();
    let recovered: SimulationEvent = serde_json::from_str(&json).unwrap();
    assert_eq!(recovered.timestep, 5);
}

#[test]
fn convergence_event() {
    let event = SimulationEvent::new(
        10,
        EventKind::Convergence {
            iterations: 15,
            final_residual: 1e-8,
            converged: true,
        },
    );
    let json = serde_json::to_string(&event).unwrap();
    assert!(json.contains("converged"));
}
