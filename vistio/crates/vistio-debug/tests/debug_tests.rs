//! Integration tests for vistio-debug.

use vistio_debug::hooks::{InspectionHook, TelemetryHook};
use vistio_debug::snapshot::StateSnapshot;

// ─── Hook Tests ───────────────────────────────────────────────

#[test]
fn telemetry_hook_collects_events() {
    let mut hook = TelemetryHook::new();
    hook.on_timestep_begin(0, 0.0);
    hook.on_iteration(0, 0, 1e-2);
    hook.on_iteration(0, 1, 1e-5);
    hook.on_timestep_end(0, 0.001);

    let events = hook.drain_events();
    assert_eq!(events.len(), 4);
    assert_eq!(events[0].timestep, 0);
}

#[test]
fn hook_drain_clears() {
    let mut hook = TelemetryHook::new();
    hook.on_timestep_begin(0, 0.0);
    let _ = hook.drain_events();
    let events = hook.drain_events();
    assert!(events.is_empty());
}

#[test]
fn hook_name() {
    let hook = TelemetryHook::new();
    assert_eq!(hook.name(), "telemetry_hook");
}

// ─── Snapshot Tests ───────────────────────────────────────────

#[test]
fn snapshot_round_trip() {
    let pos_x = vec![1.0, 2.0, 3.0];
    let pos_y = vec![4.0, 5.0, 6.0];
    let pos_z = vec![7.0, 8.0, 9.0];
    let vel_x = vec![0.1, 0.2, 0.3];
    let vel_y = vec![0.4, 0.5, 0.6];
    let vel_z = vec![0.7, 0.8, 0.9];

    let snap = StateSnapshot::from_soa(42, 0.7, &pos_x, &pos_y, &pos_z, &vel_x, &vel_y, &vel_z);

    let bytes = snap.to_bytes();
    let recovered = StateSnapshot::from_bytes(&bytes).unwrap();

    assert_eq!(recovered.timestep, 42);
    assert_eq!(recovered.vertex_count, 3);
    assert!((recovered.sim_time - 0.7).abs() < 1e-10);
    assert_eq!(recovered.positions.len(), 9);
    assert_eq!(recovered.velocities.len(), 9);
}

#[test]
fn snapshot_interleaving() {
    let snap = StateSnapshot::from_soa(
        0, 0.0,
        &[1.0, 2.0], &[3.0, 4.0], &[5.0, 6.0],
        &[0.0; 2], &[0.0; 2], &[0.0; 2],
    );
    // Positions are interleaved: [x0, y0, z0, x1, y1, z1]
    assert_eq!(snap.positions, vec![1.0, 3.0, 5.0, 2.0, 4.0, 6.0]);
}
