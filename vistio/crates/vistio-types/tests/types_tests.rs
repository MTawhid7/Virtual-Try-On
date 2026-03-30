//! Integration tests for vistio-types.

use vistio_types::{ParticleId, TriangleId, MaterialId, VistioError};

// ─── ID Tests ──────────────────────────────────────────────────

#[test]
fn particle_id_index() {
    let id = ParticleId(42);
    assert_eq!(id.index(), 42);
}

#[test]
fn triangle_id_index() {
    let id = TriangleId(7);
    assert_eq!(id.index(), 7);
}

#[test]
fn ids_are_not_interchangeable() {
    // Compile-time guarantee — these types are distinct.
    let _p = ParticleId(0);
    let _t = TriangleId(0);
    let _m = MaterialId(0);
}

#[test]
fn ids_are_serializable() {
    let id = ParticleId(100);
    let json = serde_json::to_string(&id).unwrap();
    let deserialized: ParticleId = serde_json::from_str(&json).unwrap();
    assert_eq!(id, deserialized);
}

// ─── Error Tests ──────────────────────────────────────────────

#[test]
fn error_display() {
    let err = VistioError::InvalidMesh("non-manifold edge at index 42".into());
    assert!(err.to_string().contains("non-manifold edge"));
}

#[test]
fn solver_divergence_display() {
    let err = VistioError::SolverDivergence {
        iterations: 100,
        residual: 1.5e-2,
    };
    let msg = err.to_string();
    assert!(msg.contains("100"));
    assert!(msg.contains("1.50e-2") || msg.contains("1.5e-2"));
}
