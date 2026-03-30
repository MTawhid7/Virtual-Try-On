//! Integration tests for vistio-io.

use vistio_io::contract::{SimulationInput, SimulationParams};
use vistio_io::validator::validate_input;
use vistio_mesh::generators::{quad_grid, uv_sphere};

// ─── Contract Tests ───────────────────────────────────────────

#[test]
fn default_params() {
    let params = SimulationParams::default();
    assert!((params.gravity - 9.81).abs() < 1e-3);
    assert_eq!(params.iterations, vistio_types::constants::DEFAULT_PD_ITERATIONS);
}

#[test]
fn simulation_input_round_trip() {
    let mesh = quad_grid(2, 2, 1.0, 1.0);
    let n = mesh.vertex_count();
    let input = SimulationInput {
        garment: mesh,
        body: None,
        pinned: vec![false; n],
        params: SimulationParams::default(),
    };
    let json = serde_json::to_string(&input).unwrap();
    let recovered: SimulationInput = serde_json::from_str(&json).unwrap();
    assert_eq!(recovered.garment.vertex_count(), 9);
}

// ─── Validator Tests ──────────────────────────────────────────

fn make_valid_input() -> SimulationInput {
    let mesh = quad_grid(4, 4, 1.0, 1.0);
    let n = mesh.vertex_count();
    SimulationInput {
        garment: mesh,
        body: None,
        pinned: vec![false; n],
        params: SimulationParams::default(),
    }
}

#[test]
fn valid_input_passes() {
    assert!(validate_input(&make_valid_input()).is_ok());
}

#[test]
fn wrong_pinned_length() {
    let mut input = make_valid_input();
    input.pinned = vec![false; 3];
    assert!(validate_input(&input).is_err());
}

#[test]
fn negative_dt_rejected() {
    let mut input = make_valid_input();
    input.params.dt = -0.01;
    assert!(validate_input(&input).is_err());
}

#[test]
fn zero_iterations_rejected() {
    let mut input = make_valid_input();
    input.params.iterations = 0;
    assert!(validate_input(&input).is_err());
}

#[test]
fn non_unit_gravity_direction_rejected() {
    let mut input = make_valid_input();
    input.params.gravity_direction = [10.0, 0.0, 0.0];
    assert!(validate_input(&input).is_err());
}

#[test]
fn with_body_mesh() {
    let mut input = make_valid_input();
    input.body = Some(uv_sphere(0.5, 8, 16));
    assert!(validate_input(&input).is_ok());
}
