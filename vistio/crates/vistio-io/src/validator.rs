//! Mesh and input validation.
//!
//! Validates simulation inputs before the solver receives them,
//! catching data-level errors early with clear diagnostics.

use vistio_types::{VistioError, VistioResult};

use crate::contract::SimulationInput;

/// Validates a complete simulation input.
///
/// Checks:
/// - Garment mesh integrity (SoA consistency, valid indices)
/// - Body mesh integrity (if present)
/// - Pinning array length matches garment
/// - Simulation parameters are physically reasonable
pub fn validate_input(input: &SimulationInput) -> VistioResult<()> {
    // Validate garment mesh
    input.garment.validate().map_err(|e| {
        VistioError::InvalidMesh(format!("Garment mesh: {}", e))
    })?;

    // Validate body mesh if present
    if let Some(ref body) = input.body {
        body.validate().map_err(|e| {
            VistioError::InvalidMesh(format!("Body mesh: {}", e))
        })?;
    }

    // Check pinning array
    if input.pinned.len() != input.garment.vertex_count() {
        return Err(VistioError::InvalidMesh(format!(
            "Pinned array length ({}) != garment vertex count ({})",
            input.pinned.len(),
            input.garment.vertex_count()
        )));
    }

    // Validate parameters
    validate_params(&input.params)?;

    Ok(())
}

/// Validates simulation parameters.
fn validate_params(params: &crate::contract::SimulationParams) -> VistioResult<()> {
    if params.dt <= 0.0 {
        return Err(VistioError::InvalidConfig(
            "Timestep dt must be positive".into(),
        ));
    }
    if params.dt > 1.0 {
        return Err(VistioError::InvalidConfig(
            "Timestep dt > 1.0 is unreasonably large".into(),
        ));
    }
    if params.duration <= 0.0 {
        return Err(VistioError::InvalidConfig(
            "Duration must be positive".into(),
        ));
    }
    if params.iterations == 0 {
        return Err(VistioError::InvalidConfig(
            "Solver iterations must be >= 1".into(),
        ));
    }
    if params.gravity < 0.0 {
        return Err(VistioError::InvalidConfig(
            "Gravity magnitude must be non-negative".into(),
        ));
    }
    if params.contact_thickness < 0.0 {
        return Err(VistioError::InvalidConfig(
            "Contact thickness must be non-negative".into(),
        ));
    }

    // Gravity direction should be roughly unit length
    let gd = params.gravity_direction;
    let len = (gd[0] * gd[0] + gd[1] * gd[1] + gd[2] * gd[2]).sqrt();
    if (len - 1.0).abs() > 0.01 {
        return Err(VistioError::InvalidConfig(format!(
            "Gravity direction should be unit length, got magnitude {}",
            len
        )));
    }

    Ok(())
}

