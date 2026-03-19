//! Physical constants and simulation defaults.

/// Gravitational acceleration (m/s²).
pub const GRAVITY: f32 = 9.81;

/// Default simulation timestep (seconds). 1/60th of a second.
pub const DEFAULT_DT: f32 = 1.0 / 60.0;

/// Default number of PD iterations per timestep.
pub const DEFAULT_PD_ITERATIONS: u32 = 20;

/// Default contact thickness (meters). Minimum separation between surfaces.
pub const DEFAULT_CONTACT_THICKNESS: f32 = 0.005;

/// Epsilon for floating-point comparisons.
pub const EPSILON: f32 = 1.0e-7;

/// Epsilon for degenerate triangle detection (area threshold).
pub const DEGENERATE_AREA_THRESHOLD: f32 = 1.0e-10;
