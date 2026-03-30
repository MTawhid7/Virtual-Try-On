//! Simulation input/output contract types.
//!
//! These types define the I/O boundary of the Vistio simulation engine.
//! They are serializable for API transport and CLI configuration.

use serde::{Deserialize, Serialize};
use vistio_mesh::TriangleMesh;

/// Complete input specification for a simulation run.
///
/// Contains all the data needed to set up and execute a garment simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationInput {
    /// The garment mesh to simulate.
    pub garment: TriangleMesh,

    /// The body/obstacle mesh (static or animated).
    /// `None` for free-falling cloth benchmarks.
    pub body: Option<TriangleMesh>,

    /// Per-vertex pinning constraints.
    /// `true` = pinned (infinite mass), `false` = free.
    /// Length must equal `garment.vertex_count()`.
    pub pinned: Vec<bool>,

    /// Simulation parameters.
    pub params: SimulationParams,
}

/// Physics and solver parameters for a simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationParams {
    /// Total simulation time in seconds.
    pub duration: f32,
    /// Timestep in seconds (e.g., 1/60).
    pub dt: f32,
    /// Number of solver iterations per timestep.
    pub iterations: u32,
    /// Gravitational acceleration (m/s², typically 9.81).
    pub gravity: f32,
    /// Gravity direction (unit vector, typically [0, -1, 0]).
    pub gravity_direction: [f32; 3],
    /// Contact thickness / minimum separation (meters).
    pub contact_thickness: f32,
    /// Material identifier for the garment.
    pub material_name: String,
}

impl Default for SimulationParams {
    fn default() -> Self {
        Self {
            duration: 2.0,
            dt: vistio_types::constants::DEFAULT_DT,
            iterations: vistio_types::constants::DEFAULT_PD_ITERATIONS,
            gravity: vistio_types::constants::GRAVITY,
            gravity_direction: [0.0, -1.0, 0.0],
            contact_thickness: vistio_types::constants::DEFAULT_CONTACT_THICKNESS,
            material_name: "cotton_twill".to_string(),
        }
    }
}

/// Output from a completed simulation run.
///
/// Contains the final mesh state and diagnostic metrics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimulationOutput {
    /// The draped garment mesh in its final state.
    pub draped_mesh: TriangleMesh,

    /// Per-vertex metadata.
    pub vertex_metadata: VertexMetadata,

    /// Simulation-wide metrics.
    pub metrics: SimulationMetrics,
}

/// Per-vertex diagnostic data from the simulation.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VertexMetadata {
    /// Per-triangle maximum principal strain (stretch ratio - 1.0).
    /// 0.0 = no deformation, positive = stretched, negative = compressed.
    pub strain: Vec<f32>,

    /// Per-vertex contact force magnitude. 0.0 = no contact.
    pub contact_force: Vec<f32>,
}

/// Aggregate metrics from a simulation run.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct SimulationMetrics {
    /// Total simulation wall-clock time (seconds).
    pub wall_time_seconds: f64,
    /// Number of timesteps executed.
    pub timestep_count: u32,
    /// Final total energy (kinetic + potential + elastic).
    pub final_energy: f64,
    /// Maximum penetration depth detected (meters). 0.0 = no penetration.
    pub max_penetration: f32,
    /// Maximum triangle strain at final frame.
    pub max_strain: f32,
    /// Average solver iterations per timestep.
    pub avg_iterations: f32,
}

