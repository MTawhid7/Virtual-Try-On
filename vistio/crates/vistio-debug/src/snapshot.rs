//! State snapshot serialization for replay and debugging.
//!
//! Snapshots capture the full simulation state at a point in time,
//! enabling deterministic replay and diff-based debugging.

use serde::{Deserialize, Serialize};

/// A complete simulation state snapshot.
///
/// Serialized with `bincode` for compact binary output.
/// Contains all data needed to resume or replay the simulation
/// from this point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StateSnapshot {
    /// Timestep index when this snapshot was taken.
    pub timestep: u32,
    /// Simulation time in seconds.
    pub sim_time: f64,
    /// Vertex positions (flat: [x0, y0, z0, x1, y1, z1, ...]).
    pub positions: Vec<f32>,
    /// Vertex velocities (flat: [vx0, vy0, vz0, ...]).
    pub velocities: Vec<f32>,
    /// Number of vertices.
    pub vertex_count: usize,
}

impl StateSnapshot {
    /// Creates a snapshot from SoA position and velocity buffers.
    #[allow(clippy::too_many_arguments)]
    pub fn from_soa(
        timestep: u32,
        sim_time: f64,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        vel_x: &[f32],
        vel_y: &[f32],
        vel_z: &[f32],
    ) -> Self {
        let n = pos_x.len();
        let mut positions = Vec::with_capacity(n * 3);
        let mut velocities = Vec::with_capacity(n * 3);

        for i in 0..n {
            positions.push(pos_x[i]);
            positions.push(pos_y[i]);
            positions.push(pos_z[i]);
            velocities.push(vel_x[i]);
            velocities.push(vel_y[i]);
            velocities.push(vel_z[i]);
        }

        Self {
            timestep,
            sim_time,
            positions,
            velocities,
            vertex_count: n,
        }
    }

    /// Serializes to compact binary format.
    pub fn to_bytes(&self) -> Vec<u8> {
        bincode::serialize(self).expect("Snapshot serialization should not fail")
    }

    /// Deserializes from binary format.
    pub fn from_bytes(data: &[u8]) -> Result<Self, String> {
        bincode::deserialize(data).map_err(|e| format!("Snapshot deserialization failed: {}", e))
    }
}
