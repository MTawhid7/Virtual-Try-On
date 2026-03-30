// physics/src/engine/state.rs
use glam::{Vec4, Vec2}; // Removed Vec3

/// The core data container for the particle system.
/// Uses Structure-of-Arrays (SoA) layout for better cache locality and SIMD alignment.
/// All vectors are 16-byte aligned (Vec4) to support WebAssembly SIMD implementation.
#[derive(Clone)]
pub struct PhysicsState {
    pub count: usize,

    // --- Particle Data (Aligned to 16 bytes / Vec4) ---
    /// Current position (xyz) + Padding (w)
    pub positions: Vec<Vec4>,
    /// Previous position (xyz) + Padding (w) - for Verlet integration.
    pub prev_positions: Vec<Vec4>,
    /// Calculated velocity (for damping/aerodynamics).
    pub normals: Vec<Vec4>,

    pub inv_mass: Vec<f32>,
    pub uvs: Vec<Vec2>,

    // --- Topology ---
    /// Triangle indices (3 per triangle).
    pub indices: Vec<u32>,
}

impl PhysicsState {
    pub fn new(
        raw_positions: &[f32],
        raw_indices: &[u32],
        raw_uvs: &[f32]
    ) -> Self {
        let count = raw_positions.len() / 3;

        let mut positions = Vec::with_capacity(count);
        let mut prev_positions = Vec::with_capacity(count);
        let normals = vec![Vec4::Y; count];
        let mut uvs = Vec::with_capacity(count);

        for i in 0..count {
            let p = Vec4::new(
                raw_positions[i * 3],
                raw_positions[i * 3 + 1],
                raw_positions[i * 3 + 2],
                0.0
            );
            positions.push(p);
            prev_positions.push(p);
        }

        for i in 0..count {
            if i * 2 + 1 < raw_uvs.len() {
                uvs.push(Vec2::new(raw_uvs[i * 2], raw_uvs[i * 2 + 1]));
            } else {
                uvs.push(Vec2::ZERO);
            }
        }

        let inv_mass = vec![1.0; count];

        PhysicsState {
            count,
            positions,
            prev_positions,
            inv_mass,
            normals,
            uvs,
            indices: raw_indices.to_vec(),
        }
    }
}