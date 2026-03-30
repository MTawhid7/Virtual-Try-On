//! Core triangle mesh type with SoA (Structure of Arrays) layout.
//!
//! The SoA layout stores each coordinate channel contiguously:
//! - `pos_x: [x0, x1, x2, ...]`
//! - `pos_y: [y0, y1, y2, ...]`
//! - `pos_z: [z0, z1, z2, ...]`
//!
//! This gives optimal GPU memory coalescing when compute shaders
//! iterate over all vertices in parallel.

use serde::{Deserialize, Serialize};
use vistio_types::{MaterialId, VistioError, VistioResult};

/// A triangle mesh stored in Structure-of-Arrays layout.
///
/// All position, normal, and UV data is stored in separate per-channel
/// contiguous arrays for GPU coalescing. Triangle indices reference
/// into these arrays.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TriangleMesh {
    // --- Vertex data (SoA) ---
    /// X coordinates of all vertices.
    pub pos_x: Vec<f32>,
    /// Y coordinates of all vertices.
    pub pos_y: Vec<f32>,
    /// Z coordinates of all vertices.
    pub pos_z: Vec<f32>,

    /// X components of vertex normals.
    pub normal_x: Vec<f32>,
    /// Y components of vertex normals.
    pub normal_y: Vec<f32>,
    /// Z components of vertex normals.
    pub normal_z: Vec<f32>,

    /// U texture coordinates.
    pub uv_u: Vec<f32>,
    /// V texture coordinates.
    pub uv_v: Vec<f32>,

    // --- Triangle data ---
    /// Triangle indices — each triangle is [v0, v1, v2].
    /// Stored flat: `[t0v0, t0v1, t0v2, t1v0, t1v1, t1v2, ...]`
    pub indices: Vec<u32>,

    /// Per-triangle material assignment.
    pub material_ids: Vec<MaterialId>,
}

impl TriangleMesh {
    /// Returns the number of vertices.
    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.pos_x.len()
    }

    /// Returns the number of triangles.
    #[inline]
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// Returns the position of vertex `i` as `[x, y, z]`.
    #[inline]
    pub fn position(&self, i: usize) -> [f32; 3] {
        [self.pos_x[i], self.pos_y[i], self.pos_z[i]]
    }

    /// Returns the position as a `glam::Vec3`.
    #[inline]
    pub fn position_vec3(&self, i: usize) -> vistio_math::Vec3 {
        vistio_math::Vec3::new(self.pos_x[i], self.pos_y[i], self.pos_z[i])
    }

    /// Returns the normal of vertex `i` as a `glam::Vec3`.
    #[inline]
    pub fn normal_vec3(&self, i: usize) -> vistio_math::Vec3 {
        vistio_math::Vec3::new(self.normal_x[i], self.normal_y[i], self.normal_z[i])
    }

    /// Returns the three vertex indices of triangle `t`.
    #[inline]
    pub fn triangle(&self, t: usize) -> [u32; 3] {
        let base = t * 3;
        [self.indices[base], self.indices[base + 1], self.indices[base + 2]]
    }

    /// Sets the position of vertex `i`.
    #[inline]
    pub fn set_position(&mut self, i: usize, x: f32, y: f32, z: f32) {
        self.pos_x[i] = x;
        self.pos_y[i] = y;
        self.pos_z[i] = z;
    }

    /// Creates an empty mesh with pre-allocated capacity.
    pub fn with_capacity(vertex_capacity: usize, triangle_capacity: usize) -> Self {
        Self {
            pos_x: Vec::with_capacity(vertex_capacity),
            pos_y: Vec::with_capacity(vertex_capacity),
            pos_z: Vec::with_capacity(vertex_capacity),
            normal_x: Vec::with_capacity(vertex_capacity),
            normal_y: Vec::with_capacity(vertex_capacity),
            normal_z: Vec::with_capacity(vertex_capacity),
            uv_u: Vec::with_capacity(vertex_capacity),
            uv_v: Vec::with_capacity(vertex_capacity),
            indices: Vec::with_capacity(triangle_capacity * 3),
            material_ids: Vec::with_capacity(triangle_capacity),
        }
    }

    /// Validates mesh integrity.
    ///
    /// Checks:
    /// - All SoA arrays have the same length
    /// - Triangle indices are within bounds
    /// - No degenerate triangles (repeated vertex indices)
    pub fn validate(&self) -> VistioResult<()> {
        let n = self.pos_x.len();

        // Check SoA consistency
        if self.pos_y.len() != n || self.pos_z.len() != n {
            return Err(VistioError::InvalidMesh(
                "Position arrays have inconsistent lengths".into(),
            ));
        }
        if self.normal_x.len() != n || self.normal_y.len() != n || self.normal_z.len() != n {
            return Err(VistioError::InvalidMesh(
                "Normal arrays have inconsistent lengths".into(),
            ));
        }
        if self.uv_u.len() != n || self.uv_v.len() != n {
            return Err(VistioError::InvalidMesh(
                "UV arrays have inconsistent lengths".into(),
            ));
        }

        // Check indices
        if self.indices.len() % 3 != 0 {
            return Err(VistioError::InvalidMesh(
                "Index count is not divisible by 3".into(),
            ));
        }

        let tri_count = self.triangle_count();
        if self.material_ids.len() != tri_count {
            return Err(VistioError::InvalidMesh(format!(
                "Material IDs count ({}) != triangle count ({})",
                self.material_ids.len(),
                tri_count
            )));
        }

        for (i, &idx) in self.indices.iter().enumerate() {
            if idx as usize >= n {
                return Err(VistioError::InvalidMesh(format!(
                    "Index {} at position {} is out of range (vertex count: {})",
                    idx, i, n
                )));
            }
        }

        // Check for degenerate triangles
        for t in 0..tri_count {
            let [a, b, c] = self.triangle(t);
            if a == b || b == c || a == c {
                return Err(VistioError::InvalidMesh(format!(
                    "Triangle {} has repeated vertex indices: [{}, {}, {}]",
                    t, a, b, c
                )));
            }
        }

        Ok(())
    }

    /// Constructs a mesh from interleaved AoS position data.
    ///
    /// Converts from the V1 format `[x0, y0, z0, x1, y1, z1, ...]`
    /// to SoA layout. Useful for loading data from legacy formats.
    pub fn from_interleaved(
        positions: &[f32],
        indices: &[u32],
        uvs: &[f32],
    ) -> VistioResult<Self> {
        if positions.len() % 3 != 0 {
            return Err(VistioError::InvalidMesh(
                "Interleaved positions length not divisible by 3".into(),
            ));
        }

        let n = positions.len() / 3;
        let mut mesh = Self::with_capacity(n, indices.len() / 3);

        // Deinterleave positions
        for i in 0..n {
            mesh.pos_x.push(positions[i * 3]);
            mesh.pos_y.push(positions[i * 3 + 1]);
            mesh.pos_z.push(positions[i * 3 + 2]);
        }

        // Deinterleave UVs (or fill zeros)
        if uvs.len() == n * 2 {
            for i in 0..n {
                mesh.uv_u.push(uvs[i * 2]);
                mesh.uv_v.push(uvs[i * 2 + 1]);
            }
        } else {
            mesh.uv_u.resize(n, 0.0);
            mesh.uv_v.resize(n, 0.0);
        }

        // Initialize normals to zero (recompute later)
        mesh.normal_x.resize(n, 0.0);
        mesh.normal_y.resize(n, 0.0);
        mesh.normal_z.resize(n, 0.0);

        mesh.indices = indices.to_vec();
        mesh.material_ids = vec![MaterialId(0); indices.len() / 3];

        mesh.validate()?;
        Ok(mesh)
    }
}

