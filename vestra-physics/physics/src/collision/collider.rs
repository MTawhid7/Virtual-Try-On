// physics/src/collision/collider.rs
use super::geometry::Triangle;
use super::preprocessing;
use super::spatial::StaticSpatialHash;
use glam::Vec3;

/// Represents a static collider mesh (e.g., the Mannequin).
/// Uses a Spatial Hash for efficient broad-phase collision detection.
/// Stores both raw geometry (for narrow phase) and acceleration structures.
pub struct MeshCollider {
    #[allow(dead_code)]
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
    pub indices: Vec<u32>,
    pub triangles: Vec<Triangle>,
    pub spatial_hash: StaticSpatialHash,
    // Config for updates
    pub smoothing_iterations: usize,
    pub inflation_amount: f32,
}

impl MeshCollider {
    pub fn new(
        raw_vertices: Vec<f32>,
        _raw_normals: Vec<f32>,
        indices: Vec<u32>,
        smoothing: usize,
        inflation: f32,
    ) -> Self {
        // Pass config to processor
        let processed = preprocessing::process_mesh(&raw_vertices, &indices, smoothing, inflation);

        let mut min_bound = Vec3::splat(f32::MAX);
        let mut max_bound = Vec3::splat(f32::MIN);

        for &v in &processed.vertices {
            min_bound = min_bound.min(v);
            max_bound = max_bound.max(v);
        }

        let mut spatial_hash = StaticSpatialHash::new(min_bound, max_bound, 0.1);
        let mut triangles = Vec::new();

        let num_triangles = indices.len() / 3;
        for i in 0..num_triangles {
            let idx0 = indices[i * 3] as usize;
            let idx1 = indices[i * 3 + 1] as usize;
            let idx2 = indices[i * 3 + 2] as usize;

            let v0 = processed.vertices[idx0];
            let v1 = processed.vertices[idx1];
            let v2 = processed.vertices[idx2];

            let tri = Triangle::new(v0, v1, v2, i);
            let (min, max) = tri.aabb();

            triangles.push(tri);
            spatial_hash.insert_aabb(i, min, max);
        }

        MeshCollider {
            vertices: processed.vertices,
            normals: processed.normals,
            indices,
            triangles,
            spatial_hash,
            smoothing_iterations: smoothing,
            inflation_amount: inflation,
        }
    }

    /// Updates the collider mesh with new vertex positions (e.g. for animation).
    /// Reprocesses the mesh (smoothing/inflation) and rebuilds the spatial hash.
    pub fn update(&mut self, raw_vertices: &[f32]) {
        // 1. Reprocess Mesh (this handles smoothing and inflation)
        let processed = preprocessing::process_mesh(
            raw_vertices,
            &self.indices,
            self.smoothing_iterations,
            self.inflation_amount,
        );

        self.vertices = processed.vertices;
        self.normals = processed.normals;

        // 2. Clear Spatial Hash
        self.spatial_hash.clear();

        // 3. Update Triangles & Re-insert into Hash
        let num_triangles = self.indices.len() / 3;
        for i in 0..num_triangles {
            let idx0 = self.indices[i * 3] as usize;
            let idx1 = self.indices[i * 3 + 1] as usize;
            let idx2 = self.indices[i * 3 + 2] as usize;

            // Update Triangle Geometry
            self.triangles[i].v0 = self.vertices[idx0];
            self.triangles[i].v1 = self.vertices[idx1];
            self.triangles[i].v2 = self.vertices[idx2];

            // Recompute AABB
            let (min, max) = self.triangles[i].aabb();

            // Insert into Hash
            self.spatial_hash.insert_aabb(i, min, max);
        }
    }
}
