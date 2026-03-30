// physics/src/collision/spatial/static_grid.rs

use glam::Vec3;
use rustc_hash::FxHashSet;

/// A fixed-size 3D grid for spatial partitioning.
/// Optimized for static geometry (like the mannequin) where objects do not move.
/// Allows fast O(1) lookups of triangles near a particle.
pub struct StaticSpatialHash {
    cell_size: f32,
    min: Vec3,
    max: Vec3,
    width: usize,
    height: usize,
    depth: usize,
    cells: Vec<Vec<usize>>,
}

impl StaticSpatialHash {
    pub fn new(bounds_min: Vec3, bounds_max: Vec3, cell_size: f32) -> Self {
        let padding = Vec3::splat(cell_size * 2.0);
        let min = bounds_min - padding;
        let max = bounds_max + padding;

        let size = max - min;

        let width = (size.x / cell_size).ceil() as usize;
        let height = (size.y / cell_size).ceil() as usize;
        let depth = (size.z / cell_size).ceil() as usize;

        // Safety caps to prevent OOM on huge meshes
        let safe_width = width.max(1).min(1000);
        let safe_height = height.max(1).min(1000);
        let safe_depth = depth.max(1).min(1000);

        let total_cells = safe_width * safe_height * safe_depth;

        Self {
            cell_size,
            min,
            max,
            width: safe_width,
            height: safe_height,
            depth: safe_depth,
            cells: vec![Vec::new(); total_cells],
        }
    }

    /// Fast AABB check to skip particles far outside the mesh
    pub fn contains(&self, p: Vec3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    /// Clears all cells in the spatial hash, preparing it for a new frame.
    /// Keeps the allocated memory (capacity) for performance.
    pub fn clear(&mut self) {
        for cell in &mut self.cells {
            cell.clear();
        }
    }

    /// Inserts a triangle index into all cells that overlap its Axis-Aligned Bounding Box (AABB).
    /// This ensures that even large triangles are correctly registered in the grid.
    pub fn insert_aabb(&mut self, id: usize, min: Vec3, max: Vec3) {
        let start_local = (min - self.min).max(Vec3::ZERO);
        let end_local = max - self.min;

        let min_x = (start_local.x / self.cell_size) as usize;
        let min_y = (start_local.y / self.cell_size) as usize;
        let min_z = (start_local.z / self.cell_size) as usize;

        let max_x = ((end_local.x / self.cell_size) as usize).min(self.width - 1);
        let max_y = ((end_local.y / self.cell_size) as usize).min(self.height - 1);
        let max_z = ((end_local.z / self.cell_size) as usize).min(self.depth - 1);

        for z in min_z..=max_z {
            for y in min_y..=max_y {
                for x in min_x..=max_x {
                    let idx = x + y * self.width + z * self.width * self.height;
                    self.cells[idx].push(id);
                }
            }
        }
    }

    /// Retrieves all triangles in cells overlapping the query radius.
    /// Used during Broad Phase Collision Detection.
    /// OPTIMIZATION: Uses FxHashSet for O(N) deduplication instead of O(N log N) sort.
    /// This is critical because broad phase is called for every active particle.
    /// Retrieves all triangles in cells overlapping the query radius.
    /// Used during Broad Phase Collision Detection.
    /// Thread-safe version: requires external buffers to avoid internal mutation.
    pub fn query(
        &self,
        p: Vec3,
        radius: f32,
        buffer: &mut Vec<usize>,
        dedup_set: &mut FxHashSet<usize>,
    ) {
        buffer.clear();
        dedup_set.clear();

        let min = p - Vec3::splat(radius);
        let max = p + Vec3::splat(radius);

        let start_local = (min - self.min).max(Vec3::ZERO);
        let end_local = max - self.min;

        let min_x = (start_local.x / self.cell_size) as usize;
        let min_y = (start_local.y / self.cell_size) as usize;
        let min_z = (start_local.z / self.cell_size) as usize;

        if min_x >= self.width || min_y >= self.height || min_z >= self.depth {
            return;
        }

        let max_x = ((end_local.x / self.cell_size) as usize).min(self.width - 1);
        let max_y = ((end_local.y / self.cell_size) as usize).min(self.height - 1);
        let max_z = ((end_local.z / self.cell_size) as usize).min(self.depth - 1);

        for z in min_z..=max_z {
            for y in min_y..=max_y {
                for x in min_x..=max_x {
                    let idx = x + y * self.width + z * self.width * self.height;
                    // Safe immutable access to cells
                    for &triangle_id in &self.cells[idx] {
                        if dedup_set.insert(triangle_id) {
                            buffer.push(triangle_id);
                        }
                    }
                }
            }
        }
    }
}
