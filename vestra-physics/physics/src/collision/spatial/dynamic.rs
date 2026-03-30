// physics/src/collision/spatial/dynamic.rs

use glam::Vec3;
use rustc_hash::FxHashMap;
use smallvec::SmallVec;

/// Morton code computation for cache-coherent spatial hashing.
/// Interleaves bits in ZYX order for 3D spatial locality.
#[inline]
fn expand_bits(mut v: u64) -> u64 {
    // Spread bits: 0b111 -> 0b001001001
    v = (v | (v << 32)) & 0x1f00000000ffff;
    v = (v | (v << 16)) & 0x1f0000ff0000ff;
    v = (v | (v << 8)) & 0x100f00f00f00f00f;
    v = (v | (v << 4)) & 0x10c30c30c30c30c3;
    v = (v | (v << 2)) & 0x1249249249249249;
    v
}

#[inline]
pub fn morton_encode(x: i32, y: i32, z: i32) -> u64 {
    // Offset to positive range to handle negative coordinates
    let x = (x as i64 + 512) as u64 & 0x3FF;
    let y = (y as i64 + 512) as u64 & 0x3FF;
    let z = (z as i64 + 512) as u64 & 0x3FF;
    expand_bits(x) | (expand_bits(y) << 1) | (expand_bits(z) << 2)
}

/// A hierarchical, Morton-coded spatial hash for dynamic objects.
/// Uses two grid levels: fine (collision radius) and coarse (4x) for efficient broad-phase.
/// Optimized with FxHashMap for O(1) lookups and SmallVec for stack-allocated cells.
pub struct HierarchicalSpatialHash {
    fine_cell_size: f32,
    coarse_cell_size: f32,
    fine_grid: FxHashMap<u64, SmallVec<[u32; 8]>>,
    coarse_grid: FxHashMap<u64, SmallVec<[u32; 16]>>,
}

impl HierarchicalSpatialHash {
    pub fn new(collision_radius: f32) -> Self {
        let fine_cell_size = collision_radius * 2.0;
        let coarse_cell_size = fine_cell_size * 4.0;

        Self {
            fine_cell_size,
            coarse_cell_size,
            fine_grid: FxHashMap::default(),
            coarse_grid: FxHashMap::default(),
        }
    }

    /// Clears all cells without deallocating memory (reuses capacity).
    pub fn clear(&mut self) {
        for (_, cell) in self.fine_grid.iter_mut() {
            cell.clear();
        }
        for (_, cell) in self.coarse_grid.iter_mut() {
            cell.clear();
        }
    }

    #[inline]
    fn get_fine_cell(&self, p: Vec3) -> (i32, i32, i32) {
        (
            (p.x / self.fine_cell_size).floor() as i32,
            (p.y / self.fine_cell_size).floor() as i32,
            (p.z / self.fine_cell_size).floor() as i32,
        )
    }

    #[inline]
    fn get_coarse_cell(&self, p: Vec3) -> (i32, i32, i32) {
        (
            (p.x / self.coarse_cell_size).floor() as i32,
            (p.y / self.coarse_cell_size).floor() as i32,
            (p.z / self.coarse_cell_size).floor() as i32,
        )
    }

    /// Inserts a particle into both fine and coarse grids.
    pub fn insert_point(&mut self, id: u32, p: Vec3) {
        let fine_cell = self.get_fine_cell(p);
        let fine_key = morton_encode(fine_cell.0, fine_cell.1, fine_cell.2);
        self.fine_grid.entry(fine_key).or_default().push(id);

        let coarse_cell = self.get_coarse_cell(p);
        let coarse_key = morton_encode(coarse_cell.0, coarse_cell.1, coarse_cell.2);
        self.coarse_grid.entry(coarse_key).or_default().push(id);
    }

    /// Queries particles within radius using hierarchical refinement.
    /// 1. Check coarse grid for early exit
    /// 2. Refine to fine grid for actual candidates
    /// Queries particles within radius using hierarchical refinement.
    /// 1. Check coarse grid for early exit
    /// 2. Refine to fine grid for actual candidates
    /// Thread-safe version: accepts external buffers.
    pub fn query(
        &self,
        p: Vec3,
        radius: f32,
        buffer: &mut Vec<u32>,
        dedup_set: &mut rustc_hash::FxHashSet<u32>,
    ) {
        buffer.clear();
        dedup_set.clear();

        // Early exit: check coarse grid first
        let coarse_cell = self.get_coarse_cell(p);
        let coarse_key = morton_encode(coarse_cell.0, coarse_cell.1, coarse_cell.2);
        if !self.coarse_grid.contains_key(&coarse_key) {
            // Also check immediate neighbors for boundary cases
            let mut has_neighbors = false;
            for dz in -1i32..=1 {
                for dy in -1i32..=1 {
                    for dx in -1i32..=1 {
                        let key = morton_encode(
                            coarse_cell.0 + dx,
                            coarse_cell.1 + dy,
                            coarse_cell.2 + dz,
                        );
                        if self.coarse_grid.contains_key(&key) {
                            has_neighbors = true;
                            break;
                        }
                    }
                    if has_neighbors {
                        break;
                    }
                }
                if has_neighbors {
                    break;
                }
            }
            if !has_neighbors {
                return;
            }
        }

        // Fine grid query
        let min = p - Vec3::splat(radius);
        let max = p + Vec3::splat(radius);

        let (min_x, min_y, min_z) = self.get_fine_cell(min);
        let (max_x, max_y, max_z) = self.get_fine_cell(max);

        for z in min_z..=max_z {
            for y in min_y..=max_y {
                for x in min_x..=max_x {
                    let key = morton_encode(x, y, z);
                    if let Some(cell) = self.fine_grid.get(&key) {
                        for &id in cell.iter() {
                            if dedup_set.insert(id) {
                                buffer.push(id);
                            }
                        }
                    }
                }
            }
        }
    }

    #[allow(dead_code)]
    pub fn fine_grid_capacity(&self) -> usize {
        self.fine_grid.capacity()
    }
}
