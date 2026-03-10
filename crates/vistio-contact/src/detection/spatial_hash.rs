//! Spatial hash broad phase for uniform-grid collision detection.
//!
//! Partitions space into a uniform grid and bins vertices into cells.
//! Candidate pairs are generated from vertices in the same or adjacent cells.
//! Ideal for cloth self-collision where particles are roughly uniformly distributed.

use std::collections::HashMap;

use vistio_types::VistioResult;

use crate::broad::{BroadPhase, CandidatePair};

/// Spatial hash broad phase using a uniform grid.
///
/// Cell size should be ~2× the collision thickness for optimal performance.
pub struct SpatialHash {
    /// Inverse cell size (cached for performance).
    inv_cell_size: f32,
    /// Hash map from cell key to list of vertex indices.
    grid: HashMap<(i32, i32, i32), Vec<u32>>,
    /// Number of vertices tracked.
    vertex_count: u32,
}

impl SpatialHash {
    /// Create a new spatial hash with the given cell size.
    pub fn new(cell_size: f32) -> Self {
        let cell_size = cell_size.max(1e-6);
        Self {
            inv_cell_size: 1.0 / cell_size,
            grid: HashMap::new(),
            vertex_count: 0,
        }
    }

    /// Hash a position to a cell key.
    fn cell_key(&self, x: f32, y: f32, z: f32) -> (i32, i32, i32) {
        let cx = (x * self.inv_cell_size).floor() as i32;
        let cy = (y * self.inv_cell_size).floor() as i32;
        let cz = (z * self.inv_cell_size).floor() as i32;
        (cx, cy, cz)
    }
}

impl BroadPhase for SpatialHash {
    fn update(&mut self, pos_x: &[f32], pos_y: &[f32], pos_z: &[f32], _thickness: f32) -> VistioResult<()> {
        self.grid.clear();
        self.vertex_count = pos_x.len() as u32;

        for i in 0..pos_x.len() {
            let key = self.cell_key(pos_x[i], pos_y[i], pos_z[i]);
            self.grid.entry(key).or_default().push(i as u32);
        }

        Ok(())
    }

    fn query_pairs(&self) -> Vec<CandidatePair> {
        let mut pairs = Vec::new();

        for (&(cx, cy, cz), vertices) in &self.grid {
            // Self-pairs within cell
            for i in 0..vertices.len() {
                for j in (i + 1)..vertices.len() {
                    pairs.push(CandidatePair {
                        a: vertices[i],
                        b: vertices[j],
                        is_self: true,
                    });
                }
            }

            // Pairs with 26 neighboring cells (only half to avoid duplicates)
            for dx in -1..=1_i32 {
                for dy in -1..=1_i32 {
                    for dz in -1..=1_i32 {
                        if dx == 0 && dy == 0 && dz == 0 {
                            continue;
                        }
                        // Only check neighbors with greater key to avoid duplicates
                        let nkey = (cx + dx, cy + dy, cz + dz);
                        if nkey <= (cx, cy, cz) {
                            continue;
                        }

                        if let Some(neighbor_verts) = self.grid.get(&nkey) {
                            for &a in vertices {
                                for &b in neighbor_verts {
                                    pairs.push(CandidatePair {
                                        a,
                                        b,
                                        is_self: true,
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        pairs
    }

    fn name(&self) -> &str {
        "spatial_hash"
    }
}
