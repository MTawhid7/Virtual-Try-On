// physics/src/collision/exclusion.rs

use rustc_hash::FxHashSet;

/// Precomputed topology exclusion masks for O(1) neighbor filtering.
/// Prevents self-collision between topologically connected vertices
/// (which would cause instability as they're already constrained by edges).
pub struct TopologyExclusion {
    /// For particles < 64 neighbors, use bitmask for O(1) lookup
    masks: Vec<u64>,
    /// For particles with >= 64 neighbors, fall back to hash set
    extended: Vec<Option<FxHashSet<u32>>>,
    /// Number of ring hops to exclude (default: 2)
    #[allow(dead_code)] // Stored for debugging/introspection
    ring_depth: usize,
}

impl TopologyExclusion {
    /// Builds exclusion masks from mesh topology.
    /// `ring_depth` controls how many hops of neighbors to exclude.
    /// - 1 = immediate edge neighbors only
    /// - 2 = neighbors of neighbors (recommended for stability)
    pub fn new(indices: &[u32], particle_count: usize, ring_depth: usize) -> Self {
        // Build adjacency list
        let mut adjacency: Vec<FxHashSet<u32>> = vec![FxHashSet::default(); particle_count];
        let num_triangles = indices.len() / 3;

        for t in 0..num_triangles {
            let i0 = indices[t * 3] as usize;
            let i1 = indices[t * 3 + 1] as usize;
            let i2 = indices[t * 3 + 2] as usize;

            adjacency[i0].insert(i1 as u32);
            adjacency[i0].insert(i2 as u32);
            adjacency[i1].insert(i0 as u32);
            adjacency[i1].insert(i2 as u32);
            adjacency[i2].insert(i0 as u32);
            adjacency[i2].insert(i1 as u32);
        }

        // BFS to find N-ring neighbors
        let mut masks = vec![0u64; particle_count];
        let mut extended: Vec<Option<FxHashSet<u32>>> = vec![None; particle_count];

        for i in 0..particle_count {
            let mut visited = FxHashSet::default();
            let mut frontier: Vec<u32> = vec![i as u32];
            visited.insert(i as u32);

            for _ring in 0..ring_depth {
                let mut next_frontier = Vec::new();
                for &node in &frontier {
                    for &neighbor in &adjacency[node as usize] {
                        if visited.insert(neighbor) {
                            next_frontier.push(neighbor);
                        }
                    }
                }
                frontier = next_frontier;
            }

            // Store in appropriate format
            let mut needs_extended = false;
            for &v in &visited {
                if v < 64 {
                    masks[i] |= 1u64 << v;
                } else {
                    needs_extended = true;
                }
            }

            if needs_extended || visited.len() > 64 {
                extended[i] = Some(visited);
            }
        }

        Self {
            masks,
            extended,
            ring_depth,
        }
    }

    /// O(1) check if particle j should be excluded from collision with particle i.
    /// Returns true if j is within N-ring topology of i.
    #[inline(always)]
    pub fn should_exclude(&self, i: usize, j: usize) -> bool {
        // Fast path: use bitmask for j < 64
        if j < 64 {
            return (self.masks[i] & (1u64 << j)) != 0;
        }

        // Slow path: check extended set
        if let Some(ref set) = self.extended[i] {
            return set.contains(&(j as u32));
        }

        false
    }

    /// Returns the ring depth used for exclusion.
    #[allow(dead_code)] // Used for debugging/testing
    pub fn ring_depth(&self) -> usize {
        self.ring_depth
    }
}
