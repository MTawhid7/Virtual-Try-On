// physics/src/collision/self_collision/coloring.rs

//! Phase 2: Graph coloring for parallel-safe collision resolution.

use super::SelfCollision;

impl SelfCollision {
    /// Phase 2: Color pairs for parallel-safe resolution.
    pub(crate) fn color_pairs(&mut self) {
        if self.collision_pairs.is_empty() {
            self.batch_offsets.clear();
            return;
        }

        // Build degree count for each particle
        let mut degree = vec![0usize; self.particle_count];
        for pair in &self.collision_pairs {
            degree[pair.i as usize] += 1;
            degree[pair.j as usize] += 1;
        }

        // CSR offset array
        let mut offset = vec![0usize; self.particle_count + 1];
        for i in 0..self.particle_count {
            offset[i + 1] = offset[i] + degree[i];
        }

        // Adjacency list (pair indices for each particle)
        let mut adj = vec![0usize; offset[self.particle_count]];
        let mut counter = offset.clone();
        for (idx, pair) in self.collision_pairs.iter().enumerate() {
            let i = pair.i as usize;
            let j = pair.j as usize;
            adj[counter[i]] = idx; counter[i] += 1;
            adj[counter[j]] = idx; counter[j] += 1;
        }

        // Greedy coloring
        let mut pair_colors: Vec<Option<usize>> = vec![None; self.collision_pairs.len()];
        let mut batch_indices: Vec<Vec<usize>> = Vec::new();

        for idx in 0..self.collision_pairs.len() {
            let pair = &self.collision_pairs[idx];
            let i = pair.i as usize;
            let j = pair.j as usize;
            let mut used_colors = 0u64;

            // Check neighbors of particle i
            for &c_idx in &adj[offset[i]..offset[i + 1]] {
                if let Some(c) = pair_colors[c_idx] {
                    if c < 64 {
                        used_colors |= 1u64 << c;
                    }
                }
            }
            // Check neighbors of particle j
            for &c_idx in &adj[offset[j]..offset[j + 1]] {
                if let Some(c) = pair_colors[c_idx] {
                    if c < 64 {
                        used_colors |= 1u64 << c;
                    }
                }
            }

            let color = (!used_colors).trailing_zeros() as usize;
            pair_colors[idx] = Some(color);

            if color >= batch_indices.len() {
                batch_indices.resize(color + 1, Vec::new());
            }
            batch_indices[color].push(idx);
        }

        // Reorder pairs by batch and store offsets
        let mut sorted_pairs = Vec::with_capacity(self.collision_pairs.len());
        self.batch_offsets.clear();
        let mut current_offset = 0;

        for batch in batch_indices {
            self.batch_offsets.push(current_offset);
            for idx in batch {
                sorted_pairs.push(self.collision_pairs[idx]);
            }
            current_offset = sorted_pairs.len();
        }
        self.batch_offsets.push(current_offset);
        self.collision_pairs = sorted_pairs;
    }
}
