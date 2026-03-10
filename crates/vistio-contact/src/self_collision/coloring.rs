//! Graph coloring for parallel-safe collision resolution.
//!
//! Implements greedy graph coloring with bitmask-accelerated color searching.
//! Organizes collision pairs into batches where no two pairs in a batch
//! share a vertex, enabling parallel resolution without write conflicts.

/// Collision pair coloring for batched parallel resolution.
///
/// Uses a greedy graph coloring algorithm with u64 bitmask for O(1)
/// color assignment. The conflict graph has an edge between two collision
/// pairs if they share a vertex.
pub struct CollisionColoring;

impl CollisionColoring {
    /// Color collision pairs for parallel-safe batched resolution.
    ///
    /// Returns `(sorted_pairs, batch_offsets)` where:
    /// - `sorted_pairs`: pairs reordered so each batch is contiguous
    /// - `batch_offsets`: indices into `sorted_pairs` where each batch starts
    ///
    /// Pairs within a batch can be resolved independently (no shared vertices).
    pub fn color_pairs(
        pairs: &[(u32, u32)],
        vertex_count: usize,
    ) -> (Vec<(u32, u32)>, Vec<usize>) {
        if pairs.is_empty() {
            return (Vec::new(), vec![0]);
        }

        let n_pairs = pairs.len();

        // Build vertex → pair adjacency
        let mut vertex_to_pairs: Vec<Vec<usize>> = vec![Vec::new(); vertex_count];
        for (pi, &(a, b)) in pairs.iter().enumerate() {
            vertex_to_pairs[a as usize].push(pi);
            vertex_to_pairs[b as usize].push(pi);
        }

        // Build pair-pair conflict graph (CSR-like adjacency)
        let mut adjacency: Vec<Vec<usize>> = vec![Vec::new(); n_pairs];
        for vtop in &vertex_to_pairs {
            for i in 0..vtop.len() {
                for j in (i + 1)..vtop.len() {
                    let pi = vtop[i];
                    let pj = vtop[j];
                    adjacency[pi].push(pj);
                    adjacency[pj].push(pi);
                }
            }
        }

        // Greedy coloring with bitmask
        let mut colors: Vec<usize> = vec![usize::MAX; n_pairs];
        let mut max_color = 0;

        for pi in 0..n_pairs {
            // Build bitmask of colors used by neighbors
            let mut used_mask: u64 = 0;
            for &neighbor in &adjacency[pi] {
                let c = colors[neighbor];
                if c < 64 {
                    used_mask |= 1u64 << c;
                }
            }

            // Find first available color (first zero bit)
            let color = (!used_mask).trailing_zeros() as usize;
            colors[pi] = color;
            max_color = max_color.max(color);
        }

        // Organize pairs into batches by color
        let n_colors = max_color + 1;
        let mut batches: Vec<Vec<(u32, u32)>> = vec![Vec::new(); n_colors];
        for (pi, &color) in colors.iter().enumerate() {
            batches[color].push(pairs[pi]);
        }

        // Flatten into sorted_pairs + batch_offsets
        let mut sorted_pairs = Vec::with_capacity(n_pairs);
        let mut batch_offsets = vec![0usize];
        for batch in &batches {
            sorted_pairs.extend_from_slice(batch);
            batch_offsets.push(sorted_pairs.len());
        }

        (sorted_pairs, batch_offsets)
    }
}
