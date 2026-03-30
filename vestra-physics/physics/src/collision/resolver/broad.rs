// physics/src/collision/resolver/broad.rs
use super::CollisionResolver;
use crate::collision::collider::MeshCollider;
use crate::engine::state::PhysicsState;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use rustc_hash::FxHashSet;

/// Executes the Broad Phase of collision detection.
/// Queries the Spatial Hash to find potential collision candidates (triangles close to particles).
/// Populates the `candidate_indices` buffer in the resolver.
///
/// OPTIMIZATION: Uses Rayon for parallel spatial hash queries.
pub fn perform_broad_phase(
    resolver: &mut CollisionResolver,
    state: &PhysicsState,
    collider: &mut MeshCollider,
) {
    // 1. Reset counters
    // resolver.candidate_indices.clear(); // We rewrite, so no clear needed if we resize strictly
    // Actually we need to ensure capacity.

    #[cfg(feature = "parallel")]
    {
        // Step 1: Compute candidate counts in parallel
        // We use a chunk-based approach or zip with indices
        resolver
            .candidate_counts
            .par_iter_mut()
            .enumerate()
            .take(state.count)
            .for_each(|(i, count_ref)| {
                if state.inv_mass[i] == 0.0 {
                    *count_ref = 0;
                    return;
                }

                let pos = state.positions[i].truncate();
                let prev = state.prev_positions[i].truncate();

                // Optimized check: only query if moving or near mesh
                if !collider.spatial_hash.contains(pos) && !collider.spatial_hash.contains(prev) {
                    *count_ref = 0;
                    return;
                }

                let search_radius = 0.02 + pos.distance(prev);

                // Thread-local scratch buffers
                // We waste some allocation here but SmallVec/stack might be too small for complex collision
                let mut local_buffer = Vec::with_capacity(32);
                let mut local_dedup = FxHashSet::default();

                // Query (Read-only on spatial hash)
                collider.spatial_hash.query(
                    pos,
                    search_radius,
                    &mut local_buffer,
                    &mut local_dedup,
                );

                *count_ref = local_buffer.len();
            });

        // Step 2: Prefix Sum (Scan) to compute offsets
        // Serial is fast enough for 5000 particles
        let mut current_offset = 0;
        for i in 0..state.count {
            resolver.candidate_offsets[i] = current_offset;
            current_offset += resolver.candidate_counts[i];
        }

        // Resize indices buffer to fit all results
        // Safety: We write to disjoint regions
        resolver.candidate_indices.resize(current_offset, 0);

        // Safety: We write to disjoint regions.
        // Convert pointer to usize to allow capturing in closure (usize is Send+Sync)
        let indices_ptr_addr = resolver.candidate_indices.as_mut_ptr() as usize;

        // Capture read-only slices/references to avoid capturing &mut resolver/collider
        let counts = &resolver.candidate_counts;
        let offsets = &resolver.candidate_offsets;
        let spatial_hash = &collider.spatial_hash;

        // Step 3: Parallel Write (Scatter)
        // We re-query to write directly to the target index.
        // Double-querying is often faster than dynamic allocation + merge in parallel.
        (0..state.count).into_par_iter().for_each(move |i| {
            let count = counts[i];
            if count == 0 {
                return;
            }

            let pos = state.positions[i].truncate();
            let prev = state.prev_positions[i].truncate();
            let search_radius = 0.02 + pos.distance(prev);

            let mut local_buffer = Vec::with_capacity(count);
            let mut local_dedup = FxHashSet::default();

            spatial_hash.query(pos, search_radius, &mut local_buffer, &mut local_dedup);

            let offset = offsets[i];
            let ptr = indices_ptr_addr as *mut usize;

            // Unsafe write to pre-calculated disjoint offsets
            for (j, &val) in local_buffer.iter().enumerate() {
                unsafe {
                    *ptr.add(offset + j) = val;
                }
            }
        });
    }

    #[cfg(not(feature = "parallel"))]
    {
        resolver.candidate_indices.clear();
        let mut dedup = FxHashSet::default();

        for i in 0..state.count {
            if state.inv_mass[i] == 0.0 {
                resolver.candidate_counts[i] = 0;
                continue;
            }

            let pos = state.positions[i].truncate();
            let prev = state.prev_positions[i].truncate();
            let search_radius = 0.02 + pos.distance(prev);

            if !collider.spatial_hash.contains(pos) && !collider.spatial_hash.contains(prev) {
                resolver.candidate_counts[i] = 0;
                continue;
            }

            collider
                .spatial_hash
                .query(pos, search_radius, &mut resolver.query_buffer, &mut dedup);

            let start_idx = resolver.candidate_indices.len();
            let query_len = resolver.query_buffer.len();

            // HEAP GUARD: Cap candidates
            if start_idx + query_len < resolver.candidate_indices.capacity() {
                resolver
                    .candidate_indices
                    .extend_from_slice(&resolver.query_buffer);
                resolver.candidate_offsets[i] = start_idx;
                resolver.candidate_counts[i] = query_len;
            } else {
                resolver.candidate_counts[i] = 0;
            }
        }
    }
}
