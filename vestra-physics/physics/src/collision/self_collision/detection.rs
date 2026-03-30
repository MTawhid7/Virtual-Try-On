// physics/src/collision/self_collision/detection.rs

//! Phase 1: Collision pair detection using spatial hashing.

use super::SelfCollision;
use super::config::CollisionPair;
use crate::engine::state::PhysicsState;
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use rustc_hash::FxHashSet;

impl SelfCollision {
    /// Phase 1: Detect all collision pairs (read-only on positions).
    /// Returns true if there are pairs to resolve.
    pub(crate) fn detect_pairs(&mut self, state: &PhysicsState) -> bool {
        self.collision_pairs.clear();

        // 1. Rebuild hash with current positions (Serial - fast O(N))
        self.hash.clear();
        for i in 0..state.count {
            self.hash
                .insert_point(i as u32, state.positions[i].truncate());
        }

        let thickness = self.config.thickness;
        let thickness_sq = thickness * thickness;
        let max_pairs = self.config.max_pairs;

        // 2. Detect pairs (Parallel Query)
        #[cfg(feature = "parallel")]
        {
            let results: Vec<CollisionPair> = (0..state.count)
                .into_par_iter()
                .map(|i| {
                    let p_i = state.positions[i].truncate();

                    // Thread-local scratch buffers
                    let mut query_buffer = Vec::with_capacity(32);
                    let mut dedup_set = FxHashSet::default();

                    self.hash
                        .query(p_i, thickness, &mut query_buffer, &mut dedup_set);

                    let mut local_pairs = Vec::new();

                    for &j in query_buffer.iter() {
                        let j = j as usize;

                        // Only process once per pair (i < j)
                        if i >= j {
                            continue;
                        }
                        if self.exclusion.should_exclude(i, j) {
                            continue;
                        }

                        let p_j = state.positions[j].truncate();
                        let delta = p_i - p_j;
                        let dist_sq = delta.length_squared();

                        if dist_sq < thickness_sq && dist_sq > 1e-9 {
                            local_pairs.push(CollisionPair {
                                i: i as u32,
                                j: j as u32,
                            });
                        }
                    }
                    local_pairs
                })
                .flatten() // Flatten thread results
                .collect();

            // Cap results if needed (though hard to cap strictly during parallel)
            if results.len() > max_pairs {
                self.collision_pairs
                    .extend_from_slice(&results[..max_pairs]);
            } else {
                self.collision_pairs = results;
            }
        }

        #[cfg(not(feature = "parallel"))]
        {
            // Re-use member buffer for serial
            let mut dedup_set = FxHashSet::default();

            for i in 0..state.count {
                let p_i = state.positions[i].truncate();
                // We can temporarily borrow query_buffer if we are careful, but the struct has it.
                // But wait, the function takes &mut self, so we can't borrow self.hash (immutable) and self.query_buffer (mutable) easily?
                // self.hash.query takes &self.
                // We need to use a local buffer or split borrows.
                // Let's use a local buffer/set to match the parallel logic and avoid borrow checker pain with &mut self.
                let mut query_buffer = Vec::with_capacity(32);

                self.hash
                    .query(p_i, thickness, &mut query_buffer, &mut dedup_set);

                for &j in query_buffer.iter() {
                    let j = j as usize;

                    // Only process once per pair (i < j)
                    if i >= j {
                        continue;
                    }
                    if self.exclusion.should_exclude(i, j) {
                        continue;
                    }

                    let p_j = state.positions[j].truncate();
                    let delta = p_i - p_j;
                    let dist_sq = delta.length_squared();

                    if dist_sq < thickness_sq && dist_sq > 1e-9 {
                        self.collision_pairs.push(CollisionPair {
                            i: i as u32,
                            j: j as u32,
                        });

                        // Cap pairs for performance
                        if self.collision_pairs.len() >= max_pairs {
                            break;
                        }
                    }
                }

                if self.collision_pairs.len() >= max_pairs {
                    break;
                }
            }
        }

        !self.collision_pairs.is_empty()
    }
}
