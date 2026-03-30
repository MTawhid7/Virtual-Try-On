// physics/src/collision/self_collision/mod.rs

//! Handles cloth-on-cloth collision detection and resolution.
//! Uses hierarchical spatial hashing with Morton codes for efficient broad-phase,
//! and topology-aware exclusion to prevent instability from constrained neighbors.
//!
//! OPTIMIZATION: Two-phase approach with graph coloring for batched parallel resolution.

mod coloring;
mod config;
mod detection;
mod resolution;

use config::CollisionPair;
pub use config::SelfCollisionConfig;

use super::exclusion::TopologyExclusion;
use super::spatial::HierarchicalSpatialHash;
use crate::engine::state::PhysicsState;
use crate::utils::profiler::{ProfileCategory, Profiler};

/// Handles cloth-on-cloth collision detection and resolution.
/// Uses hierarchical spatial hashing with Morton codes for efficient broad-phase,
/// and topology-aware exclusion to prevent instability from constrained neighbors.
///
/// OPTIMIZATION: Two-phase approach with graph coloring for batched parallel resolution.
pub struct SelfCollision {
    pub(crate) hash: HierarchicalSpatialHash,
    pub(crate) exclusion: TopologyExclusion,
    pub config: SelfCollisionConfig,
    /// Detected collision pairs (phase 1 output)
    pub(crate) collision_pairs: Vec<CollisionPair>,
    /// Batch offsets for graph-colored pairs
    pub(crate) batch_offsets: Vec<usize>,
    /// Particle count for coloring
    pub(crate) particle_count: usize,
}

impl SelfCollision {
    /// Creates a new self-collision system from mesh topology.
    pub fn new(state: &PhysicsState, config: SelfCollisionConfig) -> Self {
        // Build topology exclusion with 2-ring depth (recommended for stability)
        let exclusion = TopologyExclusion::new(&state.indices, state.count, 2);

        // Initialize hierarchical hash with collision radius
        let hash = HierarchicalSpatialHash::new(config.thickness);

        Self {
            hash,
            exclusion,
            config,
            collision_pairs: Vec::with_capacity(1000),
            batch_offsets: Vec::new(),
            particle_count: state.count,
        }
    }

    /// Detects and resolves self-collisions using three-phase approach:
    /// 1. Detect pairs (read-only broad-phase)
    /// 2. Color pairs for parallel-safe batching
    /// 3. Resolve in batches with SIMD acceleration
    ///
    /// PROFILING: Each phase is measured individually.
    pub fn solve(&mut self, state: &mut PhysicsState) {
        Profiler::start(ProfileCategory::SelfCollisionDetect);
        let has_pairs = self.detect_pairs(state);
        Profiler::end(ProfileCategory::SelfCollisionDetect);

        if has_pairs {
            Profiler::start(ProfileCategory::SelfCollisionColor);
            self.color_pairs();
            Profiler::end(ProfileCategory::SelfCollisionColor);

            Profiler::start(ProfileCategory::SelfCollisionResolve);
            self.resolve_batched(state);
            Profiler::end(ProfileCategory::SelfCollisionResolve);
        }
    }
}
