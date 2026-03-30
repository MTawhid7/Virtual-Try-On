// physics/src/collision/self_collision/config.rs

//! Configuration and data types for self-collision detection.

/// Configuration for self-collision behavior.
/// Allows runtime tuning of quality vs. performance trade-off.
pub struct SelfCollisionConfig {
    /// Minimum separation distance between particles (default: 0.005 = 5mm)
    pub thickness: f32,
    /// Repulsion strength (0.0 - 1.0, default: 0.5)
    pub stiffness: f32,
    /// Solve every N substeps (1 = every substep, 2 = every other, etc.)
    pub frequency: u8,
    /// Maximum collision pairs to process per frame (performance cap)
    pub max_pairs: usize,
}

impl Default for SelfCollisionConfig {
    fn default() -> Self {
        Self {
            thickness: 0.005,
            stiffness: 0.5,
            frequency: 2, // Every other substep
            max_pairs: 10000,
        }
    }
}

/// A collision pair between two particles.
#[derive(Clone, Copy)]
pub(crate) struct CollisionPair {
    pub i: u32,
    pub j: u32,
}
