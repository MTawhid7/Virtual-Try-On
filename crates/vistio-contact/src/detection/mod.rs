//! Collision detection phases — broad and narrow.

pub mod broad;
pub mod narrow;
pub mod bvh;
pub mod spatial_hash;

pub use broad::{BroadPhase, CandidatePair};
pub use narrow::NarrowPhase;
pub use bvh::BvhBroadPhase;
pub use spatial_hash::SpatialHash;
