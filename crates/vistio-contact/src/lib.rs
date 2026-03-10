//! # vistio-contact
//!
//! Collision detection and contact response for garment simulation.
//!
//! The collision pipeline is split into three phases:
//! 1. **Broad phase** — Spatial acceleration (spatial hash / BVH)
//! 2. **Narrow phase** — Exact proximity/penetration tests
//! 3. **Contact response** — Position correction / barrier forces
//!
//! Each phase is a pluggable trait, enabling different strategies
//! (e.g., spatial hash for self-collision, BVH for cloth-body).
//!
//! ## Module Structure
//!
//! - `colliders/` — Analytical collider primitives (sphere, cylinder, box, ground plane)
//! - `detection/` — Broad & narrow phase collision detection (BVH, spatial hash)
//! - `ipc/` — IPC barrier functions, CCD, and distance primitives
//! - `collision_pipeline/` — Unified pipeline orchestrating all phases
//! - `self_collision/` — Self-collision detection, coloring, and topology exclusion
//! - Top-level: contact types, response traits, vertex-triangle tests

// ─── Organized submodules ─────────────────────────────────────
pub mod colliders;
pub mod detection;
pub mod ipc;
pub mod collision_pipeline;

// ─── Top-level modules ───────────────────────────────────────
pub mod contact;
pub mod projection;
pub mod response;
pub mod self_collision;
pub mod vertex_triangle;

// Backward-compatible re-exports from self_collision/
pub use self_collision::coloring;
pub use self_collision::exclusion;

// ─── Re-exports for backward compatibility ────────────────────
// These re-exports ensure that existing `use crate::broad::*` paths
// and external `use vistio_contact::BvhBroadPhase` paths continue to work.

// Detection re-exports
pub use detection::broad;
pub use detection::narrow;
pub use detection::bvh;
pub use detection::spatial_hash;
pub use detection::{BroadPhase, BvhBroadPhase, NarrowPhase, SpatialHash};

// Collider re-exports
pub use colliders::sphere;
pub use colliders::cylinder;
pub use colliders::box_collider;
pub use colliders::ground_plane;
pub use colliders::{SphereCollider, CylinderCollider, BoxCollider, GroundPlane};

// IPC re-exports
pub use ipc::barrier;
pub use ipc::ipc_response;
pub use ipc::ccd;
pub use ipc::distance_primitives;
pub use ipc::IpcContactSet;

// Pipeline + remaining top-level re-exports
pub use collision_pipeline::CollisionPipeline;
pub use coloring::CollisionColoring;
pub use contact::{ContactPair, ContactType};
pub use exclusion::TopologyExclusion;
pub use projection::ProjectionContactResponse;
pub use response::ContactResponse;
pub use self_collision::SelfCollisionSystem;
pub use vertex_triangle::VertexTriangleTest;

#[cfg(test)]
mod tests;
