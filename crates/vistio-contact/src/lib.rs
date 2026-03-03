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
//! ## Pipeline
//!
//! The [`CollisionPipeline`] orchestrates all three phases and optional
//! ground plane collision into a single `step()` call.

pub mod barrier;
pub mod broad;
pub mod bvh;
pub mod ccd;
pub mod collision_pipeline;
pub mod coloring;
pub mod contact;
pub mod distance_primitives;
pub mod exclusion;
pub mod ground_plane;
pub mod ipc_response;
pub mod narrow;
pub mod projection;
pub mod response;
pub mod self_collision;
pub mod spatial_hash;
pub mod sphere;
pub mod cylinder;
pub mod box_collider;
pub mod vertex_triangle;

pub use broad::BroadPhase;
pub use bvh::BvhBroadPhase;
pub use collision_pipeline::CollisionPipeline;
pub use coloring::CollisionColoring;
pub use contact::{ContactPair, ContactType};
pub use exclusion::TopologyExclusion;
pub use ground_plane::GroundPlane;
pub use ipc_response::IpcContactSet;
pub use narrow::NarrowPhase;
pub use projection::ProjectionContactResponse;
pub use response::ContactResponse;
pub use self_collision::SelfCollisionSystem;
pub use spatial_hash::SpatialHash;
pub use sphere::SphereCollider;
pub use cylinder::CylinderCollider;
pub use box_collider::BoxCollider;
pub use vertex_triangle::VertexTriangleTest;

#[cfg(test)]
mod tests;
