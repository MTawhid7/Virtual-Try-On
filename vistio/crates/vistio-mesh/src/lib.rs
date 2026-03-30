//! # vistio-mesh
//!
//! Triangle mesh representation with Structure-of-Arrays (SoA) layout
//! for GPU-friendly memory access patterns.
//!
//! ## Key Types
//!
//! - [`TriangleMesh`] — The core mesh type. Stores positions, normals, UVs,
//!   and topology in contiguous SoA buffers.
//! - [`Topology`] — Adjacency queries (vertex-to-triangle, edge neighbors).
//! - Procedural generators for benchmark meshes (quad grids, UV spheres).

pub mod generators;
pub mod mesh;
pub mod normals;
pub mod topology;

pub use mesh::TriangleMesh;
pub use topology::Topology;
