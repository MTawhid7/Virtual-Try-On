//! Contact pair data types.
//!
//! `ContactPair` represents a detected proximity or penetration
//! between two primitives (vertex-triangle, edge-edge).

use serde::{Deserialize, Serialize};

/// Type of contact between two primitives.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ContactType {
    /// Vertex of mesh A against triangle of mesh B.
    VertexTriangle,
    /// Edge of mesh A against edge of mesh B.
    EdgeEdge,
}

/// A detected contact between two primitives.
///
/// Carries all geometric data needed by the contact response phase
/// to compute position corrections or barrier forces.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactPair {
    /// Contact type.
    pub contact_type: ContactType,

    /// Indices of the primitives involved.
    ///
    /// For `VertexTriangle`: `[vertex, tri_v0, tri_v1, tri_v2]`
    /// For `EdgeEdge`: `[edge_a_v0, edge_a_v1, edge_b_v0, edge_b_v1]`
    pub indices: [u32; 4],

    /// Signed distance (negative = penetration).
    pub distance: f32,

    /// Contact normal (unit vector, points from B to A).
    pub normal: [f32; 3],

    /// Barycentric coordinates of the closest point on the triangle/edge.
    pub barycentric: [f32; 3],

    /// Whether this is a self-collision (both primitives from same mesh).
    pub is_self: bool,
}

impl ContactPair {
    /// Returns the penetration depth (positive if penetrating, zero otherwise).
    pub fn penetration_depth(&self) -> f32 {
        (-self.distance).max(0.0)
    }

    /// Returns true if the contact represents actual penetration.
    pub fn is_penetrating(&self) -> bool {
        self.distance < 0.0
    }
}
