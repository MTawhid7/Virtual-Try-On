//! Dihedral bending model for Projective Dynamics.
//!
//! Adds bending resistance via dihedral angle springs between adjacent
//! triangles sharing an interior edge. Each bending element stores the
//! rest dihedral angle and contributes to both the PD system matrix
//! and the local projection step.
//!
//! ## Geometry
//!
//! For an interior edge (v0, v1) with wing vertices (wa, wb):
//! ```text
//!        wa
//!       / \
//!      /   \
//!    v0 ─── v1
//!      \   /
//!       \ /
//!        wb
//! ```
//! The dihedral angle θ is the angle between normals of the two triangles.

use vistio_math::Vec3;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;

/// A single bending element between two adjacent triangles.
#[derive(Debug, Clone, Copy)]
pub struct BendingElement {
    /// Shared edge vertex A.
    pub v0: usize,
    /// Shared edge vertex B.
    pub v1: usize,
    /// Wing vertex of triangle A (opposite to shared edge).
    pub wing_a: usize,
    /// Wing vertex of triangle B (opposite to shared edge).
    pub wing_b: usize,
    /// Rest dihedral angle (radians).
    pub rest_angle: f32,
    /// Bending stiffness weight for the PD system matrix.
    pub weight: f32,
}

/// Collection of all bending elements.
pub struct BendingData {
    /// Per-edge bending elements.
    pub elements: Vec<BendingElement>,
    /// Bending stiffness parameter.
    pub stiffness: f32,
}

/// Compute the dihedral angle between two triangles sharing edge (v0, v1).
///
/// The dihedral angle is the angle between the wing vertices as seen
/// from the shared edge. For a flat (coplanar) mesh with wings on
/// opposite sides, returns π.
pub fn compute_dihedral_angle(
    p_v0: Vec3,
    p_v1: Vec3,
    p_wa: Vec3,
    p_wb: Vec3,
) -> f32 {
    let edge = p_v1 - p_v0;
    let edge_len = edge.length();
    if edge_len < 1e-10 {
        return std::f32::consts::PI;
    }
    let edge_dir = edge / edge_len;

    // Vectors from v0 to wing vertices, projected onto plane perpendicular to edge
    let to_a = p_wa - p_v0;
    let to_b = p_wb - p_v0;

    // Remove component along edge
    let perp_a = to_a - edge_dir * to_a.dot(edge_dir);
    let perp_b = to_b - edge_dir * to_b.dot(edge_dir);

    let len_a = perp_a.length();
    let len_b = perp_b.length();

    if len_a < 1e-10 || len_b < 1e-10 {
        return std::f32::consts::PI;
    }

    let perp_a = perp_a / len_a;
    let perp_b = perp_b / len_b;

    // Angle between projected vectors
    // When coplanar and on opposite sides → cos = -1 → angle = π
    let cos_theta = perp_a.dot(perp_b).clamp(-1.0, 1.0);
    cos_theta.acos()
}

impl BendingData {
    /// Build bending elements from mesh topology.
    ///
    /// One bending element per interior edge. The rest angle is computed
    /// from the initial mesh configuration.
    pub fn from_topology(
        mesh: &TriangleMesh,
        topology: &Topology,
        stiffness: f32,
    ) -> Self {
        let mut elements = Vec::with_capacity(topology.interior_edges.len());

        for ie in &topology.interior_edges {
            let v0 = ie.v0 as usize;
            let v1 = ie.v1 as usize;
            let wa = ie.wing_a as usize;
            let wb = ie.wing_b as usize;

            let p_v0 = Vec3::new(mesh.pos_x[v0], mesh.pos_y[v0], mesh.pos_z[v0]);
            let p_v1 = Vec3::new(mesh.pos_x[v1], mesh.pos_y[v1], mesh.pos_z[v1]);
            let p_wa = Vec3::new(mesh.pos_x[wa], mesh.pos_y[wa], mesh.pos_z[wa]);
            let p_wb = Vec3::new(mesh.pos_x[wb], mesh.pos_y[wb], mesh.pos_z[wb]);

            let rest_angle = compute_dihedral_angle(p_v0, p_v1, p_wa, p_wb);

            // Clamp near-flat rest angles to exactly 0 or π.
            // Floating-point errors can produce rest angles like 3.14148 instead of π,
            // which creates small but persistent restoring forces on boundary vertices
            // with few constraints — causing visible corner twisting and edge curling.
            let rest_angle = if (rest_angle - std::f32::consts::PI).abs() < 1e-3 {
                std::f32::consts::PI
            } else if rest_angle.abs() < 1e-3 {
                0.0
            } else {
                rest_angle
            };

            // Weight based on edge length (longer edges = more influence)
            let edge_len = (p_v1 - p_v0).length();
            let weight = stiffness * edge_len;

            elements.push(BendingElement {
                v0,
                v1,
                wing_a: wa,
                wing_b: wb,
                rest_angle,
                weight,
            });
        }

        Self {
            elements,
            stiffness,
        }
    }

    /// Build bending elements from mesh topology using material properties.
    ///
    /// Derives bending stiffness from `FabricProperties`:
    /// - Average bending stiffness (warp/weft) → element weight
    /// - Scales by 100 for appropriate PD energy magnitude
    pub fn from_topology_with_material(
        mesh: &TriangleMesh,
        topology: &Topology,
        properties: &vistio_material::FabricProperties,
    ) -> Self {
        let stiffness = properties.avg_bending_stiffness() * 100.0;
        Self::from_topology(mesh, topology, stiffness)
    }

    /// Returns the number of bending elements.
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Returns true if there are no bending elements.
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Project a bending element toward its rest dihedral angle.
    ///
    /// Returns target positions for the four vertices (v0, v1, wa, wb)
    /// that would restore the rest dihedral angle while preserving
    /// the current edge length and triangle areas.
    ///
    /// The projection applies a correction proportional to the angle
    /// difference, distributed across the wing vertices.
    pub fn project(
        &self,
        elem: &BendingElement,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> (Vec3, Vec3, Vec3, Vec3) {
        let p_v0 = Vec3::new(pos_x[elem.v0], pos_y[elem.v0], pos_z[elem.v0]);
        let p_v1 = Vec3::new(pos_x[elem.v1], pos_y[elem.v1], pos_z[elem.v1]);
        let p_wa = Vec3::new(pos_x[elem.wing_a], pos_y[elem.wing_a], pos_z[elem.wing_a]);
        let p_wb = Vec3::new(pos_x[elem.wing_b], pos_y[elem.wing_b], pos_z[elem.wing_b]);

        let current_angle = compute_dihedral_angle(p_v0, p_v1, p_wa, p_wb);
        let angle_diff = current_angle - elem.rest_angle;

        // Small angle difference → no correction needed
        if angle_diff.abs() < 1e-8 {
            return (p_v0, p_v1, p_wa, p_wb);
        }

        // Compute rotation axis (the shared edge)
        let edge = p_v1 - p_v0;
        let edge_len = edge.length();
        if edge_len < 1e-10 {
            return (p_v0, p_v1, p_wa, p_wb);
        }
        let axis = edge / edge_len;

        // Correction: rotate wing vertices toward the rest configuration
        let e_dir = axis;
        let to_a = p_wa - p_v0;
        let to_b = p_wb - p_v0;
        let p_a = to_a - e_dir * to_a.dot(e_dir);
        let p_b = to_b - e_dir * to_b.dot(e_dir);

        let n_cross = p_a.cross(p_b);
        let sign = if n_cross.dot(e_dir) >= 0.0 { 1.0 } else { -1.0 };

        let half_corr = angle_diff * 0.5 * sign;
        let mid = (p_v0 + p_v1) * 0.5;

        let wa_proj = rotate_around_axis(p_wa, mid, axis, half_corr);
        let wb_proj = rotate_around_axis(p_wb, mid, axis, -half_corr);

        (p_v0, p_v1, wa_proj, wb_proj)
    }
}

/// Rotate point `p` around an axis through `center` by `angle` radians.
fn rotate_around_axis(p: Vec3, center: Vec3, axis: Vec3, angle: f32) -> Vec3 {
    let v = p - center;
    let cos_a = angle.cos();
    let sin_a = angle.sin();
    let dot = axis.dot(v);

    let rotated = v * cos_a + axis.cross(v) * sin_a + axis * dot * (1.0 - cos_a);
    center + rotated
}
