//! Discrete Shells bending model (Grinspun et al. 2003).
//!
//! Replaces the dihedral-angle springs with a curvature-based bending energy
//! that uses **cotangent-weighted** Laplacian operators. This produces
//! resolution-independent fold behavior: curvature is measured per-unit-area,
//! so mesh refinement doesn't change the effective bending stiffness.
//!
//! ## Energy
//!
//! The bending energy for each interior edge is:
//!
//!   E_bend = (k_b / 2) · (3 / A_e) · (θ - θ₀)²
//!
//! where:
//! - `A_e` = combined area of the two triangles sharing the edge
//! - `θ` = current dihedral angle
//! - `θ₀` = rest dihedral angle
//! - `k_b` = bending stiffness
//!
//! ## PD Integration
//!
//! In the Projective Dynamics framework, each bending element contributes:
//! - **System matrix:** `w_b · Lᵀ L` where `L` is the cotangent-weighted
//!   Laplacian stencil for the 4-vertex bending element
//! - **RHS:** `w_b · Lᵀ · p_b` where `p_b` is the local projection target
//!
//! The cotangent-weighted stencil replaces the uniform `[-1, -1, 1, 1]`
//! from the dihedral model, giving area-dependent curvature estimation.
//!
//! ## Weight Scaling
//!
//! The PD weight per bending element uses the same magnitude as the
//! proven dihedral model (`stiffness * edge_len`). The improved curvature
//! estimation comes from the cotangent-weighted stencil coefficients, NOT
//! from scaling the weight by 3/A_e (which produces catastrophically large
//! values that make the system ill-conditioned).
//!
//! ## Geometry
//!
//! ```text
//!        wa
//!       / \
//!      /   \
//!    v0 ─── v1
//!      \   /
//!       \ /
//!        wb
//! ```

use vistio_math::Vec3;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;

use crate::bending::compute_dihedral_angle;

/// A single bending element using the Discrete Shells formulation.
#[derive(Debug, Clone, Copy)]
pub struct DiscreteShellsElement {
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
    /// Combined area of the two adjacent triangles (m²).
    pub combined_area: f32,
    /// Cotangent-weighted Laplacian stencil coefficients for [v0, v1, wa, wb].
    /// These encode the curvature operator for this edge.
    /// Normalized so that the stencil vector has unit L2 norm.
    pub stencil: [f32; 4],
    /// Bending stiffness weight for the PD system matrix.
    /// w_b = stiffness · edge_len (matching dihedral model magnitude)
    pub weight: f32,
}

/// Collection of all Discrete Shells bending elements.
pub struct DiscreteShellsBendingData {
    /// Per-edge bending elements.
    pub elements: Vec<DiscreteShellsElement>,
    /// Global bending stiffness parameter.
    pub stiffness: f32,
}

/// Compute the area of triangle (a, b, c).
fn triangle_area(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let e1 = b - a;
    let e2 = c - a;
    0.5 * e1.cross(e2).length()
}

impl DiscreteShellsBendingData {
    /// Build Discrete Shells bending elements from mesh topology.
    ///
    /// For each interior edge, computes:
    /// - Rest dihedral angle from initial geometry
    /// - Cotangent-weighted Laplacian stencil (normalized)
    /// - Combined triangle area for reference
    ///
    /// The cotangent weights come from the angles opposite the shared edge
    /// in each adjacent triangle. The stencil is normalized to unit L2 norm
    /// so that the weight alone controls the energy magnitude.
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

            // Rest dihedral angle (with clamping for flat meshes)
            let rest_angle = {
                let angle = compute_dihedral_angle(p_v0, p_v1, p_wa, p_wb);
                if (angle - std::f32::consts::PI).abs() < 1e-3 {
                    std::f32::consts::PI
                } else if angle.abs() < 1e-3 {
                    0.0
                } else {
                    angle
                }
            };

            // Combined area of the two adjacent triangles
            let area_a = triangle_area(p_v0, p_v1, p_wa);
            let area_b = triangle_area(p_v0, p_v1, p_wb);
            let combined_area = (area_a + area_b).max(1e-8);

            let edge = p_v1 - p_v0;
            let edge_len = edge.length();
            if edge_len < 1e-10 {
                continue;
            }

            let inv_e2 = 1.0 / (edge_len * edge_len);

            // Scalar coefficients derived from the gradient of the dihedral angle.
            // These guarantee translation invariance (sum to EXACTLY 0) for any geometry.
            let c_wa = edge_len / (2.0 * area_a.max(1e-8));
            let c_wb = edge_len / (2.0 * area_b.max(1e-8));

            let dot0_a = (p_v1 - p_wa).dot(edge) * inv_e2;
            let dot0_b = (p_v1 - p_wb).dot(edge) * inv_e2;
            let c_0 = -dot0_a * c_wa - dot0_b * c_wb;

            let dot1_a = (p_wa - p_v0).dot(edge) * inv_e2;
            let dot1_b = (p_wb - p_v0).dot(edge) * inv_e2;
            let c_1 = -dot1_a * c_wa - dot1_b * c_wb;

            // Raw geometric stencil: [c_0, c_1, c_wa, c_wb]
            // Normalized to unit L2 norm so the weight alone controls energy magnitude.
            let raw = [c_0, c_1, c_wa, c_wb];
            let norm = (raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2] + raw[3] * raw[3]).sqrt();
            let stencil = if norm > 1e-10 {
                [raw[0] / norm, raw[1] / norm, raw[2] / norm, raw[3] / norm]
            } else {
                // Fallback to uniform stencil for degenerate geometry
                let s = 0.5_f32;
                [-s, -s, s, s]
            };

            // Weight: match dihedral model magnitude (stiffness * edge_len).
            // The improved curvature estimation comes from the cotangent stencil
            // direction, not from scaling the weight.
            let weight = stiffness * edge_len;

            elements.push(DiscreteShellsElement {
                v0,
                v1,
                wing_a: wa,
                wing_b: wb,
                rest_angle,
                combined_area,
                stencil,
                weight,
            });
        }

        Self {
            elements,
            stiffness,
        }
    }

    /// Build Discrete Shells bending elements with material-derived stiffness.
    ///
    /// Uses the average bending stiffness from `FabricProperties`.
    /// Scales by 100 for appropriate PD energy magnitude (matching dihedral model).
    pub fn from_topology_with_material(
        mesh: &TriangleMesh,
        topology: &Topology,
        properties: &vistio_material::FabricProperties,
    ) -> Self {
        let stiffness = properties.avg_bending_stiffness() * 100.0;
        Self::from_topology(mesh, topology, stiffness)
    }

    /// Build Discrete Shells with anisotropic (warp/weft) bending stiffness.
    ///
    /// For each edge, the bending stiffness is interpolated between warp and weft
    /// based on the edge's alignment with the material warp direction (assumed to
    /// be the local u-axis in the mesh's rest configuration).
    ///
    /// `k_edge = k_warp · cos²(θ) + k_weft · sin²(θ)`
    ///
    /// where θ is the angle between the edge direction and the warp direction.
    pub fn from_topology_with_anisotropic_material(
        mesh: &TriangleMesh,
        topology: &Topology,
        properties: &vistio_material::FabricProperties,
    ) -> Self {
        // Use the average bending stiffness as the base, then modulate per-edge.
        let k_avg = properties.avg_bending_stiffness() * 100.0;
        let k_warp = properties.bending_stiffness_warp;
        let k_weft = properties.bending_stiffness_weft;
        let avg_raw = (k_warp + k_weft) / 2.0;

        let mut data = Self::from_topology(mesh, topology, k_avg);

        // Modulate per-element weight by the edge-to-warp-axis alignment.
        // The weight ratio is (k_edge / k_avg) so the overall weight stays
        // in the same magnitude range.
        if avg_raw > 1e-8 {
            for elem in &mut data.elements {
                let p_v0 = Vec3::new(mesh.pos_x[elem.v0], mesh.pos_y[elem.v0], mesh.pos_z[elem.v0]);
                let p_v1 = Vec3::new(mesh.pos_x[elem.v1], mesh.pos_y[elem.v1], mesh.pos_z[elem.v1]);

                let edge = p_v1 - p_v0;
                let edge_len = edge.length();
                if edge_len < 1e-10 {
                    continue;
                }
                let edge_dir = edge / edge_len;

                // Project edge onto the XZ plane (warp = X, weft = Z)
                let horizontal = Vec3::new(edge_dir.x, 0.0, edge_dir.z);
                let horiz_len = horizontal.length();
                if horiz_len < 1e-10 {
                    // Vertical edge — keep average stiffness (ratio = 1.0)
                    continue;
                }
                let horiz_dir = horizontal / horiz_len;

                let cos_theta = horiz_dir.x.abs();
                let cos2 = cos_theta * cos_theta;
                let sin2 = 1.0 - cos2;

                // Edge aligned with warp → bend with weft stiffness (and vice versa)
                let k_edge = k_weft * cos2 + k_warp * sin2;
                let ratio = k_edge / avg_raw;
                elem.weight *= ratio;
            }
        }

        data
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
    pub fn project(
        &self,
        elem: &DiscreteShellsElement,
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

        // Correction: rotate wing vertices toward the rest configuration.
        // We must determine the orientation of the current wing vertices relative to the edge.
        let e_dir = axis;
        let to_a = p_wa - p_v0;
        let to_b = p_wb - p_v0;
        let p_a = to_a - e_dir * to_a.dot(e_dir);
        let p_b = to_b - e_dir * to_b.dot(e_dir);

        // n_cross points along e_dir if p_a, p_b are oriented positively around e_dir.
        let n_cross = p_a.cross(p_b);
        let sign = if n_cross.dot(e_dir) >= 0.0 { 1.0 } else { -1.0 };

        // angle_diff > 0 means current > rest (we need to fold them together).
        // Since rotating around e_dir by positive angle moves p_a towards p_b (when sign > 0),
        // we scale the rotation by `sign`.
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
