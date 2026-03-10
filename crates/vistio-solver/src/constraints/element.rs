//! FEM element computation for Projective Dynamics.
//!
//! Precomputes per-triangle rest-state data and provides the ARAP (co-rotational)
//! local projection used in the PD local step. Each triangle element stores its
//! rest-state edge matrix inverse, area, and stiffness weight.
//!
//! ## PD Local Step
//!
//! For each triangle, the local step finds the closest valid configuration:
//! 1. Compute deformation gradient F = Ds · Dm⁻¹
//! 2. Polar decomposition F = R · S
//! 3. Project: target = R · rest_edges (rotation-aware, resists stretch)
//!
//! ## Material-Aware Mode
//!
//! When constructed with `from_mesh_with_material()`, per-element stiffness
//! is derived from `FabricProperties`. Use `project_with_model()` to delegate
//! projection to a pluggable `ConstitutiveModel`.

use vistio_math::Vec2;
use vistio_math::Vec3;
use vistio_math::decomposition::{deformation_gradient, polar_decomposition_3x2};
use vistio_mesh::TriangleMesh;

/// Precomputed rest-state data for a single triangle element.
#[derive(Debug, Clone, Copy)]
pub struct RestTriangle {
    /// Triangle vertex indices (into the global vertex buffer).
    pub indices: [usize; 3],
    /// Inverse of the rest-state edge matrix Dm (2×2, column-major).
    /// Dm = [e1, e2] where e1 = p1-p0, e2 = p2-p0 (projected to 2D).
    pub dm_inv: [f32; 4],
    /// Rest-state area (half the cross product magnitude of the edge vectors).
    pub rest_area: f32,
    /// Stiffness weight for this element in the PD system matrix.
    /// weight = stiffness * area (energy contribution per element).
    pub weight: f32,
}

/// Collection of all triangle elements with precomputed rest-state data.
pub struct ElementData {
    /// Per-triangle element data.
    pub elements: Vec<RestTriangle>,
    /// Global stiffness parameter (Young's modulus proxy).
    pub stiffness: f32,
}

impl ElementData {
    /// Compute rest-state data for all triangles in a mesh.
    ///
    /// # Arguments
    /// * `mesh` — Triangle mesh (vertex positions + triangle indices)
    /// * `stiffness` — Global stiffness parameter (higher = stiffer fabric)
    pub fn from_mesh(mesh: &TriangleMesh, stiffness: f32) -> Self {
        let tri_count = mesh.triangle_count();
        let mut elements = Vec::with_capacity(tri_count);

        for t in 0..tri_count {
            let idx_base = t * 3;
            let i0 = mesh.indices[idx_base] as usize;
            let i1 = mesh.indices[idx_base + 1] as usize;
            let i2 = mesh.indices[idx_base + 2] as usize;

            let p0 = Vec3::new(mesh.pos_x[i0], mesh.pos_y[i0], mesh.pos_z[i0]);
            let p1 = Vec3::new(mesh.pos_x[i1], mesh.pos_y[i1], mesh.pos_z[i1]);
            let p2 = Vec3::new(mesh.pos_x[i2], mesh.pos_y[i2], mesh.pos_z[i2]);

            let e1 = p1 - p0;
            let e2 = p2 - p0;

            // Rest-state area = 0.5 * ||e1 × e2||
            let cross = e1.cross(e2);
            let area = 0.5 * cross.length();

            let dm_inv = compute_dm_inv(e1, e2);

            let weight = stiffness * area;

            elements.push(RestTriangle {
                indices: [i0, i1, i2],
                dm_inv,
                rest_area: area,
                weight,
            });
        }

        Self {
            elements,
            stiffness,
        }
    }

    /// Compute rest-state data using material properties for stiffness.
    ///
    /// Derives per-element stiffness from `FabricProperties`:
    /// - Average stretch stiffness (warp/weft) → element weight
    /// - Scales by 1000 for appropriate PD energy magnitude
    ///
    /// # Arguments
    /// * `mesh` — Triangle mesh
    /// * `properties` — Physical fabric properties (KES-derived)
    pub fn from_mesh_with_material(
        mesh: &TriangleMesh,
        properties: &vistio_material::FabricProperties,
    ) -> Self {
        let stiffness = properties.avg_stretch_stiffness() * 10.0; // scale down slightly so it balances with the local projection instead of overpowering it
        Self::from_mesh(mesh, stiffness)
    }

    /// Returns the number of elements.
    pub fn len(&self) -> usize {
        self.elements.len()
    }

    /// Returns true if there are no elements.
    pub fn is_empty(&self) -> bool {
        self.elements.is_empty()
    }

    /// Compute the local projection for one element (hardcoded ARAP).
    ///
    /// Given current vertex positions, computes the deformation gradient F,
    /// performs polar decomposition F = R·S, and returns the projected
    /// target positions for the three vertices.
    ///
    /// Returns `(target_p0, target_p1, target_p2)`.
    pub fn project(
        &self,
        elem: &RestTriangle,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
    ) -> (Vec3, Vec3, Vec3) {
        let [i0, i1, i2] = elem.indices;

        let p0 = Vec3::new(pos_x[i0], pos_y[i0], pos_z[i0]);
        let p1 = Vec3::new(pos_x[i1], pos_y[i1], pos_z[i1]);
        let p2 = Vec3::new(pos_x[i2], pos_y[i2], pos_z[i2]);

        let f = deformation_gradient(p0, p1, p2, elem.dm_inv);
        let pd = polar_decomposition_3x2(&f);

        self.reconstruct_from_target_f(&pd.rotation, elem, p0, p1, p2)
    }

    /// Compute the local projection using a pluggable ConstitutiveModel.
    ///
    /// Computes the deformation gradient F, passes it to the model's `project()`,
    /// then reconstructs target positions from the projected deformation gradient.
    ///
    /// This is the Tier 2+ entry point — different models (co-rotational,
    /// isotropic, anisotropic) produce different target gradients.
    pub fn project_with_model(
        &self,
        elem: &RestTriangle,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        model: &dyn vistio_material::ConstitutiveModel,
    ) -> (Vec3, Vec3, Vec3) {
        let [i0, i1, i2] = elem.indices;

        let p0 = Vec3::new(pos_x[i0], pos_y[i0], pos_z[i0]);
        let p1 = Vec3::new(pos_x[i1], pos_y[i1], pos_z[i1]);
        let p2 = Vec3::new(pos_x[i2], pos_y[i2], pos_z[i2]);

        let f = deformation_gradient(p0, p1, p2, elem.dm_inv);

        // Recover effective stiffness: weight = stiffness * area
        let effective_stiffness = elem.weight / elem.rest_area.max(1e-12);
        let projected = model.project(&f, elem.rest_area, effective_stiffness);

        self.reconstruct_from_target_f(&projected.target_f, elem, p0, p1, p2)
    }

    /// Reconstruct target vertex positions from a projected 3×2 deformation gradient.
    ///
    /// Given a target F (rotation for ARAP, or any projected gradient from a model),
    /// computes positions by applying F to the rest-state edges and anchoring
    /// at the current centroid.
    fn reconstruct_from_target_f(
        &self,
        target_f: &vistio_math::mat3x2::Mat3x2,
        elem: &RestTriangle,
        p0: Vec3,
        p1: Vec3,
        p2: Vec3,
    ) -> (Vec3, Vec3, Vec3) {
        let centroid = (p0 + p1 + p2) / 3.0;

        // Reconstruct rest-state edges from Dm_inv
        let det = elem.dm_inv[0] * elem.dm_inv[3] - elem.dm_inv[1] * elem.dm_inv[2];
        let (rest_e1_2d, rest_e2_2d) = if det.abs() > 1e-12 {
            let inv_det = 1.0 / det;
            let dm00 = elem.dm_inv[3] * inv_det;
            let dm01 = -elem.dm_inv[2] * inv_det;
            let dm10 = -elem.dm_inv[1] * inv_det;
            let dm11 = elem.dm_inv[0] * inv_det;
            (Vec2::new(dm00, dm10), Vec2::new(dm01, dm11))
        } else {
            return (p0, p1, p2);
        };

        // Apply target_f to rest edges
        let re1 = target_f.col0 * rest_e1_2d.x + target_f.col1 * rest_e1_2d.y;
        let re2 = target_f.col0 * rest_e2_2d.x + target_f.col1 * rest_e2_2d.y;

        // Rest centroid offset from vertex 0
        let rest_c = (re1 + re2) / 3.0;

        // Projected positions anchored at current centroid
        let p0_proj = centroid - rest_c;
        let p1_proj = p0_proj + re1;
        let p2_proj = p0_proj + re2;

        (p0_proj, p1_proj, p2_proj)
    }
}

/// Compute the 2×2 inverse of the rest-state edge matrix.
///
/// Projects 3D edge vectors into the triangle's local 2D frame to build
/// a 2×2 matrix Dm, then inverts it.
///
/// Local frame construction:
/// - u-axis: normalized e1
/// - v-axis: component of e2 perpendicular to e1 (in the triangle plane)
fn compute_dm_inv(e1: Vec3, e2: Vec3) -> [f32; 4] {
    let len_e1 = e1.length();
    if len_e1 < 1e-10 {
        return [1.0, 0.0, 0.0, 1.0]; // Degenerate fallback
    }

    let u = e1 / len_e1;
    let n = e1.cross(e2);
    let n_len = n.length();
    if n_len < 1e-10 {
        return [1.0, 0.0, 0.0, 1.0]; // Degenerate fallback
    }

    let v = n.cross(u).normalize();

    // Project edges into local 2D frame:
    // Dm = [[e1·u, e2·u], [e1·v, e2·v]]
    let dm00 = e1.dot(u); // = len_e1
    let dm10 = e1.dot(v); // = 0 by construction
    let dm01 = e2.dot(u);
    let dm11 = e2.dot(v);

    // Invert the 2×2 matrix (column-major storage)
    let det = dm00 * dm11 - dm01 * dm10;
    if det.abs() < 1e-10 {
        return [1.0, 0.0, 0.0, 1.0]; // Degenerate fallback
    }

    let inv_det = 1.0 / det;
    [
        dm11 * inv_det,   // [0,0]
        -dm10 * inv_det,  // [1,0]
        -dm01 * inv_det,  // [0,1]
        dm00 * inv_det,   // [1,1]
    ]
}
