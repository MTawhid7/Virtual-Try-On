//! Vertex normal computation from triangle mesh data.
//!
//! Computes area-weighted vertex normals by accumulating
//! face normals from each adjacent triangle.

use crate::mesh::TriangleMesh;

/// Recompute vertex normals from triangle geometry (area-weighted).
///
/// Each triangle's face normal (weighted by its area) is accumulated
/// at each vertex. The result is normalized. This produces smooth
/// normals that reflect the local surface orientation.
///
/// This modifies the mesh's `normal_x`, `normal_y`, `normal_z` arrays in place.
pub fn compute_vertex_normals(mesh: &mut TriangleMesh) {
    let n = mesh.vertex_count();

    // Zero out normals
    for i in 0..n {
        mesh.normal_x[i] = 0.0;
        mesh.normal_y[i] = 0.0;
        mesh.normal_z[i] = 0.0;
    }

    // Accumulate area-weighted face normals
    let tri_count = mesh.triangle_count();
    for t in 0..tri_count {
        let [ia, ib, ic] = mesh.triangle(t);
        let a = ia as usize;
        let b = ib as usize;
        let c = ic as usize;

        // Edge vectors
        let e1x = mesh.pos_x[b] - mesh.pos_x[a];
        let e1y = mesh.pos_y[b] - mesh.pos_y[a];
        let e1z = mesh.pos_z[b] - mesh.pos_z[a];

        let e2x = mesh.pos_x[c] - mesh.pos_x[a];
        let e2y = mesh.pos_y[c] - mesh.pos_y[a];
        let e2z = mesh.pos_z[c] - mesh.pos_z[a];

        // Cross product (area-weighted normal, magnitude = 2 × triangle area)
        let nx = e1y * e2z - e1z * e2y;
        let ny = e1z * e2x - e1x * e2z;
        let nz = e1x * e2y - e1y * e2x;

        // Accumulate at each vertex
        mesh.normal_x[a] += nx;
        mesh.normal_y[a] += ny;
        mesh.normal_z[a] += nz;

        mesh.normal_x[b] += nx;
        mesh.normal_y[b] += ny;
        mesh.normal_z[b] += nz;

        mesh.normal_x[c] += nx;
        mesh.normal_y[c] += ny;
        mesh.normal_z[c] += nz;
    }

    // Normalize
    for i in 0..n {
        let x = mesh.normal_x[i];
        let y = mesh.normal_y[i];
        let z = mesh.normal_z[i];
        let len = (x * x + y * y + z * z).sqrt();
        if len > 1e-10 {
            let inv = 1.0 / len;
            mesh.normal_x[i] = x * inv;
            mesh.normal_y[i] = y * inv;
            mesh.normal_z[i] = z * inv;
        }
    }
}

