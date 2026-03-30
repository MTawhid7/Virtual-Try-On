// physics/src/utils/normals.rs

use glam::Vec4;

/// Computes vertex normals from triangle mesh data.
/// This is optimized to run in WASM instead of JavaScript, avoiding main-thread blocking.
///
/// Algorithm:
/// 1. Zero out all normals
/// 2. For each triangle, compute face normal and accumulate to each vertex
/// 3. Normalize all vertex normals
///
/// PERFORMANCE: This runs in WASM to avoid the O(N) JavaScript bottleneck on the main thread.
/// It uses area-weighted face normals (cross product of edges) which correctly favors
/// larger triangles and provides more accurate curvature representation.
pub fn compute_vertex_normals(
    positions: &[Vec4],
    indices: &[u32],
    normals: &mut [Vec4]
) {
    let count = normals.len();

    // 1. Zero out normals
    for n in normals.iter_mut() {
        *n = Vec4::ZERO;
    }

    // 2. Accumulate face normals to vertices
    let num_triangles = indices.len() / 3;

    for t in 0..num_triangles {
        let i0 = indices[t * 3] as usize;
        let i1 = indices[t * 3 + 1] as usize;
        let i2 = indices[t * 3 + 2] as usize;

        // Skip invalid indices
        if i0 >= count || i1 >= count || i2 >= count {
            continue;
        }

        let p0 = positions[i0].truncate();
        let p1 = positions[i1].truncate();
        let p2 = positions[i2].truncate();

        // Edge vectors
        let e1 = p1 - p0;
        let e2 = p2 - p0;

        // Face normal (not normalized - magnitude is proportional to area)
        // This gives area-weighted normals which is more accurate for curved surfaces
        let face_normal = e1.cross(e2);
        let face_normal_v4 = Vec4::from((face_normal, 0.0));

        // Accumulate to each vertex of the triangle
        normals[i0] += face_normal_v4;
        normals[i1] += face_normal_v4;
        normals[i2] += face_normal_v4;
    }

    // 3. Normalize all vertex normals
    for n in normals.iter_mut() {
        let n3 = n.truncate();
        let len_sq = n3.length_squared();

        if len_sq > 1e-12 {
            let normalized = n3 / len_sq.sqrt();
            *n = Vec4::from((normalized, 0.0));
        } else {
            // Default to up vector for degenerate cases
            *n = Vec4::new(0.0, 1.0, 0.0, 0.0);
        }
    }
}
