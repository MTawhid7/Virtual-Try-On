// physics/src/collision/preprocessing.rs
use glam::Vec3;

/// A mesh that has been pre-processed for physics collision (smoothed, inflated, etc).
pub struct ProcessedMesh {
    pub vertices: Vec<Vec3>,
    pub normals: Vec<Vec3>,
}

/// Prepares a raw mesh for use as a collider.
/// Steps:
/// 1. Converts flat float buffers to Vec3.
/// 2. Performs Laplacian Smoothing to remove sharp edges that snag cloth.
/// 3. Computes Face Normals (flat shading style for accurate collisions).
/// 4. Inflates the mesh to create a "safety buffer".
pub fn process_mesh(
    raw_vertices: &[f32],
    indices: &[u32],
    smoothing_iterations: usize,
    inflation_amount: f32
) -> ProcessedMesh {
    let num_verts = raw_vertices.len() / 3;
    let mut vertices = Vec::with_capacity(num_verts);

    // 1. Convert to Vec3
    for i in 0..num_verts {
        vertices.push(Vec3::new(
            raw_vertices[i * 3],
            raw_vertices[i * 3 + 1],
            raw_vertices[i * 3 + 2],
        ));
    }

    // 2. Build Adjacency (Only if smoothing is needed)
    if smoothing_iterations > 0 {
        let mut adj = vec![Vec::new(); num_verts];
        let num_triangles = indices.len() / 3;

        for i in 0..num_triangles {
            let idx0 = indices[i * 3] as usize;
            let idx1 = indices[i * 3 + 1] as usize;
            let idx2 = indices[i * 3 + 2] as usize;
            add_neighbor(&mut adj, idx0, idx1);
            add_neighbor(&mut adj, idx0, idx2);
            add_neighbor(&mut adj, idx1, idx2);
        }

        // 3. Laplacian Smoothing
        let lambda = 0.5;
        for _ in 0..smoothing_iterations {
            let old_verts = vertices.clone();
            for i in 0..num_verts {
                let neighbors = &adj[i];
                if neighbors.is_empty() { continue; }
                let mut sum = Vec3::ZERO;
                for &n_idx in neighbors { sum += old_verts[n_idx]; }
                let avg = sum / (neighbors.len() as f32);
                vertices[i] = old_verts[i].lerp(avg, lambda);
            }
        }
    }

    // 4. Compute Normals
    let mut normals = vec![Vec3::ZERO; num_verts];
    let num_triangles = indices.len() / 3;
    for i in 0..num_triangles {
        let idx0 = indices[i * 3] as usize;
        let idx1 = indices[i * 3 + 1] as usize;
        let idx2 = indices[i * 3 + 2] as usize;

        let v0 = vertices[idx0];
        let v1 = vertices[idx1];
        let v2 = vertices[idx2];

        let edge1 = v1 - v0;
        let edge2 = v2 - v0;
        let face_normal = edge1.cross(edge2);

        normals[idx0] += face_normal;
        normals[idx1] += face_normal;
        normals[idx2] += face_normal;
    }

    for n in &mut normals {
        *n = n.normalize_or_zero();
    }

    // 5. Inflation
    if inflation_amount.abs() > 1e-6 {
        for i in 0..num_verts {
            vertices[i] += normals[i] * inflation_amount;
        }
    }

    ProcessedMesh { vertices, normals }
}

fn add_neighbor(adj: &mut Vec<Vec<usize>>, a: usize, b: usize) {
    if !adj[a].contains(&b) { adj[a].push(b); }
    if !adj[b].contains(&a) { adj[b].push(a); }
}