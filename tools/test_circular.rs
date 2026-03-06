use vistio_mesh::TriangleMesh;
use vistio_types::MaterialId;

pub fn circular_grid_quad(radius: f32, resolution: usize) -> TriangleMesh {
    let cols = resolution;
    let rows = resolution;
    let width = radius * 2.0;
    let height = radius * 2.0;
    let half_w = width / 2.0;
    let half_h = height / 2.0;

    let mut mesh = TriangleMesh::with_capacity(cols * rows, cols * rows * 2);

    let mut vertex_map = vec![usize::MAX; (cols + 1) * (rows + 1)];
    let mut current_vertex = 0;

    // First pass: add vertices that belong to quads which are at least partially inside
    for j in 0..=rows {
        for i in 0..=cols {
            let u = i as f32 / cols as f32;
            let v = j as f32 / rows as f32;
            let px = -half_w + u * width;
            let py = half_h - v * height;

            // Check if this vertex is inside the circle
            if (px * px + py * py).sqrt() <= radius * 1.01 { // slight epsilon
                mesh.pos_x.push(px);
                mesh.pos_y.push(py);
                mesh.pos_z.push(0.0);
                mesh.normal_x.push(0.0);
                mesh.normal_y.push(0.0);
                mesh.normal_z.push(1.0);
                mesh.uv_u.push(u);
                mesh.uv_v.push(v);

                vertex_map[j * (cols + 1) + i] = current_vertex;
                current_vertex += 1;
            }
        }
    }

    // Second pass: add triangles if all their vertices exist
    for j in 0..rows {
        for i in 0..cols {
            let top_left = j * (cols + 1) + i;
            let top_right = top_left + 1;
            let bot_left = top_left + (cols + 1);
            let bot_right = bot_left + 1;

            let tl_v = vertex_map[top_left];
            let tr_v = vertex_map[top_right];
            let bl_v = vertex_map[bot_left];
            let br_v = vertex_map[bot_right];

            if tl_v != usize::MAX && tr_v != usize::MAX && bl_v != usize::MAX && br_v != usize::MAX {
                if (i + j) % 2 == 0 {
                    mesh.indices.push(tl_v as u32);
                    mesh.indices.push(bl_v as u32);
                    mesh.indices.push(tr_v as u32);

                    mesh.indices.push(tr_v as u32);
                    mesh.indices.push(bl_v as u32);
                    mesh.indices.push(br_v as u32);
                } else {
                    mesh.indices.push(tl_v as u32);
                    mesh.indices.push(bl_v as u32);
                    mesh.indices.push(br_v as u32);

                    mesh.indices.push(tl_v as u32);
                    mesh.indices.push(br_v as u32);
                    mesh.indices.push(tr_v as u32);
                }
                mesh.material_ids.push(MaterialId(0));
                mesh.material_ids.push(MaterialId(0));
            }
        }
    }

    mesh
}

fn main() {
    let mesh = circular_grid_quad(0.15, 30);
    println!("Generated mesh with {} vertices and {} triangles.", mesh.vertex_count(), mesh.triangle_count());
}
