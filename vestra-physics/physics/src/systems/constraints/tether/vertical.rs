// physics/src/systems/constraints/tether/vertical.rs

use crate::engine::state::PhysicsState;
use std::collections::HashMap;

/// Generates vertical tethers (Shoulder-to-Hem).
/// Sorts particles into vertical columns and connects top-point to bottom-point by UV or normal alignment.
/// Prevents the "Super-Elastic" effect where gravity stretches the cloth down like gum.
pub fn generate(state: &PhysicsState) -> (Vec<[usize; 2]>, Vec<f32>) {
    let mut constraints = Vec::new();
    let mut rest_lengths = Vec::new();

    let cell_size = 0.03;
    let mut columns: HashMap<(i32, i32), Vec<usize>> = HashMap::new();

    for i in 0..state.count {
        let p = state.positions[i];
        let cell_x = (p.x / cell_size).floor() as i32;
        let cell_z = (p.z / cell_size).floor() as i32;

        columns.entry((cell_x, cell_z))
            .or_insert_with(Vec::new)
            .push(i);
    }

    for (_, indices) in columns {
        if indices.len() < 2 { continue; }

        let mut sorted = indices.clone();
        sorted.sort_by(|&a, &b| {
            state.positions[b].y.partial_cmp(&state.positions[a].y).unwrap()
        });

        let top_idx = sorted[0];
        let top_n = state.normals[top_idx];

        for &bottom_idx in sorted.iter().rev() {
            if top_idx == bottom_idx { continue; }
            let bottom_n = state.normals[bottom_idx];

            if top_n.dot(bottom_n) > 0.8 {
                let p1 = state.positions[top_idx];
                let p2 = state.positions[bottom_idx];
                let dist = p1.distance(p2);

                if dist > 0.10 {
                    constraints.push([top_idx, bottom_idx]);
                    rest_lengths.push(dist);
                    break;
                }
            }
        }
    }

    (constraints, rest_lengths)
}