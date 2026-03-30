// physics/src/systems/constraints/tether/horizontal.rs

use crate::engine::state::PhysicsState;
use std::collections::HashMap;

/// Generates horizontal tethers (Shoulder-to-Shoulder, Torso-to-Torso).
/// Scans the mesh in Z-slices and connects particles on the left side to the right side.
/// This limits the garment's ability to stretch infinitely wide.
pub fn generate(state: &PhysicsState) -> (Vec<[usize; 2]>, Vec<f32>) {
    let mut constraints = Vec::new();
    let mut rest_lengths = Vec::new();

    let mut max_y = f32::MIN;
    for p in &state.positions {
        if p.y > max_y { max_y = p.y; }
    }

    let shoulder_threshold = max_y - 0.15;
    let z_cell_size = 0.04;
    let mut rows: HashMap<i32, Vec<usize>> = HashMap::new();

    for i in 0..state.count {
        let p = state.positions[i];
        if p.y < shoulder_threshold { continue; }

        let cell_z = (p.z / z_cell_size).floor() as i32;
        rows.entry(cell_z).or_insert_with(Vec::new).push(i);
    }

    for (_, indices) in rows {
        if indices.len() < 2 { continue; }

        let mut sorted = indices.clone();
        sorted.sort_by(|&a, &b| {
            state.positions[a].x.partial_cmp(&state.positions[b].x).unwrap()
        });

        let count = sorted.len();
        let steps = count / 2;

        for i in 0..steps {
            let left = sorted[i];
            let right = sorted[count - 1 - i];

            let p1 = state.positions[left];
            let p2 = state.positions[right];
            let dist = p1.distance(p2);

            if dist > 0.15 {
                let n1 = state.normals[left];
                let n2 = state.normals[right];

                if n1.dot(n2) > 0.5 {
                    constraints.push([left, right]);
                    rest_lengths.push(dist);
                }
            }
        }
    }

    (constraints, rest_lengths)
}