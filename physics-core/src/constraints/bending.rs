// physics-core/src/constraints/bending.rs
use crate::data::PhysicsData;
use std::collections::HashSet;

pub struct BendingConstraint {
    constraints: Vec<usize>,
    rest_lengths: Vec<f32>,
}

impl BendingConstraint {
    pub fn new(data: &PhysicsData) -> Self {
        let mut constraints = Vec::new();
        let mut rest_lengths = Vec::new();

        let indices = &data.indices;
        let num_triangles = indices.len() / 3;

        // 1. Build Adjacency Map
        let mut adj = vec![HashSet::new(); data.count];
        let mut edge_count = 0;
        for i in 0..num_triangles {
            let a = indices[i * 3] as usize;
            let b = indices[i * 3 + 1] as usize;
            let c = indices[i * 3 + 2] as usize;

            if adj[a].insert(b) { edge_count += 1; }
            if adj[a].insert(c) { edge_count += 1; }
            if adj[b].insert(a) { edge_count += 1; }
            if adj[b].insert(c) { edge_count += 1; }
            if adj[c].insert(a) { edge_count += 1; }
            if adj[c].insert(b) { edge_count += 1; }
        }

        crate::console_log!("[Rust] Adjacency Graph Built. Edges: {}", edge_count);

        let mut processed = HashSet::new();
        let mut bending_count = 0;

        for i in 0..data.count {
            for &neighbor in &adj[i] {
                for &far_neighbor in &adj[neighbor] {
                    if i == far_neighbor { continue; }
                    if adj[i].contains(&far_neighbor) { continue; }

                    let (min, max) = if i < far_neighbor { (i, far_neighbor) } else { (far_neighbor, i) };
                    if processed.contains(&(min, max)) { continue; }
                    processed.insert((min, max));

                    BendingConstraint::add_constraint(i, far_neighbor, &data.positions, &mut constraints, &mut rest_lengths);
                    bending_count += 1;
                }
            }
        }

        crate::console_log!("[Rust] Bending Constraints Generated: {}", bending_count);

        BendingConstraint {
            constraints,
            rest_lengths,
        }
    }

    fn add_constraint(i1: usize, i2: usize, pos: &[f32], constraints: &mut Vec<usize>, lengths: &mut Vec<f32>) {
        let idx1 = i1 * 3;
        let idx2 = i2 * 3;
        let dx = pos[idx1] - pos[idx2];
        let dy = pos[idx1 + 1] - pos[idx2 + 1];
        let dz = pos[idx1 + 2] - pos[idx2 + 2];
        let dist = (dx*dx + dy*dy + dz*dz).sqrt();

        constraints.push(i1);
        constraints.push(i2);
        lengths.push(dist);
    }

    pub fn solve(&self, data: &mut PhysicsData, dt: f32) {
        // TUNING: Bending Compliance
        // 0.5 = Soft (Cotton), 0.01 = Stiff (Leather)
        let compliance = 0.005;
        let alpha = compliance / (dt * dt);

        for i in 0..self.rest_lengths.len() {
            let i1 = self.constraints[i * 2];
            let i2 = self.constraints[i * 2 + 1];

            let w1 = data.inv_mass[i1];
            let w2 = data.inv_mass[i2];
            let w_sum = w1 + w2;
            if w_sum == 0.0 { continue; }

            let idx1 = i1 * 3;
            let idx2 = i2 * 3;

            let dx = data.positions[idx1] - data.positions[idx2];
            let dy = data.positions[idx1 + 1] - data.positions[idx2 + 1];
            let dz = data.positions[idx1 + 2] - data.positions[idx2 + 2];
            let dist = (dx*dx + dy*dy + dz*dz).sqrt();

            if dist < 1e-6 { continue; }

            let rest = self.rest_lengths[i];
            let c = dist - rest;
            let lambda = -c / (w_sum + alpha);

            let nx = dx / dist;
            let ny = dy / dist;
            let nz = dz / dist;

            if w1 > 0.0 {
                data.positions[idx1] += nx * lambda * w1;
                data.positions[idx1 + 1] += ny * lambda * w1;
                data.positions[idx1 + 2] += nz * lambda * w1;
            }
            if w2 > 0.0 {
                data.positions[idx2] -= nx * lambda * w2;
                data.positions[idx2 + 1] -= ny * lambda * w2;
                data.positions[idx2 + 2] -= nz * lambda * w2;
            }
        }
    }
}