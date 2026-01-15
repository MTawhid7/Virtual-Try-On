// physics-core/src/constraints/distance.rs
use crate::data::PhysicsData;

pub struct DistanceConstraint {
    constraints: Vec<usize>,
    rest_lengths: Vec<f32>,
}

impl DistanceConstraint {
    pub fn new(data: &PhysicsData) -> Self {
        let mut constraints = Vec::new();
        let mut rest_lengths = Vec::new();

        let indices = &data.indices;
        let num_triangles = indices.len() / 3;

        for i in 0..num_triangles {
            let idx0 = indices[i * 3] as usize;
            let idx1 = indices[i * 3 + 1] as usize;
            let idx2 = indices[i * 3 + 2] as usize;

            DistanceConstraint::add_edge(idx0, idx1, &data.positions, &mut constraints, &mut rest_lengths);
            DistanceConstraint::add_edge(idx1, idx2, &data.positions, &mut constraints, &mut rest_lengths);
            DistanceConstraint::add_edge(idx2, idx0, &data.positions, &mut constraints, &mut rest_lengths);
        }

        // --- DEBUG: Stats ---
        let mut min_len = f32::MAX;
        let mut max_len = f32::MIN;
        let mut sum_len = 0.0;
        for l in &rest_lengths {
            if *l < min_len { min_len = *l; }
            if *l > max_len { max_len = *l; }
            sum_len += *l;
        }
        let avg = sum_len / rest_lengths.len() as f32;
        crate::console_log!("[Rust] Constraints: {}. Min: {:.4}, Max: {:.4}, Avg: {:.4}", rest_lengths.len(), min_len, max_len, avg);
        // --------------------

        DistanceConstraint {
            constraints,
            rest_lengths,
        }
    }

    fn add_edge(i1: usize, i2: usize, pos: &[f32], constraints: &mut Vec<usize>, lengths: &mut Vec<f32>) {
        let idx1 = i1 * 3;
        let idx2 = i2 * 3;
        let dx = pos[idx1] - pos[idx2];
        let dy = pos[idx1 + 1] - pos[idx2 + 1];
        let dz = pos[idx1 + 2] - pos[idx2 + 2];
        let dist = (dx * dx + dy * dy + dz * dz).sqrt();

        // Filter duplicates or zero-length
        if dist < 1e-4 { return; }

        constraints.push(i1);
        constraints.push(i2);
        lengths.push(dist);
    }

    pub fn solve(&self, data: &mut PhysicsData, dt: f32) {
        let compliance = 0.000001; 
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
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();

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