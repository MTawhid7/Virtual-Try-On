// physics-core/src/constraints/distance.rs
use crate::data::PhysicsData;

pub struct TetherConstraint {
    indices: Vec<usize>,      // [particle_idx, anchor_idx]
    max_distances: Vec<f32>,
}

impl TetherConstraint {
    pub fn new(data: &PhysicsData) -> Self {
        let mut indices = Vec::new();
        let mut max_distances = Vec::new();

        // 1. Find Anchors (Pinned vertices)
        let mut anchors = Vec::new();
        for i in 0..data.count {
            if data.inv_mass[i] == 0.0 {
                anchors.push(i);
            }
        }

        if anchors.is_empty() {
            return TetherConstraint { indices, max_distances };
        }

        // 2. Create Tether for every dynamic particle
        for i in 0..data.count {
            if data.inv_mass[i] == 0.0 { continue; }

            // Find closest anchor (Euclidean approximation)
            // In V3 we used Geodesic, but Euclidean is a good start for Rust MVP
            let mut best_anchor = 0;
            let mut min_dist = f32::MAX;

            let px = data.positions[i*3];
            let py = data.positions[i*3+1];
            let pz = data.positions[i*3+2];

            for &anchor in &anchors {
                let ax = data.positions[anchor*3];
                let ay = data.positions[anchor*3+1];
                let az = data.positions[anchor*3+2];

                let dx = px - ax;
                let dy = py - ay;
                let dz = pz - az;
                let d2 = dx*dx + dy*dy + dz*dz;

                if d2 < min_dist {
                    min_dist = d2;
                    best_anchor = anchor;
                }
            }

            // 3. Add Constraint
            // Allow 10% slack so it doesn't look rigid
            let slack = 1.10;
            let limit = min_dist.sqrt() * slack;

            indices.push(i);
            indices.push(best_anchor);
            max_distances.push(limit);
        }

        crate::console_log!("[Rust] Tethers Generated: {}", indices.len() / 2);

        TetherConstraint {
            indices,
            max_distances,
        }
    }

    pub fn solve(&self, data: &mut PhysicsData) {
        // Tethers are "Hard Limits" (Stiffness = 1.0)
        // We project them directly.

        for i in 0..self.max_distances.len() {
            let particle_idx = self.indices[i * 2];
            let anchor_idx = self.indices[i * 2 + 1];
            let max_dist = self.max_distances[i];

            let w = data.inv_mass[particle_idx];
            if w == 0.0 { continue; } // Should not happen for particle

            let idx_p = particle_idx * 3;
            let idx_a = anchor_idx * 3;

            let dx = data.positions[idx_p] - data.positions[idx_a];
            let dy = data.positions[idx_p+1] - data.positions[idx_a+1];
            let dz = data.positions[idx_p+2] - data.positions[idx_a+2];

            let dist_sq = dx*dx + dy*dy + dz*dz;

            if dist_sq > max_dist * max_dist {
                let dist = dist_sq.sqrt();
                let correction = (dist - max_dist) / dist; // Normalized scalar

                // Move particle towards anchor
                // We assume anchor is static (mass=0), so we move particle 100%
                data.positions[idx_p] -= dx * correction;
                data.positions[idx_p+1] -= dy * correction;
                data.positions[idx_p+2] -= dz * correction;
            }
        }
    }
}