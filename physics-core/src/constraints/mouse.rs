// physics-core/src/constraints/mouse.rs
use crate::data::PhysicsData;

pub struct MouseConstraint {
    pub active: bool,
    // Store (index, weight) tuples. Weight 0.0 to 1.0.
    pub grabbed_particles: Vec<(usize, f32)>,
    pub target: [f32; 3],
    pub stiffness: f32,
}

impl MouseConstraint {
    pub fn new() -> Self {
        MouseConstraint {
            active: false,
            grabbed_particles: Vec::new(),
            target: [0.0; 3],
            stiffness: 0.1,
        }
    }

    // NEW: Grab a radius around a center point
    pub fn grab(&mut self, data: &PhysicsData, center_idx: usize, x: f32, y: f32, z: f32, radius: f32) {
        self.active = true;
        self.target = [x, y, z];
        self.grabbed_particles.clear();

        // 1. Get center position
        let cx = data.positions[center_idx * 3];
        let cy = data.positions[center_idx * 3 + 1];
        let cz = data.positions[center_idx * 3 + 2];

        // 2. Find neighbors in radius (Brute force is fine for interaction start)
        let r_sq = radius * radius;

        for i in 0..data.count {
            if data.inv_mass[i] == 0.0 { continue; }

            let px = data.positions[i * 3];
            let py = data.positions[i * 3 + 1];
            let pz = data.positions[i * 3 + 2];

            let dx = px - cx;
            let dy = py - cy;
            let dz = pz - cz;
            let dist_sq = dx*dx + dy*dy + dz*dz;

            if dist_sq < r_sq {
                let dist = dist_sq.sqrt();
                // Falloff: 1.0 at center, 0.0 at edge
                // Using a curve (smoothstep-like) feels better than linear
                let t = dist / radius;
                let weight = (1.0 - t).powi(2); // Quadratic falloff

                self.grabbed_particles.push((i, weight));
            }
        }

        // Ensure the center is always included with max weight
        if self.grabbed_particles.is_empty() {
             self.grabbed_particles.push((center_idx, 1.0));
        }
    }

    pub fn update_target(&mut self, x: f32, y: f32, z: f32) {
        self.target = [x, y, z];
    }

    pub fn release(&mut self) {
        self.active = false;
        self.grabbed_particles.clear();
    }

    pub fn solve(&self, data: &mut PhysicsData, dt: f32) {
        if !self.active { return; }

        let compliance = 0.00001;
        let alpha = compliance / (dt * dt);

        // Iterate over ALL grabbed particles
        for &(i, weight) in &self.grabbed_particles {
            let idx = i * 3;
            let w = data.inv_mass[i];
            if w == 0.0 { continue; }

            let px = data.positions[idx];
            let py = data.positions[idx + 1];
            let pz = data.positions[idx + 2];

            // Calculate vector to target
            // Note: We apply the offset relative to where the particle started?
            // Simplified: Pull everyone to the single mouse target, but scaled by weight.
            // Better: Pull them towards (Target + Offset), but for now, pulling to Target
            // with weight creates a "pinching" effect which is acceptable.

            let dx = self.target[0] - px;
            let dy = self.target[1] - py;
            let dz = self.target[2] - pz;

            let dist = (dx*dx + dy*dy + dz*dz).sqrt();
            if dist < 1e-6 { continue; }

            // Scale stiffness by weight (Outer particles are looser)
            let effective_alpha = alpha / weight;
            let scale = w / (w + effective_alpha);

            data.positions[idx] += dx * scale;
            data.positions[idx + 1] += dy * scale;
            data.positions[idx + 2] += dz * scale;
        }
    }
}