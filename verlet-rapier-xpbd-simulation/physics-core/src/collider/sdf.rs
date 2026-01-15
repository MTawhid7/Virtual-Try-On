// physics-core/src/collider/sdf.rs
use crate::data::PhysicsData;

pub struct SDFCollider {
    // Spine
    pub spine_radius: f32,
    pub spine_base: [f32; 3],
    pub spine_top: [f32; 3],

    // Shoulders
    pub shoulder_radius: f32,
    pub shoulder_left: [f32; 3],
    pub shoulder_right: [f32; 3],
}

impl SDFCollider {
    pub fn new() -> Self {
        SDFCollider {
            // Spine: Thinner and slightly shorter to fit inside neck hole
            spine_radius: 0.09,
            spine_base: [0.0, 0.90, 0.0],
            spine_top: [0.0, 1.55, 0.0],

            // Shoulders: Lower and narrower to allow shirt to land ON them
            shoulder_radius: 0.10,
            // Y=1.42 means top surface is 1.52. Shirt top is 1.56.
            // X=0.19 means tip is 0.29. Shirt width is 0.31.
            shoulder_left: [-0.19, 1.42, 0.0],
            shoulder_right: [0.19, 1.42, 0.0],
        }
    }

    // Helper: Distance to a capsule segment
    fn dist_capsule(p: [f32; 3], a: [f32; 3], b: [f32; 3], r: f32) -> (f32, [f32; 3]) {
        let pa = [p[0]-a[0], p[1]-a[1], p[2]-a[2]];
        let ba = [b[0]-a[0], b[1]-a[1], b[2]-a[2]];

        let ba_mag_sq = ba[0]*ba[0] + ba[1]*ba[1] + ba[2]*ba[2];
        let dot = pa[0]*ba[0] + pa[1]*ba[1] + pa[2]*ba[2];

        let h = (dot / ba_mag_sq).clamp(0.0, 1.0);

        let closest = [
            a[0] + ba[0] * h,
            a[1] + ba[1] * h,
            a[2] + ba[2] * h
        ];

        let dx = p[0] - closest[0];
        let dy = p[1] - closest[1];
        let dz = p[2] - closest[2];

        let dist = (dx*dx + dy*dy + dz*dz).sqrt() - r;

        (dist, closest)
    }

    pub fn solve(&self, data: &mut PhysicsData, _dt: f32) {
        // High friction to grab the cloth
        let friction = 0.9;
        let thickness = 0.015;

        for i in 0..data.count {
            if data.inv_mass[i] == 0.0 { continue; }

            data.debug_data[i] = 0.0;

            let idx = i * 3;
            let p = [
                data.positions[idx],
                data.positions[idx + 1],
                data.positions[idx + 2]
            ];

            // Floor
            if p[1] < 0.0 {
                data.positions[idx + 1] = 0.0;
                data.prev_positions[idx] = data.positions[idx];
                data.prev_positions[idx + 2] = data.positions[idx + 2];
                continue;
            }

            // 1. Check Spine
            let (d_spine, c_spine) = Self::dist_capsule(p, self.spine_base, self.spine_top, self.spine_radius);

            // 2. Check Shoulders
            let (d_shoulder, c_shoulder) = Self::dist_capsule(p, self.shoulder_left, self.shoulder_right, self.shoulder_radius);

            // 3. Union (Min Distance)
            let (dist, closest, _radius) = if d_spine < d_shoulder {
                (d_spine, c_spine, self.spine_radius)
            } else {
                (d_shoulder, c_shoulder, self.shoulder_radius)
            };

            // 4. Collision Response
            if dist < thickness {
                data.debug_data[i] = 1.0;

                let dx = p[0] - closest[0];
                let dy = p[1] - closest[1];
                let dz = p[2] - closest[2];
                let len = (dx*dx + dy*dy + dz*dz).sqrt();

                let (nx, ny, nz) = if len > 1e-6 {
                    (dx/len, dy/len, dz/len)
                } else {
                    (0.0, 1.0, 0.0)
                };

                let penetration = thickness - dist;

                data.positions[idx] += nx * penetration;
                data.positions[idx + 1] += ny * penetration;
                data.positions[idx + 2] += nz * penetration;

                // Friction
                let prev_x = data.prev_positions[idx];
                let prev_y = data.prev_positions[idx + 1];
                let prev_z = data.prev_positions[idx + 2];

                let vx = data.positions[idx] - prev_x;
                let vy = data.positions[idx + 1] - prev_y;
                let vz = data.positions[idx + 2] - prev_z;

                data.prev_positions[idx] = data.positions[idx] - (vx * (1.0 - friction));
                data.prev_positions[idx + 1] = data.positions[idx + 1] - (vy * (1.0 - friction));
                data.prev_positions[idx + 2] = data.positions[idx + 2] - (vz * (1.0 - friction));

                // FRICTION LOGIC
                let mut current_friction = friction;

                // HACK: If hitting the top of the shoulder (Normal pointing UP), stick it.
                // This simulates the friction of fabric on skin/hanger.
                if ny > 0.8 {
                    current_friction = 1.0; // Glue
                }

                // Apply Friction
                data.prev_positions[idx] = data.positions[idx] - (vx * (1.0 - current_friction));
                data.prev_positions[idx + 1] = data.positions[idx + 1] - (vy * (1.0 - current_friction));
                data.prev_positions[idx + 2] = data.positions[idx + 2] - (vz * (1.0 - current_friction));
            }
        }
    }
}