//! Continuous Collision Detection step computation.
//!
//! Computes the maximum safe step fraction to prevent any vertex from
//! tunneling through a collision boundary.

use super::CollisionPipeline;

impl CollisionPipeline {
    /// Compute the maximum safe step fraction to prevent any vertex from passing
    /// through a collision boundary explicitly modelled by IPC.
    #[allow(clippy::too_many_arguments)]
    pub fn compute_ccd_step(
        &self,
        mesh_indices: &[u32],
        prev_x: &[f32], prev_y: &[f32], prev_z: &[f32],
        new_x: &[f32], new_y: &[f32], new_z: &[f32],
        padding: f32,
    ) -> f32 {
        let mut min_toi = 1.0_f32;

        // 1. Self Collision
        if self.self_collision.is_some() {
            let tri_pairs = self.broad.query_triangle_pairs();
            let tri_pairs_usize: Vec<(usize, usize)> = tri_pairs.into_iter().map(|p| (p.ta as usize, p.tb as usize)).collect();
            let toi = crate::ccd::compute_max_step_size(
                prev_x, prev_y, prev_z,
                new_x, new_y, new_z,
                mesh_indices, &tri_pairs_usize
            );
            min_toi = min_toi.min(toi);
        }

        // 2. Ground Plane
        if let Some(ref ground) = self.ground {
            let toi = ground.compute_ccd_step(prev_y, new_y, padding);
            min_toi = min_toi.min(toi);
        }

        // 3. Sphere Collider
        if let Some(ref sphere) = self.sphere {
            let toi = sphere.compute_ccd_step(prev_x, prev_y, prev_z, new_x, new_y, new_z, padding);
            min_toi = min_toi.min(toi);
        }

        // 4. Cylinder Collider
        if let Some(ref cylinder) = self.cylinder {
            let toi = cylinder.compute_ccd_step(prev_x, prev_y, prev_z, new_x, new_y, new_z, padding);
            min_toi = min_toi.min(toi);
        }

        // 5. Box Collider
        if let Some(ref box_col) = self.box_collider {
            let toi = box_col.compute_ccd_step(prev_x, prev_y, prev_z, new_x, new_y, new_z, padding);
            min_toi = min_toi.min(toi);
        }

        min_toi.max(1e-6)
    }
}
