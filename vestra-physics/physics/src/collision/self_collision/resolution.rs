// physics/src/collision/self_collision/resolution.rs

//! Phase 3: SIMD-accelerated collision resolution.

use crate::engine::state::PhysicsState;
use crate::utils::simd::{F32x4, Vec3x4};
use glam::Vec4;
use super::SelfCollision;

impl SelfCollision {
    /// Phase 3: Resolve collisions in batches (SIMD-accelerated).
    pub(crate) fn resolve_batched(&self, state: &mut PhysicsState) {
        let stiffness = self.config.stiffness;
        let thickness = self.config.thickness;

        for b in 0..(self.batch_offsets.len().saturating_sub(1)) {
            let start = self.batch_offsets[b];
            let end = self.batch_offsets[b + 1];
            let count = end - start;

            // SIMD: process 4 pairs at a time
            let chunks = count / 4;
            let remainder = count % 4;

            for chunk in 0..chunks {
                let base = start + chunk * 4;
                self.resolve_simd_4(state, base, stiffness, thickness);
            }

            // Scalar remainder
            for k in (start + chunks * 4)..(start + chunks * 4 + remainder) {
                self.resolve_single(state, k, stiffness, thickness);
            }
        }
    }

    /// SIMD-accelerated resolution for 4 collision pairs.
    #[inline(always)]
    pub(crate) fn resolve_simd_4(&self, state: &mut PhysicsState, base: usize, stiffness: f32, thickness: f32) {
        let p0 = &self.collision_pairs[base];
        let p1 = &self.collision_pairs[base + 1];
        let p2 = &self.collision_pairs[base + 2];
        let p3 = &self.collision_pairs[base + 3];

        let i0 = p0.i as usize; let j0 = p0.j as usize;
        let i1 = p1.i as usize; let j1 = p1.j as usize;
        let i2 = p2.i as usize; let j2 = p2.j as usize;
        let i3 = p3.i as usize; let j3 = p3.j as usize;

        // Load positions
        let pos_i = Vec3x4::from_vec4s(
            state.positions[i0],
            state.positions[i1],
            state.positions[i2],
            state.positions[i3],
        );
        let pos_j = Vec3x4::from_vec4s(
            state.positions[j0],
            state.positions[j1],
            state.positions[j2],
            state.positions[j3],
        );

        // Load inverse masses
        let w_i = F32x4::new(
            state.inv_mass[i0],
            state.inv_mass[i1],
            state.inv_mass[i2],
            state.inv_mass[i3],
        );
        let w_j = F32x4::new(
            state.inv_mass[j0],
            state.inv_mass[j1],
            state.inv_mass[j2],
            state.inv_mass[j3],
        );

        let delta = pos_i.sub(pos_j);
        let dist_sq = delta.length_squared();
        let dist = dist_sq.sqrt();

        // Overlap = thickness - dist
        let overlap = F32x4::splat(thickness).sub(dist);

        // Only apply if overlap > 0
        let zero = F32x4::splat(0.0);
        let positive_overlap = overlap.max(zero);

        // Normal = delta / dist
        let safe_dist = dist.max(F32x4::splat(1e-8));
        let normal = delta.div_scalar(safe_dist);

        // Correction magnitude = overlap * stiffness
        let correction_mag = positive_overlap.mul(F32x4::splat(stiffness));

        // Weight ratios
        let w_sum = w_i.add(w_j);
        let safe_w_sum = w_sum.max(F32x4::splat(1e-8));
        let ratio_i = w_i.div(safe_w_sum);
        let ratio_j = w_j.div(safe_w_sum);

        let corr_i = normal.mul_scalar(correction_mag.mul(ratio_i));
        let corr_j = normal.mul_scalar(correction_mag.mul(ratio_j));

        // Apply corrections
        let mask_wi = w_i.gt_mask(zero);
        let mask_wj = w_j.gt_mask(zero);

        if mask_wi.lane0().to_bits() != 0 {
            state.positions[i0] += corr_i.extract_lane0();
        }
        if mask_wj.lane0().to_bits() != 0 {
            state.positions[j0] -= corr_j.extract_lane0();
        }

        if mask_wi.lane1().to_bits() != 0 {
            state.positions[i1] += corr_i.extract_lane1();
        }
        if mask_wj.lane1().to_bits() != 0 {
            state.positions[j1] -= corr_j.extract_lane1();
        }

        if mask_wi.lane2().to_bits() != 0 {
            state.positions[i2] += corr_i.extract_lane2();
        }
        if mask_wj.lane2().to_bits() != 0 {
            state.positions[j2] -= corr_j.extract_lane2();
        }

        if mask_wi.lane3().to_bits() != 0 {
            state.positions[i3] += corr_i.extract_lane3();
        }
        if mask_wj.lane3().to_bits() != 0 {
            state.positions[j3] -= corr_j.extract_lane3();
        }
    }

    /// Scalar fallback for single pair resolution.
    #[inline(always)]
    pub(crate) fn resolve_single(&self, state: &mut PhysicsState, k: usize, stiffness: f32, thickness: f32) {
        let pair = &self.collision_pairs[k];
        let i = pair.i as usize;
        let j = pair.j as usize;

        let p_i = state.positions[i].truncate();
        let p_j = state.positions[j].truncate();
        let delta = p_i - p_j;
        let dist = delta.length();

        if dist < 1e-9 { return; }

        let overlap = thickness - dist;
        if overlap <= 0.0 { return; }

        let normal = delta / dist;
        let correction = normal * overlap * stiffness;

        let w1 = state.inv_mass[i];
        let w2 = state.inv_mass[j];
        let w_sum = w1 + w2;

        if w_sum > 0.0 {
            let ratio1 = w1 / w_sum;
            let ratio2 = w2 / w_sum;

            if w1 > 0.0 {
                state.positions[i] += Vec4::from((correction * ratio1, 0.0));
            }
            if w2 > 0.0 {
                state.positions[j] -= Vec4::from((correction * ratio2, 0.0));
            }
        }
    }
}
