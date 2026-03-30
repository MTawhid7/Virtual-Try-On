// physics/src/systems/constraints/tether/mod.rs

mod vertical;
mod horizontal;

use crate::engine::state::PhysicsState;
use crate::utils::coloring;
use crate::utils::simd::{F32x4, Vec3x4};

/// Enforces global length limits (Long-Range Attachment).
/// "Tethers" particles to stable anchor points to prevent excessive stretching
/// that local distance constraints cannot prevent alone (the "Super-Elastic" effect).
///
/// OPTIMIZATION: Uses SIMD vectorization for 4-wide parallel processing.
pub struct TetherConstraint {
    pub constraints: Vec<[usize; 2]>,
    pub rest_lengths: Vec<f32>,
    pub batch_offsets: Vec<usize>,
}

impl TetherConstraint {
    /// Generates Vertical and Horizontal tethers based on UV coordinates or topology.
    pub fn new(state: &PhysicsState) -> Self {
        let (v_constraints, v_lengths) = vertical::generate(state);
        let (h_constraints, h_lengths) = horizontal::generate(state);

        let mut raw_constraints = v_constraints;
        raw_constraints.extend(h_constraints);

        let mut raw_rest_lengths = v_lengths;
        raw_rest_lengths.extend(h_lengths);

        let (sorted_indices, batch_offsets) = coloring::color_constraints(&raw_constraints, state.count);

        let mut constraints = Vec::with_capacity(raw_constraints.len());
        let mut rest_lengths = Vec::with_capacity(raw_constraints.len());

        for idx in sorted_indices {
            constraints.push(raw_constraints[idx]);
            rest_lengths.push(raw_rest_lengths[idx]);
        }

        Self {
            constraints,
            rest_lengths,
            batch_offsets,
        }
    }

    /// Solves tether constraints using SIMD vectorization.
    /// OPTIMIZATION: Processes 4 constraints at a time.
    #[inline(never)]
    pub fn solve(&self, state: &mut PhysicsState, omega: f32, _dt: f32) {
        for b in 0..(self.batch_offsets.len() - 1) {
            let start = self.batch_offsets[b];
            let end = self.batch_offsets[b + 1];
            let count = end - start;

            let chunks = count / 4;
            let remainder = count % 4;

            for chunk in 0..chunks {
                let base = start + chunk * 4;
                self.solve_simd_4(state, base, omega);
            }

            for k in (start + chunks * 4)..(start + chunks * 4 + remainder) {
                self.solve_single(state, k, omega);
            }
        }
    }

    /// SIMD-accelerated tether solver for 4 constraints.
    #[inline(always)]
    fn solve_simd_4(&self, state: &mut PhysicsState, base: usize, omega: f32) {
        // Load indices
        let [i1_0, i2_0] = self.constraints[base];
        let [i1_1, i2_1] = self.constraints[base + 1];
        let [i1_2, i2_2] = self.constraints[base + 2];
        let [i1_3, i2_3] = self.constraints[base + 3];

        // Load inverse masses
        let w1 = F32x4::new(
            state.inv_mass[i1_0],
            state.inv_mass[i1_1],
            state.inv_mass[i1_2],
            state.inv_mass[i1_3],
        );
        let w2 = F32x4::new(
            state.inv_mass[i2_0],
            state.inv_mass[i2_1],
            state.inv_mass[i2_2],
            state.inv_mass[i2_3],
        );
        let w_sum = w1.add(w2);

        // Load positions
        let p1 = Vec3x4::from_vec4s(
            state.positions[i1_0],
            state.positions[i1_1],
            state.positions[i1_2],
            state.positions[i1_3],
        );
        let p2 = Vec3x4::from_vec4s(
            state.positions[i2_0],
            state.positions[i2_1],
            state.positions[i2_2],
            state.positions[i2_3],
        );

        // Compute delta vectors
        let delta = p1.sub(p2);
        let len = delta.length();

        // Load rest lengths
        let rest = F32x4::new(
            self.rest_lengths[base],
            self.rest_lengths[base + 1],
            self.rest_lengths[base + 2],
            self.rest_lengths[base + 3],
        );

        // Tether only activates if len > rest
        // C = max(0, len - rest)
        let c = len.sub(rest).max(F32x4::splat(0.0));

        // Skip if no constraint violation (handled per-lane in scalar)
        // For SIMD, we process all but zero violations naturally produce zero correction

        // delta_lambda = -C / w_sum (alpha = 0 for infinite stiffness)
        let safe_w_sum = w_sum.max(F32x4::splat(1e-8));
        let delta_lambda = c.neg().div(safe_w_sum);

        // Normalize delta
        let safe_len = len.max(F32x4::splat(1e-8));
        let direction = delta.div_scalar(safe_len);

        // Correction vector with Chebyshev acceleration
        let omega_vec = F32x4::splat(omega);
        let correction_mag = delta_lambda.mul(omega_vec);
        let correction = direction.mul_scalar(correction_mag);

        // Apply corrections
        let corr1 = correction.mul_scalar(w1);
        let corr2 = correction.mul_scalar(w2);

        let mask_w1 = w1.gt_mask(F32x4::splat(0.0));
        let mask_w2 = w2.gt_mask(F32x4::splat(0.0));

        // Only apply if w > 0 and there's a violation
        if mask_w1.lane0().to_bits() != 0 {
            state.positions[i1_0] += corr1.extract_lane0();
        }
        if mask_w2.lane0().to_bits() != 0 {
            state.positions[i2_0] -= corr2.extract_lane0();
        }

        if mask_w1.lane1().to_bits() != 0 {
            state.positions[i1_1] += corr1.extract_lane1();
        }
        if mask_w2.lane1().to_bits() != 0 {
            state.positions[i2_1] -= corr2.extract_lane1();
        }

        if mask_w1.lane2().to_bits() != 0 {
            state.positions[i1_2] += corr1.extract_lane2();
        }
        if mask_w2.lane2().to_bits() != 0 {
            state.positions[i2_2] -= corr2.extract_lane2();
        }

        if mask_w1.lane3().to_bits() != 0 {
            state.positions[i1_3] += corr1.extract_lane3();
        }
        if mask_w2.lane3().to_bits() != 0 {
            state.positions[i2_3] -= corr2.extract_lane3();
        }
    }

    /// Scalar fallback for remainder constraints.
    #[inline(always)]
    fn solve_single(&self, state: &mut PhysicsState, k: usize, omega: f32) {
        let [i1, i2] = self.constraints[k];
        let w1 = state.inv_mass[i1];
        let w2 = state.inv_mass[i2];
        let w_sum = w1 + w2;
        if w_sum == 0.0 { return; }

        let p1 = state.positions[i1];
        let p2 = state.positions[i2];
        let delta = p1 - p2;
        let len = delta.length();
        if len < 1e-6 { return; }

        let rest = self.rest_lengths[k];
        if len <= rest { return; }

        let c = len - rest;
        let delta_lambda = -c / w_sum;
        let correction_vector = (delta / len) * delta_lambda * omega;

        if w1 > 0.0 { state.positions[i1] += correction_vector * w1; }
        if w2 > 0.0 { state.positions[i2] -= correction_vector * w2; }
    }
}