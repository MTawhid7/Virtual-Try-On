// physics/src/systems/constraints/area.rs

use crate::engine::state::PhysicsState;
use crate::utils::coloring;
use glam::Vec4;

/// Enforces triangle area preservation.
/// Prevents the mesh from shearing or collapsing, simulating the material's resistance to planar deformation.
///
/// NOTE: Area constraints involve 3-body interactions with complex gradient computations.
/// Full SIMD is less beneficial here due to data dependencies.
pub struct AreaConstraint {
    indices: Vec<[usize; 3]>,
    rest_areas: Vec<f32>,
    batch_offsets: Vec<usize>,
}

impl AreaConstraint {
    /// Computes the rest area of every triangle in the mesh.
    pub fn new(state: &PhysicsState) -> Self {
        let num_triangles = state.indices.len() / 3;
        let mut raw_indices = Vec::with_capacity(num_triangles);
        let mut raw_rest_areas = Vec::with_capacity(num_triangles);

        for i in 0..num_triangles {
            let idx0 = state.indices[i * 3] as usize;
            let idx1 = state.indices[i * 3 + 1] as usize;
            let idx2 = state.indices[i * 3 + 2] as usize;

            let p0 = state.positions[idx0];
            let p1 = state.positions[idx1];
            let p2 = state.positions[idx2];

            let u = p1 - p0;
            let v = p2 - p0;
            let u3 = u.truncate();
            let v3 = v.truncate();
            let cross = u3.cross(v3);
            let area = 0.5 * cross.length();

            if area > 1e-6 {
                raw_indices.push([idx0, idx1, idx2]);
                raw_rest_areas.push(area);
            }
        }

        let (sorted_indices, batch_offsets) = coloring::color_constraints_3(&raw_indices, state.count);

        let mut indices = Vec::with_capacity(raw_indices.len());
        let mut rest_areas = Vec::with_capacity(raw_indices.len());

        for idx in sorted_indices {
            indices.push(raw_indices[idx]);
            rest_areas.push(raw_rest_areas[idx]);
        }

        Self {
            indices,
            rest_areas,
            batch_offsets,
        }
    }

    /// Solves the area constraint using XPBD.
    /// Uses 4x loop unrolling for instruction-level parallelism.
    #[inline(never)]
    pub fn solve(&self, state: &mut PhysicsState, compliance: f32, omega: f32, dt: f32) {
        let alpha = compliance / (dt * dt);

        for b in 0..(self.batch_offsets.len() - 1) {
            let start = self.batch_offsets[b];
            let end = self.batch_offsets[b + 1];
            let count = end - start;

            let chunks = count / 4;
            let remainder = count % 4;

            for chunk in 0..chunks {
                let base = start + chunk * 4;
                Self::solve_single(state, &self.indices[base], self.rest_areas[base], alpha, omega);
                Self::solve_single(state, &self.indices[base + 1], self.rest_areas[base + 1], alpha, omega);
                Self::solve_single(state, &self.indices[base + 2], self.rest_areas[base + 2], alpha, omega);
                Self::solve_single(state, &self.indices[base + 3], self.rest_areas[base + 3], alpha, omega);
            }

            for k in (start + chunks * 4)..(start + chunks * 4 + remainder) {
                Self::solve_single(state, &self.indices[k], self.rest_areas[k], alpha, omega);
            }
        }
    }

    #[inline(always)]
    fn solve_single(state: &mut PhysicsState, indices: &[usize; 3], rest_area: f32, alpha: f32, omega: f32) {
        let [i0, i1, i2] = *indices;

        let w0 = state.inv_mass[i0];
        let w1 = state.inv_mass[i1];
        let w2 = state.inv_mass[i2];
        let w_sum = w0 + w1 + w2;
        if w_sum == 0.0 { return; }

        let p0 = state.positions[i0];
        let p1 = state.positions[i1];
        let p2 = state.positions[i2];

        let u = p1 - p0;
        let v = p2 - p0;
        let u3 = u.truncate();
        let v3 = v.truncate();
        let cross = u3.cross(v3);
        let current_area = 0.5 * cross.length();

        let c = current_area - rest_area;
        if c.abs() < 1e-6 { return; }
        if current_area < 1e-9 { return; }

        let n = cross / (2.0 * current_area);

        let grad0 = 0.5 * (p2.truncate() - p1.truncate()).cross(n);
        let grad1 = 0.5 * (p0.truncate() - p2.truncate()).cross(n);
        let grad2 = 0.5 * (p1.truncate() - p0.truncate()).cross(n);

        let denom = w0 * grad0.length_squared() +
                    w1 * grad1.length_squared() +
                    w2 * grad2.length_squared();

        if denom < 1e-9 { return; }

        let delta_lambda = -c / (denom + alpha);
        let lambda_omega = delta_lambda * omega;

        if w0 > 0.0 { state.positions[i0] += Vec4::from((grad0 * (lambda_omega * w0), 0.0)); }
        if w1 > 0.0 { state.positions[i1] += Vec4::from((grad1 * (lambda_omega * w1), 0.0)); }
        if w2 > 0.0 { state.positions[i2] += Vec4::from((grad2 * (lambda_omega * w2), 0.0)); }
    }
}