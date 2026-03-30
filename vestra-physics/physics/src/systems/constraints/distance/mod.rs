// physics/src/systems/constraints/distance/mod.rs

//! Enforces edge length preservation (Stretch Resistance).
//! Uses XPBD (Extended Position Based Dynamics) to handle stiffness compliance.
//! Constraints are colored (batched) to allow stable sequential solving.
//!
//! OPTIMIZATION: Uses true SIMD vectorization to process 4 constraints in parallel.

mod solver;

use crate::engine::state::PhysicsState;
use crate::utils::coloring;
use std::collections::HashMap;

/// Enforces edge length preservation (Stretch Resistance).
/// Uses XPBD (Extended Position Based Dynamics) to handle stiffness compliance.
/// Constraints are colored (batched) to allow stable sequential solving.
///
/// OPTIMIZATION: Uses true SIMD vectorization to process 4 constraints in parallel.
pub struct DistanceConstraint {
    pub constraints: Vec<[usize; 2]>,
    pub rest_lengths: Vec<f32>,
    pub compliances: Vec<f32>,
    pub batch_offsets: Vec<usize>,
}

impl DistanceConstraint {
    /// Builds distance constraints for every unique edge in the mesh.
    pub fn new(state: &PhysicsState, compliance: f32) -> Self {
        let mut raw_constraints = Vec::new();
        let mut raw_rest_lengths = Vec::new();
        let mut raw_compliances = Vec::new();

        let mut edge_counts = HashMap::new();
        let num_triangles = state.indices.len() / 3;

        for i in 0..num_triangles {
            let idx0 = state.indices[i * 3] as usize;
            let idx1 = state.indices[i * 3 + 1] as usize;
            let idx2 = state.indices[i * 3 + 2] as usize;

            let e1 = if idx0 < idx1 {
                (idx0, idx1)
            } else {
                (idx1, idx0)
            };
            let e2 = if idx1 < idx2 {
                (idx1, idx2)
            } else {
                (idx2, idx1)
            };
            let e3 = if idx2 < idx0 {
                (idx2, idx0)
            } else {
                (idx0, idx2)
            };

            *edge_counts.entry(e1).or_insert(0) += 1;
            *edge_counts.entry(e2).or_insert(0) += 1;
            *edge_counts.entry(e3).or_insert(0) += 1;
        }

        for ((i1, i2), _count) in edge_counts {
            let p1 = state.positions[i1];
            let p2 = state.positions[i2];
            let dist = p1.distance(p2);

            raw_constraints.push([i1, i2]);
            raw_rest_lengths.push(dist);
            raw_compliances.push(compliance);
        }

        let (sorted_indices, batch_offsets) =
            coloring::color_constraints(&raw_constraints, state.count);

        let mut constraints = Vec::with_capacity(raw_constraints.len());
        let mut rest_lengths = Vec::with_capacity(raw_constraints.len());
        let mut compliances = Vec::with_capacity(raw_constraints.len());

        for idx in sorted_indices {
            constraints.push(raw_constraints[idx]);
            rest_lengths.push(raw_rest_lengths[idx]);
            compliances.push(raw_compliances[idx]);
        }

        Self {
            constraints,
            rest_lengths,
            compliances,
            batch_offsets,
        }
    }
}
