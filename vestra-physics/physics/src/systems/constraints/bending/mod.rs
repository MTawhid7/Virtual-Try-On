// physics/src/systems/constraints/bending/mod.rs

//! Enforces dihedral angle preservation (Bend Resistance).
//! Connects vertices that are two edges apart (bends).
//! Uses limits and reduced compliance for "folding" behavior along anatomical creases.
//!
//! OPTIMIZATION: Uses true SIMD vectorization to process 4 constraints in parallel.

mod solver;

use crate::engine::state::PhysicsState;
use crate::utils::coloring;
use std::collections::HashSet;

/// Enforces dihedral angle preservation (Bend Resistance).
/// Connects vertices that are two edges apart (bends).
/// Uses limits and reduced compliance for "folding" behavior along anatomical creases.
///
/// OPTIMIZATION: Uses true SIMD vectorization to process 4 constraints in parallel.
pub struct BendingConstraint {
    pub constraints: Vec<[usize; 2]>,
    pub rest_lengths: Vec<f32>,
    pub compliances: Vec<f32>,
    pub batch_offsets: Vec<usize>,
}

impl BendingConstraint {
    /// Identifies bending pairs (neighbors of neighbors) and initializes constraints.
    pub fn new(state: &PhysicsState, compliance_factor: f32) -> Self {
        let mut raw_constraints = Vec::new();
        let mut raw_rest_lengths = Vec::new();
        let mut raw_compliances = Vec::new();

        let mut adj = vec![HashSet::new(); state.count];
        let num_triangles = state.indices.len() / 3;

        for i in 0..num_triangles {
            let idx0 = state.indices[i * 3] as usize;
            let idx1 = state.indices[i * 3 + 1] as usize;
            let idx2 = state.indices[i * 3 + 2] as usize;

            adj[idx0].insert(idx1); adj[idx0].insert(idx2);
            adj[idx1].insert(idx0); adj[idx1].insert(idx2);
            adj[idx2].insert(idx0); adj[idx2].insert(idx1);
        }

        let mut processed = HashSet::new();

        for i in 0..state.count {
            for &neighbor in &adj[i] {
                for &far_neighbor in &adj[neighbor] {
                    if i == far_neighbor { continue; }
                    if adj[i].contains(&far_neighbor) { continue; }

                    let pair = if i < far_neighbor { (i, far_neighbor) } else { (far_neighbor, i) };
                    if processed.contains(&pair) { continue; }
                    processed.insert(pair);

                    let p1 = state.positions[i];
                    let p2 = state.positions[far_neighbor];

                    raw_constraints.push([i, far_neighbor]);
                    raw_rest_lengths.push(p1.distance(p2));

                    let uv1 = state.uvs[i];
                    let uv2 = state.uvs[far_neighbor];

                    let du = (uv1.x - uv2.x).abs();
                    let dv = (uv1.y - uv2.y).abs();
                    let is_axis_aligned = du > 2.0 * dv || dv > 2.0 * du;

                    // SOFTENED: 0.5 allows the cloth to fold.
                    if is_axis_aligned {
                        raw_compliances.push(0.5 * compliance_factor);
                    } else {
                        raw_compliances.push(1.0 * compliance_factor);
                    }
                }
            }
        }

        let (sorted_indices, batch_offsets) = coloring::color_constraints(&raw_constraints, state.count);

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
