//! Self-collision detection and resolution.
//!
//! Uses vertex-triangle proximity tests for accurate collision detection.
//! The pipeline:
//! 1. **Broad phase**: Spatial hash → candidate vertex pairs
//! 2. **Narrow phase**: For each pair (a,b), test vertex a against triangles
//!    containing b (and vice versa), filtered by topology exclusion
//! 3. **Resolution**: Push vertex out of triangle along the contact normal

use vistio_math::Vec3;
use vistio_mesh::TriangleMesh;
use vistio_mesh::topology::Topology;
use vistio_solver::state::SimulationState;

use crate::broad::BroadPhase;
use crate::exclusion::TopologyExclusion;

/// Self-collision system: detect → resolve using vertex-triangle tests.
pub struct SelfCollisionSystem {
    /// Topology-based exclusion (n-ring neighborhood).
    exclusion: TopologyExclusion,
    /// Collision thickness / proximity threshold.
    thickness: f32,
    /// Response stiffness (0.0–1.0).
    stiffness: f32,
    /// Triangle indices (flattened: [a0,b0,c0, a1,b1,c1, ...]).
    triangles: Vec<[u32; 3]>,
    /// For each vertex, which triangles contain it.
    vertex_to_triangles: Vec<Vec<usize>>,
}

/// Result of self-collision resolution.
#[derive(Debug, Clone)]
pub struct SelfCollisionResult {
    /// Number of candidate pairs from broad phase.
    pub candidate_pairs: u32,
    /// Number of pairs after topology filtering.
    pub filtered_pairs: u32,
    /// Number of pairs where vertices are within thickness.
    pub proximity_pairs: u32,
    /// Number of color batches used.
    pub batch_count: u32,
    /// Number of position corrections applied.
    pub corrections_applied: u32,
}

impl SelfCollisionSystem {
    /// Create a new self-collision system.
    pub fn new(
        mesh: &TriangleMesh,
        topology: &Topology,
        exclusion_depth: usize,
        thickness: f32,
        stiffness: f32,
    ) -> Self {
        let exclusion = TopologyExclusion::new(mesh, topology, exclusion_depth);

        // Build triangle list
        let tri_count = mesh.triangle_count();
        let mut triangles = Vec::with_capacity(tri_count);
        for t in 0..tri_count {
            triangles.push(mesh.triangle(t));
        }

        // Build vertex → triangle map
        let n = mesh.vertex_count();
        let mut vertex_to_triangles: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (ti, tri) in triangles.iter().enumerate() {
            vertex_to_triangles[tri[0] as usize].push(ti);
            vertex_to_triangles[tri[1] as usize].push(ti);
            vertex_to_triangles[tri[2] as usize].push(ti);
        }

        Self {
            exclusion,
            thickness,
            stiffness,
            triangles,
            vertex_to_triangles,
        }
    }

    /// Run the full self-collision pipeline: detect → resolve.
    pub fn solve(
        &mut self,
        state: &mut SimulationState,
        broad: &mut dyn BroadPhase,
    ) -> SelfCollisionResult {
        // Phase 1: Broad phase — get candidate vertex pairs
        broad.update(&state.pos_x, &state.pos_y, &state.pos_z, self.thickness).ok();
        let candidates = broad.query_pairs();
        let candidate_count = candidates.len() as u32;

        let mut corrections = 0u32;
        let mut proximity_count = 0u32;

        // Phase 2+3: For each candidate pair, do vertex-triangle tests + resolve
        for candidate in &candidates {
            let vi = candidate.a as usize;
            let vj = candidate.b as usize;

            // Skip if topologically adjacent
            if self.exclusion.should_exclude(vi, vj) {
                continue;
            }

            // Test vertex vi against triangles containing vj
            corrections += self.test_and_resolve_vertex_against_triangles(
                vi, vj, state, &mut proximity_count,
            );

            // Test vertex vj against triangles containing vi
            corrections += self.test_and_resolve_vertex_against_triangles(
                vj, vi, state, &mut proximity_count,
            );
        }

        SelfCollisionResult {
            candidate_pairs: candidate_count,
            filtered_pairs: proximity_count,
            proximity_pairs: proximity_count,
            batch_count: 1,
            corrections_applied: corrections,
        }
    }

    /// Test vertex `vi` against all triangles containing vertex `vj`.
    /// Applies position corrections if penetration is found.
    fn test_and_resolve_vertex_against_triangles(
        &self,
        vi: usize,
        vj: usize,
        state: &mut SimulationState,
        proximity_count: &mut u32,
    ) -> u32 {
        let mut corrections = 0u32;

        let p = Vec3::new(state.pos_x[vi], state.pos_y[vi], state.pos_z[vi]);

        for &ti in &self.vertex_to_triangles[vj] {
            let [a, b, c] = self.triangles[ti];
            let a = a as usize;
            let b = b as usize;
            let c = c as usize;

            // Skip if vertex is part of this triangle
            if vi == a || vi == b || vi == c {
                continue;
            }

            // Skip if any triangle vertex is topologically adjacent to vi
            if self.exclusion.should_exclude(vi, a)
                || self.exclusion.should_exclude(vi, b)
                || self.exclusion.should_exclude(vi, c)
            {
                continue;
            }

            let pa = Vec3::new(state.pos_x[a], state.pos_y[a], state.pos_z[a]);
            let pb = Vec3::new(state.pos_x[b], state.pos_y[b], state.pos_z[b]);
            let pc = Vec3::new(state.pos_x[c], state.pos_y[c], state.pos_z[c]);

            // Test proximity
            if let Some((penetration, normal)) =
                point_triangle_proximity(p, pa, pb, pc, self.thickness)
            {
                *proximity_count += 1;

                // Apply correction: push vertex along triangle normal
                if penetration > 0.0 && state.inv_mass[vi] > 0.0 {
                    let correction = penetration * self.stiffness;
                    state.pos_x[vi] += normal.x * correction;
                    state.pos_y[vi] += normal.y * correction;
                    state.pos_z[vi] += normal.z * correction;

                    // Inelastic velocity correction
                    let vn = state.vel_x[vi] * normal.x
                           + state.vel_y[vi] * normal.y
                           + state.vel_z[vi] * normal.z;
                    if vn < 0.0 {
                        state.vel_x[vi] -= vn * normal.x;
                        state.vel_y[vi] -= vn * normal.y;
                        state.vel_z[vi] -= vn * normal.z;
                    }

                    corrections += 1;
                }
            }
        }

        corrections
    }

    /// Detect self-collision contacts and compute IPC barrier gradients.
    ///
    /// This is the Tier 4 IPC step which does not modify positions directly,
    /// but instead builds an `IpcContactSet` which is returned to the solver.
    #[allow(clippy::too_many_arguments)]
    pub fn detect_ipc_contacts(
        &self,
        broad_phase: &mut dyn crate::broad::BroadPhase,
        pos_x: &[f32],
        pos_y: &[f32],
        pos_z: &[f32],
        thickness: f32,
        d_hat: f32,
        kappa: f32,
    ) -> crate::ipc_response::IpcContactSet {
        // Update broad phase with current slices
        broad_phase.update(pos_x, pos_y, pos_z, thickness).ok();

        let mut ipc_set = crate::ipc_response::IpcContactSet::new_with_thickness(d_hat, kappa, thickness);

        // We use triangle pairs from the BVH because IPC needs both
        // vertex-triangle and edge-edge contacts for robustness.
        let tri_pairs = broad_phase.query_triangle_pairs();

        for pair in tri_pairs {
            let [a0, a1, a2] = self.triangles[pair.ta as usize];
            let [b0, b1, b2] = self.triangles[pair.tb as usize];
            let a_verts = [a0 as usize, a1 as usize, a2 as usize];
            let b_verts = [b0 as usize, b1 as usize, b2 as usize];

            // 1. Point-Triangle tests
            // Vertices of A vs Triangle B
            for &va in &a_verts {
                if b_verts.contains(&va) { continue; }
                if self.exclusion.should_exclude(va, b_verts[0])
                    || self.exclusion.should_exclude(va, b_verts[1])
                    || self.exclusion.should_exclude(va, b_verts[2])
                { continue; }
                ipc_set.detect_vertex_triangle(va, b_verts[0], b_verts[1], b_verts[2], pos_x, pos_y, pos_z, true);
            }

            // Vertices of B vs Triangle A
            for &vb in &b_verts {
                if a_verts.contains(&vb) { continue; }
                if self.exclusion.should_exclude(vb, a_verts[0])
                    || self.exclusion.should_exclude(vb, a_verts[1])
                    || self.exclusion.should_exclude(vb, a_verts[2])
                { continue; }
                ipc_set.detect_vertex_triangle(vb, a_verts[0], a_verts[1], a_verts[2], pos_x, pos_y, pos_z, true);
            }

            // 2. Edge-Edge tests
            let a_edges = [(a0, a1), (a1, a2), (a2, a0)];
            let b_edges = [(b0, b1), (b1, b2), (b2, b0)];

            for &(ea0, ea1) in &a_edges {
                let ea0 = ea0 as usize;
                let ea1 = ea1 as usize;
                for &(eb0, eb1) in &b_edges {
                    let eb0 = eb0 as usize;
                    let eb1 = eb1 as usize;

                    // Skip if they share a vertex
                    if ea0 == eb0 || ea0 == eb1 || ea1 == eb0 || ea1 == eb1 {
                        continue;
                    }

                    // Skip if topologically close
                    if self.exclusion.should_exclude(ea0, eb0)
                        && self.exclusion.should_exclude(ea0, eb1)
                        && self.exclusion.should_exclude(ea1, eb0)
                        && self.exclusion.should_exclude(ea1, eb1)
                    {
                        continue;
                    }

                    ipc_set.detect_edge_edge(ea0, ea1, eb0, eb1, pos_x, pos_y, pos_z, true);
                }
            }
        }

        ipc_set
    }
}

/// Test proximity between a point and a triangle.
///
/// Returns `Some((penetration_depth, normal))` if the point is within
/// `thickness` of the triangle, `None` otherwise.
/// The normal points from the triangle toward the vertex.
fn point_triangle_proximity(
    p: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    thickness: f32,
) -> Option<(f32, Vec3)> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let tri_normal = ab.cross(ac);
    let area2 = tri_normal.length();
    if area2 < 1e-10 {
        return None; // Degenerate triangle
    }
    let tri_normal = tri_normal / area2;

    // Signed distance from point to triangle plane
    let signed_dist = ap.dot(tri_normal);
    let abs_dist = signed_dist.abs();

    if abs_dist > thickness {
        return None; // Too far
    }

    // Project point onto the triangle plane
    let projected = p - tri_normal * signed_dist;
    let ap_proj = projected - a;

    // Barycentric coordinates using Cramer's rule
    let d00 = ab.dot(ab);
    let d01 = ab.dot(ac);
    let d11 = ac.dot(ac);
    let d20 = ap_proj.dot(ab);
    let d21 = ap_proj.dot(ac);

    let denom = d00 * d11 - d01 * d01;
    if denom.abs() < 1e-10 {
        return None;
    }
    let inv_denom = 1.0 / denom;

    let v = (d11 * d20 - d01 * d21) * inv_denom;
    let w = (d00 * d21 - d01 * d20) * inv_denom;
    let u = 1.0 - v - w;

    // Check if projection is inside the triangle
    let tol = -0.01;
    if u < tol || v < tol || w < tol {
        return None; // Outside triangle
    }

    // Normal direction: always point from triangle toward vertex
    let normal = if signed_dist >= 0.0 { tri_normal } else { -tri_normal };

    // Penetration depth = how much the vertex needs to be pushed out
    let penetration = thickness - abs_dist;

    Some((penetration, normal))
}
