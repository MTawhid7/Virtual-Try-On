//! Vertex-triangle proximity narrow phase.
//!
//! For each candidate pair from the broad phase, tests whether a vertex
//! is within `thickness` distance of a triangle. When it is, produces
//! a `ContactPair` with the signed distance, contact normal, and
//! barycentric coordinates.

use vistio_math::Vec3;
use vistio_mesh::TriangleMesh;
use vistio_solver::state::SimulationState;
use vistio_types::VistioResult;

use crate::broad::CandidatePair;
use crate::contact::{ContactPair, ContactType};
use crate::narrow::NarrowPhase;

/// Vertex-triangle proximity test narrow phase.
///
/// For each broad-phase candidate pair (a, b), treats `a` as a vertex
/// and finds the closest triangle containing `b`, then tests proximity.
///
/// This is the Tier 1 narrow phase — exact geometric testing without CCD.
pub struct VertexTriangleTest;

impl NarrowPhase for VertexTriangleTest {
    fn detect(
        &self,
        candidates: &[CandidatePair],
        state: &SimulationState,
        mesh: &TriangleMesh,
        thickness: f32,
    ) -> VistioResult<Vec<ContactPair>> {
        let mut contacts = Vec::new();
        let tri_count = mesh.triangle_count();

        for candidate in candidates {
            let vi = candidate.a as usize;
            let vj = candidate.b as usize;

            let p = Vec3::new(state.pos_x[vi], state.pos_y[vi], state.pos_z[vi]);

            // Test vertex vi against all triangles containing vj
            for t in 0..tri_count {
                let [a, b, c] = mesh.triangle(t);
                let a = a as usize;
                let b = b as usize;
                let c = c as usize;

                // Skip if vertex vi is part of this triangle
                if vi == a || vi == b || vi == c {
                    continue;
                }

                // Only consider triangles containing vj
                if vj != a && vj != b && vj != c {
                    continue;
                }

                let pa = Vec3::new(state.pos_x[a], state.pos_y[a], state.pos_z[a]);
                let pb = Vec3::new(state.pos_x[b], state.pos_y[b], state.pos_z[b]);
                let pc = Vec3::new(state.pos_x[c], state.pos_y[c], state.pos_z[c]);

                if let Some(contact) = point_triangle_proximity(
                    p,
                    pa,
                    pb,
                    pc,
                    vi as u32,
                    a as u32,
                    b as u32,
                    c as u32,
                    thickness,
                ) {
                    contacts.push(contact);
                }
            }
        }

        Ok(contacts)
    }

    fn name(&self) -> &str {
        "vertex_triangle_test"
    }
}

/// Test proximity between a point and a triangle.
///
/// Returns `Some(ContactPair)` if the distance is < `thickness`.
/// `None` otherwise.
#[allow(clippy::too_many_arguments)]
fn point_triangle_proximity(
    p: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    vi: u32,
    ai: u32,
    bi: u32,
    ci: u32,
    thickness: f32,
) -> Option<ContactPair> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let normal = ab.cross(ac);
    let area2 = normal.length();
    if area2 < 1e-10 {
        return None; // Degenerate triangle
    }
    let normal = normal / area2;

    // Signed distance from point to triangle plane
    let signed_dist = ap.dot(normal);
    let abs_dist = signed_dist.abs();

    if abs_dist > thickness {
        return None; // Too far
    }

    // Project point onto the triangle plane
    let projected = p - normal * signed_dist;
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

    // Check if the projection is inside the triangle (with small tolerance)
    let tol = -0.01;
    if u < tol || v < tol || w < tol {
        return None; // Outside triangle
    }

    // Contact normal direction: ensure it points from triangle to vertex
    let contact_normal = if signed_dist >= 0.0 { normal } else { -normal };

    Some(ContactPair {
        contact_type: ContactType::VertexTriangle,
        indices: [vi, ai, bi, ci],
        distance: -abs_dist, // Negative = penetration/proximity
        normal: [contact_normal.x, contact_normal.y, contact_normal.z],
        barycentric: [u, v, w],
        is_self: true,
    })
}
