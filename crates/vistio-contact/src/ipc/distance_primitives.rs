//! Numerically robust distance primitives for IPC contact detection.
//!
//! Provides squared distance and gradient computations between:
//! - **Point-Triangle**: vertex vs. triangle face/edge/vertex
//! - **Edge-Edge**: two line segments
//!
//! All functions classify the closest feature (face, edge, or vertex)
//! to handle degenerate cases correctly.

use vistio_math::Vec3;

/// Classification of the nearest feature in a distance query.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceType {
    /// Closest point is on the triangle face interior.
    PointTriangleFace,
    /// Closest point is on a triangle edge (index 0=AB, 1=BC, 2=CA).
    PointTriangleEdge(u8),
    /// Closest point is a triangle vertex (index 0=A, 1=B, 2=C).
    PointTriangleVertex(u8),
    /// Closest points are in the interior of both edge segments.
    EdgeEdgeInterior,
    /// Degenerate case — closest feature couldn't be classified.
    Degenerate,
}

// ─── Point-Triangle Distance ─────────────────────────────────────

/// Compute the squared distance from point `p` to triangle `(a, b, c)`.
///
/// Returns `(distance², DistanceType)`. The distance type classifies
/// which feature (face, edge, vertex) contains the closest point.
///
/// Handles degenerate triangles (zero area) gracefully by falling back
/// to point-edge or point-point distance.
pub fn point_triangle_distance_squared(
    p: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
) -> (f32, DistanceType) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let bp = p - b;
    let cp = p - c;

    let normal = ab.cross(ac);
    let area2_sq = normal.length_squared();

    // Degenerate triangle — fall back to edge distances
    if area2_sq < 1e-20 {
        let d_ab = point_segment_distance_squared(p, a, b);
        let d_bc = point_segment_distance_squared(p, b, c);
        let d_ca = point_segment_distance_squared(p, c, a);
        return if d_ab <= d_bc && d_ab <= d_ca {
            (d_ab, DistanceType::PointTriangleEdge(0))
        } else if d_bc <= d_ca {
            (d_bc, DistanceType::PointTriangleEdge(1))
        } else {
            (d_ca, DistanceType::PointTriangleEdge(2))
        };
    }

    // Check Voronoi region of vertex A
    let d1 = ab.dot(ap);
    let d2 = ac.dot(ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (ap.length_squared(), DistanceType::PointTriangleVertex(0));
    }

    // Check Voronoi region of vertex B
    let d3 = ab.dot(bp);
    let d4 = ac.dot(bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (bp.length_squared(), DistanceType::PointTriangleVertex(1));
    }

    // Check Voronoi region of vertex C
    let d5 = ab.dot(cp);
    let d6 = ac.dot(cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (cp.length_squared(), DistanceType::PointTriangleVertex(2));
    }

    // Check edge AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let closest = a + ab * v;
        return ((p - closest).length_squared(), DistanceType::PointTriangleEdge(0));
    }

    // Check edge BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let closest = b + (c - b) * w;
        return ((p - closest).length_squared(), DistanceType::PointTriangleEdge(1));
    }

    // Check edge CA
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let closest = a + ac * w;
        return ((p - closest).length_squared(), DistanceType::PointTriangleEdge(2));
    }

    // Inside the triangle face — project onto plane
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + ab * v + ac * w;
    ((p - closest).length_squared(), DistanceType::PointTriangleFace)
}

/// Compute the gradient of point-triangle squared distance w.r.t. all 4 vertices.
///
/// Returns `[∂d²/∂p, ∂d²/∂a, ∂d²/∂b, ∂d²/∂c]`.
///
/// This is an approximate gradient computed via the closest-point projection
/// for the face case.
pub fn point_triangle_distance_gradient(
    p: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
) -> [Vec3; 4] {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    let area2_sq = normal.length_squared();

    if area2_sq < 1e-20 {
        // Degenerate — return zero gradients
        return [Vec3::ZERO; 4];
    }

    let n = normal / area2_sq.sqrt();
    let ap = p - a;
    let signed_dist = ap.dot(n);
    let projected = p - n * signed_dist;

    // Barycentric coords of projected point
    let v0 = b - a;
    let v1 = c - a;
    let v2 = projected - a;
    let d00 = v0.dot(v0);
    let d01 = v0.dot(v1);
    let d11 = v1.dot(v1);
    let d20 = v2.dot(v0);
    let d21 = v2.dot(v1);
    let denom = d00 * d11 - d01 * d01;

    if denom.abs() < 1e-20 {
        return [Vec3::ZERO; 4];
    }
    let inv = 1.0 / denom;
    let v = (d11 * d20 - d01 * d21) * inv;
    let w = (d00 * d21 - d01 * d20) * inv;
    let u = 1.0 - v - w;

    // Closest point on triangle
    let closest = a * u + b * v + c * w;
    let diff = p - closest;

    // ∂(d²)/∂p = 2 * (p - closest)
    let grad_p = diff * 2.0;

    // ∂(d²)/∂a = -2 * u * (p - closest)
    let grad_a = -diff * (2.0 * u);
    let grad_b = -diff * (2.0 * v);
    let grad_c = -diff * (2.0 * w);

    [grad_p, grad_a, grad_b, grad_c]
}

// ─── Edge-Edge Distance ──────────────────────────────────────────

/// Compute the squared distance between two line segments:
/// segment 1: `(a0, a1)` and segment 2: `(b0, b1)`.
///
/// Returns `(distance², DistanceType)`.
pub fn edge_edge_distance_squared(
    a0: Vec3,
    a1: Vec3,
    b0: Vec3,
    b1: Vec3,
) -> (f32, DistanceType) {
    let d1 = a1 - a0; // Direction of segment 1
    let d2 = b1 - b0; // Direction of segment 2
    let r = a0 - b0;

    let a = d1.dot(d1); // |d1|²
    let e = d2.dot(d2); // |d2|²
    let f = d2.dot(r);

    // Check for degenerate segments
    let eps = 1e-10;
    if a < eps && e < eps {
        // Both segments are points
        return (r.length_squared(), DistanceType::Degenerate);
    }

    let (s, t) = if a < eps {
        // First segment degenerates to a point
        let t = (f / e).clamp(0.0, 1.0);
        (0.0, t)
    } else {
        let c = d1.dot(r);
        if e < eps {
            // Second segment degenerates to a point
            let s = (-c / a).clamp(0.0, 1.0);
            (s, 0.0)
        } else {
            // General case
            let b = d1.dot(d2);
            let denom = a * e - b * b;

            let mut s = if denom.abs() > eps {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0 // Parallel segments — pick s=0
            };

            let mut t = (b * s + f) / e;

            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }

            (s, t)
        }
    };

    let closest1 = a0 + d1 * s;
    let closest2 = b0 + d2 * t;
    let diff = closest1 - closest2;

    (diff.length_squared(), DistanceType::EdgeEdgeInterior)
}

/// Compute the gradient of edge-edge squared distance w.r.t. all 4 vertices.
///
/// Returns `[∂d²/∂a0, ∂d²/∂a1, ∂d²/∂b0, ∂d²/∂b1]`.
pub fn edge_edge_distance_gradient(
    a0: Vec3,
    a1: Vec3,
    b0: Vec3,
    b1: Vec3,
) -> [Vec3; 4] {
    let d1 = a1 - a0;
    let d2 = b1 - b0;
    let r = a0 - b0;

    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let c = d1.dot(r);
    let b_val = d1.dot(d2);

    let eps = 1e-10;

    let (s, t) = if a < eps || e < eps {
        (0.0, 0.0)
    } else {
        let denom = a * e - b_val * b_val;
        let mut s = if denom.abs() > eps {
            ((b_val * f - c * e) / denom).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let mut t = (b_val * s + f) / e;
        if t < 0.0 {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else if t > 1.0 {
            t = 1.0;
            s = ((b_val - c) / a).clamp(0.0, 1.0);
        }
        (s, t)
    };

    let closest1 = a0 + d1 * s;
    let closest2 = b0 + d2 * t;
    let diff = closest1 - closest2;

    // ∂(d²)/∂a0 = 2 * (1 - s) * diff
    // ∂(d²)/∂a1 = 2 * s * diff
    let grad_a0 = diff * (2.0 * (1.0 - s));
    let grad_a1 = diff * (2.0 * s);
    // ∂(d²)/∂b0 = -2 * (1 - t) * diff
    // ∂(d²)/∂b1 = -2 * t * diff
    let grad_b0 = -diff * (2.0 * (1.0 - t));
    let grad_b1 = -diff * (2.0 * t);

    [grad_a0, grad_a1, grad_b0, grad_b1]
}

// ─── Helpers ─────────────────────────────────────────────────────

/// Squared distance from point `p` to line segment `(a, b)`.
pub(crate) fn point_segment_distance_squared(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let ap = p - a;
    let ab_sq = ab.length_squared();
    if ab_sq < 1e-20 {
        return ap.length_squared();
    }
    let t = (ap.dot(ab) / ab_sq).clamp(0.0, 1.0);
    let closest = a + ab * t;
    (p - closest).length_squared()
}

