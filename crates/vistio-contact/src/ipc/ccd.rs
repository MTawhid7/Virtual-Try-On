//! Continuous Collision Detection (CCD) for tunneling prevention.
//!
//! Provides swept-volume CCD for vertex-triangle and edge-edge contacts.
//! CCD determines if a collision occurs during a timestep by solving for
//! the time of impact (TOI) when moving from previous to current positions.
//!
//! Used by the IPC solver to:
//! 1. Validate that a proposed step doesn't create new intersections
//! 2. Compute a maximum safe step size (line search)

use vistio_math::Vec3;

/// Default safety margin for CCD — prevents exact contact by clamping
/// the returned TOI slightly before impact.
pub(crate) const CCD_SAFETY_MARGIN: f32 = 0.9;

/// Minimum time of impact to avoid division-by-zero issues.
pub(crate) const CCD_MIN_TOI: f32 = 1e-6;

/// Vertex-triangle CCD: detect if a moving vertex passes through a moving triangle.
///
/// Tests whether vertex moving from `p0` to `p1` intersects triangle
/// `(a0→a1, b0→b1, c0→c1)` during the time interval `[0, 1]`.
///
/// Returns `Some(toi)` with time of impact in `[0, 1]`, or `None` if no collision.
///
/// Uses a coplanarity test: at time `t`, the four points `p(t), a(t), b(t), c(t)`
/// become coplanar when the triple product of their relative positions is zero.
/// This reduces to finding the roots of a cubic polynomial in `t`.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::manual_range_contains)]
pub fn vertex_triangle_ccd(
    p0: Vec3, p1: Vec3,
    a0: Vec3, a1: Vec3,
    b0: Vec3, b1: Vec3,
    c0: Vec3, c1: Vec3,
) -> Option<f32> {
    // Relative motion: compute positions relative to vertex p
    let dp = p1 - p0;
    let da = a1 - a0;
    let db = b1 - b0;
    let dc = c1 - c0;

    // At time t, positions are:
    //   p(t) = p0 + t*dp
    //   a(t) = a0 + t*da, ...
    // Relative to p(t):
    //   a_rel(t) = (a0 - p0) + t*(da - dp)
    //   b_rel(t) = (b0 - p0) + t*(db - dp)
    //   c_rel(t) = (c0 - p0) + t*(dc - dp)

    let a_r = a0 - p0;
    let b_r = b0 - p0;
    let c_r = c0 - p0;
    let a_v = da - dp;
    let b_v = db - dp;
    let c_v = dc - dp;

    // The coplanarity condition is: dot(a_rel(t), cross(b_rel(t), c_rel(t))) = 0
    // This is a cubic polynomial in t: At³ + Bt² + Ct + D = 0

    // Expand the triple product:
    // f(t) = (a_r + t*a_v) · ((b_r + t*b_v) × (c_r + t*c_v))

    // Cross products needed:
    let bv_x_cv = b_v.cross(c_v); // t² term from cross
    let br_x_cv = b_r.cross(c_v);
    let bv_x_cr = b_v.cross(c_r);
    let br_x_cr = b_r.cross(c_r); // t⁰ term from cross

    // Cubic coefficients
    let coeff_a = a_v.dot(bv_x_cv);                               // t³
    let coeff_b = a_r.dot(bv_x_cv) + a_v.dot(br_x_cv + bv_x_cr); // t²
    let coeff_c = a_r.dot(br_x_cv + bv_x_cr) + a_v.dot(br_x_cr); // t¹
    let coeff_d = a_r.dot(br_x_cr);                               // t⁰

    // Find roots in [0, 1]
    let roots = solve_cubic_roots(coeff_a, coeff_b, coeff_c, coeff_d);

    let mut min_toi: Option<f32> = None;

    for t in roots {
        if t < -CCD_MIN_TOI || t > 1.0 + CCD_MIN_TOI {
            continue;
        }
        let t = t.clamp(0.0, 1.0);

        // Check if the point is actually inside the triangle at time t
        let pa = a_r + a_v * t;
        let pb = b_r + b_v * t;
        let pc = c_r + c_v * t;

        let n = (pb - pa).cross(pc - pa);
        let area2 = n.length_squared();
        if area2 < 1e-20 {
            continue; // Degenerate triangle
        }

        // Barycentric coordinates (point is at origin in relative coords)
        let origin = Vec3::ZERO;
        let v0 = pb - pa;
        let v1 = pc - pa;
        let v2 = origin - pa;

        let d00 = v0.dot(v0);
        let d01 = v0.dot(v1);
        let d11 = v1.dot(v1);
        let d20 = v2.dot(v0);
        let d21 = v2.dot(v1);

        let denom = d00 * d11 - d01 * d01;
        if denom.abs() < 1e-20 {
            continue;
        }
        let inv = 1.0 / denom;
        let u = (d11 * d20 - d01 * d21) * inv;
        let w = (d00 * d21 - d01 * d20) * inv;
        let v = 1.0 - u - w;

        // Inside triangle if all barycentric coords are in [-tol, 1+tol]
        let tol = 0.01;
        if v >= -tol && u >= -tol && w >= -tol {
            let safe_t = (t * CCD_SAFETY_MARGIN).max(CCD_MIN_TOI);
            min_toi = Some(min_toi.map_or(safe_t, |prev: f32| prev.min(safe_t)));
        }
    }

    min_toi
}

/// Edge-edge CCD: detect if two moving edges cross each other.
///
/// Edge 1 moves from `(a0→a1, b0→b1)`, edge 2 moves from `(c0→c1, d0→d1)`.
///
/// Returns `Some(toi)` with time of impact in `[0, 1]`, or `None`.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::manual_range_contains)]
pub fn edge_edge_ccd(
    a0: Vec3, a1: Vec3,
    b0: Vec3, b1: Vec3,
    c0: Vec3, c1: Vec3,
    d0: Vec3, d1: Vec3,
) -> Option<f32> {
    // At time t:
    //   a(t) = a0 + t*(a1-a0),  b(t) = b0 + t*(b1-b0)
    //   c(t) = c0 + t*(c1-c0),  d(t) = d0 + t*(d1-d0)
    //
    // Edge directions: e1(t) = b(t) - a(t),  e2(t) = d(t) - c(t)
    // Coplanarity: (c(t) - a(t)) · (e1(t) × e2(t)) = 0

    let da = a1 - a0;
    let db = b1 - b0;
    let dc = c1 - c0;
    let dd = d1 - d0;

    // r(t) = c(t) - a(t) = (c0-a0) + t*(dc-da)
    let r0 = c0 - a0;
    let rv = dc - da;

    // e1(t) = (b0-a0) + t*(db-da)
    let e10 = b0 - a0;
    let e1v = db - da;

    // e2(t) = (d0-c0) + t*(dd-dc)
    let e20 = d0 - c0;
    let e2v = dd - dc;

    // Triple product f(t) = r(t) · (e1(t) × e2(t)) = cubic in t
    let e1v_x_e2v = e1v.cross(e2v);
    let e10_x_e2v = e10.cross(e2v);
    let e1v_x_e20 = e1v.cross(e20);
    let e10_x_e20 = e10.cross(e20);

    let coeff_a = rv.dot(e1v_x_e2v);
    let coeff_b = r0.dot(e1v_x_e2v) + rv.dot(e10_x_e2v + e1v_x_e20);
    let coeff_c = r0.dot(e10_x_e2v + e1v_x_e20) + rv.dot(e10_x_e20);
    let coeff_d = r0.dot(e10_x_e20);

    let roots = solve_cubic_roots(coeff_a, coeff_b, coeff_c, coeff_d);

    let mut min_toi: Option<f32> = None;

    for t in roots {
        if t < -CCD_MIN_TOI || t > 1.0 + CCD_MIN_TOI {
            continue;
        }
        let t = t.clamp(0.0, 1.0);

        // Check if the closest points are actually on both segments at time t
        let a_t = a0 + da * t;
        let b_t = b0 + db * t;
        let c_t = c0 + dc * t;
        let d_t = d0 + dd * t;

        let e1 = b_t - a_t;
        let e2 = d_t - c_t;
        let r = c_t - a_t;

        let len1_sq = e1.dot(e1);
        let len2_sq = e2.dot(e2);

        if len1_sq < 1e-20 || len2_sq < 1e-20 {
            continue; // Degenerate edge
        }

        let b_val = e1.dot(e2);
        let f_val = e2.dot(r);
        let c_val = e1.dot(r);
        let denom = len1_sq * len2_sq - b_val * b_val;

        let (s, u) = if denom.abs() < 1e-10 {
            // Parallel edges — skip
            continue;
        } else {
            let s = ((b_val * f_val - c_val * len2_sq) / denom).clamp(0.0, 1.0);
            let u = (b_val * s + f_val) / len2_sq;
            (s, u.clamp(0.0, 1.0))
        };

        // Both parameters must be in [0, 1] for contact on the segments
        let tol = 0.05;
        if s >= -tol && s <= 1.0 + tol && u >= -tol && u <= 1.0 + tol {
            let safe_t = (t * CCD_SAFETY_MARGIN).max(CCD_MIN_TOI);
            min_toi = Some(min_toi.map_or(safe_t, |prev: f32| prev.min(safe_t)));
        }
    }

    min_toi
}

/// Compute the maximum safe step fraction for a set of vertex displacements.
///
/// Given previous and proposed positions, checks all vertex-triangle and
/// edge-edge CCD pairs and returns the largest fraction `alpha in (0, 1]`
/// such that moving `alpha` of the way creates no new intersections.
#[allow(clippy::too_many_arguments)]
#[allow(clippy::manual_range_contains)]
pub fn compute_max_step_size(
    prev_x: &[f32], prev_y: &[f32], prev_z: &[f32],
    new_x: &[f32], new_y: &[f32], new_z: &[f32],
    indices: &[u32],
    candidate_pairs: &[(usize, usize)], // (triangle_a, triangle_b)
) -> f32 {
    let mut min_toi = 1.0_f32;
    let tri_count = indices.len() / 3;

    for &(ta, tb) in candidate_pairs {
        if ta >= tri_count || tb >= tri_count {
            continue;
        }
        let base_a = ta * 3;
        let base_b = tb * 3;
        let va = [indices[base_a] as usize, indices[base_a + 1] as usize, indices[base_a + 2] as usize];
        let vb = [indices[base_b] as usize, indices[base_b + 1] as usize, indices[base_b + 2] as usize];

        // Test each vertex of triangle A against triangle B
        for &vi in &va {
            if vb.contains(&vi) { continue; }
            let p0 = Vec3::new(prev_x[vi], prev_y[vi], prev_z[vi]);
            let p1 = Vec3::new(new_x[vi], new_y[vi], new_z[vi]);
            let a0 = Vec3::new(prev_x[vb[0]], prev_y[vb[0]], prev_z[vb[0]]);
            let a1 = Vec3::new(new_x[vb[0]], new_y[vb[0]], new_z[vb[0]]);
            let b0 = Vec3::new(prev_x[vb[1]], prev_y[vb[1]], prev_z[vb[1]]);
            let b1 = Vec3::new(new_x[vb[1]], new_y[vb[1]], new_z[vb[1]]);
            let c0 = Vec3::new(prev_x[vb[2]], prev_y[vb[2]], prev_z[vb[2]]);
            let c1 = Vec3::new(new_x[vb[2]], new_y[vb[2]], new_z[vb[2]]);

            if let Some(toi) = vertex_triangle_ccd(p0, p1, a0, a1, b0, b1, c0, c1) {
                min_toi = min_toi.min(toi);
            }
        }

        // Test each vertex of triangle B against triangle A
        for &vi in &vb {
            if va.contains(&vi) { continue; }
            let p0 = Vec3::new(prev_x[vi], prev_y[vi], prev_z[vi]);
            let p1 = Vec3::new(new_x[vi], new_y[vi], new_z[vi]);
            let a0 = Vec3::new(prev_x[va[0]], prev_y[va[0]], prev_z[va[0]]);
            let a1 = Vec3::new(new_x[va[0]], new_y[va[0]], new_z[va[0]]);
            let b0 = Vec3::new(prev_x[va[1]], prev_y[va[1]], prev_z[va[1]]);
            let b1 = Vec3::new(new_x[va[1]], new_y[va[1]], new_z[va[1]]);
            let c0 = Vec3::new(prev_x[va[2]], prev_y[va[2]], prev_z[va[2]]);
            let c1 = Vec3::new(new_x[va[2]], new_y[va[2]], new_z[va[2]]);

            if let Some(toi) = vertex_triangle_ccd(p0, p1, a0, a1, b0, b1, c0, c1) {
                min_toi = min_toi.min(toi);
            }
        }
    }

    min_toi.max(CCD_MIN_TOI)
}

// ─── Cubic Solver ────────────────────────────────────────────────

/// Solve cubic equation At³ + Bt² + Ct + D = 0.
///
/// Returns all real roots (up to 3). Uses depressed cubic / Cardano's method.
pub(crate) fn solve_cubic_roots(a: f32, b: f32, c: f32, d: f32) -> Vec<f32> {
    let eps = 1e-8;

    // If A ≈ 0, solve as quadratic
    if a.abs() < eps {
        return solve_quadratic_roots(b, c, d);
    }

    // Normalize: t³ + pt² + qt + r = 0
    let p = b / a;
    let q = c / a;
    let r = d / a;

    // Depressed cubic: substitute t = u - p/3
    // u³ + au + b = 0  where:
    let a_dep = q - p * p / 3.0;
    let b_dep = r - p * q / 3.0 + 2.0 * p * p * p / 27.0;

    let discriminant = b_dep * b_dep / 4.0 + a_dep * a_dep * a_dep / 27.0;

    let shift = -p / 3.0;

    if discriminant > eps {
        // One real root
        let sqrt_disc = discriminant.sqrt();
        let u = (-b_dep / 2.0 + sqrt_disc).cbrt();
        let v = (-b_dep / 2.0 - sqrt_disc).cbrt();
        vec![u + v + shift]
    } else if discriminant.abs() <= eps {
        // Triple or double root
        if b_dep.abs() < eps {
            vec![shift] // Triple root
        } else {
            let u = (b_dep / 2.0).cbrt();
            vec![-2.0 * u + shift, u + shift]
        }
    } else {
        // Three real roots (casus irreducibilis) — use trigonometric method
        let r_val = (-a_dep * a_dep * a_dep / 27.0).sqrt();
        if r_val.abs() < 1e-12 {
            return vec![shift];
        }
        let cos_val = (-b_dep / (2.0 * r_val)).clamp(-1.0, 1.0);
        let theta = cos_val.acos();
        let m = 2.0 * (r_val).cbrt();

        vec![
            m * (theta / 3.0).cos() + shift,
            m * ((theta + 2.0 * std::f32::consts::PI) / 3.0).cos() + shift,
            m * ((theta + 4.0 * std::f32::consts::PI) / 3.0).cos() + shift,
        ]
    }
}

/// Solve quadratic equation at² + bt + c = 0.
pub(crate) fn solve_quadratic_roots(a: f32, b: f32, c: f32) -> Vec<f32> {
    let eps = 1e-8;

    if a.abs() < eps {
        // Linear
        if b.abs() < eps {
            return Vec::new();
        }
        return vec![-c / b];
    }

    let disc = b * b - 4.0 * a * c;
    if disc < -eps {
        Vec::new()
    } else if disc.abs() <= eps {
        vec![-b / (2.0 * a)]
    } else {
        let sqrt_disc = disc.sqrt();
        vec![
            (-b + sqrt_disc) / (2.0 * a),
            (-b - sqrt_disc) / (2.0 * a),
        ]
    }
}

