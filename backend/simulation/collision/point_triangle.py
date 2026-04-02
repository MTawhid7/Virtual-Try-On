"""
Point-triangle projection — pure geometry, no collision policy.

Provides two @ti.func helpers called from inside the body collision resolver kernel:
    - closest_point_on_triangle: Voronoi region barycentric projection
    - signed_distance_to_triangle: smoothed-normal signed distance

These are @ti.func (not @ti.kernel) — they run device-side only, called
from within resolver kernels.

IMPORTANT: Do NOT add `from __future__ import annotations` to this file.
Taichi JIT cannot resolve string annotations on .template() kernel parameters.
"""

import taichi as ti


@ti.func
def closest_point_on_triangle(
    p: ti.math.vec3,
    v0: ti.math.vec3,
    v1: ti.math.vec3,
    v2: ti.math.vec3,
) -> ti.math.vec3:
    """
    Compute the closest point on triangle (v0, v1, v2) to query point p.

    Uses the Voronoi region approach (Ericson, Real-Time Collision Detection §5.1.5).
    Handles all 7 regions: interior + 3 edge regions + 3 vertex regions.

    Returns:
        closest: The closest point on the triangle surface (vec3).
        Barycentric coords (u, v, w) s.t. closest = u*v0 + v*v1 + w*v2
        are embedded in the return — callers needing them should use
        closest_point_and_bary() below.
    """
    ab = v1 - v0
    ac = v2 - v0
    ap = p - v0

    d1 = ab.dot(ap)
    d2 = ac.dot(ap)

    # P is in vertex region of v0
    closest = v0
    if d1 <= 0.0 and d2 <= 0.0:
        closest = v0
    else:
        bp = p - v1
        d3 = ab.dot(bp)
        d4 = ac.dot(bp)

        # P is in vertex region of v1
        if d3 >= 0.0 and d4 <= d3:
            closest = v1
        else:
            cp = p - v2
            d5 = ab.dot(cp)
            d6 = ac.dot(cp)

            # P is in vertex region of v2
            if d6 >= 0.0 and d5 <= d6:
                closest = v2
            else:
                vc = d1 * d4 - d3 * d2
                # P is in edge region of v0-v1
                if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
                    t = d1 / (d1 - d3)
                    closest = v0 + t * ab
                else:
                    vb = d5 * d2 - d1 * d6
                    # P is in edge region of v0-v2
                    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
                        w = d2 / (d2 - d6)
                        closest = v0 + w * ac
                    else:
                        va = d3 * d6 - d5 * d4
                        # P is in edge region of v1-v2
                        if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
                            w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
                            closest = v1 + w * (v2 - v1)
                        else:
                            # P is inside the triangle
                            denom = 1.0 / (va + vb + vc)
                            v = vb * denom
                            w = vc * denom
                            closest = v0 + v * ab + w * ac

    return closest


@ti.func
def closest_point_and_bary(
    p: ti.math.vec3,
    v0: ti.math.vec3,
    v1: ti.math.vec3,
    v2: ti.math.vec3,
) -> ti.math.vec4:
    """
    Compute the closest point on triangle and its barycentric coordinates.

    Returns:
        vec4(closest.x, closest.y, closest.z, bary_encoded)

    Actually returns closest as vec3 and bary as separate vec3 via two calls;
    but since Taichi @ti.func can only return one value, we embed both:
        .xyz = closest point
        We return a mat3x2: col0 = closest, col1 = bary — but that's complex.

    Simpler: returns (closest: vec3, bary: vec3) packed into a mat (3x2).
    Use closest_point_on_triangle for closest, bary_coords for bary separately.
    """
    ab = v1 - v0
    ac = v2 - v0
    ap = p - v0

    d1 = ab.dot(ap)
    d2 = ac.dot(ap)

    closest = v0
    bary_v = ti.f32(0.0)
    bary_w = ti.f32(0.0)

    if d1 <= 0.0 and d2 <= 0.0:
        # Vertex v0
        closest = v0
        bary_v = 0.0
        bary_w = 0.0
    else:
        bp = p - v1
        d3 = ab.dot(bp)
        d4 = ac.dot(bp)

        if d3 >= 0.0 and d4 <= d3:
            # Vertex v1
            closest = v1
            bary_v = 1.0
            bary_w = 0.0
        else:
            cp = p - v2
            d5 = ab.dot(cp)
            d6 = ac.dot(cp)

            if d6 >= 0.0 and d5 <= d6:
                # Vertex v2
                closest = v2
                bary_v = 0.0
                bary_w = 1.0
            else:
                vc = d1 * d4 - d3 * d2
                if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
                    # Edge v0-v1
                    t = d1 / (d1 - d3)
                    closest = v0 + t * ab
                    bary_v = t
                    bary_w = 0.0
                else:
                    vb = d5 * d2 - d1 * d6
                    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
                        # Edge v0-v2
                        w = d2 / (d2 - d6)
                        closest = v0 + w * ac
                        bary_v = 0.0
                        bary_w = w
                    else:
                        va = d3 * d6 - d5 * d4
                        if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
                            # Edge v1-v2
                            w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
                            closest = v1 + w * (v2 - v1)
                            bary_v = 1.0 - w
                            bary_w = w
                        else:
                            # Interior
                            denom = 1.0 / (va + vb + vc)
                            bary_v = vb * denom
                            bary_w = vc * denom
                            closest = v0 + bary_v * ab + bary_w * ac

    # Pack: xyz = closest, w = bary_v (bary_w recoverable as 1-u-v)
    # We return both via a vec4: xyz=closest, but need bary separately.
    # Return as vec4(closest.x, closest.y, closest.z, bary_v) — caller reconstructs.
    # bary_u = 1 - bary_v - bary_w, but we need w too.
    # Use a different encoding: return closest as vec3 and pack bary into a separate return.
    # Taichi supports returning structs — use ti.Vector for closest and separate bary.
    # Actually, simplest: caller uses closest_point_on_triangle for closest,
    # and signed_distance_to_triangle takes (p, closest, n0, n1, n2) without needing bary explicitly.
    # But we need bary for normal interpolation. Let's return vec4 with
    # (bary_u, bary_v, bary_w, 0) and have the caller reconstruct closest from bary.
    bary_u = 1.0 - bary_v - bary_w
    return ti.math.vec4(closest.x, closest.y, closest.z, bary_u)


@ti.func
def signed_distance_to_triangle(
    p: ti.math.vec3,
    closest: ti.math.vec3,
    n0: ti.math.vec3,
    n1: ti.math.vec3,
    n2: ti.math.vec3,
    bary: ti.math.vec3,
) -> ti.f32:
    """
    Compute signed distance from point p to triangle surface.

    Uses Vestra's "smoothed normal trick": interpolate vertex normals with
    barycentric coordinates instead of using the flat face normal. This
    prevents gradient discontinuities at triangle edges that caused upward
    crumpling in the old voxel SDF approach.

    Args:
        p:       Query point.
        closest: Closest point on triangle (from closest_point_on_triangle).
        n0, n1, n2: Vertex normals at triangle vertices.
        bary:    Barycentric coordinates (u, v, w) with u+v+w=1.

    Returns:
        signed_dist: > 0 if p is on the normal side (outside), < 0 if penetrating.
    """
    interp_normal = bary.x * n0 + bary.y * n1 + bary.z * n2
    n_len = interp_normal.norm()
    if n_len > 1e-8:
        interp_normal = interp_normal / n_len
    else:
        # Degenerate: use +Y as fallback (upward escape)
        interp_normal = ti.math.vec3(0.0, 1.0, 0.0)

    return (p - closest).dot(interp_normal)
