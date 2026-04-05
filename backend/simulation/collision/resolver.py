"""
Body mesh collision resolver — the hot-path kernel.

For each cloth particle, queries the static spatial hash for candidate body
triangles, finds the closest penetrating triangle, and pushes the particle
to the surface with position-based friction.

Collision is interleaved inside the XPBD solver iteration loop (not post-process)
per the Vestra pattern — prevents constraint-vs-collision fighting.

IMPORTANT: Do NOT add `from __future__ import annotations` to this file.
Taichi JIT cannot resolve string annotations on .template() kernel parameters.
"""

import taichi as ti

from simulation.collision.point_triangle import closest_point_and_bary, signed_distance_to_triangle

# Hash function primes — must match spatial_hash.py
_P1 = 73856093
_P2 = 19349663
_P3 = 83492791


@ti.kernel
def resolve_body_collision(
    positions: ti.template(),
    predicted: ti.template(),
    inv_mass: ti.template(),
    n_particles: ti.i32,
    # Spatial hash arrays
    cell_start: ti.template(),
    cell_count: ti.template(),
    entries: ti.template(),
    # Triangle geometry
    tri_v0: ti.template(),
    tri_v1: ti.template(),
    tri_v2: ti.template(),
    tri_n0: ti.template(),
    tri_n1: ti.template(),
    tri_n2: ti.template(),
    n_triangles: ti.i32,
    # Hash parameters
    cell_size: ti.f32,
    table_size: ti.i32,
    # Collision parameters
    thickness: ti.f32,
    friction: ti.f32,
):
    """
    Resolve body mesh collisions for all cloth particles.

    For each particle:
        1. Skip if pinned (inv_mass <= 0)
        2. Query spatial hash: search 27 cells (3×3×3 neighborhood) for candidate triangles
        3. For each candidate: compute closest point + signed distance
        4. Track closest penetrating triangle (min signed_dist < thickness)
        5. Push particle to surface + thickness along interpolated normal
        6. Apply position-based friction (same as SphereCollider)

    Using the closest penetrating triangle (not the first) prevents conflicting
    push directions when a particle is near multiple body triangles simultaneously
    — an issue from Vestra's early mesh collision implementation.
    """
    for i in range(n_particles):
        if inv_mass[i] <= 0.0:
            continue

        p = positions[i]

        # Cell containing this particle
        ix0 = ti.cast(ti.floor(p.x / cell_size), ti.i32)
        iy0 = ti.cast(ti.floor(p.y / cell_size), ti.i32)
        iz0 = ti.cast(ti.floor(p.z / cell_size), ti.i32)

        # Track the NEAREST candidate triangle (minimum Euclidean distance to
        # closest point). The nearest surface is physically what the cloth touches.
        # Hash-collision false hits from distant mesh regions are by definition
        # farther away and will not win the min-euclidean comparison.
        # Response is only applied if the nearest triangle is actually penetrating
        # (best_sd < thickness). This replaces the previous MAX-sd (shallowest
        # penetration) strategy, which was unstable with a tight Euclidean guard:
        # when a wrong-normal triangle happened to win max-sd, it injected energy.
        best_euclidean = ti.f32(1e9)  # track MIN euclidean (nearest surface)
        best_sd = ti.f32(0.0)         # sd of the nearest candidate
        best_closest = ti.math.vec3(0.0, 0.0, 0.0)
        best_normal = ti.math.vec3(0.0, 1.0, 0.0)
        found = ti.i32(0)

        # Euclidean guard: discard candidates whose closest point is further than
        # 2×cell_size from the particle. The 3×3×3 search covers ±1.5×cell_size,
        # so all geometrically plausible contacts are within this range. Hash
        # collisions placing triangles from distant body regions (e.g. a foot
        # triangle appearing for a shoulder particle) are well outside it.
        max_contact_dist = cell_size * 2.0

        # Search 27-cell (3×3×3) neighborhood
        for di in ti.static(range(-1, 2)):
            for dj in ti.static(range(-1, 2)):
                for dk in ti.static(range(-1, 2)):
                    ix = ix0 + di
                    iy = iy0 + dj
                    iz = iz0 + dk

                    # Hash cell
                    h = (ix * _P1) ^ (iy * _P2) ^ (iz * _P3)
                    h = h % table_size
                    if h < 0:
                        h += table_size

                    start = cell_start[h]
                    count = cell_count[h]

                    for k in range(count):
                        tri_idx = entries[start + k]
                        if tri_idx >= n_triangles:
                            continue

                        v0 = tri_v0[tri_idx]
                        v1 = tri_v1[tri_idx]
                        v2 = tri_v2[tri_idx]
                        n0 = tri_n0[tri_idx]
                        n1 = tri_n1[tri_idx]
                        n2 = tri_n2[tri_idx]

                        # closest_point_and_bary returns vec4(closest.xyz, bary_u)
                        result = closest_point_and_bary(p, v0, v1, v2)
                        closest = ti.math.vec3(result.x, result.y, result.z)
                        bary_u = result.w
                        # Reconstruct bary_v, bary_w from triangle coordinates
                        # For the triangle: closest = u*v0 + v*v1 + w*v2
                        # We recover bary_v and bary_w by solving the linear system
                        # Use the fact that closest - v0 = bary_v*(v1-v0) + bary_w*(v2-v0)
                        ab = v1 - v0
                        ac = v2 - v0
                        ap_closest = closest - v0
                        d1 = ab.dot(ab)
                        d2 = ab.dot(ac)
                        d3 = ac.dot(ac)
                        d4 = ap_closest.dot(ab)
                        d5 = ap_closest.dot(ac)
                        denom = d1 * d3 - d2 * d2
                        bary_v = ti.f32(0.0)
                        bary_w = ti.f32(0.0)
                        if denom > 1e-10:
                            bary_v = (d4 * d3 - d5 * d2) / denom
                            bary_w = (d5 * d1 - d4 * d2) / denom
                        bary = ti.math.vec3(bary_u, bary_v, bary_w)

                        # Euclidean distance guard: skip triangles that are too far away.
                        # Hash collisions can place false-positive candidate triangles
                        # from completely different mesh regions (e.g. foot triangles
                        # appearing as candidates for a shoulder particle). These have
                        # large Euclidean distance to the particle. Legitimate contacts
                        # occur within max_contact_dist (0.1m >> cell_size ≈ 0.016m).
                        euclidean = (p - closest).norm()
                        if euclidean > max_contact_dist:
                            continue

                        sd = signed_distance_to_triangle(p, closest, n0, n1, n2, bary)

                        # Select the nearest candidate (min euclidean). We track ALL
                        # candidates within the Euclidean guard, updating whenever a
                        # closer triangle is found. The collision response is applied
                        # at the end only if the nearest triangle is penetrating.
                        if euclidean < best_euclidean:
                            best_euclidean = euclidean
                            best_sd = sd
                            best_closest = closest
                            # Reconstruct interpolated normal for response
                            interp_n = bary_u * n0 + bary_v * n1 + bary_w * n2
                            n_len = interp_n.norm()
                            if n_len > 1e-8:
                                best_normal = interp_n / n_len
                            else:
                                best_normal = ti.math.vec3(0.0, 1.0, 0.0)
                            found = 1

        # Apply collision response only if the nearest triangle is penetrating.
        # We also implement a "backface cull" with a depth limit: `outward_check >= -0.05`.
        # By rejecting particles slightly "inside" a seam edge's interpolating normal,
        # we prevent edge normal artifacts from causing severe crumpling. The `-0.05m` 
        # depth threshold ensures that genuinely deep penetrations (tunneling pulled by
        # connected constraints) are still resolutely pushed out.
        outward_check = (p - best_closest).dot(best_normal)
        if found == 1 and best_sd < thickness and outward_check >= -0.05:
            surface_pos = best_closest + thickness * best_normal

            # Position-based friction (same as SphereCollider):
            # Decompose displacement from substep start into normal + tangential,
            # scale tangential by (1 - friction). update_velocities() picks up
            # the friction effect through the reduced position delta.
            disp = surface_pos - predicted[i]
            d_normal = disp.dot(best_normal) * best_normal
            d_tangential = disp - d_normal
            positions[i] = predicted[i] + d_normal + d_tangential * (1.0 - friction)

        # Resting contact friction: particle is in the contact zone but NOT penetrating.
        # Using elif guarantees mutual exclusion with the penetration block above —
        # the same outward_check value is reused, no second spatial query needed.
        # Applies tangential velocity damping without a normal push-out correction.
        # This prevents cloth from sliding off curved surfaces under gravity when
        # not actively penetrating. (Bridson et al. 2002 §4.3 — velocity-level friction)
        elif found == 1 and best_euclidean < thickness * 5.0 and outward_check >= -0.05:
            curr_disp = positions[i] - predicted[i]
            d_normal_rest = curr_disp.dot(best_normal) * best_normal
            d_tang_rest = curr_disp - d_normal_rest
            positions[i] = predicted[i] + d_normal_rest + d_tang_rest * (1.0 - friction)
