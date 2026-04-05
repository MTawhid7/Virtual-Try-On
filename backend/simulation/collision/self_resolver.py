"""
Cloth self-collision resolver — hot-path kernel.

For each cloth particle, queries the dynamic spatial hash for candidate cloth
triangles, applies 1-ring exclusion (skip triangles incident to the query vertex),
finds the nearest penetrating triangle, and pushes the particle to the surface.

The hash is rebuilt in numpy each substep by ClothSelfCollider (self_collider.py)
before this kernel runs.

Bidirectionality emerges naturally: every vertex is processed as a query particle,
so both sides of a cloth-cloth contact receive corrections.

IMPORTANT: Do NOT add `from __future__ import annotations` to this file.
Taichi JIT cannot resolve string annotations on .template() kernel parameters.
"""

import taichi as ti

from simulation.collision.point_triangle import closest_point_on_triangle

# Hash function primes — must match spatial_hash.py
_P1 = 73856093
_P2 = 19349663
_P3 = 83492791


@ti.kernel
def resolve_self_collision(
    positions: ti.template(),
    inv_mass: ti.template(),
    n_particles: ti.i32,
    faces: ti.template(),
    n_faces: ti.i32,
    cell_start: ti.template(),
    cell_count: ti.template(),
    entries: ti.template(),
    table_size: ti.i32,
    cell_size: ti.f32,
    thickness: ti.f32,
):
    """
    Resolve cloth self-collisions for all free particles.

    Algorithm (mirrors body collision resolver):
        1. Skip pinned particles (inv_mass <= 0)
        2. Search 27-cell (3×3×3) neighborhood in the dynamic hash
        3. 1-ring exclusion: skip candidate triangles incident to the query vertex
        4. Euclidean guard: discard triangles farther than 2×cell_size
        5. Track the nearest cloth triangle (min Euclidean distance)
        6. If |signed_distance| < thickness, push particle to thickness along face normal
        7. Apply friction tangentially

    Signed distance convention:
        sd = (p - closest).dot(face_n)
        sd > 0: particle on the front face side (outside — no response)
        sd < 0: particle has passed through to the back side (penetration)

    Only actual penetrations are corrected (sd < -thickness * 0.1). This
    filters out the coplanar flat-cloth case where sd = 0.0 exactly, which
    would otherwise cause every particle to be pushed upward on every iteration.
    Push direction is always +face_n (back toward the outside/front of the triangle).
    """
    for i in range(n_particles):
        if inv_mass[i] <= 0.0:
            continue

        p = positions[i]
        ix0 = ti.cast(ti.floor(p.x / cell_size), ti.i32)
        iy0 = ti.cast(ti.floor(p.y / cell_size), ti.i32)
        iz0 = ti.cast(ti.floor(p.z / cell_size), ti.i32)

        best_euclidean = ti.f32(1e9)
        best_normal = ti.math.vec3(0.0, 1.0, 0.0)
        best_sd = ti.f32(0.0)
        found = ti.i32(0)

        # Euclidean guard: same rationale as body collision.
        # Hash collisions can map distant cloth regions into the same cell;
        # they will have large Euclidean distance and be discarded here.
        max_contact_dist = cell_size * 2.0

        for di in ti.static(range(-1, 2)):
            for dj in ti.static(range(-1, 2)):
                for dk in ti.static(range(-1, 2)):
                    ix = ix0 + di
                    iy = iy0 + dj
                    iz = iz0 + dk

                    h = (ix * _P1) ^ (iy * _P2) ^ (iz * _P3)
                    h = h % table_size
                    if h < 0:
                        h += table_size

                    start = cell_start[h]
                    count = cell_count[h]

                    for k in range(count):
                        tri_idx = entries[start + k]
                        if tri_idx >= n_faces:
                            continue

                        f0 = faces[tri_idx][0]
                        f1 = faces[tri_idx][1]
                        f2 = faces[tri_idx][2]

                        # 1-ring exclusion: skip triangles incident to particle i.
                        # Prevents the cloth from pushing itself away from its own
                        # adjacent triangles (which always appear to "penetrate" at
                        # shared edges).
                        if f0 == i or f1 == i or f2 == i:
                            continue

                        v0 = positions[f0]
                        v1 = positions[f1]
                        v2 = positions[f2]

                        e1 = v1 - v0
                        e2 = v2 - v0
                        face_n = e1.cross(e2)
                        n_len = face_n.norm()
                        if n_len < 1e-8:
                            continue
                        face_n = face_n / n_len

                        closest = closest_point_on_triangle(p, v0, v1, v2)
                        euclidean = (p - closest).norm()
                        if euclidean > max_contact_dist:
                            continue

                        sd = (p - closest).dot(face_n)

                        # Track nearest triangle (min Euclidean)
                        if euclidean < best_euclidean:
                            best_euclidean = euclidean
                            best_sd = sd
                            best_normal = face_n
                            found = 1

        # Penetration condition: particle must be within the thickness band AND on
        # the back side of the triangle (sd < 0 means it has crossed through).
        #
        # Using `best_euclidean < thickness` as the primary guard is critical:
        #   - Natural draping folds: particle is "below" a fold (sd < 0) but the
        #     Euclidean distance to the fold surface is large (e.g. 2.4 cm). These
        #     should NOT trigger — the particle is just folded naturally, not
        #     penetrating. sd-only threshold (e.g. sd < -thickness*0.1) is fooled
        #     by these cases and causes explosive upward corrections.
        #   - True penetrations: particle has passed through the cloth surface; the
        #     Euclidean distance to the closest point is < thickness (< 4 mm). These
        #     should trigger a push-out.
        #   - Flat coplanar case: sd = 0 exactly for all coplanar non-adjacent
        #     triangles → sd < 0 is false → no response. Safe without any additional
        #     threshold gymnastics.
        #
        # Correction: push particle to `thickness` above the surface along face_n.
        # `thickness - best_sd` where sd ∈ (-thickness, 0) → correction ∈ (thickness, 2×thickness).
        # Max 8 mm per substep — well within constraint restore range.
        if found == 1 and best_euclidean < thickness and best_sd < 0.0:
            positions[i] = positions[i] + (thickness - best_sd) * best_normal
