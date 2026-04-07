"""
Pattern panel triangulation — converts a 2D polygon into a triangulated mesh.

Uses a grid-clip approach:
  1. Generate a regular NxN grid inside the polygon bounding box
  2. Keep only grid vertices that lie inside the polygon (point-in-polygon)
  3. Add the polygon outline vertices (boundary)
  4. Triangulate the combined point set using mapbox-earcut
  5. Extract unique structural edges + cross-diagonal shear edges

The density of the mesh is controlled by `resolution` (number of grid
subdivisions along the longer axis). A 0.4 × 0.7m panel at resolution=20
produces approximately 200–400 interior particles.

UV coordinates are the normalized 2D positions within the polygon bounding box.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

import mapbox_earcut


@dataclass
class TriangulatedPanel:
    """Result of triangulating a single 2D pattern panel."""

    positions: NDArray[np.float32]   # (N, 3) — 2D vertices lifted to 3D (Y=0)
    faces: NDArray[np.int32]         # (F, 3) — triangle vertex indices
    edges: NDArray[np.int32]         # (E, 2) — unique structural + shear edges
    uvs: NDArray[np.float32]         # (N, 2) — UV coords normalized to [0,1]²
    boundary_indices: NDArray[np.int32]  # (B,) — indices of polygon boundary vertices


def _point_in_polygon(points: NDArray[np.float64], polygon: NDArray[np.float64]) -> NDArray[np.bool_]:
    """
    Ray-casting point-in-polygon test.

    Args:
        points:  (M, 2) array of query points.
        polygon: (P, 2) array of polygon vertices (in order, NOT closed).

    Returns:
        (M,) boolean array — True if point is inside or on boundary.
    """
    n = len(polygon)
    inside = np.zeros(len(points), dtype=bool)

    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        px = points[:, 0]
        py = points[:, 1]

        # Edge intersects ray from point going right (+X direction)
        cond = ((yi > py) != (yj > py)) & (
            px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
        )
        inside ^= cond
        j = i

    return inside


def triangulate_panel(
    vertices_2d: list[list[float]],
    resolution: int = 20,
) -> TriangulatedPanel:
    """
    Triangulate a 2D polygon panel into a particle mesh.

    Args:
        vertices_2d: List of [x, z] pairs defining the polygon outline
                     (counterclockwise, NOT closed — do not repeat first vertex).
        resolution:  Number of grid subdivisions along the longer axis.
                     The other axis scales proportionally. Higher = more particles.

    Returns:
        TriangulatedPanel with positions (Y=0), faces, edges, UVs, and
        boundary_indices.
    """
    poly = np.array(vertices_2d, dtype=np.float64)  # (P, 2)
    if len(poly) < 3:
        raise ValueError(f"Polygon must have at least 3 vertices, got {len(poly)}")

    # --- Bounding box ---
    bb_min = poly.min(axis=0)
    bb_max = poly.max(axis=0)
    extent = bb_max - bb_min  # [width, height]

    if extent[0] < 1e-8 or extent[1] < 1e-8:
        raise ValueError(f"Degenerate polygon: bounding box too small {extent}")

    # --- Grid resolution scaled to aspect ratio ---
    longer = max(extent[0], extent[1])
    shorter = min(extent[0], extent[1])
    res_long = resolution
    res_short = max(2, int(round(resolution * shorter / longer)))

    if extent[0] >= extent[1]:
        res_x, res_z = res_long, res_short
    else:
        res_x, res_z = res_short, res_long

    # --- Generate interior grid candidates ---
    xs = np.linspace(bb_min[0], bb_max[0], res_x)
    zs = np.linspace(bb_min[1], bb_max[1], res_z)
    gx, gz = np.meshgrid(xs, zs)
    grid_pts = np.stack([gx.ravel(), gz.ravel()], axis=1)  # (res_x*res_z, 2)

    # Keep only points strictly inside the polygon
    inside_mask = _point_in_polygon(grid_pts, poly)
    interior_pts = grid_pts[inside_mask]  # (M, 2)

    # --- Combine boundary + interior points ---
    # Boundary comes first so boundary_indices are [0..P-1]
    all_pts_2d = np.concatenate([poly, interior_pts], axis=0)  # (P+M, 2)
    boundary_indices = np.arange(len(poly), dtype=np.int32)

    n_pts = len(all_pts_2d)

    # --- Earcut triangulation ---
    # earcut requires: vertices as float32 (N, 2), rings as uint32 [N]
    verts_f32 = all_pts_2d.astype(np.float32)
    rings = np.array([n_pts], dtype=np.uint32)
    flat_indices = mapbox_earcut.triangulate_float32(verts_f32, rings)  # (T*3,) uint32
    flat_indices = flat_indices.astype(np.int32)

    if len(flat_indices) == 0:
        raise ValueError("Earcut returned no triangles — polygon may be degenerate or self-intersecting")

    faces = flat_indices.reshape(-1, 3)  # (F, 3)

    # Remove degenerate triangles (zero area)
    v0 = all_pts_2d[faces[:, 0]]
    v1 = all_pts_2d[faces[:, 1]]
    v2 = all_pts_2d[faces[:, 2]]
    cross_z = (v1[:, 0] - v0[:, 0]) * (v2[:, 1] - v0[:, 1]) - \
               (v1[:, 1] - v0[:, 1]) * (v2[:, 0] - v0[:, 0])
    valid = np.abs(cross_z) > 1e-10
    faces = faces[valid].astype(np.int32)

    if len(faces) == 0:
        raise ValueError("No valid (non-degenerate) triangles after filtering")

    # --- Lift 2D → 3D (XZ plane, Y=0) ---
    positions = np.zeros((n_pts, 3), dtype=np.float32)
    positions[:, 0] = all_pts_2d[:, 0].astype(np.float32)
    positions[:, 2] = all_pts_2d[:, 1].astype(np.float32)
    # Y=0 — placement.py will apply the world-space transform

    # --- Extract edges from faces ---
    edge_set: set[tuple[int, int]] = set()
    for f in faces:
        for k in range(3):
            a, b = int(f[k]), int(f[(k + 1) % 3])
            edge_set.add((min(a, b), max(a, b)))

    edges = np.array(sorted(edge_set), dtype=np.int32)

    # --- UV coordinates: normalize 2D positions to [0,1]² ---
    uvs = np.zeros((n_pts, 2), dtype=np.float32)
    uvs[:, 0] = ((all_pts_2d[:, 0] - bb_min[0]) / extent[0]).astype(np.float32)
    uvs[:, 1] = ((all_pts_2d[:, 1] - bb_min[1]) / extent[1]).astype(np.float32)

    return TriangulatedPanel(
        positions=positions,
        faces=faces,
        edges=edges,
        uvs=uvs,
        boundary_indices=boundary_indices,
    )
