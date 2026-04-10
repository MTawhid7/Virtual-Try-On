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
    original_vertex_mapping: NDArray[np.int32]  # (V,) — maps original poly vertex idx to index in boundary_indices


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
    # Calculate target area and edge
    longer = max(extent[0], extent[1])
    # Target 1.5cm segments for high detail
    target_edge = 0.015 
    target_area = (np.sqrt(3) / 4) * (target_edge ** 2)

    # --- Subdivide boundary to enable 'triangle' refinement ---
    original_vertices = list(poly)
    steiner_points = []
    
    n_poly = len(poly)
    # the ordered path of indices along the boundary
    path_indices = []
    
    for i in range(n_poly):
        path_indices.append(i)
        p0 = poly[i]
        p1 = poly[(i + 1) % n_poly]
        dist = np.linalg.norm(p1 - p0)
        
        n_splits = int(np.ceil(dist / target_edge))
        if n_splits > 1:
            for k in range(1, n_splits):
                steiner_idx = n_poly + len(steiner_points)
                path_indices.append(steiner_idx)
                steiner_points.append(p0 + (p1 - p0) * (k / n_splits))
                
    poly_subdiv = np.array(original_vertices + steiner_points, dtype=np.float64)
    n_pts = len(poly_subdiv)
    
    # build segments from path_indices
    seg_list = []
    for i in range(len(path_indices)):
        seg_list.append([path_indices[i], path_indices[(i + 1) % len(path_indices)]])
    segments = np.array(seg_list, dtype=np.int32)
    
    A = {"vertices": poly_subdiv, "segments": segments}
    
    import triangle as tr
    B = tr.triangulate(A, f"pq30Ya{target_area:.6f}")
    
    all_pts_2d = B["vertices"].astype(np.float32)
    faces = B["triangles"].astype(np.int32)
    n_total_pts = len(all_pts_2d)
    
    # Robustly find the indices of the original corners in the new mesh
    # This prevents the 'webbing' caused by vertex reordering in triangle.lib
    from scipy.spatial import KDTree
    tree = KDTree(all_pts_2d)
    _, original_vertex_mapping = tree.query(poly)
    old_to_new_mapping = original_vertex_mapping.astype(np.int32)
    
    # Path indices now refer to the high-res boundary
    # Find closest vertices for all path points to be safe
    _, boundary_indices = tree.query(poly_subdiv)
    boundary_indices = boundary_indices.astype(np.int32)

    # --- Lift 2D → 3D (XZ plane, Y=0) ---
    positions = np.zeros((n_total_pts, 3), dtype=np.float32)
    positions[:, 0] = all_pts_2d[:, 0]
    positions[:, 2] = all_pts_2d[:, 1]

    # Y=0 — placement.py will apply the world-space transform

    # --- Extract edges from faces ---
    edge_set: set[tuple[int, int]] = set()
    for f in faces:
        for k in range(3):
            a, b = int(f[k]), int(f[(k + 1) % 3])
            edge_set.add((min(a, b), max(a, b)))

    edges = np.array(sorted(edge_set), dtype=np.int32)

    # --- UV coordinates: normalize 2D positions to [0,1]² ---
    uvs = np.zeros((n_total_pts, 2), dtype=np.float32)
    uvs[:, 0] = ((all_pts_2d[:, 0] - bb_min[0]) / extent[0])
    uvs[:, 1] = ((all_pts_2d[:, 1] - bb_min[1]) / extent[1])


    return TriangulatedPanel(
        positions=positions,
        faces=faces,
        edges=edges,
        uvs=uvs,
        boundary_indices=boundary_indices,
        original_vertex_mapping=np.array(old_to_new_mapping, dtype=np.int32),
    )
