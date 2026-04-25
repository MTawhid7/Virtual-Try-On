"""
Pattern panel triangulation — converts a 2D polygon into a triangulated mesh.

Uses the 'triangle' library for constrained Delaunay triangulation:
  1. Resample polygon boundary to enforce minimum edge length (eliminates
     pathologically short DXF curve edges)
  2. Feed resampled boundary + segments to 'triangle' with area constraints
  3. Extract structural edges from triangle faces
  4. Build boundary_indices with full path ordering (including Steiner points)

The density of the mesh is controlled by `target_edge` (meters). The default
of 0.030m (3cm) produces ~200–600 particles per panel — balanced for
real-time (30fps) interactive visualization.

UV coordinates are the normalized 2D positions within the polygon bounding box.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass
class TriangulatedPanel:
    """Result of triangulating a single 2D pattern panel."""

    positions: NDArray[np.float32]   # (N, 3) — 2D vertices lifted to 3D (Y=0)
    faces: NDArray[np.int32]         # (F, 3) — triangle vertex indices
    edges: NDArray[np.int32]         # (E, 2) — unique structural + shear edges
    uvs: NDArray[np.float32]         # (N, 2) — UV coords normalized to [0,1]²
    boundary_indices: NDArray[np.int32]  # (B,) — indices of boundary vertices in path order
    original_vertex_mapping: NDArray[np.int32]  # (V,) — maps original poly vertex idx to index in positions


def _resample_polygon(
    poly: NDArray[np.float64],
    min_edge: float,
) -> NDArray[np.float64]:
    """
    Resample polygon boundary to enforce minimum edge length.

    Walks the polygon and drops vertices that are too close to the previous
    kept vertex. This eliminates the 1-3mm edges from dense DXF curve sampling
    while preserving the overall shape.

    The first vertex is always kept. A vertex is kept only if its distance
    from the last kept vertex exceeds `min_edge`.

    Args:
        poly:     (P, 2) polygon vertices in order.
        min_edge: Minimum allowed edge length in meters.

    Returns:
        (P', 2) resampled polygon with P' <= P vertices.
    """
    if len(poly) < 3:
        return poly

    kept = [poly[0]]
    for i in range(1, len(poly)):
        dist = np.linalg.norm(poly[i] - kept[-1])
        if dist >= min_edge:
            kept.append(poly[i])

    # Ensure the last vertex doesn't create a tiny edge back to the first
    if len(kept) > 3:
        close_dist = np.linalg.norm(kept[-1] - kept[0])
        if close_dist < min_edge:
            kept.pop()

    if len(kept) < 3:
        # Fallback: return original if resampling collapsed the polygon
        return poly

    return np.array(kept, dtype=np.float64)


def triangulate_panel(
    vertices_2d: list[list[float]],
    resolution: int = 20,
    target_edge: float = 0.030,
    min_edge: float = 0.007,
) -> TriangulatedPanel:
    """
    Triangulate a 2D polygon panel into a particle mesh.

    Args:
        vertices_2d: List of [x, z] pairs defining the polygon outline
                     (counterclockwise, NOT closed — do not repeat first vertex).
        resolution:  Triangulation density. If target_edge is 0.030 (default),
                     target_edge will be computed as (max_dim / resolution).
        target_edge: Target interior edge length in meters. 0.030 = 3cm.
        min_edge:    Minimum boundary edge length in meters.

    Returns:
        TriangulatedPanel with positions (Y=0), faces, edges, UVs, and
        boundary_indices.
    """
    poly_raw = np.array(vertices_2d, dtype=np.float64)  # (P, 2)
    if len(poly_raw) < 3:
        raise ValueError(f"Polygon must have at least 3 vertices, got {len(poly_raw)}")

    # --- Bounding box ---
    bb_min = poly_raw.min(axis=0)
    bb_max = poly_raw.max(axis=0)
    extent = bb_max - bb_min  # [width, height]

    if extent[0] < 1e-8 or extent[1] < 1e-8:
        raise ValueError(f"Degenerate polygon: bounding box too small {extent}")

    # --- Derive target_edge from resolution if not default ---
    if target_edge == 0.030 and resolution != 20:
        max_dim = max(extent[0], extent[1])
        target_edge = max_dim / max(resolution, 1)
        # Avoid extremes
        target_edge = max(0.008, min(target_edge, 0.10))

    # --- Target area for triangle refinement ---
    target_area = (np.sqrt(3) / 4) * (target_edge ** 2)

    # --- Phase 0: Resample boundary to eliminate very short edges ---
    poly = _resample_polygon(poly_raw, min_edge)
    n_poly = len(poly)

    # Recompute bounding box after resampling
    bb_min = poly.min(axis=0)
    bb_max = poly.max(axis=0)
    extent = bb_max - bb_min

    # --- Target area for triangle refinement ---
    target_area = (np.sqrt(3) / 4) * (target_edge ** 2)

    # --- Subdivide boundary to create evenly-spaced boundary segments ---
    original_vertices = list(poly)
    steiner_points = []

    # Track which indices in the combined array are path-ordered boundary points
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

    # Build segments from path_indices
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

    # --- Map EVERY original polygon corner to its closest position in the new mesh ---
    # We use poly_raw (the input vertices) instead of the resampled 'poly'
    # so that original_vertex_mapping has length P (original vertex count).
    # This ensures stitch definitions using original indices work correctly.
    from scipy.spatial import KDTree
    tree = KDTree(all_pts_2d)
    _, original_vertex_mapping = tree.query(poly_raw)
    old_to_new_mapping = original_vertex_mapping.astype(np.int32)

    # --- Build boundary_indices: ALL boundary vertices in path order ---
    # Must query poly_subdiv in PATH ORDER (original verts interleaved with their
    # Steiner points) so that _find_edge_particles() can walk from v_start to v_end
    # and collect the Steiner points in between.
    #
    # poly_subdiv layout: [original_verts(0..n_poly-1)] + [steiner_points(n_poly..)]
    # path_indices layout: [v0, st_0_1, st_0_2, ..., v1, st_1_1, ..., v2, ...]
    #
    # Querying poly_subdiv directly would give boundary_indices with original verts
    # first and all Steiners at the end — causing _find_edge_particles() to walk
    # through original corners only (e.g. 6 pairs on a 70cm side seam instead of ~35).
    path_ordered_pts = poly_subdiv[np.array(path_indices, dtype=int)]
    _, boundary_indices = tree.query(path_ordered_pts)
    boundary_indices = boundary_indices.astype(np.int32)

    # --- Lift 2D → 3D (XZ plane, Y=0) ---
    positions = np.zeros((n_total_pts, 3), dtype=np.float32)
    positions[:, 0] = all_pts_2d[:, 0]
    positions[:, 2] = all_pts_2d[:, 1]

    # --- Extract edges from faces ---
    edge_set: set[tuple[int, int]] = set()
    for f in faces:
        for k in range(3):
            a, b = int(f[k]), int(f[(k + 1) % 3])
            edge_set.add((min(a, b), max(a, b)))

    edges = np.array(sorted(edge_set), dtype=np.int32)

    # --- UV coordinates: normalize 2D positions to [0,1]² ---
    uvs = np.zeros((n_total_pts, 2), dtype=np.float32)
    uvs[:, 0] = ((all_pts_2d[:, 0] - bb_min[0]) / max(extent[0], 1e-8))
    uvs[:, 1] = ((all_pts_2d[:, 1] - bb_min[1]) / max(extent[1], 1e-8))

    return TriangulatedPanel(
        positions=positions,
        faces=faces,
        edges=edges,
        uvs=uvs,
        boundary_indices=boundary_indices,
        original_vertex_mapping=np.array(old_to_new_mapping, dtype=np.int32),
    )
