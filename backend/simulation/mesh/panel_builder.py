"""
Panel builder — loads a pattern JSON, triangulates each 2D panel,
applies 3D placement transforms, merges all panels into a single
GarmentMesh, and resolves stitch vertex pairs.

Pattern JSON format:
    {
      "name": "TankTop",
      "panels": [
        {
          "id": "front",
          "vertices_2d": [[0,0],[0.4,0],[0.4,0.7],[0,0.7]],
          "placement": { "position": [0.0, 1.0, 0.12], "rotation_y_deg": 0 }
        },
        {
          "id": "back",
          "vertices_2d": [[0,0],[0.4,0],[0.4,0.7],[0,0.7]],
          "placement": { "position": [0.0, 1.0, -0.12], "rotation_y_deg": 180 }
        }
      ],
      "stitches": [
        { "panel_a": "front", "edge_a": [0, 3], "panel_b": "back", "edge_b": [0, 3] }
      ],
      "fabric": "cotton"
    }

Placement convention:
    - The 2D panel lives in the local XZ plane (X = pattern width, Z = pattern height).
    - Y=0 in local space. After transform: translated to placement.position.
    - rotation_y_deg rotates around the Y axis (180° flips a back panel inward).
    - The bottom-left corner of the polygon bounding box maps to placement.position.

Stitch edge convention:
    - edge_a: [vertex_index_start, vertex_index_end] referencing the POLYGON outline
      vertices (the boundary_indices of the triangulated panel).
    - Vertices along the edge are matched 1-to-1 by position along the edge,
      sorted from start to end. Both edges must produce the same number of
      boundary particles (same edge length / same density).
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from simulation.mesh.triangulation import triangulate_panel, TriangulatedPanel


@dataclass
class GarmentMesh:
    """Merged multi-panel mesh ready for ParticleState.load_from_numpy()."""

    positions: NDArray[np.float32]       # (N_total, 3) world-space, all panels merged
    faces: NDArray[np.int32]             # (F_total, 3) triangles with global indices
    edges: NDArray[np.int32]             # (E_total, 2) structural edges
    uvs: NDArray[np.float32]             # (N_total, 2) UV coordinates
    stitch_pairs: NDArray[np.int32]      # (S, 2) global vertex index pairs to stitch
    panel_offsets: list[int]             # start vertex index of each panel
    panel_ids: list[str]                 # panel id strings in order
    fabric: str                          # fabric name from pattern JSON


def _rotation_x(deg: float) -> NDArray[np.float64]:
    """3×3 rotation matrix around X axis (right-hand rule)."""
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [1,  0,  0],
        [0,  c, -s],
        [0,  s,  c],
    ], dtype=np.float64)


def _rotation_y(deg: float) -> NDArray[np.float64]:
    """3×3 rotation matrix around Y axis."""
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ], dtype=np.float64)


def _apply_placement(
    positions_local: NDArray[np.float32],
    placement: dict,
) -> NDArray[np.float32]:
    """
    Transform panel vertices from local space to world space.

    Local space: panel lies in XZ plane (X=width, Z=height), Y=0.
    World space: apply Rx(rotation_x_deg) first, then Ry(rotation_y_deg),
    then translate to position.

    rotation_x_deg=-90 converts the XZ panel to a vertical XY plane:
        local (x, 0, z)  →Rx(-90)→  (x, z, 0)
    Combined with translation [-0.2, 0.65, 0.12] this places the panel
    upright in the torso region (Y=0.65–1.35) at Z=+0.12.

    Args:
        positions_local: (N, 3) positions in local panel space.
        placement: dict with keys:
            'position'       ([x, y, z]) — world-space translation
            'rotation_y_deg' — Y-axis rotation in degrees (default 0)
            'rotation_x_deg' — X-axis rotation applied first (default 0)

    Returns:
        (N, 3) float32 world-space positions.
    """
    rot_x_deg = float(placement.get("rotation_x_deg", 0.0))
    rot_y_deg = float(placement.get("rotation_y_deg", 0.0))
    translation = np.array(placement["position"], dtype=np.float64)

    pts = positions_local.astype(np.float64)   # (N, 3)

    if rot_x_deg != 0.0:
        pts = pts @ _rotation_x(rot_x_deg).T   # apply X rotation first

    rotated = pts @ _rotation_y(rot_y_deg).T   # then Y rotation
    world = rotated + translation               # broadcast translate

    return world.astype(np.float32)


def _find_edge_particles(
    panel: TriangulatedPanel,
    v_start: int,
    v_end: int,
) -> NDArray[np.int32]:
    """
    Find all boundary particle indices that lie along the edge from
    polygon vertex v_start to polygon vertex v_end, sorted by position
    along the edge.

    Args:
        panel: TriangulatedPanel (local space, Y=0).
        v_start: Index into the polygon outline (boundary_indices).
        v_end:   Index into the polygon outline (boundary_indices).

    Returns:
        (M,) int32 array of local particle indices along the edge,
        ordered from v_start to v_end (inclusive at both ends).
    """
    # World positions of the two polygon boundary vertices
    idx_start = int(panel.boundary_indices[v_start])
    idx_end   = int(panel.boundary_indices[v_end])

    p_start = panel.positions[idx_start, [0, 2]].astype(np.float64)  # XZ
    p_end   = panel.positions[idx_end,   [0, 2]].astype(np.float64)

    edge_vec = p_end - p_start
    edge_len = np.linalg.norm(edge_vec)

    if edge_len < 1e-8:
        return np.array([idx_start], dtype=np.int32)

    edge_dir = edge_vec / edge_len

    # Check every vertex: project onto edge direction, keep those near the segment
    pts_xz = panel.positions[:, [0, 2]].astype(np.float64)  # (N, 2)
    rel = pts_xz - p_start                                    # (N, 2)
    t   = rel @ edge_dir                                       # scalar projection
    # perpendicular distance to the edge line
    perp = rel - np.outer(t, edge_dir)
    dist_perp = np.linalg.norm(perp, axis=1)

    # Tolerance: must capture at least the first interior grid column.
    # 10% of edge length — for a 0.7m edge at resolution=20, this gives 0.07m,
    # which includes the first grid column at ~0.04m from the boundary.
    tol = edge_len * 0.10 + 1e-4

    on_segment = (t >= -tol) & (t <= edge_len + tol) & (dist_perp < tol)
    indices = np.where(on_segment)[0]

    if len(indices) == 0:
        # Fallback: return just the two endpoints
        return np.array([idx_start, idx_end], dtype=np.int32)

    # Sort by t (projection along edge direction)
    t_vals = t[indices]
    order = np.argsort(t_vals)
    return indices[order].astype(np.int32)


def build_garment_mesh(
    pattern_path: str | Path,
    resolution: int = 20,
) -> GarmentMesh:
    """
    Load a pattern JSON and build a merged GarmentMesh.

    Steps:
        1. Parse JSON — panels + stitches + fabric name
        2. Triangulate each 2D panel
        3. Apply placement transform (rotate Y + translate)
        4. Merge all panels into global arrays
        5. Resolve stitch definitions → global vertex index pairs

    Args:
        pattern_path: Path to the pattern JSON file.
        resolution:   Triangulation density (passed to triangulate_panel).

    Returns:
        GarmentMesh with merged positions/faces/edges/uvs, stitch_pairs,
        panel_offsets, panel_ids, and fabric name.
    """
    path = Path(pattern_path)
    if not path.exists():
        raise FileNotFoundError(f"Pattern file not found: {path}")

    with open(path) as f:
        spec = json.load(f)

    panel_specs  = spec["panels"]
    stitch_specs = spec.get("stitches", [])
    fabric       = spec.get("fabric", "cotton")

    # --- Step 1+2+3: Triangulate and transform each panel ---
    panels_local: list[TriangulatedPanel] = []
    panels_world_pos: list[NDArray[np.float32]] = []
    panel_ids: list[str] = []

    for pspec in panel_specs:
        panel_local = triangulate_panel(pspec["vertices_2d"], resolution=resolution)
        world_pos   = _apply_placement(panel_local.positions, pspec["placement"])
        panels_local.append(panel_local)
        panels_world_pos.append(world_pos)
        panel_ids.append(pspec["id"])

    # --- Step 4: Merge into global arrays ---
    panel_offsets: list[int] = []
    offset = 0
    all_positions = []
    all_faces = []
    all_edges = []
    all_uvs = []

    for i, (panel, world_pos) in enumerate(zip(panels_local, panels_world_pos)):
        panel_offsets.append(offset)
        all_positions.append(world_pos)
        all_faces.append(panel.faces + offset)
        all_edges.append(panel.edges + offset)
        all_uvs.append(panel.uvs)
        offset += world_pos.shape[0]

    positions = np.concatenate(all_positions, axis=0).astype(np.float32)
    faces     = np.concatenate(all_faces,     axis=0).astype(np.int32)
    edges     = np.concatenate(all_edges,     axis=0).astype(np.int32)
    uvs       = np.concatenate(all_uvs,       axis=0).astype(np.float32)

    # --- Step 5: Resolve stitch definitions → global index pairs ---
    panel_id_to_idx = {pid: i for i, pid in enumerate(panel_ids)}
    stitch_pairs_list: list[tuple[int, int]] = []

    for sdef in stitch_specs:
        pa_idx = panel_id_to_idx[sdef["panel_a"]]
        pb_idx = panel_id_to_idx[sdef["panel_b"]]
        ea = sdef["edge_a"]   # [v_start, v_end] in polygon outline space
        eb = sdef["edge_b"]

        local_a = _find_edge_particles(panels_local[pa_idx], ea[0], ea[1])
        local_b = _find_edge_particles(panels_local[pb_idx], eb[0], eb[1])

        # Match by relative position along edge (zip shortest)
        n_match = min(len(local_a), len(local_b))
        if n_match == 0:
            continue

        # Subsample the longer list to match the shorter one
        if len(local_a) != len(local_b):
            idx_a = np.round(np.linspace(0, len(local_a) - 1, n_match)).astype(int)
            idx_b = np.round(np.linspace(0, len(local_b) - 1, n_match)).astype(int)
            local_a = local_a[idx_a]
            local_b = local_b[idx_b]

        off_a = panel_offsets[pa_idx]
        off_b = panel_offsets[pb_idx]

        for la, lb in zip(local_a, local_b):
            stitch_pairs_list.append((int(la) + off_a, int(lb) + off_b))

    stitch_pairs = (
        np.array(stitch_pairs_list, dtype=np.int32)
        if stitch_pairs_list
        else np.zeros((0, 2), dtype=np.int32)
    )

    return GarmentMesh(
        positions=positions,
        faces=faces,
        edges=edges,
        uvs=uvs,
        stitch_pairs=stitch_pairs,
        panel_offsets=panel_offsets,
        panel_ids=panel_ids,
        fabric=fabric,
    )
