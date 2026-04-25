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
    stitch_seam_ids: list[str] | None = None  # one seam label per stitch pair (from JSON comment)
    verts_2d: NDArray[np.float32] | None = None  # (N_total, 2) unrotated 2D panel coordinates


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


def _cylindrical_wrap_sleeve(
    world_pos: NDArray[np.float32],
    placement: dict,
) -> NDArray[np.float32]:
    """
    Replace the flat sleeve placement with a cylindrical pre-wrap around the arm.

    After rotation_x_deg=-90 and translation, all sleeve vertices share the same
    Z (= placement["position"][2]) and vary in X across the sleeve width.  The
    underarm seam (left-most and right-most edges in u) therefore start ~sleeve_pw
    apart, with the arm in between — body collision prevents closure.

    This transform maps the flat sleeve into a tube whose circumference equals
    sleeve_pw, centered on the arm.  Both ends of the underarm seam start at the
    same 3-D point (~0 initial gap), so the stitch closes immediately.

    Cylinder geometry (Y-axis):
        angle   = 2π * u / sleeve_pw      (u = X offset from placement X origin)
        world_x = arm_cx + r * sin(angle)
        world_z = arm_cz - r * cos(angle)
        world_y = unchanged
    where r = sleeve_pw / (2π).
    """
    pts = world_pos.astype(np.float64).copy()

    pos_x0 = float(placement["position"][0])   # X origin of the flat sleeve
    u_vals  = pts[:, 0] - pos_x0               # u ∈ [0, sleeve_pw]
    sleeve_pw = float(u_vals.max() - u_vals.min())
    if sleeve_pw < 1e-6:
        return world_pos

    r = sleeve_pw / (2.0 * np.pi)

    # Arm center: use the world-space target position from the pattern JSON
    arm_cx = float(placement["position"][0])
    arm_cz = float(placement["position"][2])

    u_norm  = (u_vals - u_vals.min()) / sleeve_pw   # [0, 1]
    
    # We want u=0 (inner armpit) to be our starting phase.
    # For Right Arm (cx > 0), inner is at x = cx - r. 
    # For Left Arm (cx < 0), inner is at x = cx + r.
    # We want u=0.25 (the middle of the first half) to face the FRONT (+Z).
    
    if arm_cx > 0:
        # Right Arm: Start at -90deg, rotate POSITIVE.
        # u=0.25 -> angle = 0. sin=0, cos=1. Z = 0.05 - r (BACK). 
        # Wait, we want FRONT (+Z). So we want cos to be -1.
        # Let's use: angle = 2*pi*u + pi/2
        # u=0: 90deg. sin=1, cos=0. x = cx+r (Outer). NO.
        
        # FINAL LOGIC:
        # Right Arm: angle = -2*pi*u - pi/2
        #   u=0: -90deg. sin=-1, cos=0. x = cx-r (inner). Correct.
        #   u=0.25: -180deg. sin=0, cos=-1. z = 0.05+r (FRONT). Correct.
        angle = -2.0 * np.pi * u_norm - (np.pi / 2.0)
    else:
        # Left Arm: inner is at x = cx + r. 
        # Start at +90deg, rotate POSITIVE.
        #   u=0: 90deg. sin=1, cos=0. x = cx+r (inner). Correct.
        #   u=0.25: 180deg. sin=0, cos=-1. z = 0.05+r (FRONT). Correct.
        angle = 2.0 * np.pi * u_norm + (np.pi / 2.0)

    pts[:, 0] = arm_cx + r * np.sin(angle)
    pts[:, 2] = arm_cz - r * np.cos(angle)
    # pts[:, 1] (Y) unchanged — preserves cap height and cuff height

    return pts.astype(np.float32)


def _apply_placement(
    positions_local: NDArray[np.float32],
    placement: dict,
    global_scale: float = 1.0,
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

    With global_scale=1.0 (sew-then-drape default), this is a pure
    rotation + translation. The old scaling logic is retained for
    backward compatibility but should not be used with new patterns.

    Args:
        positions_local: (N, 3) positions in local panel space.
        placement: dict with keys:
            'position'       ([x, y, z]) — world-space translation
            'rotation_y_deg' — Y-axis rotation in degrees (default 0)
            'rotation_x_deg' — X-axis rotation applied first (default 0)
        global_scale: Scale factor for panel size. 1.0 = no scaling (default).

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

    if global_scale != 1.0:
        rotated *= global_scale
        body_center = np.array([0.0, 1.25, 0.0], dtype=np.float64)
        direction = translation - body_center
        translation = body_center + direction * global_scale

    world = rotated + translation               # broadcast translate

    return world.astype(np.float32)


def _find_edge_particles(
    panel: TriangulatedPanel,
    v_start: int,
    v_end: int,
) -> NDArray[np.int32]:
    """
    Return particle indices along the polygon boundary from v_start to v_end,
    walking in the shorter direction around the perimeter.

    This version includes ALL boundary vertices between the two endpoints —
    including Steiner points inserted by the triangulator. This gives dense
    stitch coverage (≥1 stitch pair per ~3cm of seam length instead of only
    at original polygon corners).

    Algorithm:
        1. Find where the original polygon vertices v_start and v_end appear
           in the full boundary_indices array (which includes Steiner points)
        2. Walk boundary_indices between those positions, collecting every
           vertex index along the path

    Args:
        panel:   TriangulatedPanel (local space, Y=0).
        v_start: Polygon vertex index (start of edge) in the original polygon.
        v_end:   Polygon vertex index (end of edge) in the original polygon.

    Returns:
        (M,) int32 array of particle indices along the boundary, ordered
        from v_start to v_end inclusive. Includes all Steiner points.
    """
    n = len(panel.boundary_indices)

    # Map original polygon vertex indices to their position indices in the mesh
    start_pos_idx = int(panel.original_vertex_mapping[v_start])
    end_pos_idx = int(panel.original_vertex_mapping[v_end])

    # Find where these position indices appear in boundary_indices
    # boundary_indices is ordered along the boundary path and may contain
    # many more entries than the original polygon (due to Steiner points)
    start_bi = -1
    end_bi = -1
    for k in range(n):
        if panel.boundary_indices[k] == start_pos_idx and start_bi == -1:
            start_bi = k
        if panel.boundary_indices[k] == end_pos_idx and end_bi == -1:
            end_bi = k

    if start_bi == -1 or end_bi == -1:
        # Fallback: return just the two endpoints
        return np.array([start_pos_idx, end_pos_idx], dtype=np.int32)

    # Walk the shorter direction around the boundary
    d_fwd = (end_bi - start_bi) % n
    d_bwd = (start_bi - end_bi) % n

    if d_fwd <= d_bwd:
        indices = [int(panel.boundary_indices[(start_bi + k) % n])
                   for k in range(d_fwd + 1)]
    else:
        indices = [int(panel.boundary_indices[(start_bi - k) % n])
                   for k in range(d_bwd + 1)]

    # Deduplicate while preserving order (triangle library may map multiple
    # boundary_indices entries to the same mesh vertex)
    seen = set()
    unique = []
    for idx in indices:
        if idx not in seen:
            seen.add(idx)
            unique.append(idx)

    return np.array(unique, dtype=np.int32)


def build_garment_mesh(
    pattern_path: str | Path,
    resolution: int = 20,
    global_scale: float = 1.0,
    target_edge: float = 0.020,
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
        target_edge:  Target boundary/interior edge length in metres.
                      Controls stitch density: 0.020m gives ~50% more stitch
                      pairs than the old 0.030m default. Passed directly to
                      triangulate_panel(), overriding the resolution-derived
                      value when explicitly set.

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
        panel_local = triangulate_panel(pspec["vertices_2d"], resolution=resolution, target_edge=target_edge)
        world_pos   = _apply_placement(panel_local.positions, pspec["placement"], global_scale=global_scale)
        if "sleeve" in pspec["id"].lower():
            # Force cylindrical wrap for ALL sleeves regardless of aspect ratio
            # This is critical for the placement to clear the body and for
            # the sewing heuristic to calculate non-crossed distances properly.
            world_pos = _cylindrical_wrap_sleeve(world_pos, pspec["placement"])

            # Fix face winding after cylindrical wrap.  The wrap formula rotates
            # left-arm vs right-arm in opposite directions, which reverses the
            # CCW winding for one of the sleeves so its bending normals point
            # inward (concave) instead of outward (convex).  Sample up to 10
            # faces: if the majority of normals dot negative against the outward
            # radial from the arm centre, flip all triangles in the panel.
            arm_cx = float(pspec["placement"]["position"][0])
            arm_cz = float(pspec["placement"]["position"][2])
            n_sample = min(10, len(panel_local.faces))
            sf = panel_local.faces[:n_sample]
            sv0, sv1, sv2 = world_pos[sf[:, 0]], world_pos[sf[:, 1]], world_pos[sf[:, 2]]
            snormals  = np.cross(sv1.astype(np.float64) - sv0, sv2.astype(np.float64) - sv0)
            scents    = ((sv0 + sv1 + sv2) / 3.0).astype(np.float64)
            sradial   = scents - np.array([arm_cx, 0.0, arm_cz])
            sradial[:, 1] = 0.0
            srad_norm = np.linalg.norm(sradial, axis=1, keepdims=True)
            srad_norm = np.maximum(srad_norm, 1e-9)
            sdots     = np.sum(snormals * (sradial / srad_norm), axis=1)
            if sdots.mean() < 0:
                panel_local.faces[:, [1, 2]] = panel_local.faces[:, [2, 1]]
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
    edges_raw = np.concatenate(all_edges,     axis=0).astype(np.int32)
    uvs       = np.concatenate(all_uvs,       axis=0).astype(np.float32)

    # Use all edges provided by the high-quality triangulator
    edges = edges_raw

    # --- Step 5: Resolve stitch definitions → global index pairs ---
    panel_id_to_idx = {pid: i for i, pid in enumerate(panel_ids)}
    stitch_pairs_list: list[tuple[int, int]] = []
    seam_ids: list[str] = []

    for s_idx, sdef in enumerate(stitch_specs):
        pa_idx = panel_id_to_idx[sdef["panel_a"]]
        pb_idx = panel_id_to_idx[sdef["panel_b"]]
        ea = sdef["edge_a"]   # [v_start, v_end] in polygon outline space
        eb = sdef["edge_b"]
        seam_label = sdef.get("comment", f"seam_{s_idx}")

        local_a = _find_edge_particles(panels_local[pa_idx], ea[0], ea[1])
        local_b = _find_edge_particles(panels_local[pb_idx], eb[0], eb[1])

        # Match by relative position along edge.  Use max() so every vertex on
        # both edges gets at least one stitch partner (no orphan seam tips).
        # When the two edges differ by ≤ 15 % (ratio ≤ 1.15), switch to min()
        # instead: the difference is only 1–2 vertices and using min() avoids
        # the shorter side being oversampled (double stitch force on those
        # vertices causes surface ridges on tight-mesh seams like sleeve caps).
        len_a, len_b = len(local_a), len(local_b)
        longer, shorter = max(len_a, len_b), min(len_a, len_b)
        if shorter > 0 and longer / shorter <= 1.15:
            n_pts = shorter   # tiny imbalance — avoid clustering
        else:
            n_pts = longer    # large imbalance — keep full coverage
        if n_pts == 0:
            continue

        # Subsample linearly
        idx_a = np.linspace(0, len(local_a) - 1, n_pts, dtype=int)
        idx_b = np.linspace(0, len(local_b) - 1, n_pts, dtype=int)

        off_a = panel_offsets[pa_idx]
        off_b = panel_offsets[pb_idx]

        for i in range(n_pts):
            stitch_pairs_list.append((int(local_a[idx_a[i]]) + off_a, int(local_b[idx_b[i]]) + off_b))
        seam_ids.extend([seam_label] * n_pts)

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
        stitch_seam_ids=seam_ids if seam_ids else None,
    )
