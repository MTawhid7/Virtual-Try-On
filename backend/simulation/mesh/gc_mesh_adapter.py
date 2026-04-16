"""
GarmentCode → garment-sim adapter.

Converts a loaded `BoxMesh` (from vendored pygarment) into a `GarmentMesh`
that can be fed directly into garment-sim's simulation pipeline.

Key design decisions:
    * We do NOT use BoxMesh's collapsed mesh (where stitch vertices are merged).
      Instead we keep panels separate and generate stitch_pairs — spring
      constraints processed by the XPBD solver.
    * BoxMesh already handles edge subdivision matching (both sides of a
      stitch have the same number of vertices), so stitch_pairs line up 1:1.
    * Coordinate conversion: GarmentCode uses centimetres (Y-up), garment-sim
      uses metres (Y-up). We scale by 0.01.
"""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from simulation.mesh.panel_builder import GarmentMesh
from pygarment.meshgen.boxmeshgen import BoxMesh


def _extract_edges_from_faces(
    faces: NDArray[np.int32],
    positions: NDArray[np.float32] | None = None,
    min_edge_length: float = 0.0,
) -> NDArray[np.int32]:
    """Extract unique undirected structural edges from triangle faces.

    Args:
        faces: (F, 3) triangle indices
        positions: (N, 3) vertex positions — required if min_edge_length > 0
        min_edge_length: Minimum edge length (metres). Edges shorter than this
            are excluded from the structural constraint set to prevent extreme
            stretch ratios from GarmentCode's tiny dart/notch geometry.
    """
    edge_set: set[tuple[int, int]] = set()
    for f in faces:
        for k in range(3):
            a, b = int(f[k]), int(f[(k + 1) % 3])
            edge_set.add((min(a, b), max(a, b)))

    edges = np.array(sorted(edge_set), dtype=np.int32)

    if min_edge_length > 0 and positions is not None and len(edges) > 0:
        lengths = np.linalg.norm(
            positions[edges[:, 1]] - positions[edges[:, 0]], axis=1
        )
        mask = lengths >= min_edge_length
        edges = edges[mask]

    return edges


def boxmesh_to_garment_mesh(
    bm: BoxMesh,
    cm_to_m: float = 0.01,
    fabric: str = "cotton",
    body_z_offset: float = 0.0,
) -> GarmentMesh:
    """
    Convert a loaded (but NOT collapsed) BoxMesh into a GarmentMesh.

    This function uses the per-panel mesh data from BoxMesh (after
    triangulation but BEFORE stitch vertex collapse) to produce a mesh
    format compatible with garment-sim's XPBD simulation pipeline.

    The stitch vertex collapse that BoxMesh does is designed for Warp-based
    simulation (manifold mesh). Our Taichi-based XPBD solver uses spring
    constraints instead, so we keep panels separate and generate stitch_pairs.

    Args:
        bm:             A BoxMesh that has been loaded via bm.load().
        cm_to_m:        Scale factor from pattern units to metres (0.01 for cm→m).
        fabric:         Fabric preset name for the GarmentMesh.
        body_z_offset:  Metres to add to every panel's Z coordinate after cm→m
                        scaling. Use this to align GarmentCode's SMPL-derived
                        panel positions onto a different body mesh.

                        GarmentCode's default body (SMPL mean) has its torso
                        centred at Z≈+0.025m (after cm→m). Our mannequin_physics
                        body is centred at Z≈+0.156m, so pass body_z_offset=0.131
                        when simulating on that mannequin.

                        Default is 0.0 so existing unit tests that use synthetic
                        panels at known absolute positions are unaffected.

    Returns:
        GarmentMesh ready for ParticleState.load_from_numpy().
    """
    if not bm.loaded:
        raise RuntimeError("BoxMesh must be loaded before conversion. Call bm.load() first.")

    panel_names = bm.panelNames
    all_positions: list[NDArray[np.float32]] = []
    all_faces: list[NDArray[np.int32]] = []
    all_edges: list[NDArray[np.int32]] = []
    all_uvs: list[NDArray[np.float32]] = []
    panel_offsets: list[int] = []
    panel_ids: list[str] = []

    # Per-panel vertex offset into the global array
    offset = 0

    for panel_name in panel_names:
        panel = bm.panels[panel_name]

        # --- 3D positions (cm → m) ---
        # panel.panel_vertices are 2D, rot_trans_panel lifts to 3D
        verts_3d = panel.rot_trans_panel(panel.panel_vertices)
        verts_3d = np.array(verts_3d, dtype=np.float64) * cm_to_m
        positions = verts_3d.astype(np.float32)  # (N_panel, 3)

        # Align GarmentCode SMPL body space → target body mesh space.
        # Applied per-panel (before stitch densification) so all geometry
        # is in the correct coordinate space when midpoints are interpolated.
        if body_z_offset != 0.0:
            positions[:, 2] += body_z_offset

        # --- Faces (offset into global) ---
        faces_local = np.array([np.asarray(f) for f in panel.panel_faces], dtype=np.int32)
        faces_global = faces_local + offset

        # --- Structural edges from faces ---
        # Filter edges shorter than 40% of mesh_resolution (in m) to prevent
        # extreme stretch from GarmentCode dart/notch geometry
        min_edge_m = 0.4 * cm_to_m * getattr(bm, 'mesh_resolution', 2.0)
        edges_local = _extract_edges_from_faces(faces_local, positions, min_edge_m)
        edges_global = edges_local + offset

        # --- UV coordinates (2D panel positions normalized to [0,1]²) ---
        verts_2d = np.array([np.asarray(v) for v in panel.panel_vertices], dtype=np.float32)
        bb_min = verts_2d.min(axis=0)
        bb_max = verts_2d.max(axis=0)
        extent = bb_max - bb_min
        uvs = np.zeros((len(verts_2d), 2), dtype=np.float32)
        uvs[:, 0] = (verts_2d[:, 0] - bb_min[0]) / max(float(extent[0]), 1e-8)
        uvs[:, 1] = (verts_2d[:, 1] - bb_min[1]) / max(float(extent[1]), 1e-8)

        panel_offsets.append(offset)
        panel_ids.append(panel_name)
        all_positions.append(positions)
        all_faces.append(faces_global)
        all_edges.append(edges_global)
        all_uvs.append(uvs)
        offset += len(positions)

    # --- Merge global arrays ---
    positions_merged = np.concatenate(all_positions, axis=0).astype(np.float32)
    faces_merged = np.concatenate(all_faces, axis=0).astype(np.int32)
    edges_merged = np.concatenate(all_edges, axis=0).astype(np.int32)
    uvs_merged = np.concatenate(all_uvs, axis=0).astype(np.float32)

    # --- Build stitch_pairs from BoxMesh stitch edge vertex_ranges ---
    # BoxMesh guarantees that stitch_range_1 and stitch_range_2 have the same
    # length and are in matching order. We pair them as spring constraints.
    stitch_pairs_list: list[tuple[int, int]] = []
    seam_ids: list[str] = []

    # Build panel name → index map for offset lookup
    panel_name_to_idx = {name: i for i, name in enumerate(panel_ids)}

    # Minimum stitch pairs per seam. Short seams on coarse meshes can produce
    # very few natural pairs; densification brings them to this target so the
    # XPBD solver has enough spring density to close the gap.
    _MIN_PAIRS_PER_SEAM = 12

    for stitch_idx, stitch in enumerate(bm.stitches):
        stitch_range_1, stitch_range_2 = bm._swap_stitch_ranges(stitch)

        off_1 = panel_offsets[panel_name_to_idx[stitch.panel_1]]
        off_2 = panel_offsets[panel_name_to_idx[stitch.panel_2]]

        seam_label = stitch.label if stitch.label else f"seam_{stitch_idx}"

        seam_pairs: list[tuple[int, int]] = []
        for loc_1, loc_2 in zip(stitch_range_1, stitch_range_2):
            global_1 = loc_1 + off_1
            global_2 = loc_2 + off_2
            if global_1 == global_2:
                continue
            seam_pairs.append((int(global_1), int(global_2)))

        # Densify sparse seams iteratively.
        #
        # On coarse meshes a short stitch edge may produce only 2–4 base pairs.
        # We insert extra pairs at the midpoints between consecutive pairs,
        # looping until we reach _MIN_PAIRS_PER_SEAM or until an iteration
        # adds no new pairs (converged — mesh is too coarse for more).
        #
        # The nearest-vertex search operates on non-overlapping per-panel index
        # ranges, so near_1 and near_2 can never be the same global index.
        # We deduplicate using a set so repeated nearest-vertex hits (due to
        # coarse mesh resolution) don't inflate the count without adding coverage.
        if 0 < len(seam_pairs) < _MIN_PAIRS_PER_SEAM:
            p1_idx = panel_name_to_idx[stitch.panel_1]
            p2_idx = panel_name_to_idx[stitch.panel_2]
            p1_start = panel_offsets[p1_idx]
            p1_end = (panel_offsets[p1_idx + 1]
                      if p1_idx + 1 < len(panel_offsets)
                      else len(positions_merged))
            p2_start = panel_offsets[p2_idx]
            p2_end = (panel_offsets[p2_idx + 1]
                      if p2_idx + 1 < len(panel_offsets)
                      else len(positions_merged))
            panel_1_pos = positions_merged[p1_start:p1_end]
            panel_2_pos = positions_merged[p2_start:p2_end]

            existing: set[tuple[int, int]] = set(seam_pairs)

            for _ in range(4):  # up to 4 densification passes
                if len(seam_pairs) >= _MIN_PAIRS_PER_SEAM:
                    break
                added_this_pass = 0
                new_pairs: list[tuple[int, int]] = []
                for i in range(len(seam_pairs) - 1):
                    g1_a, g2_a = seam_pairs[i]
                    g1_b, g2_b = seam_pairs[i + 1]
                    mid_1 = (positions_merged[g1_a] + positions_merged[g1_b]) / 2
                    mid_2 = (positions_merged[g2_a] + positions_merged[g2_b]) / 2
                    near_1 = int(np.argmin(np.linalg.norm(panel_1_pos - mid_1, axis=1))) + p1_start
                    near_2 = int(np.argmin(np.linalg.norm(panel_2_pos - mid_2, axis=1))) + p2_start
                    candidate = (near_1, near_2)
                    if candidate not in existing:
                        new_pairs.append(candidate)
                        existing.add(candidate)
                        added_this_pass += 1
                seam_pairs = seam_pairs + new_pairs
                if added_this_pass == 0:
                    break  # mesh too coarse; no new vertices to discover

        stitch_pairs_list.extend(seam_pairs)
        seam_ids.extend([seam_label] * len(seam_pairs))

    stitch_pairs = (
        np.array(stitch_pairs_list, dtype=np.int32)
        if stitch_pairs_list
        else np.zeros((0, 2), dtype=np.int32)
    )

    return GarmentMesh(
        positions=positions_merged,
        faces=faces_merged,
        edges=edges_merged,
        uvs=uvs_merged,
        stitch_pairs=stitch_pairs,
        panel_offsets=panel_offsets,
        panel_ids=panel_ids,
        fabric=fabric,
        stitch_seam_ids=seam_ids if seam_ids else None,
    )


def build_garment_mesh_gc(
    pattern_path: str | Path,
    mesh_resolution: float = 2.0,
    fabric: str = "cotton",
    body_z_offset: float = 0.0,
) -> GarmentMesh:
    """
    Build a GarmentMesh from a GarmentCode pattern specification JSON.

    This is the main entry point for Phase 2 — it replaces / complements
    the existing `build_garment_mesh()` function for GarmentCode patterns.

    The pipeline:
        1. Load the GarmentCode pattern JSON into a BoxMesh
        2. BoxMesh.load() → triangulate panels + match stitch edges
        3. boxmesh_to_garment_mesh() → convert to GarmentMesh (cm → m)

    Args:
        pattern_path:   Path to a GarmentCode pattern specification JSON.
        mesh_resolution: Edge length in cm for triangulation (default 2.0 cm).
                        Lower = denser mesh. GarmentCode default is 1.0 cm,
                        but 2.0 gives good balance for simulation.
        fabric:         Fabric preset name (default "cotton").
        body_z_offset:  Metres added to every panel's Z after cm→m scaling.
                        Pass 0.131 when simulating on mannequin_physics.glb
                        (default 0.0 keeps unit tests unaffected).

    Returns:
        GarmentMesh ready for the XPBD simulation pipeline.
    """
    path = Path(pattern_path)
    if not path.exists():
        raise FileNotFoundError(f"Pattern file not found: {path}")

    bm = BoxMesh(str(path), res=mesh_resolution)
    bm.load()

    return boxmesh_to_garment_mesh(bm, fabric=fabric, body_z_offset=body_z_offset)
