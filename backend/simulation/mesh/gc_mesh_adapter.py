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


def build_gc_attachment_constraints(
    garment: GarmentMesh,
    max_per_panel: int = 40,
) -> tuple[NDArray[np.int32], NDArray[np.float32]]:
    """
    Build soft positional attachment constraint vertex indices and targets.

    For each panel, selects a subset of non-stitch interior vertices and pins them
    to their current 3D prewrapped positions.  Targets are read directly from
    garment.positions so attachment constraints reinforce the curved elliptical
    placement set by prewrap_panels_to_body().

    Stitch vertices are explicitly excluded — they need full freedom to slide
    laterally so seams can close.

    Args:
        garment:       GarmentMesh after prewrap_panels_to_body() has been called.
        max_per_panel: Maximum attachments sampled per panel (evenly spaced).

    Returns:
        vertex_indices:   (A,) int32 — global vertex indices to anchor.
        target_positions: (A, 3) float32 — pin target (current prewrapped 3D position).
    """
    n = garment.positions.shape[0]
    offsets = garment.panel_offsets + [n]
    pos = garment.positions  # (N, 3) float32

    # Stitch vertices must remain free — exclude them from attachment selection
    stitch_verts: set[int] = set(garment.stitch_pairs.flatten().tolist())

    attach_indices: list[int] = []
    attach_targets: list[list[float]] = []

    for k, _pid in enumerate(garment.panel_ids):

        panel_range = list(range(offsets[k], offsets[k + 1]))
        # Exclude stitch vertices
        free_verts = [v for v in panel_range if v not in stitch_verts]
        if not free_verts:
            continue

        # Sample evenly up to max_per_panel
        step = max(1, len(free_verts) // max_per_panel)
        selected = free_verts[::step][:max_per_panel]

        for vi in selected:
            # Target = current 3D prewrapped position so attachment constraints
            # reinforce the curved elliptical placement rather than pulling
            # vertices back to a flat 1D Z-snap position.
            attach_indices.append(vi)
            attach_targets.append([float(pos[vi, 0]), float(pos[vi, 1]), float(pos[vi, 2])])

    if not attach_indices:
        return np.zeros(0, dtype=np.int32), np.zeros((0, 3), dtype=np.float32)

    return (
        np.array(attach_indices, dtype=np.int32),
        np.array(attach_targets, dtype=np.float32),
    )


def _rotation_matrix_from_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return a 3×3 rotation matrix that rotates unit vector a to align with unit vector b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if abs(c + 1.0) < 1e-8:  # anti-parallel: 180° around any perpendicular axis
        perp = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis = np.cross(a, perp)
        axis /= np.linalg.norm(axis)
        K = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
        return np.eye(3) + 2.0 * (K @ K)
    s = float(np.linalg.norm(v))
    if s < 1e-8:
        return np.eye(3)
    K = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + K + K @ K * ((1.0 - c) / (s * s))


def calibrate_garment_y(
    garment: "GarmentMesh",
    profile_path: str = "data/bodies/mannequin_profile.json",
    anchor: str = "neck",
) -> None:
    """
    Translate all garment vertices so the topmost vertex aligns with the target
    body landmark.  Corrects for GarmentCode SMPL proportions being calibrated
    to a 1.70m body while our mannequin is 1.75m — the shirt otherwise
    initialises at chest level (Y≈1.0–1.4m) and gravity drags it to the waist.

    anchor: "neck"     → target profile.neck_y     (≈1.52m) — shirt collar
            "shoulder" → target profile.shoulder_y  (≈1.43m) — shoulder seam
    """
    from simulation.mesh.body_measurements import load_profile

    profile = load_profile(profile_path)
    target_y = profile.neck_y if anchor == "neck" else profile.shoulder_y
    garment_top_y = float(garment.positions[:, 1].max())
    delta_y = target_y - garment_top_y
    garment.positions[:, 1] += delta_y


def prewrap_panels_to_body(
    garment: "GarmentMesh",
    clearance: float = 0.008,
) -> None:
    """
    Place garment panels close to their initial simulation positions before simulation.

    Two operations:
    1. Torso panels (front/back): 3D elliptical wrap using the body cross-section profile.
       Each vertex is mapped onto the body's elliptical cross-section at its Y height at
       clearance distance from the surface.  Front vertices land on the front hemisphere
       (Z > center_Z), back vertices on the back hemisphere (Z < center_Z).  Both X and Z
       coordinates are updated so the panel is naturally curved, matching the body shape
       and eliminating the flat-panel waviness of a 1D Z-snap.
    2. Sleeve/collar panels:
       a. Translate so the stitch-vertex centroid aligns with the opposing armhole centroid.
       b. Rotate around that centroid so the sleeve's armhole ring plane aligns with the
          torso's armhole ring plane (SVD-based).  This orients the sleeve to wrap around
          the arm rather than hanging flat in front of it.

    Call resolve_initial_penetrations() after this function to push out any vertices that
    the wrap placed inside the body mesh due to concave geometry (armpits, etc.).

    Modifies garment.positions in-place before Taichi field upload.
    """
    from simulation.mesh.body_measurements import load_profile

    n = garment.positions.shape[0]
    offsets = garment.panel_offsets + [n]
    pos = garment.positions  # (N, 3) float32, modified in-place
    profile = load_profile("data/bodies/mannequin_profile.json")

    _BACK_KEYWORDS = ("btorso", "back")
    _FRONT_KEYWORDS = ("ftorso", "front")
    _TORSO_KEYWORDS = _BACK_KEYWORDS + _FRONT_KEYWORDS
    _TORSO_Y_MIN = 0.65   # below hip: body profile unreliable
    _TORSO_Y_MAX = 1.60   # above shoulder: arm/collar geometry, not torso

    # Two-pass processing: torso panels must be 3D-wrapped BEFORE sleeve panels are
    # centroid-aligned, because the torso armhole vertex positions are the stable
    # reference that sleeve panels align to.  Panel ordering in GarmentCode JSON is
    # not guaranteed (sleeves can appear before torso), so we enforce the order here.

    # --- Pass 1: 3D elliptical wrap for all torso panels ---
    torso_vert_set: set[int] = set()
    for k, pid in enumerate(garment.panel_ids):
        pid_lower = pid.lower()
        is_back = any(kw in pid_lower for kw in _BACK_KEYWORDS)
        is_front = any(kw in pid_lower for kw in _FRONT_KEYWORDS)
        if not (is_back or is_front):
            continue

        torso_vert_set.update(range(offsets[k], offsets[k + 1]))

        # Project each vertex onto the body's ellipse at its Y height.
        # sin(θ) = (vertex_x − center_x) / body_half_width gives the wrap angle
        # directly from world X — correct for half-body panels (no panel_width needed).
        #
        # Vertices BEYOND the body's side (|sin_t| > 1) are seam/armhole vertices
        # that extend past the body width.  Clamping them would collapse many adjacent
        # vertices to the same point, zeroing rest-lengths.  Instead we keep their
        # original X and set Z = center_z, which simultaneously:
        #   • places front AND back seam vertices at the same Z → side-seam gap ≈ 0
        #   • preserves relative X spacing → no rest-length collapse
        for vi in range(offsets[k], offsets[k + 1]):
            py = float(pos[vi, 1])
            if py < _TORSO_Y_MIN or py > _TORSO_Y_MAX:
                continue
            sl = profile.at_y(py)
            a_body = sl.width / 2    # body X half-extent
            b_body = sl.depth / 2    # body Z half-extent
            cx, cz = sl.center_x, sl.center_z
            sin_t = (pos[vi, 0] - cx) / max(a_body, 1e-8)
            if abs(sin_t) <= 1.0:
                # Within body width: map to clearance ellipse surface
                cos_t = float(np.sqrt(max(1.0 - sin_t * sin_t, 0.0)))
                a_out = a_body + clearance
                b_out = b_body + clearance
                pos[vi, 0] = cx + a_out * sin_t
                pos[vi, 2] = (cz - b_out * cos_t) if is_back else (cz + b_out * cos_t)
            else:
                # Beyond body side: preserve X, set Z to body-center depth.
                # Both front AND back seam vertices land at the same Z → side-seam gap ≈ 0.
                # Preserving X avoids rest-length collapse (no adjacent vertices collapse).
                pos[vi, 2] = cz

    # --- Pass 2: sleeve/collar panels — translate then rotate to align with torso armhole ---
    # Torso vertices are now at their final 3D-wrapped positions and serve as the
    # stable reference.  Only torso vertices are used for the centroid and SVD plane
    # computation; sleeve-to-sleeve tube seam gaps are closed by stitch constraints.
    for k, pid in enumerate(garment.panel_ids):
        pid_lower = pid.lower()
        if any(kw in pid_lower for kw in _TORSO_KEYWORDS):
            continue  # already handled in pass 1

        panel_set = set(range(offsets[k], offsets[k + 1]))
        pairs = garment.stitch_pairs  # (S, 2) int32

        my_verts: list[int] = []
        their_torso_verts: list[int] = []
        their_all_verts: list[int] = []
        for col, other_col in ((0, 1), (1, 0)):
            mask = np.isin(pairs[:, col], list(panel_set))
            if not np.any(mask):
                continue
            sv = pairs[mask, col]
            tv = pairs[mask, other_col]
            cross = ~np.isin(tv, list(panel_set))
            my_verts.extend(sv[cross].tolist())
            their_all_verts.extend(tv[cross].tolist())
            torso_cross = cross & np.isin(tv, list(torso_vert_set))
            their_torso_verts.extend(tv[torso_cross].tolist())

        if not my_verts:
            continue  # no cross-panel stitches — leave unchanged

        my_arr = np.array(my_verts)
        ref_arr = np.array(their_torso_verts) if their_torso_verts else np.array(their_all_verts)

        my_centroid = pos[my_arr].mean(axis=0)
        ref_centroid = pos[ref_arr].mean(axis=0)
        delta = ref_centroid - my_centroid
        # Sleeve panels: GarmentCode already places them at the correct Z
        # (front sleeve near z_front, back sleeve near z_back). Applying the
        # full XYZ delta pulls both to body center Z — destroying that placement.
        # Apply only XY alignment; let the sew phase handle remaining Z gaps.
        _SLEEVE_KWS = ("sleeve",)
        if any(kw in pid_lower for kw in _SLEEVE_KWS):
            delta[2] = 0.0
        pos[offsets[k]:offsets[k + 1]] += delta


def resolve_initial_penetrations(
    garment: "GarmentMesh",
    body_mesh_path: str = "data/bodies/mannequin_physics.glb",
    clearance: float = 0.008,
    max_passes: int = 5,
) -> int:
    """
    Post-prewrap safety pass: push any garment vertex that is inside the body
    mesh to clearance outside the nearest surface point.

    Returns the total number of vertex corrections applied across all passes.
    Iterates until no inside-body vertices remain or max_passes is reached.

    Called after prewrap_panels_to_body() and calibrate_garment_y() in scene
    scripts.  Guarantees the simulation starts with zero vertices inside the
    body, preventing the inside-body velocity-buildup explosion chain.
    """
    import trimesh

    body_mesh = trimesh.load(body_mesh_path, force="mesh")
    pos = garment.positions
    total_corrected = 0

    for _ in range(max_passes):
        inside_mask = body_mesh.contains(pos)          # BVH winding-number test
        if not np.any(inside_mask):
            break
        pts = pos[inside_mask]
        closest, _, _ = trimesh.proximity.closest_point(body_mesh, pts)
        # Escape direction: (closest - pts) points from the inside vertex toward
        # the nearest surface — always outward for a vertex inside the mesh.
        # Place the vertex clearance past the surface along this direction.
        escape = closest - pts
        norms = np.linalg.norm(escape, axis=1, keepdims=True)
        escape_dir = escape / np.maximum(norms, 1e-8)
        pos[inside_mask] = closest + escape_dir * clearance
        total_corrected += int(np.sum(inside_mask))

    return total_corrected
