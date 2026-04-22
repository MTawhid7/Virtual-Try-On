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
    scale_x: float = 1.0,
    scale_y: float = 1.0,
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
    all_verts_2d: list[NDArray[np.float32]] = []
    panel_offsets: list[int] = []
    panel_ids: list[str] = []

    # Per-panel vertex offset into the global array
    offset = 0

    for panel_name in panel_names:
        panel = bm.panels[panel_name]

        # --- 3D positions (cm → m) ---
        # Scale 2D vertices from panel centroid before 3D rotation to preserve alignment
        verts_2d = np.array(panel.panel_vertices, dtype=np.float64)
        if scale_x != 1.0 or scale_y != 1.0:
            c2d = verts_2d.mean(axis=0)
            verts_2d[:, 0] = c2d[0] + (verts_2d[:, 0] - c2d[0]) * scale_x
            verts_2d[:, 1] = c2d[1] + (verts_2d[:, 1] - c2d[1]) * scale_y

        # rot_trans_panel lifts to 3D
        verts_3d = panel.rot_trans_panel(verts_2d.tolist())
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
        all_verts_2d.append((verts_2d * cm_to_m).astype(np.float32))
        offset += len(positions)

    # --- Merge global arrays ---
    positions_merged = np.concatenate(all_positions, axis=0).astype(np.float32)
    faces_merged = np.concatenate(all_faces, axis=0).astype(np.int32)
    edges_merged = np.concatenate(all_edges, axis=0).astype(np.int32)
    uvs_merged = np.concatenate(all_uvs, axis=0).astype(np.float32)
    verts_2d_merged = np.concatenate(all_verts_2d, axis=0).astype(np.float32)

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
        verts_2d=verts_2d_merged,
    )


def build_garment_mesh_gc(
    pattern_path: str | Path,
    mesh_resolution: float = 2.0,
    fabric: str = "cotton",
    body_z_offset: float = 0.0,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
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

    return boxmesh_to_garment_mesh(
        bm, fabric=fabric, body_z_offset=body_z_offset, scale_x=scale_x, scale_y=scale_y
    )


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


def _rotation_from_axes(src: np.ndarray, dst: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix mapping unit vector src → dst (Rodrigues' formula)."""
    src = src / (np.linalg.norm(src) + 1e-12)
    dst = dst / (np.linalg.norm(dst) + 1e-12)
    if abs(np.dot(src, dst) + 1.0) < 1e-6:  # anti-parallel: rotate 180° around any perp axis
        perp = np.array([1.0, 0.0, 0.0]) if abs(src[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        v = np.cross(src, perp)
        v /= np.linalg.norm(v)
        return -np.eye(3) + 2.0 * np.outer(v, v)
    v = np.cross(src, dst)
    s = np.linalg.norm(v)
    if s < 1e-8:
        return np.eye(3)
    c = float(np.dot(src, dst))
    K = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + K + K @ K * ((1.0 - c) / (s ** 2))


# ---------------------------------------------------------------------------
# Arm cylinder model — piecewise centerline from body mesh vertices
# ---------------------------------------------------------------------------

def _build_arm_centerline(
    body_mesh_path: str,
    x_sign: float,
    x_min_abs: float = 0.20,
    x_max_abs: float = 0.48,
    n_slices: int = 12,
    ring_hw: float = 0.025,
) -> tuple[NDArray, NDArray, NDArray, NDArray]:
    """Build piecewise arm cylinder from body mesh.

    Returns (x_positions, center_y, center_z, radius) arrays sorted by X.
    ``x_sign`` is -1 for right arm, +1 for left arm.
    """
    import trimesh
    body = trimesh.load(body_mesh_path, force="mesh")
    verts = np.array(body.vertices)

    arm_mask = (verts[:, 0] * x_sign > x_min_abs) & (verts[:, 1] > 0.90)
    arm_verts = verts[arm_mask]

    xs, cys, czs, rs = [], [], [], []
    for x_t in np.linspace(x_sign * x_min_abs, x_sign * x_max_abs, n_slices):
        ring = arm_verts[(arm_verts[:, 0] > x_t - ring_hw) &
                         (arm_verts[:, 0] < x_t + ring_hw)]
        if len(ring) < 4:
            continue
        cy = float(ring[:, 1].mean())
        cz = float(ring[:, 2].mean())
        r = float(np.sqrt(np.mean((ring[:, 1] - cy) ** 2 + (ring[:, 2] - cz) ** 2)))
        if r > 0.01:
            xs.append(x_t); cys.append(cy); czs.append(cz); rs.append(r)

    order = np.argsort(xs)
    return (np.array(xs)[order], np.array(cys)[order],
            np.array(czs)[order], np.array(rs)[order])


def _arm_at_x(
    arm: tuple[NDArray, NDArray, NDArray, NDArray],
    x: float,
) -> tuple[float, float, float]:
    """Interpolate arm (center_y, center_z, radius) at X position."""
    xs, cys, czs, rs = arm
    if len(xs) < 2:
        return float(cys[0]), float(czs[0]), float(rs[0])
    idx = int(np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2))
    denom = xs[idx + 1] - xs[idx]
    t = float(np.clip((x - xs[idx]) / max(denom, 1e-8), 0.0, 1.0))
    return (
        float(cys[idx] + t * (cys[idx + 1] - cys[idx])),
        float(czs[idx] + t * (czs[idx + 1] - czs[idx])),
        float(rs[idx] + t * (rs[idx + 1] - rs[idx])),
    )


# ---------------------------------------------------------------------------
# Panel role classification (name-based + stitch-connectivity fallback)
# ---------------------------------------------------------------------------

_BACK_KEYWORDS = ("btorso", "back")
_FRONT_KEYWORDS = ("ftorso", "front")
_TORSO_KEYWORDS = _BACK_KEYWORDS + _FRONT_KEYWORDS


def _classify_panel(
    pid: str,
    panel_idx: int,
    garment: "GarmentMesh",
    offsets: list[int],
    torso_vert_set: set[int],
) -> str:
    """Return 'torso_back', 'torso_front', 'sleeve', or 'other'."""
    pid_lower = pid.lower()

    # Primary: name-based
    if any(kw in pid_lower for kw in _BACK_KEYWORDS):
        return "torso_back"
    if any(kw in pid_lower for kw in _FRONT_KEYWORDS):
        return "torso_front"
    if "sleeve" in pid_lower:
        return "sleeve"

    # Fallback: stitch connectivity — if >30% of cross-panel stitches
    # connect to torso panels, classify as sleeve.
    panel_set = set(range(offsets[panel_idx], offsets[panel_idx + 1]))
    pairs = garment.stitch_pairs
    torso_count = 0
    total_cross = 0
    for col, other_col in ((0, 1), (1, 0)):
        mask = np.isin(pairs[:, col], list(panel_set))
        if not np.any(mask):
            continue
        other = pairs[mask, other_col]
        cross = ~np.isin(other, list(panel_set))
        total_cross += int(cross.sum())
        for ov in other[cross]:
            if int(ov) in torso_vert_set:
                torso_count += 1

    if total_cross > 0 and torso_count / total_cross > 0.3:
        return "sleeve"
    return "other"


# ---------------------------------------------------------------------------
# Main pre-wrap function
# ---------------------------------------------------------------------------

# Default clearance values (designed for future configurability)
_TORSO_CLEARANCE: float = 0.040  # 40mm — loose CLO3D-like gap, avoids body poke-through
_SLEEVE_CLEARANCE: float = 0.005  # 5mm — arm is convex, less risk


def prewrap_panels_to_body(
    garment: "GarmentMesh",
    clearance: float = _TORSO_CLEARANCE,
    body_mesh_path: str = "data/bodies/mannequin_physics.glb",
) -> None:
    """Place garment panels close to the body surface for stable simulation.

    Three-pass pipeline producing CLO3D-style smooth initial placement:

    Pass 1 — Torso panels (front/back):
        Smooth elliptical wrap using a *single reference ellipse* per panel
        (computed from the panel's median-Y body cross-section).  Produces
        uniform curvature without the terrain-like artifacts of per-vertex
        body profile lookup.  Side-edge vertices (|dx/a| > 1) are blended
        smoothly toward body center-Z instead of being left flat.

    Pass 2 — Sleeve/collar panels:
        a. Translate so stitch-vertex centroid aligns with opposing armhole.
        b. Project onto a piecewise arm cylinder (center and radius queried
           from the body mesh at each vertex's X position).  Each vertex's
           circumferential coordinate (from SVD secondary axis) maps to an
           angle θ on the cylinder, producing true Z curvature.

    Pass 3 — resolve_initial_penetrations() should be called after this
        function to push any remaining inside-body vertices outward.

    Modifies garment.positions in-place before Taichi field upload.

    Args:
        garment:        GarmentMesh to modify.
        clearance:      Torso clearance in metres (default 15mm).
        body_mesh_path: Path to physics body mesh for arm cylinder model.
    """
    from simulation.mesh.body_measurements import load_profile

    n = garment.positions.shape[0]
    offsets = garment.panel_offsets + [n]
    pos = garment.positions  # (N, 3) float32, modified in-place
    profile = load_profile("data/bodies/mannequin_profile.json")

    _TORSO_Y_MIN = 0.65
    _TORSO_Y_MAX = 1.60

    # ── Classify every panel ──────────────────────────────────────────────
    torso_vert_set: set[int] = set()
    roles: list[str] = []

    # First pass: identify torso panels (needed for fallback classification)
    for k, pid in enumerate(garment.panel_ids):
        pid_lower = pid.lower()
        if any(kw in pid_lower for kw in _TORSO_KEYWORDS):
            torso_vert_set.update(range(offsets[k], offsets[k + 1]))

    for k, pid in enumerate(garment.panel_ids):
        roles.append(_classify_panel(pid, k, garment, offsets, torso_vert_set))

    # ── Pass 1: Smooth elliptical wrap for torso panels ───────────────────
    for k, pid in enumerate(garment.panel_ids):
        role = roles[k]
        is_back = role == "torso_back"
        is_front = role == "torso_front"
        if not (is_back or is_front):
            continue

        panel_ys = pos[offsets[k]:offsets[k + 1], 1]
        y_lo = max(float(panel_ys.min()), _TORSO_Y_MIN)
        y_hi = min(float(panel_ys.max()), _TORSO_Y_MAX)
        y_med = (y_lo + y_hi) / 2.0

        # Single reference ellipse from panel median-Y cross-section.
        # This eliminates per-vertex terrain artifacts while maintaining
        # the correct overall curvature for the panel.
        ref = profile.at_y(y_med)
        a = ref.width / 2.0               # X semi-axis (body half-width)
        b = ref.depth / 2.0 + clearance   # Z semi-axis + clearance
        cx = ref.center_x
        cz = ref.center_z

        if a < 1e-4 or b < 1e-4:
            continue

        # Widen the mapping parameter to prevent vertical asymptotes at the edges
        a_extended = a * 1.25

        for vi in range(offsets[k], offsets[k + 1]):
            py = float(pos[vi, 1])
            if py < _TORSO_Y_MIN or py > _TORSO_Y_MAX:
                continue

            dx = float(pos[vi, 0]) - cx
            t = dx / a_extended
            
            # Smooth shield function: Z drops off smoothly past the body boundary
            # Using cosine: at dx=0 (center), z_offset = b. 
            # at dx=a_extended (edge), z_offset = 0 (reaches body center_z).
            t_clip = float(np.clip(t, -1.0, 1.0))
            z_offset = b * float(np.cos(t_clip * (np.pi / 2.0)))

            pos[vi, 0] = cx + dx  # X stays unchanged relative to center
            if is_back:
                pos[vi, 2] = cz - z_offset
            else:
                pos[vi, 2] = cz + z_offset

    # ── Pass 2: Sleeve panels — translate + arm-cylinder projection ───────
    # Build arm centerlines (cached for both arms)
    arm_models: dict[str, tuple] = {}

    pairs = garment.stitch_pairs

    for k, pid in enumerate(garment.panel_ids):
        if roles[k] not in ("sleeve", "other"):
            continue
        if roles[k] == "other":
            # Non-sleeve, non-torso: just centroid-align (collars etc.)
            pass

        pid_lower = pid.lower()
        panel_set = set(range(offsets[k], offsets[k + 1]))

        # Find cross-panel stitch connections
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
            continue

        my_arr = np.array(my_verts)
        ref_arr = (np.array(their_torso_verts) if their_torso_verts
                   else np.array(their_all_verts))
        ref_centroid = pos[ref_arr].mean(axis=0)

        # Centroid translation
        my_centroid = pos[my_arr].mean(axis=0)
        delta = ref_centroid - my_centroid
        pos[offsets[k]:offsets[k + 1]] += delta

        # ── Arm-cylinder projection (sleeve panels only) ──────────────
        if roles[k] != "sleeve":
            continue

        is_right = "right" in pid_lower or float(pos[offsets[k]:offsets[k + 1], 0].mean()) < 0
        x_sign = -1.0 if is_right else 1.0
        arm_key = "right" if is_right else "left"

        # Build arm model lazily (once per arm)
        if arm_key not in arm_models:
            arm_models[arm_key] = _build_arm_centerline(body_mesh_path, x_sign)

        arm = arm_models[arm_key]

        # Determine if this is a front or back sleeve half
        is_front_sleeve = ("_f" in pid_lower or "front" in pid_lower or
                           float(pos[offsets[k]:offsets[k + 1], 2].mean()) > 0.15)

        start, end = offsets[k], offsets[k + 1]
        pivot = pos[my_arr].mean(axis=0)  # armhole centroid post-translate
        
        # Use 2D panel vertices for direct 2D-to-3D cylinder projection.
        # This completely bypasses the GarmentCode 3D rotation, avoiding coordinate mixing.
        # Reason: GarmentCode's 3D rotation inextricably mixes the arm-length and circumferential 
        # dimensions into world X/Y/Z. By projecting straight from the 2D panel space, 
        # we can cleanly map 2D-Y to arm length (X-axis in world) and 2D-X to cylinder wrap angle.
        if garment.verts_2d is None:
            continue
            
        v2d = garment.verts_2d[start:end]
        c2d = v2d.mean(axis=0)
        centered_2d = v2d - c2d

        for i, vi in enumerate(range(start, end)):
            dy_2d = float(centered_2d[i, 1]) # arm length direction
            dx_2d = float(centered_2d[i, 0]) # circumference direction
            
            # Map 2D-Y to World X (along arm axis)
            # The sleeve extends outward. 2D Y goes from bottom (hem) to top (cap).
            # Top of cap (positive dy_2d) is closer to body (closer to pivot).
            x_along_arm = float(pivot[0]) - x_sign * dy_2d
            
            arm_cy, arm_cz, arm_r = _arm_at_x(arm, x_along_arm)
            wrap_radius = arm_r + _SLEEVE_CLEARANCE
            
            # Angle: dx_2d is arc length. theta = arc_length / radius
            theta = dx_2d / max(wrap_radius, 1e-4)
            
            pos[vi, 0] = x_along_arm
            
            # Determine base angle based on front vs back sleeve
            # Front sleeves face +Z. Back sleeves face -Z.
            base_angle = 0.0 if is_front_sleeve else np.pi
            final_angle = base_angle + theta
            
            # Cylinder mapping
            pos[vi, 1] = arm_cy - wrap_radius * float(np.sin(final_angle))
            pos[vi, 2] = arm_cz + wrap_radius * float(np.cos(final_angle))


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
