"""
Export a debug GLB showing GC panels at their initial 3D positions.

Each panel is rendered as a distinct colour so you can immediately tell
which panel is which in the viewer.  Stitch pairs are shown as thin flat
ribbons (two triangles per pair) so seam connectivity is visible.  The
body mesh is included as a semi-opaque reference.

The output is designed to answer:
  - Are panels wrapping around the body at the correct height / depth?
  - Which panels are connected by stitches and are the connection points
    at the correct geometric location on each panel edge?
  - How far are the stitch endpoints from each other before simulation?

Usage (run from backend/ with virtualenv active):
    python -m scripts.debug_gc_panel_placement
    python -m scripts.debug_gc_panel_placement --pattern data/patterns/garmentcode/hoody_mean.json
    python -m scripts.debug_gc_panel_placement --no-body --no-stitches
    python -m scripts.debug_gc_panel_placement --z-offset 0.0   # inspect before Z offset

Output: storage/debug_gc_panels.glb
"""

from __future__ import annotations

import argparse
from pathlib import Path

# One RGB colour per panel (cycles if more than 10 panels)
_PANEL_COLORS = [
    (0.90, 0.30, 0.30),  # red
    (0.30, 0.65, 0.30),  # green
    (0.25, 0.45, 0.90),  # blue
    (0.90, 0.70, 0.15),  # yellow
    (0.70, 0.25, 0.75),  # purple
    (0.20, 0.78, 0.78),  # cyan
    (0.90, 0.50, 0.15),  # orange
    (0.45, 0.85, 0.25),  # lime
    (0.85, 0.25, 0.55),  # pink
    (0.25, 0.55, 0.85),  # sky
]


def _build_stitch_ribbon_mesh(
    positions: "np.ndarray",
    stitch_pairs: "np.ndarray",
    ribbon_half_width: float = 0.003,
) -> tuple["np.ndarray", "np.ndarray"]:
    """
    Build a single ribbon mesh representing all stitch pairs.

    Each pair (va, vb) becomes a thin flat quad (2 triangles) oriented
    perpendicular to the pair direction.  This is fast, exports cleanly
    to GLB, and is clearly visible in any 3D viewer.

    Returns:
        verts: (4*S, 3) float64
        faces: (2*S, 3) int64
    """
    import numpy as np

    S = len(stitch_pairs)
    verts = np.zeros((4 * S, 3), dtype=np.float64)
    faces = np.zeros((2 * S, 3), dtype=np.int64)

    up = np.array([0.0, 1.0, 0.0])

    for k, (va, vb) in enumerate(stitch_pairs):
        pa = positions[va].astype(np.float64)
        pb = positions[vb].astype(np.float64)

        direction = pb - pa
        d_len = float(np.linalg.norm(direction))
        if d_len < 1e-8:
            direction = np.array([0.0, 0.0, 1e-4])

        # Perpendicular vector in the plane perpendicular to 'up'
        perp = np.cross(direction, up)
        perp_len = float(np.linalg.norm(perp))
        if perp_len < 1e-8:
            perp = np.cross(direction, np.array([1.0, 0.0, 0.0]))
            perp_len = float(np.linalg.norm(perp))
        perp = perp / perp_len * ribbon_half_width

        base = 4 * k
        verts[base + 0] = pa - perp
        verts[base + 1] = pa + perp
        verts[base + 2] = pb - perp
        verts[base + 3] = pb + perp

        faces[2 * k + 0] = [base, base + 1, base + 2]
        faces[2 * k + 1] = [base + 1, base + 3, base + 2]

    return verts, faces


def run_debug(
    pattern_path: str,
    mesh_resolution: float = 1.5,
    body_z_offset: float = 0.131,
    scale_x: float = 1.0,
    scale_y: float = 1.0,
    output_path: str = "storage/debug_gc_panels.glb",
    include_body: bool = True,
    include_stitches: bool = True,
    include_prewrap: bool = False,
) -> None:
    import numpy as np
    import trimesh
    from trimesh.visual import ColorVisuals

    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    print(f"\n  Loading GC pattern: {pattern_path}")
    gm = build_garment_mesh_gc(
        pattern_path,
        mesh_resolution=mesh_resolution,
        body_z_offset=body_z_offset,
        scale_x=scale_x,
        scale_y=scale_y,
    )

    if include_prewrap:
        from simulation.mesh.gc_mesh_adapter import (
            calibrate_garment_y,
            prewrap_panels_to_body,
            resolve_initial_penetrations,
        )
        calibrate_garment_y(gm, "data/bodies/mannequin_profile.json")
        prewrap_panels_to_body(gm, clearance=0.008)
        n_corr = resolve_initial_penetrations(gm, clearance=0.008)
        print(f"  Prewrap applied — {n_corr} penetration corrections")
        if output_path == "storage/debug_gc_panels.glb":
            output_path = "storage/debug_gc_panels_wrapped.glb"

    n = gm.positions.shape[0]
    offsets = gm.panel_offsets + [n]
    pos = gm.positions

    scene = trimesh.scene.Scene()

    # -----------------------------------------------------------------------
    # Per-panel coloured meshes
    # -----------------------------------------------------------------------
    print(f"\n  Panels ({len(gm.panel_ids)}):")
    for k, pid in enumerate(gm.panel_ids):
        start, end = offsets[k], offsets[k + 1]
        verts = pos[start:end].astype(np.float64)

        # Remap global faces to panel-local indices.
        # Only keep triangles whose vertices are entirely within this panel.
        mask = (
            (gm.faces[:, 0] >= start) & (gm.faces[:, 0] < end) &
            (gm.faces[:, 1] >= start) & (gm.faces[:, 1] < end) &
            (gm.faces[:, 2] >= start) & (gm.faces[:, 2] < end)
        )
        local_faces = (gm.faces[mask] - start).astype(np.int64)

        if len(local_faces) == 0:
            print(f"    {pid:<24}  ⚠️  no faces — skipped")
            continue

        r, g, b = _PANEL_COLORS[k % len(_PANEL_COLORS)]
        rgba = np.array(
            [[int(r * 255), int(g * 255), int(b * 255), 210]] * len(verts),
            dtype=np.uint8,
        )

        mesh = trimesh.Trimesh(vertices=verts, faces=local_faces, process=False)
        mesh.visual = ColorVisuals(mesh=mesh, vertex_colors=rgba)
        scene.add_geometry(mesh, node_name=f"panel_{pid}")

        ps = pos[start:end]
        print(
            f"    {pid:<24}  {len(verts):>5} verts  "
            f"Y=[{ps[:, 1].min():.3f},{ps[:, 1].max():.3f}]  "
            f"Z=[{ps[:, 2].min():+.3f},{ps[:, 2].max():+.3f}]  "
            f"color=RGB({r:.2f},{g:.2f},{b:.2f})"
        )

    # -----------------------------------------------------------------------
    # Stitch ribbons
    # -----------------------------------------------------------------------
    if include_stitches and gm.stitch_pairs.shape[0] > 0:
        ribbon_verts, ribbon_faces = _build_stitch_ribbon_mesh(pos, gm.stitch_pairs)
        ribbon_mesh = trimesh.Trimesh(
            vertices=ribbon_verts, faces=ribbon_faces, process=False
        )
        # Bright magenta so ribbons stand out against panel colours
        rgba = np.full((len(ribbon_verts), 4), [255, 0, 220, 255], dtype=np.uint8)
        ribbon_mesh.visual = ColorVisuals(mesh=ribbon_mesh, vertex_colors=rgba)
        scene.add_geometry(ribbon_mesh, node_name="stitch_ribbons")
        print(f"\n  Stitch ribbons: {gm.stitch_pairs.shape[0]} pairs")

    # -----------------------------------------------------------------------
    # Body mesh (reference)
    # -----------------------------------------------------------------------
    if include_body:
        body_path = "data/bodies/mannequin_physics.glb"
        try:
            body_mesh = trimesh.load(body_path, force="mesh")
            bv = np.array(body_mesh.vertices, dtype=np.float64)
            bf = np.array(body_mesh.faces, dtype=np.int64)
            body_tm = trimesh.Trimesh(vertices=bv, faces=bf, process=False)
            # Grey semi-opaque body
            rgba_body = np.full((len(bv), 4), [160, 145, 130, 80], dtype=np.uint8)
            body_tm.visual = ColorVisuals(mesh=body_tm, vertex_colors=rgba_body)
            scene.add_geometry(body_tm, node_name="body")
            print(f"  Body mesh: {len(bv)} verts")
        except Exception as exc:
            print(f"  ⚠️  Body mesh not loaded: {exc}")

    # -----------------------------------------------------------------------
    # Export
    # -----------------------------------------------------------------------
    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    scene.export(str(out))

    print(f"\n  ✅  Debug GLB → {out.resolve()}")
    print("  Load in the frontend viewer (http://localhost:3000) or drag into Blender.")
    print("\n  Panel legend:")
    for k, pid in enumerate(gm.panel_ids):
        r, g, b = _PANEL_COLORS[k % len(_PANEL_COLORS)]
        print(f"    {pid:<24}  RGB({int(r*255):3d},{int(g*255):3d},{int(b*255):3d})")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern", default="data/patterns/garmentcode/shirt_mean.json",
        help="GarmentCode spec JSON",
    )
    parser.add_argument(
        "--z-offset", type=float, default=0.131, dest="z_offset",
        help="Body Z offset in metres (default: 0.131)",
    )
    parser.add_argument(
        "--res", type=float, default=1.5,
        help="Mesh resolution in cm (default: 1.5)",
    )
    parser.add_argument(
        "--scale-x", type=float, default=1.0, dest="scale_x",
        help="Width scaling factor for 2D patterns (default: 1.0)",
    )
    parser.add_argument(
        "--scale-y", type=float, default=1.0, dest="scale_y",
        help="Height scaling factor for 2D patterns (default: 1.0)",
    )
    parser.add_argument(
        "--output", default="storage/debug_gc_panels.glb",
        help="Output GLB path",
    )
    parser.add_argument(
        "--no-body", action="store_true",
        help="Omit the body mesh from the output",
    )
    parser.add_argument(
        "--no-stitches", action="store_true",
        help="Omit stitch ribbon geometry from the output",
    )
    parser.add_argument(
        "--prewrap", action="store_true",
        help="Apply calibrate_garment_y + prewrap_panels_to_body before export "
             "(output: debug_gc_panels_wrapped.glb)",
    )
    args = parser.parse_args()

    run_debug(
        pattern_path=args.pattern,
        mesh_resolution=args.res,
        body_z_offset=args.z_offset,
        scale_x=args.scale_x,
        scale_y=args.scale_y,
        output_path=args.output,
        include_body=not args.no_body,
        include_stitches=not args.no_stitches,
        include_prewrap=args.prewrap,
    )


if __name__ == "__main__":
    main()
