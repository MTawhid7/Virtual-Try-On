"""
Visual check — Phase 1: Pattern Triangulation

Exports a GLB for each test polygon so you can inspect the mesh in any
3D viewer (macOS Preview, Blender, or online at gltf.report).

Usage:
    cd backend
    source .venv/bin/activate
    python -m scripts.visualize_phase1_triangulation

Output files (in storage/):
    phase1_rectangle.glb    — 0.4m × 0.7m rectangle (front panel)
    phase1_trapezoid.glb    — 0.5m × 0.6m trapezoid (shaped panel)
    phase1_tshirt_front.glb — simplified T-shirt front bodice shape
    phase1_all_panels.glb   — all panels placed side-by-side for comparison

What to verify:
    - Flat mesh lying in the XZ plane (horizontal when viewed from above)
    - Vertices distributed evenly across the interior — no large gaps
    - Clean boundary edges matching the original polygon shape
    - No degenerate triangles (jagged holes or overlapping geometry)
    - UV coverage: vertex colors encode UV (0=black → 1=white gradient)
"""

from __future__ import annotations

import numpy as np
import trimesh
from pathlib import Path

import simulation  # noqa: F401 — initialise Taichi
from simulation.mesh.triangulation import triangulate_panel

STORAGE = Path(__file__).parent.parent / "storage"
STORAGE.mkdir(exist_ok=True)


def _panel_to_mesh(vertices_2d: list, resolution: int, color: list[int]) -> trimesh.Trimesh:
    """Triangulate a 2D polygon and return a coloured trimesh."""
    panel = triangulate_panel(vertices_2d, resolution=resolution)

    mesh = trimesh.Trimesh(
        vertices=panel.positions,
        faces=panel.faces,
        process=False,
    )

    # Colour vertices by U coordinate (0 = dark blue, 1 = yellow) for UV check
    u = panel.uvs[:, 0]  # (N,)
    r = (u * 255).astype(np.uint8)
    g = ((1.0 - u) * 128).astype(np.uint8)
    b = np.full(len(u), color[2], dtype=np.uint8)
    a = np.full(len(u), 255, dtype=np.uint8)
    vertex_colors = np.stack([r, g, b, a], axis=1)
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=vertex_colors)

    return mesh, panel


def main() -> None:
    panels = {
        "phase1_rectangle": {
            "vertices": [[0, 0], [0.4, 0], [0.4, 0.7], [0, 0.7]],
            "resolution": 20,
            "color": [80, 120, 200],
            "desc": "0.4m × 0.7m rectangle — standard front/back panel",
        },
        "phase1_trapezoid": {
            "vertices": [[0, 0], [0.5, 0], [0.4, 0.6], [0.1, 0.6]],
            "resolution": 15,
            "color": [200, 80, 80],
            "desc": "Trapezoid — shaped panel (wider at hem)",
        },
        "phase1_tshirt_front": {
            "vertices": [
                [0.05, 0.0], [0.35, 0.0],          # hem
                [0.40, 0.55], [0.35, 0.60],         # right shoulder
                [0.25, 0.60], [0.22, 0.65],         # right neck
                [0.18, 0.65], [0.15, 0.60],         # left neck
                [0.05, 0.60], [0.00, 0.55],         # left shoulder
            ],
            "resolution": 18,
            "color": [80, 200, 120],
            "desc": "Simplified T-shirt front bodice outline",
        },
    }

    all_meshes = []
    x_offset = 0.0

    for name, cfg in panels.items():
        mesh, panel = _panel_to_mesh(cfg["vertices"], cfg["resolution"], cfg["color"])

        # Export individual file
        path = STORAGE / f"{name}.glb"
        mesh.export(str(path))

        n_v = panel.positions.shape[0]
        n_f = panel.faces.shape[0]
        print(f"  {name}.glb  →  {n_v} vertices, {n_f} faces  |  {cfg['desc']}")

        # Offset for combined view
        shifted = mesh.copy()
        shifted.vertices[:, 0] += x_offset
        all_meshes.append(shifted)
        x_offset += max(np.array(cfg["vertices"])[:, 0].max(), 0.5) + 0.1

    # Export all panels side-by-side
    combined = trimesh.util.concatenate(all_meshes)
    combined_path = STORAGE / "phase1_all_panels.glb"
    combined.export(str(combined_path))
    print(f"  phase1_all_panels.glb  →  all panels side-by-side")

    print()
    print("Verification checklist:")
    print("  [ ] Flat mesh in horizontal plane (no Y displacement)")
    print("  [ ] Interior filled evenly — no large empty patches")
    print("  [ ] Boundary matches polygon outline exactly")
    print("  [ ] UV gradient visible (dark → bright left to right)")
    print("  [ ] T-shirt front has correct notched shoulder/neck shape")
    print()
    print(f"Files saved to: {STORAGE}/")


if __name__ == "__main__":
    main()
