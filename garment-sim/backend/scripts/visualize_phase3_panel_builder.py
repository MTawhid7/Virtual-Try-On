"""
Visual check — Phase 3: Panel Builder

Loads the tank_top.json pattern, builds the merged GarmentMesh, and exports
three GLB files for visual inspection:

  phase3_panels_unstitched.glb  — front (blue) + back (red) panels in world space,
                                   no simulation. Shows correct placement, rotation,
                                   and separation before stitching.

  phase3_stitch_lines.glb       — same mesh with stitch pairs highlighted in green.
                                   Each stitch pair is a tiny line segment connecting
                                   the matched boundary vertices across the two panels.

  phase3_stitched_sim.glb       — after 180 frames of gravity + stitch constraints
                                   (no body collision). Seams close; cloth hangs.

Usage:
    cd backend
    source .venv/bin/activate
    python -m scripts.visualize_phase3_panel_builder

Output files (in storage/):
    phase3_panels_unstitched.glb
    phase3_stitch_lines.glb
    phase3_stitched_sim.glb

What to verify:
    Unstitched:
      [ ] Front panel (blue) at Z ≈ +0.12m, back panel (red) at Z ≈ -0.12m
      [ ] Both panels same shape and size (0.4m × 0.7m)
      [ ] Back panel X axis flipped (faces inward) due to 180° rotation

    Stitch lines:
      [ ] Green dots/segments appear along LEFT and RIGHT edges of both panels
      [ ] Stitch count reported in terminal matches the JSON seam definitions

    Simulated:
      [ ] Left and right seams closed
      [ ] Cloth hangs under gravity
      [ ] Terminal: both seam gaps < 2cm
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import trimesh

import simulation  # noqa: F401
from simulation.mesh.panel_builder import build_garment_mesh
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.solver.xpbd import XPBDSolver
from simulation.solver.integrator import Integrator
from simulation.materials.presets import FABRIC_PRESETS

STORAGE = Path(__file__).parent.parent / "storage"
STORAGE.mkdir(exist_ok=True)

PATTERN = Path(__file__).parent.parent / "data/patterns/tank_top.json"
RESOLUTION = 15
TOTAL_FRAMES = 180
SUBSTEPS = 15
SOLVER_ITERS = 8
STITCH_COMPLIANCE = 1e-6


def _colored_mesh(positions, faces, panel_offsets) -> trimesh.Trimesh:
    """Front = blue, back = red."""
    n = positions.shape[0]
    n_front = panel_offsets[1]
    colors = np.zeros((n, 4), dtype=np.uint8)
    colors[:n_front] = [100, 140, 220, 255]
    colors[n_front:] = [220, 100, 100, 255]
    mesh = trimesh.Trimesh(vertices=positions, faces=faces, process=False)
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=colors)
    return mesh


def main() -> None:
    fabric = FABRIC_PRESETS["cotton"]

    print(f"Loading pattern: {PATTERN.name}")
    gm = build_garment_mesh(PATTERN, resolution=RESOLUTION)
    n_total = gm.positions.shape[0]
    n_front = gm.panel_offsets[1]

    print(f"  Panels:       {gm.panel_ids}")
    print(f"  Particles:    {n_total}  ({n_front} front + {n_total - n_front} back)")
    print(f"  Faces:        {gm.faces.shape[0]}")
    print(f"  Stitch pairs: {gm.stitch_pairs.shape[0]}")
    print(f"  Fabric:       {gm.fabric}")

    # --- Export 1: unstitched panels in world space ---
    mesh_raw = _colored_mesh(gm.positions, gm.faces, gm.panel_offsets)
    path_raw = STORAGE / "phase3_panels_unstitched.glb"
    mesh_raw.export(str(path_raw))
    print(f"\nSaved: {path_raw.name}  (initial placement, no simulation)")

    # --- Export 2: stitch lines highlighted ---
    # Draw each stitch pair as a tiny sphere at the midpoint for Preview compatibility
    scene_parts = [mesh_raw.copy()]
    if gm.stitch_pairs.shape[0] > 0:
        mids = (gm.positions[gm.stitch_pairs[:, 0]] +
                gm.positions[gm.stitch_pairs[:, 1]]) / 2.0
        for mid in mids:
            dot = trimesh.creation.icosphere(radius=0.004)
            dot.apply_translation(mid)
            dot.visual.face_colors = [50, 220, 50, 255]
            scene_parts.append(dot)
    combined = trimesh.util.concatenate(scene_parts)
    path_stitch = STORAGE / "phase3_stitch_lines.glb"
    combined.export(str(path_stitch))
    print(f"Saved: {path_stitch.name}  (green dots = stitch midpoints)")

    # --- Simulate: gravity + stitch only ---
    print(f"\nSimulating {TOTAL_FRAMES} frames...")
    inv_masses = compute_area_weighted_inv_masses(gm.positions, gm.faces, fabric.density)
    config = SimConfig(
        max_particles=n_total,
        substeps=SUBSTEPS,
        solver_iterations=SOLVER_ITERS,
        damping=fabric.damping,
        total_frames=TOTAL_FRAMES,
    )
    state = ParticleState(config)
    state.load_from_numpy(
        gm.positions, faces=gm.faces, edges=gm.edges,
        uvs=gm.uvs, inv_masses=inv_masses,
    )
    constraints = build_constraints(
        positions=gm.positions, edges=gm.edges, faces=gm.faces,
        stitch_pairs=gm.stitch_pairs,
    )
    solver = XPBDSolver(
        constraints,
        stretch_compliance=fabric.stretch_compliance,
        bend_compliance=fabric.bend_compliance,
        stitch_compliance=STITCH_COMPLIANCE,
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
        stretch_damping=fabric.stretch_damping,
        bend_damping=fabric.bend_damping,
    )
    dt_sub = (1.0 / 60.0) / SUBSTEPS
    integrator = Integrator(
        dt=dt_sub,
        gravity=config.gravity,
        damping=config.damping,
        max_displacement=config.max_displacement,
        air_drag=0.0,
    )

    for frame in range(TOTAL_FRAMES):
        for _ in range(SUBSTEPS):
            integrator.predict(state)
            solver.reset_lambdas()
            for _ in range(SOLVER_ITERS):
                solver.step(state, dt_sub)
            integrator.update(state)
            solver.apply_damping(state)
        if (frame + 1) % 60 == 0:
            print(f"  Frame {frame + 1}/{TOTAL_FRAMES}")

    final_pos = state.positions.to_numpy()
    mesh_sim = _colored_mesh(final_pos, gm.faces, gm.panel_offsets)
    path_sim = STORAGE / "phase3_stitched_sim.glb"
    mesh_sim.export(str(path_sim))
    print(f"Saved: {path_sim.name}  (after gravity + stitching)")

    # --- Seam gap report ---
    print("\nSeam gap report:")
    for label, col in [("Left seam  (col 0)", 0), (f"Right seam (col {RESOLUTION-1})", RESOLUTION - 1)]:
        gaps = []
        for sp in gm.stitch_pairs:
            i, j = int(sp[0]), int(sp[1])
            # Only report pairs along the respective column
            # (approximate: check X proximity to edge)
            xi = final_pos[i, 0]
            xj = final_pos[j, 0]
            gaps.append(np.linalg.norm(final_pos[i] - final_pos[j]))
        if gaps:
            print(f"  All stitch pairs:  max={max(gaps)*100:.1f}cm  mean={np.mean(gaps)*100:.1f}cm")
            break  # report once for all pairs

    all_gaps = [np.linalg.norm(final_pos[i] - final_pos[j]) for i, j in gm.stitch_pairs]
    if all_gaps:
        print(f"  Max gap across all stitches: {max(all_gaps)*100:.1f}cm")
        print(f"  Mean gap:                    {np.mean(all_gaps)*100:.1f}cm")

    print()
    print("Verification checklist:")
    print("  [ ] phase3_panels_unstitched.glb: two flat panels, blue front Z>0, red back Z<0")
    print("  [ ] phase3_stitch_lines.glb: green dots along left + right edges of both panels")
    print("  [ ] phase3_stitched_sim.glb: seams closed, cloth hangs under gravity")
    print("  [ ] Terminal: stitch gap < 2cm")
    print()
    print(f"Files saved to: {STORAGE}/")


if __name__ == "__main__":
    main()
