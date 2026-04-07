"""
Visual check — Phase 2: Stitch Constraints

Simulates two flat panels (front + back) with gravity and stitch constraints
only — no body collision. Shows that the stitch kernel pulls seam vertices
together while gravity pulls the cloth down.

Usage:
    cd backend
    source .venv/bin/activate
    python -m scripts.visualize_phase2_stitch

Output files (in storage/):
    phase2_stitch_t0.glb    — initial state: two panels floating apart (0.24m gap)
    phase2_stitch_t120.glb  — final state: seams closed, cloth hanging under gravity

What to verify:
    - t0:   Two separate flat panels with a gap between them
    - t120: Left and right edges of both panels pulled together (seams closed)
    - t120: Panels hang downward under gravity — not horizontal any more
    - t120: The cloth folds naturally at the seam line
    - Terminal output: seam gap should be < 2cm on both sides
"""

from __future__ import annotations

import numpy as np
import trimesh
from pathlib import Path

import simulation  # noqa: F401
from simulation.mesh.grid import generate_grid, compute_area_weighted_inv_masses
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.solver.xpbd import XPBDSolver
from simulation.solver.integrator import Integrator
from simulation.materials.presets import FABRIC_PRESETS

STORAGE = Path(__file__).parent.parent / "storage"
STORAGE.mkdir(exist_ok=True)

COLS = 10
ROWS = 15
TOTAL_FRAMES = 120
SUBSTEPS = 15
SOLVER_ITERS = 8
STITCH_COMPLIANCE = 1e-6


def _build_two_panel_mesh():
    """Create front + back panels as a merged numpy mesh."""
    fabric = FABRIC_PRESETS["cotton"]

    # Front panel at Z=+0.12, back at Z=-0.12
    front = generate_grid(width=0.4, height=0.7, cols=COLS, rows=ROWS,
                          center=(0.0, 1.2, 0.12))
    back  = generate_grid(width=0.4, height=0.7, cols=COLS, rows=ROWS,
                          center=(0.0, 1.2, -0.12))

    n_front = front.positions.shape[0]

    all_pos   = np.concatenate([front.positions, back.positions], axis=0)
    all_faces = np.concatenate([front.faces,     back.faces + n_front], axis=0)
    all_edges = np.concatenate([front.edges,     back.edges + n_front], axis=0)

    # Stitch left (col=0) and right (col=COLS-1) edges of both panels
    pairs = []
    for r in range(ROWS):
        pairs.append([r * COLS,           n_front + r * COLS])           # left
        pairs.append([r * COLS + COLS-1,  n_front + r * COLS + COLS-1])  # right
    stitch_pairs = np.array(pairs, dtype=np.int32)

    inv_masses = compute_area_weighted_inv_masses(all_pos, all_faces, fabric.density)

    return all_pos, all_faces, all_edges, stitch_pairs, inv_masses, n_front


def _export_mesh(positions: np.ndarray, faces: np.ndarray,
                 n_front: int, path: Path) -> None:
    """Export mesh with front panel in blue, back panel in red."""
    n_total = positions.shape[0]
    colors = np.zeros((n_total, 4), dtype=np.uint8)
    colors[:n_front]  = [100, 140, 220, 255]  # front = blue
    colors[n_front:]  = [220, 100, 100, 255]  # back  = red

    mesh = trimesh.Trimesh(vertices=positions, faces=faces, process=False)
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, vertex_colors=colors)
    mesh.export(str(path))


def main() -> None:
    fabric = FABRIC_PRESETS["cotton"]

    print("Building two-panel mesh...")
    all_pos, all_faces, all_edges, stitch_pairs, inv_masses, n_front = \
        _build_two_panel_mesh()
    n_total = all_pos.shape[0]
    print(f"  Particles: {n_total}  ({n_front} front + {n_front} back)")
    print(f"  Stitch pairs: {len(stitch_pairs)}")

    # Export initial state (T=0)
    t0_path = STORAGE / "phase2_stitch_t0.glb"
    _export_mesh(all_pos, all_faces, n_front, t0_path)
    print(f"  Saved: {t0_path.name}  (initial — panels 0.24m apart)")

    # Build simulation
    config = SimConfig(
        max_particles=n_total,
        substeps=SUBSTEPS,
        solver_iterations=SOLVER_ITERS,
        damping=fabric.damping,
        total_frames=TOTAL_FRAMES,
    )
    state = ParticleState(config)
    state.load_from_numpy(all_pos, faces=all_faces, edges=all_edges,
                          inv_masses=inv_masses)

    constraints = build_constraints(
        positions=all_pos,
        edges=all_edges,
        faces=all_faces,
        stitch_pairs=stitch_pairs,
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

    # Simulate
    print(f"\nSimulating {TOTAL_FRAMES} frames ({SUBSTEPS} substeps × {SOLVER_ITERS} iters)...")
    for frame in range(TOTAL_FRAMES):
        for _ in range(SUBSTEPS):
            integrator.predict(state)
            solver.reset_lambdas()
            for _ in range(SOLVER_ITERS):
                solver.step(state, dt_sub)
            integrator.update(state)
            solver.apply_damping(state)
        if (frame + 1) % 30 == 0:
            print(f"  Frame {frame + 1}/{TOTAL_FRAMES}")

    # Export final state
    final_pos = state.positions.to_numpy()
    t120_path = STORAGE / "phase2_stitch_t120.glb"
    _export_mesh(final_pos, all_faces, n_front, t120_path)
    print(f"\n  Saved: {t120_path.name}  (after stitching + gravity)")

    # Report seam gaps
    print("\nSeam gap report:")
    for label, col in [("Left seam  (col 0)", 0), (f"Right seam (col {COLS-1})", COLS-1)]:
        gaps = [
            np.linalg.norm(final_pos[r * COLS + col] - final_pos[n_front + r * COLS + col])
            for r in range(ROWS)
        ]
        print(f"  {label}:  max={max(gaps)*100:.1f}cm  mean={np.mean(gaps)*100:.1f}cm")

    # Mean speed (settling check)
    vel = state.velocities.to_numpy()
    mean_speed = float(np.linalg.norm(vel, axis=1).mean())
    print(f"\n  Mean speed at frame {TOTAL_FRAMES}: {mean_speed:.3f} m/s")
    print(f"  {'PASS ✓' if mean_speed < 1.0 else 'FAIL — still oscillating'}")

    print()
    print("Verification checklist:")
    print("  [ ] phase2_stitch_t0.glb:  two separate flat panels (blue + red)")
    print("  [ ] phase2_stitch_t120.glb: left and right edges pulled together")
    print("  [ ] Panels hang downward under gravity (not horizontal)")
    print("  [ ] Seam gaps < 2cm on both sides (see terminal output above)")
    print("  [ ] No NaN or explosion in final positions")
    print()
    print(f"Files saved to: {STORAGE}/")


if __name__ == "__main__":
    main()
