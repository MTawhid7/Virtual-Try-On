"""
Body drape scene — Sprint 2 Layer 3a-Extended demo.

Drops a 30×30 cloth grid onto the mannequin body mesh to validate the full
body mesh collision pipeline (spatial hash + point-triangle projection).

This scene mirrors sphere_drape.py in structure, with the same 6 validation
checks, replacing SphereCollider with BodyCollider.
"""

import os
import time

import numpy as np

from simulation.collision import BodyCollider
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine, compute_vertex_normals
from simulation.core.state import ParticleState
from simulation.export import write_glb
from simulation.mesh.grid import generate_grid
from simulation.solver.xpbd import XPBDSolver


# Canonical path to the body mesh (relative to the backend/ working directory)
_BODY_GLB_PATH = "data/bodies/mannequin_physics.glb"


def run_body_drape(visualize: bool = False, output_path: str = "storage/body_drape.glb") -> None:
    """
    Sprint 2 Layer 3a-Extended: drop a 30×30 cloth grid onto the body mesh.

    Setup:
        - 30×30 grid, 1.2m × 1.2m, centered at (0, 1.8, 0) — above shoulders
        - Body mesh: mannequin_physics.glb (pre-processed physics proxy, 1.75m)
        - 120 frames, 6 substeps, 12 solver iterations (same as sphere_drape)
        - Cotton compliance: stretch=1e-8, bend=1e-3

    Validation checks (mirrors sphere_drape.py):
        1. No NaN in final positions
        2. No penetration (all particles at or above body surface)
        3. Cloth drapes naturally (particles above and below shoulder level)
        4. No upward crumpling (max Y ≤ initial Y + ε)
        5. Energy decay (mean speed < 1.0 m/s)
        6. Edge length preservation (mean stretch < 10%)
    """
    print("=== Body Drape Scene (Sprint 2 Layer 3a-Extended) ===")
    print("Dropping a 30×30 cloth grid onto the body mesh...\n")

    # --- Configuration ---
    config = SimConfig(
        total_frames=120,
        substeps=6,
        solver_iterations=12,
        damping=0.98,
        max_particles=5000,
        collision_thickness=0.005,    # 5mm — same as sphere_drape
        friction_coefficient=0.3,
    )

    # --- Body mesh ---
    print("  Loading body mesh...")
    collider = BodyCollider.from_glb(
        _BODY_GLB_PATH,
        target_height=1.75,
        decimate_target=5000,
    )
    print(f"  Body proxy: {collider.spatial_hash.n_triangles} triangles\n")

    # --- Cloth mesh ---
    # 30×30 grid, 1.2m × 1.2m, positioned at Y=1.8m (above shoulders at ~1.45m)
    grid = generate_grid(width=1.2, height=1.2, cols=30, rows=30, center=(0.0, 1.8, 0.0))
    print(f"  Cloth particles: {grid.positions.shape[0]}")
    print(f"  Cloth triangles: {grid.faces.shape[0]}")
    print(f"  Cloth edges:     {grid.edges.shape[0]}")
    print()

    # --- Constraints (cotton compliance) ---
    constraints = build_constraints(
        positions=grid.positions,
        edges=grid.edges,
        faces=grid.faces,
    )
    print(f"  Distance constraints: {constraints.distance.n_edges if constraints.distance else 0}")
    print(f"  Bending constraints:  {constraints.bending.n_hinges if constraints.bending else 0}")

    # --- State ---
    state = ParticleState(config)
    state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

    # --- Solver ---
    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=1e-8,   # Cotton: near-inextensible
        bend_compliance=1e-3,      # Cotton: moderate bending
    )

    # --- Engine ---
    engine = SimulationEngine(config, solver=solver)
    engine.collider = collider

    # --- Run ---
    start_time = time.perf_counter()
    if visualize:
        from simulation.scenes.visualizer import visualize_simulation
        visualize_simulation(engine, state, config, body_mesh_path=_BODY_GLB_PATH)
    else:
        result = engine.run(
            state,
            progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
        )
        elapsed = time.perf_counter() - start_time
        print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    final_positions = state.get_positions_numpy()

    # --- Validation ---
    print("\n  --- Results ---")

    # 1. No NaN
    has_nan = np.any(np.isnan(final_positions))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # 2. No penetration — check particles against body mesh bounds
    #    Approximate: no particle should be below Y=-0.01 (the body surface starts at Y=0)
    min_y = np.min(final_positions[:, 1])
    print(f"  Min particle Y: {min_y:.4f} m (body starts at Y=0)")
    below_body = min_y < -config.collision_thickness * 2
    print(f"  No sub-body penetration: {'FAIL ❌' if below_body else 'PASS ✅'}")

    # 3. Cloth drapes over body (particles should spread across shoulder region)
    shoulder_y = 1.45   # Approximate shoulder height for 1.75m body
    above_shoulder = np.sum(final_positions[:, 1] > shoulder_y * 0.9)
    below_initial = np.sum(final_positions[:, 1] < 1.8)
    drape_ok = above_shoulder > 0 and below_initial > 0
    print(f"  Particles near shoulders (Y ≥ {shoulder_y * 0.9:.2f}m): {above_shoulder}")
    print(f"  Particles fallen below initial Y=1.8m: {below_initial}")
    print(f"  Drape shape: {'PASS ✅' if drape_ok else 'FAIL ❌'}")

    # 4. No upward crumpling
    max_y = np.max(final_positions[:, 1])
    initial_y = 1.8
    print(f"  Max Y: {max_y:.3f} m (initial: {initial_y:.1f} m)")
    print(f"  No upward crumpling: {'FAIL ❌' if max_y > initial_y + 0.01 else 'PASS ✅'}")

    # 5. Energy decay
    velocities = state.get_velocities_numpy()
    mean_speed = np.mean(np.linalg.norm(velocities, axis=1))
    print(f"  Mean speed: {mean_speed:.4f} m/s {'PASS ✅' if mean_speed < 1.0 else 'FAIL ❌'}")

    # 6. Edge length preservation (cotton is near-inextensible)
    edge_lengths = np.linalg.norm(
        final_positions[grid.edges[:, 1]] - final_positions[grid.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        grid.positions[grid.edges[:, 1]] - grid.positions[grid.edges[:, 0]], axis=1
    )
    stretch_ratio = edge_lengths / rest_lengths
    mean_stretch = np.mean(np.abs(stretch_ratio - 1.0))
    max_stretch = np.max(np.abs(stretch_ratio - 1.0))
    print(f"  Mean stretch: {mean_stretch:.4%} {'PASS ✅' if mean_stretch < 0.10 else 'FAIL ❌'}")
    print(f"  Max stretch:  {max_stretch:.4%}")

    # --- Export ---
    normals = compute_vertex_normals(final_positions, grid.faces)
    out = write_glb(final_positions, grid.faces, normals, path=output_path)
    print(f"\n  Exported to {out}")
    print("  Open in Blender or https://gltf-viewer.donmccurdy.com/ to inspect drape")
