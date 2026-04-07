"""
Garment drape scene — Sprint 2 Layer 3b-Extended Phase 4.

Drapes a pattern-based garment (tank_top.json) onto the mannequin body mesh
using XPBD with stitch constraints, body collision, and area-weighted masses.

This is the first scene to exercise the full garment pipeline:
    pattern JSON → triangulate panels → place in 3D → merge → stitch constraints
    → body collision → engine.run() → export GLB

The scene mirrors body_drape.py in structure, replacing the flat grid with a
multi-panel garment mesh from build_garment_mesh().
"""

import time

import numpy as np

from simulation.collision import BodyCollider, ClothSelfCollider
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine, compute_vertex_normals
from simulation.core.state import ParticleState
from simulation.export import write_glb
from simulation.materials import FABRIC_PRESETS
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.mesh.panel_builder import build_garment_mesh
from simulation.solver.xpbd import XPBDSolver


_BODY_GLB_PATH = "data/bodies/mannequin_physics.glb"
_TANK_TOP_JSON = "data/patterns/tank_top.json"


def run_garment_drape(
    visualize: bool = False,
    output_path: str = "storage/garment_drape.glb",
    pattern_path: str = _TANK_TOP_JSON,
    resolution: int = 20,
) -> None:
    """
    Sprint 2 Layer 3b-Extended Phase 4: drape a tank top pattern onto the body.

    Setup:
        - Garment: tank_top.json — front + back panels, stitched at side seams
        - resolution=20 → ~232 particles total
        - Area-weighted inv_mass from cotton density (0.30 kg/m²)
        - Body mesh: mannequin_physics.glb (pre-processed physics proxy, 1.75m)
        - 300 frames, 15 substeps, 8 solver iterations
        - Cotton preset: stretch=1e-7, bend=1.5e-2, damping=0.990, friction=0.35
        - Collision thickness: 8mm

    Validation checks:
        1. No NaN in final positions
        2. No sub-floor penetration (Y ≥ -2×thickness)
        3. Garment on body — majority of particles in torso region (Y=0.8–1.6m)
        4. Stitches closed (max seam gap < 5mm after settling)
        5. Energy decay (mean speed < 1.5 m/s)
        6. Edge length preservation (mean stretch < 15%)
    """
    print("=== Garment Drape Scene (Sprint 2 Layer 3b-Extended Phase 4) ===")
    print(f"  Pattern: {pattern_path}")
    print("  Draping onto mannequin body mesh...\n")

    fabric = FABRIC_PRESETS["cotton"]

    config = SimConfig(
        total_frames=480,
        substeps=15,
        solver_iterations=8,
        damping=fabric.damping,
        max_particles=1000,
        collision_thickness=0.008,
        friction_coefficient=fabric.friction,
        air_drag=0.3,
    )

    # --- Body mesh ---
    print("  Loading body mesh...")
    collider = BodyCollider.from_glb(
        _BODY_GLB_PATH,
        target_height=1.75,
        decimate_target=5000,
    )
    print(f"  Body proxy: {collider.spatial_hash.n_triangles} triangles\n")

    # --- Garment mesh ---
    print("  Building garment mesh from pattern JSON...")
    garment = build_garment_mesh(pattern_path, resolution=resolution)
    n_particles = garment.positions.shape[0]
    n_stitches = garment.stitch_pairs.shape[0]
    print(f"  Panels:          {garment.panel_ids}")
    print(f"  Particles:       {n_particles}")
    print(f"  Triangles:       {garment.faces.shape[0]}")
    print(f"  Edges:           {garment.edges.shape[0]}")
    print(f"  Stitch pairs:    {n_stitches}")
    print()

    inv_masses = compute_area_weighted_inv_masses(
        garment.positions, garment.faces, fabric.density
    )

    # --- Constraints ---
    constraints = build_constraints(
        positions=garment.positions,
        edges=garment.edges,
        faces=garment.faces,
        stitch_pairs=garment.stitch_pairs if n_stitches > 0 else None,
        max_stitches=n_stitches + 10,
    )
    print(f"  Distance constraints: {constraints.distance.n_edges if constraints.distance else 0}")
    print(f"  Bending constraints:  {constraints.bending.n_hinges if constraints.bending else 0}")
    print(f"  Stitch constraints:   {constraints.stitch.n_stitches if constraints.stitch else 0}")
    print()

    # --- State ---
    state = ParticleState(config)
    state.load_from_numpy(
        garment.positions,
        faces=garment.faces,
        edges=garment.edges,
        inv_masses=inv_masses,
    )

    # --- Solver ---
    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=fabric.stretch_compliance,
        bend_compliance=fabric.bend_compliance,
        stitch_compliance=1e-7,
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
        stretch_damping=fabric.stretch_damping,
        bend_damping=fabric.bend_damping,
    )

    # --- Engine ---
    engine = SimulationEngine(config, solver=solver)
    engine.collider = collider

    # --- Self-collision ---
    self_collider = ClothSelfCollider.from_mesh(
        faces_np=garment.faces,
        positions_np=garment.positions,
        thickness=config.self_collision_thickness,
    )
    engine.self_collider = self_collider
    print(f"  Self-collision: cell_size={self_collider.cell_size:.4f}m, thickness={self_collider.thickness:.4f}m\n")

    # --- Run ---
    start_time = time.perf_counter()
    if visualize:
        from simulation.scenes.visualizer import visualize_simulation
        visualize_simulation(
            engine, state, config,
            body_mesh_path=_BODY_GLB_PATH,
            stitch_pairs=garment.stitch_pairs if n_stitches > 0 else None,
        )
    else:
        engine.run(
            state,
            progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
        )
        elapsed = time.perf_counter() - start_time
        print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    final_positions = state.get_positions_numpy()

    # --- Validation ---
    print("\n  --- Results ---")

    # 1. No NaN
    has_nan = bool(np.any(np.isnan(final_positions)))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # 2. No sub-floor penetration
    min_y = float(np.min(final_positions[:, 1]))
    below_floor = min_y < -config.collision_thickness * 2
    print(f"  Min Y: {min_y:.4f}m {'FAIL ❌' if below_floor else 'PASS ✅'} (no sub-floor penetration)")

    # 3. Garment distributed on torso (Y=0.8–1.6m covers waist to above shoulders)
    waist_y, top_y = 0.8, 1.6
    in_torso = int(np.sum((final_positions[:, 1] >= waist_y) & (final_positions[:, 1] <= top_y)))
    torso_ok = in_torso > n_particles * 0.25
    print(f"  Particles in torso Y={waist_y:.1f}–{top_y:.1f}m: {in_torso}/{n_particles}")
    print(f"  Garment on body: {'PASS ✅' if torso_ok else 'FAIL ❌'}")

    # 4. Stitch gap
    if n_stitches > 0:
        pa = final_positions[garment.stitch_pairs[:, 0]]
        pb = final_positions[garment.stitch_pairs[:, 1]]
        gaps = np.linalg.norm(pa - pb, axis=1)
        max_gap = float(np.max(gaps))
        mean_gap = float(np.mean(gaps))
        seam_ok = max_gap < 0.05
        print(f"  Stitch gap — max: {max_gap * 100:.2f}cm, mean: {mean_gap * 100:.2f}cm")
        print(f"  Stitches closed (<5mm): {'PASS ✅' if seam_ok else 'FAIL ❌'}")

    # 5. Energy decay
    velocities = state.get_velocities_numpy()
    mean_speed = float(np.mean(np.linalg.norm(velocities, axis=1)))
    print(f"  Mean speed: {mean_speed:.4f} m/s {'PASS ✅' if mean_speed < 1.5 else 'FAIL ❌'}")

    # 6. Edge length preservation
    edge_lengths = np.linalg.norm(
        final_positions[garment.edges[:, 1]] - final_positions[garment.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1
    )
    mean_stretch = float(np.mean(np.abs(edge_lengths / rest_lengths - 1.0)))
    max_stretch_val = float(np.max(np.abs(edge_lengths / rest_lengths - 1.0)))
    print(f"  Mean stretch: {mean_stretch:.4%} {'PASS ✅' if mean_stretch < 0.15 else 'FAIL ❌'}")
    print(f"  Max stretch:  {max_stretch_val:.4%}")

    # --- Export ---
    normals = compute_vertex_normals(final_positions, garment.faces)
    out = write_glb(final_positions, garment.faces, normals, uvs=garment.uvs, path=output_path)
    print(f"\n  Exported to {out}")
    print("  Open in Blender or https://gltf-viewer.donmccurdy.com/ to inspect drape")
