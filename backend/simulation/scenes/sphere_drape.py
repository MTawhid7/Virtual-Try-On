import time
import os
import numpy as np

from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid
from simulation.scenes.visualizer import visualize_simulation
from simulation.constraints import build_constraints
from simulation.solver.xpbd import XPBDSolver
from simulation.collision import SphereCollider

def run_sphere_drape(visualize: bool = False, output_path: str = "storage/sphere_drape.glb") -> None:
    """Layer 3a test: drop a 20×20 cloth grid onto an analytical sphere."""
    print("=== Sphere Drape Scene ===")
    print("Dropping a 20×20 cloth grid onto a sphere with collision...\n")

    # --- Configuration ---
    config = SimConfig(
        total_frames=120,      # 2 seconds
        substeps=6,
        solver_iterations=12,
        damping=0.98,
        max_particles=2000,
        collision_thickness=0.005,   # 5mm
        friction_coefficient=0.3,
    )

    # --- Mesh ---
    grid = generate_grid(width=1.0, height=1.0, cols=20, rows=20, center=(0, 2.0, 0))
    print(f"  Particles: {grid.positions.shape[0]}")
    print(f"  Triangles: {grid.faces.shape[0]}")
    print(f"  Edges:     {grid.edges.shape[0]}")

    # --- Sphere ---
    sphere_center = (0.0, 0.8, 0.0)
    sphere_radius = 0.5
    print(f"  Sphere:    center={sphere_center}, radius={sphere_radius}")
    print()

    # --- Constraints ---
    constraints = build_constraints(
        positions=grid.positions,
        edges=grid.edges,
        faces=grid.faces,
    )
    print(f"  Distance constraints: {constraints.distance.n_edges if constraints.distance else 0}")
    print(f"  Bending constraints:  {constraints.bending.n_hinges if constraints.bending else 0}")

    # --- State (no pinned particles — free drape) ---
    state = ParticleState(config)
    state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

    # --- Solver + Collider ---
    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=1e-8,
        bend_compliance=1e-3,
    )
    collider = SphereCollider(center=sphere_center, radius=sphere_radius)

    engine = SimulationEngine(config, solver=solver)
    engine.collider = collider

    # --- Run ---
    start_time = time.perf_counter()
    if visualize:
        visualize_simulation(engine, state, config, sphere_center, sphere_radius)
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

    # 2. No penetration
    center_np = np.array(sphere_center, dtype=np.float32)
    distances = np.linalg.norm(final_positions - center_np, axis=1)
    min_dist = np.min(distances)
    target_dist = sphere_radius + config.collision_thickness
    penetration = target_dist - min_dist
    print(f"  Min distance from center: {min_dist:.4f} m (target: ≥{target_dist:.4f} m)")
    print(f"  Penetration: {penetration:.4f} m {'FAIL ❌' if penetration > 0.001 else 'PASS ✅'}")

    # 3. Cloth drapes over sphere (some particles near top and below top)
    sphere_top_y = sphere_center[1] + sphere_radius
    sphere_mid_y = sphere_center[1] + sphere_radius * 0.5

    above_mid = np.sum(final_positions[:, 1] > sphere_mid_y)
    below_top = np.sum(final_positions[:, 1] < sphere_top_y)
    print(f"  Particles above sphere mid-height: {above_mid}")
    print(f"  Particles below sphere top: {below_top}")
    drape_ok = above_mid > 0 and below_top > 0
    print(f"  Drape shape: {'PASS ✅' if drape_ok else 'FAIL ❌'}")

    # 4. No upward crumpling
    max_y = np.max(final_positions[:, 1])
    initial_y = 2.0
    print(f"  Max Y: {max_y:.3f} m (initial: {initial_y:.1f} m)")
    print(f"  No upward crumpling: {'FAIL ❌' if max_y > initial_y + 0.01 else 'PASS ✅'}")

    # 5. Energy decay
    velocities = state.get_velocities_numpy()
    mean_speed = np.mean(np.linalg.norm(velocities, axis=1))
    print(f"  Mean speed: {mean_speed:.4f} m/s {'PASS ✅' if mean_speed < 1.0 else 'FAIL ❌'}")

    # 6. Edge length preservation
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

    # --- Export to glTF (.glb) ---
    from simulation.core.engine import compute_vertex_normals
    normals = compute_vertex_normals(final_positions, grid.faces)
    from simulation.export import write_glb
    out = write_glb(final_positions, grid.faces, normals, path=output_path)
    print(f"\n  Exported to {out}")
    print("  Open in Blender, three.js, or https://gltf-viewer.donmccurdy.com/")
