"""
CLI entry point for the simulation engine.

Usage:
    python -m simulation --scene freefall
    python -m simulation --scene constrained_fall  (Sprint 1 Layer 2)
    python -m simulation --scene sphere_drape      (Sprint 1 Layer 3a)
    python -m simulation --pattern tshirt --fabric cotton  (Sprint 2)
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid


def run_freefall() -> None:
    """Layer 1 test: drop a 10×10 grid under gravity, no constraints."""
    print("=== Freefall Scene ===")
    print("Dropping a 10×10 cloth grid under gravity (no constraints)...\n")

    config = SimConfig(
        total_frames=60,     # 1 second at 60fps
        substeps=6,
        solver_iterations=0,  # No constraints
        damping=1.0,          # No damping (pure freefall)
    )

    # Generate grid at height y=2.0
    grid = generate_grid(width=1.0, height=1.0, cols=10, rows=10, center=(0, 2.0, 0))
    print(f"  Particles: {grid.positions.shape[0]}")
    print(f"  Triangles: {grid.faces.shape[0]}")
    print(f"  Edges:     {grid.edges.shape[0]}")
    print(f"  Initial Y:  {grid.positions[0, 1]:.3f} m\n")

    # Initialize state
    state = ParticleState(config)
    state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

    # Run simulation
    engine = SimulationEngine(config)

    start_time = time.perf_counter()
    result = engine.run(
        state,
        progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
    )
    elapsed = time.perf_counter() - start_time
    print()  # newline after progress

    # Report results
    final_y = result.positions[:, 1]
    print(f"\n  Final Y (mean): {np.mean(final_y):.4f} m")
    print(f"  Final Y (std):  {np.std(final_y):.6f} m")

    # Expected freefall: y = y0 + 0.5 * g * t^2
    t_total = config.total_frames * config.dt
    expected_y = 2.0 + 0.5 * config.gravity * t_total**2
    print(f"  Expected Y:     {expected_y:.4f} m (analytical freefall)")
    print(f"  Error:          {abs(np.mean(final_y) - expected_y):.6f} m")
    print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    has_nan = np.any(np.isnan(result.positions))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")


def run_constrained_fall() -> None:
    """Layer 2 test: pin top-two corners, drop grid with distance + bending constraints."""
    from simulation.constraints import build_constraints
    from simulation.solver.xpbd import XPBDSolver

    print("=== Constrained Fall Scene ===")
    print("Pinning top-left + top-right corners, dropping with distance + bending...\n")

    config = SimConfig(
        total_frames=120,     # 2 seconds
        substeps=6,
        solver_iterations=12,
        damping=0.98,
        max_particles=2000,
    )

    # 20×20 grid for a good visual test
    grid = generate_grid(width=1.0, height=1.0, cols=20, rows=20, center=(0, 2.0, 0))
    print(f"  Particles: {grid.positions.shape[0]}")
    print(f"  Triangles: {grid.faces.shape[0]}")
    print(f"  Edges:     {grid.edges.shape[0]}")

    # Build constraints
    constraints = build_constraints(
        positions=grid.positions,
        edges=grid.edges,
        faces=grid.faces,
    )
    print(f"  Distance constraints: {constraints.distance.n_edges if constraints.distance else 0}")
    print(f"  Bending constraints:  {constraints.bending.n_hinges if constraints.bending else 0}")

    # Initialize state
    state = ParticleState(config)
    state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

    # Pin top-left and top-right corners
    cols = 20
    pin_tl = 0            # top-left: row 0, col 0
    pin_tr = cols - 1     # top-right: row 0, col (cols-1)
    state.pin_particle(pin_tl)
    state.pin_particle(pin_tr)
    print(f"  Pinned particles: [{pin_tl}, {pin_tr}]")
    print(f"  Pin positions: TL={grid.positions[pin_tl]}, TR={grid.positions[pin_tr]}\n")

    # Create solver
    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=1e-8,   # Very stiff edges (cotton-like)
        bend_compliance=1e-3,      # Moderate bending flexibility
    )

    engine = SimulationEngine(config, solver=solver)

    start_time = time.perf_counter()
    result = engine.run(
        state,
        progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
    )
    elapsed = time.perf_counter() - start_time
    print()

    # --- Validation ---
    print("\n  --- Results ---")

    # 1. No NaN
    has_nan = np.any(np.isnan(result.positions))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # 2. Pinned corners didn't move
    pin_tl_err = np.linalg.norm(result.positions[pin_tl] - grid.positions[pin_tl])
    pin_tr_err = np.linalg.norm(result.positions[pin_tr] - grid.positions[pin_tr])
    print(f"  Pin TL error: {pin_tl_err:.6f} m {'PASS ✅' if pin_tl_err < 1e-5 else 'FAIL ❌'}")
    print(f"  Pin TR error: {pin_tr_err:.6f} m {'PASS ✅' if pin_tr_err < 1e-5 else 'FAIL ❌'}")

    # 3. Bottom of cloth hangs below pins (gravity worked)
    min_y = np.min(result.positions[:, 1])
    pin_y = grid.positions[pin_tl, 1]
    print(f"  Pin Y:     {pin_y:.3f} m")
    print(f"  Min Y:     {min_y:.3f} m")
    print(f"  Hang dist: {pin_y - min_y:.3f} m {'PASS ✅' if min_y < pin_y else 'FAIL ❌'}")

    # 4. Edge length preservation (check mean edge stretch)
    edge_lengths = np.linalg.norm(
        result.positions[grid.edges[:, 1]] - result.positions[grid.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        grid.positions[grid.edges[:, 1]] - grid.positions[grid.edges[:, 0]], axis=1
    )
    stretch_ratio = edge_lengths / rest_lengths
    mean_stretch = np.mean(np.abs(stretch_ratio - 1.0))
    max_stretch = np.max(np.abs(stretch_ratio - 1.0))
    print(f"  Mean stretch: {mean_stretch:.4%} {'PASS ✅' if mean_stretch < 0.05 else 'FAIL ❌'}")
    print(f"  Max stretch:  {max_stretch:.4%}")

    # 5. Performance
    print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    # 6. Export to OBJ for visual confirmation
    os.makedirs("outputs", exist_ok=True)
    print("\n  Exporting to outputs/constrained_fall.obj for visual confirmation...")
    with open("outputs/constrained_fall.obj", "w") as f:
        for v in result.positions:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for face in grid.faces:
            # OBJ faces are 1-indexed
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")
    print("  Done. You can open outputs/constrained_fall.obj in macOS Preview or Blender.")

def run_sphere_drape() -> None:
    """Layer 3a test: drop a 20×20 cloth grid onto an analytical sphere."""
    from simulation.constraints import build_constraints
    from simulation.solver.xpbd import XPBDSolver
    from simulation.collision import SphereCollider

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
    result = engine.run(
        state,
        progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
    )
    elapsed = time.perf_counter() - start_time
    print()

    # --- Validation ---
    print("\n  --- Results ---")

    # 1. No NaN
    has_nan = np.any(np.isnan(result.positions))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # 2. No penetration
    center_np = np.array(sphere_center, dtype=np.float32)
    distances = np.linalg.norm(result.positions - center_np, axis=1)
    min_dist = np.min(distances)
    target_dist = sphere_radius + config.collision_thickness
    penetration = target_dist - min_dist
    print(f"  Min distance from center: {min_dist:.4f} m (target: ≥{target_dist:.4f} m)")
    print(f"  Penetration: {penetration:.4f} m {'FAIL ❌' if penetration > 0.001 else 'PASS ✅'}")

    # 3. Cloth drapes over sphere (some particles near top and below top)
    sphere_top_y = sphere_center[1] + sphere_radius
    sphere_mid_y = sphere_center[1] + sphere_radius * 0.5

    above_mid = np.sum(result.positions[:, 1] > sphere_mid_y)
    below_top = np.sum(result.positions[:, 1] < sphere_top_y)
    print(f"  Particles above sphere mid-height: {above_mid}")
    print(f"  Particles below sphere top: {below_top}")
    drape_ok = above_mid > 0 and below_top > 0
    print(f"  Drape shape: {'PASS ✅' if drape_ok else 'FAIL ❌'}")

    # 4. No upward crumpling
    max_y = np.max(result.positions[:, 1])
    initial_y = 2.0
    print(f"  Max Y: {max_y:.3f} m (initial: {initial_y:.1f} m)")
    print(f"  No upward crumpling: {'FAIL ❌' if max_y > initial_y + 0.01 else 'PASS ✅'}")

    # 5. Energy decay
    velocities = state.get_velocities_numpy()
    mean_speed = np.mean(np.linalg.norm(velocities, axis=1))
    print(f"  Mean speed: {mean_speed:.4f} m/s {'PASS ✅' if mean_speed < 1.0 else 'FAIL ❌'}")

    # 6. Edge length preservation
    edge_lengths = np.linalg.norm(
        result.positions[grid.edges[:, 1]] - result.positions[grid.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        grid.positions[grid.edges[:, 1]] - grid.positions[grid.edges[:, 0]], axis=1
    )
    stretch_ratio = edge_lengths / rest_lengths
    mean_stretch = np.mean(np.abs(stretch_ratio - 1.0))
    max_stretch = np.max(np.abs(stretch_ratio - 1.0))
    print(f"  Mean stretch: {mean_stretch:.4%} {'PASS ✅' if mean_stretch < 0.10 else 'FAIL ❌'}")
    print(f"  Max stretch:  {max_stretch:.4%}")

    # 7. Performance
    print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    # 8. Export to OBJ
    os.makedirs("outputs", exist_ok=True)
    print("\n  Exporting to outputs/sphere_drape.obj for visual confirmation...")
    with open("outputs/sphere_drape.obj", "w") as f:
        # Write cloth vertices
        for v in result.positions:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
            
        import trimesh
        # Add sphere geometry
        sphere = trimesh.creation.icosphere(subdivisions=3, radius=sphere_radius)
        sphere.apply_translation(sphere_center)
        
        # Write sphere vertices
        for v in sphere.vertices:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")

        # Write cloth faces
        for face in grid.faces:
            f.write(f"f {face[0]+1} {face[1]+1} {face[2]+1}\n")

        # Write sphere faces (offset by the number of cloth vertices)
        offset = len(result.positions)
        for face in sphere.faces:
            f.write(f"f {face[0]+offset+1} {face[1]+offset+1} {face[2]+offset+1}\n")
            
    print("  Done. Open outputs/sphere_drape.obj in macOS Preview or Blender.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Garment Simulation Engine")
    parser.add_argument(
        "--scene", type=str, default="freefall",
        choices=["freefall", "constrained_fall", "sphere_drape"],
        help="Built-in test scene to run",
    )
    # Future args:
    # parser.add_argument("--pattern", type=str, help="Pattern JSON file")
    # parser.add_argument("--fabric", type=str, help="Fabric preset name")

    args = parser.parse_args()

    if args.scene == "freefall":
        run_freefall()
    elif args.scene == "constrained_fall":
        run_constrained_fall()
    elif args.scene == "sphere_drape":
        run_sphere_drape()
    else:
        print(f"Unknown scene: {args.scene}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
