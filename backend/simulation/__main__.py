"""
CLI entry point for the simulation engine.

Usage:
    python -m simulation --scene freefall
    python -m simulation --scene sphere_drape     (Sprint 1 Layer 3a)
    python -m simulation --pattern tshirt --fabric cotton  (Sprint 2)
"""

from __future__ import annotations

import argparse
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


def main() -> None:
    parser = argparse.ArgumentParser(description="Garment Simulation Engine")
    parser.add_argument(
        "--scene", type=str, default="freefall",
        choices=["freefall"],
        help="Built-in test scene to run",
    )
    # Future args:
    # parser.add_argument("--pattern", type=str, help="Pattern JSON file")
    # parser.add_argument("--fabric", type=str, help="Fabric preset name")

    args = parser.parse_args()

    if args.scene == "freefall":
        run_freefall()
    else:
        print(f"Unknown scene: {args.scene}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
