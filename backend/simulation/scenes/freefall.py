import time
import numpy as np

from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid
from simulation.scenes.visualizer import visualize_simulation

def run_freefall(visualize: bool = False, output_path: str = "storage/freefall.glb") -> None:
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
    if visualize:
        visualize_simulation(engine, state, config)
    else:
        result = engine.run(
            state,
            progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
        )
        elapsed = time.perf_counter() - start_time
        print()  # newline after progress

    final_positions = state.get_positions_numpy()
    final_y = final_positions[:, 1]
    print(f"\n  Final Y (mean): {np.mean(final_y):.4f} m")
    print(f"  Final Y (std):  {np.std(final_y):.6f} m")

    # Expected freefall: y = y0 + 0.5 * g * t^2
    t_total = config.total_frames * config.dt
    expected_y = 2.0 + 0.5 * config.gravity * t_total**2
    print(f"  Expected Y:     {expected_y:.4f} m (analytical freefall)")
    print(f"  Error:          {abs(np.mean(final_y) - expected_y):.6f} m")

    has_nan = np.any(np.isnan(final_positions))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # --- Export to glTF (.glb) ---
    from simulation.core.engine import compute_vertex_normals
    normals = compute_vertex_normals(final_positions, grid.faces)
    from simulation.export import write_glb
    out = write_glb(final_positions, grid.faces, normals, path=output_path)
    print(f"\n  Exported to {out}")
