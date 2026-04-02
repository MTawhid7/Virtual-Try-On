import time
import numpy as np

from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid
from simulation.scenes.visualizer import visualize_simulation
from simulation.constraints import build_constraints
from simulation.solver.xpbd import XPBDSolver

def run_constrained_fall(visualize: bool = False, output_path: str = "storage/constrained_fall.glb") -> None:
    """Layer 2 test: pin top-two corners, drop grid with distance + bending constraints."""
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
    if visualize:
        visualize_simulation(engine, state, config)
    else:
        result = engine.run(
            state,
            progress_callback=lambda f, t: print(f"\r  Frame {f}/{t}", end="", flush=True),
        )
        elapsed = time.perf_counter() - start_time
        print(f"\n  Elapsed: {elapsed:.3f}s ({elapsed / config.total_frames * 1000:.1f}ms/frame)")

    # Fetch resulting state
    final_positions = state.get_positions_numpy()

    # --- Validation ---
    print("\n  --- Results ---")

    # 1. No NaN
    has_nan = np.any(np.isnan(final_positions))
    print(f"  NaN check: {'FAIL ❌' if has_nan else 'PASS ✅'}")

    # 2. Pinned corners didn't move
    pin_tl_err = np.linalg.norm(final_positions[pin_tl] - grid.positions[pin_tl])
    pin_tr_err = np.linalg.norm(final_positions[pin_tr] - grid.positions[pin_tr])
    print(f"  Pin TL error: {pin_tl_err:.6f} m {'PASS ✅' if pin_tl_err < 1e-5 else 'FAIL ❌'}")
    print(f"  Pin TR error: {pin_tr_err:.6f} m {'PASS ✅' if pin_tr_err < 1e-5 else 'FAIL ❌'}")

    # 3. Bottom of cloth hangs below pins (gravity worked)
    min_y = np.min(final_positions[:, 1])
    pin_y = grid.positions[pin_tl, 1]
    print(f"  Pin Y:     {pin_y:.3f} m")
    print(f"  Min Y:     {min_y:.3f} m")
    print(f"  Hang dist: {pin_y - min_y:.3f} m {'PASS ✅' if min_y < pin_y else 'FAIL ❌'}")

    # 4. Edge length preservation (check mean edge stretch)
    edge_lengths = np.linalg.norm(
        final_positions[grid.edges[:, 1]] - final_positions[grid.edges[:, 0]], axis=1
    )
    rest_lengths = np.linalg.norm(
        grid.positions[grid.edges[:, 1]] - grid.positions[grid.edges[:, 0]], axis=1
    )
    stretch_ratio = edge_lengths / rest_lengths
    mean_stretch = np.mean(np.abs(stretch_ratio - 1.0))
    max_stretch = np.max(np.abs(stretch_ratio - 1.0))
    print(f"  Mean stretch: {mean_stretch:.4%} {'PASS ✅' if mean_stretch < 0.05 else 'FAIL ❌'}")
    print(f"  Max stretch:  {max_stretch:.4%}")

    # --- Export to glTF (.glb) ---
    from simulation.core.engine import compute_vertex_normals
    normals = compute_vertex_normals(final_positions, grid.faces)
    from simulation.export import write_glb
    out = write_glb(final_positions, grid.faces, normals, path=output_path)
    print(f"\n  Exported to {out}")
