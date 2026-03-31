import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid


class TestConstrainedFall:
    """
    Validate that pinned cloth with distance + bending constraints
    drapes naturally without rubber-banding or explosions.
    """

    @staticmethod
    def _run_constrained_cloth(
        cols: int = 10,
        rows: int = 10,
        total_frames: int = 60,
        stretch_compliance: float = 1e-8,
        bend_compliance: float = 1e-3,
    ):
        """Helper: set up, run, and return result for a pinned constrained grid."""
        from simulation.constraints import build_constraints
        from simulation.solver.xpbd import XPBDSolver

        config = SimConfig(
            total_frames=total_frames,
            substeps=6,
            solver_iterations=12,
            damping=0.98,
            max_particles=cols * rows + 100,
        )

        grid = generate_grid(
            width=1.0, height=1.0, cols=cols, rows=rows, center=(0, 2.0, 0)
        )

        constraints = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
        )

        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        # Pin top-left and top-right corners
        state.pin_particle(0)
        state.pin_particle(cols - 1)

        solver = XPBDSolver(
            constraints=constraints,
            stretch_compliance=stretch_compliance,
            bend_compliance=bend_compliance,
        )

        engine = SimulationEngine(config, solver=solver)
        result = engine.run(state)

        return result, grid, config, state

    def test_constrained_fall_no_nan(self):
        """No NaN or Inf values after constrained simulation."""
        result, grid, config, state = self._run_constrained_cloth(
            cols=10, rows=10, total_frames=60
        )

        assert not np.any(np.isnan(result.positions)), "NaN detected"
        assert not np.any(np.isinf(result.positions)), "Inf detected"

    def test_pinned_corners_stay(self):
        """Pinned corners should not move during constrained simulation."""
        result, grid, config, state = self._run_constrained_cloth(
            cols=10, rows=10, total_frames=60
        )

        # Top-left (index 0) and top-right (index cols-1)
        np.testing.assert_allclose(
            result.positions[0], grid.positions[0], atol=1e-5,
            err_msg="Top-left pin moved"
        )
        np.testing.assert_allclose(
            result.positions[9], grid.positions[9], atol=1e-5,
            err_msg="Top-right pin moved"
        )

    def test_cloth_hangs_below_pins(self):
        """All non-pinned particles should hang below the pin height."""
        result, grid, config, state = self._run_constrained_cloth(
            cols=10, rows=10, total_frames=60
        )

        pin_y = grid.positions[0, 1]  # Y of pinned particles

        # Bottom vertices should be below pins
        min_y = np.min(result.positions[:, 1])
        assert min_y < pin_y - 0.1, (
            f"Cloth not hanging: min_y={min_y:.3f}, pin_y={pin_y:.3f}"
        )

    def test_edge_lengths_preserved(self):
        """
        Distance constraints should preserve edge lengths.
        Mean stretch error should be < 5%.
        """
        result, grid, config, state = self._run_constrained_cloth(
            cols=10, rows=10, total_frames=60,
            stretch_compliance=0.0,  # Rigid
        )

        edge_lengths = np.linalg.norm(
            result.positions[grid.edges[:, 1]] - result.positions[grid.edges[:, 0]],
            axis=1,
        )
        rest_lengths = np.linalg.norm(
            grid.positions[grid.edges[:, 1]] - grid.positions[grid.edges[:, 0]],
            axis=1,
        )

        stretch_ratios = edge_lengths / rest_lengths
        mean_stretch = np.mean(np.abs(stretch_ratios - 1.0))

        assert mean_stretch < 0.05, (
            f"Mean stretch {mean_stretch:.4%} exceeds 5% threshold"
        )

    def test_no_rubber_banding(self):
        """
        Check that the cloth doesn't oscillate wildly.
        The lowest point should not go below a reasonable limit
        (no rubber-band effect from overcorrection).
        """
        result, grid, config, state = self._run_constrained_cloth(
            cols=10, rows=10, total_frames=120,
        )

        min_y = np.min(result.positions[:, 1])
        # A 1.0m wide cloth pinned at top shouldn't hang more than
        # ~1.5m below the pins (with reasonable stiffness)
        pin_y = grid.positions[0, 1]
        hang_distance = pin_y - min_y

        assert hang_distance < 2.0, (
            f"Cloth hanging {hang_distance:.2f}m — possible rubber-banding"
        )
        assert hang_distance > 0.1, (
            f"Cloth barely hanging {hang_distance:.2f}m — constraints too stiff?"
        )

    def test_energy_decays(self):
        """
        With damping, kinetic energy should decrease from the initial drop.
        Compare velocities at end should be small.
        """
        result, grid, config, state = self._run_constrained_cloth(
            cols=10, rows=10, total_frames=120,
        )

        velocities = state.get_velocities_numpy()
        mean_speed = np.mean(np.linalg.norm(velocities, axis=1))

        # After 2 seconds of damped simulation, velocity should be small
        assert mean_speed < 1.0, (
            f"Mean speed {mean_speed:.4f} m/s — not settling"
        )
