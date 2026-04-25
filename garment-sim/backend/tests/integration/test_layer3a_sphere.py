"""
Integration tests for Layer 3a: Sphere Collision.

Validates that a cloth grid dropped onto an analytical sphere drapes
correctly: no penetration, no upward crumpling, energy decays, and
structural constraints still work under collision perturbation.

Follows the same pattern as test_layer2_structure.py — a shared helper
method sets up and runs the scenario, individual tests check specific
physical properties.
"""

import numpy as np
import pytest

import simulation  # noqa: F401 — Taichi initialization
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid
from simulation.constraints import build_constraints
from simulation.solver.xpbd import XPBDSolver
from simulation.collision import SphereCollider


# Sphere parameters (shared across all tests)
SPHERE_CENTER = (0.0, 0.8, 0.0)
SPHERE_RADIUS = 0.5


class TestSphereDrape:
    """
    Validate that cloth dropped onto a sphere drapes without penetration,
    settles with decaying energy, and maintains structural integrity.
    """

    @staticmethod
    def _run_sphere_drape(
        cols: int = 15,
        rows: int = 15,
        total_frames: int = 60,
        stretch_compliance: float = 1e-8,
        bend_compliance: float = 1e-3,
        friction: float = 0.3,
    ):
        """
        Helper: set up and run a sphere drape scenario.

        Uses a smaller grid (15×15) than the CLI scene (20×20) for faster
        test execution while still being large enough to drape visually.

        Returns:
            (result, grid, config, state, sphere_center, sphere_radius)
        """
        config = SimConfig(
            total_frames=total_frames,
            substeps=6,
            solver_iterations=12,
            damping=0.98,
            max_particles=cols * rows + 100,
            collision_thickness=0.005,
            friction_coefficient=friction,
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

        # No pinned particles — free drape onto sphere

        solver = XPBDSolver(
            constraints=constraints,
            stretch_compliance=stretch_compliance,
            bend_compliance=bend_compliance,
        )
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        engine = SimulationEngine(config, solver=solver)
        engine.collider = collider
        result = engine.run(state)

        return result, grid, config, state

    def test_sphere_drape_no_nan(self):
        """No NaN or Inf values after sphere collision simulation."""
        result, grid, config, state = self._run_sphere_drape(
            cols=10, rows=10, total_frames=30
        )

        assert not np.any(np.isnan(result.positions)), "NaN detected in positions"
        assert not np.any(np.isinf(result.positions)), "Inf detected in positions"

    def test_no_penetration(self):
        """
        No particle should be inside the sphere after simulation.

        Every particle's distance from the sphere center must be at least
        radius + thickness - epsilon (1mm tolerance for numerical precision).
        """
        result, grid, config, state = self._run_sphere_drape(
            cols=15, rows=15, total_frames=60
        )

        center_np = np.array(SPHERE_CENTER, dtype=np.float32)
        distances = np.linalg.norm(result.positions - center_np, axis=1)
        min_dist = np.min(distances)
        target = SPHERE_RADIUS + config.collision_thickness

        # Allow 1mm tolerance for numerical precision
        assert min_dist >= target - 0.001, (
            f"Penetration detected: min_dist={min_dist:.4f}, "
            f"target={target:.4f}, penetration={target - min_dist:.4f}m"
        )

    def test_cloth_drapes_over_sphere(self):
        """
        Cloth should wrap around the sphere — some particles near the top
        of the sphere, and some particles hanging below the sphere top.
        This confirms the cloth is actually draping, not bouncing off or
        sliding away entirely.

        Note: With a 1.0m cloth on a 0.5m radius sphere, the cloth doesn't
        extend far below the sphere center — it mostly rests on the upper
        hemisphere. We check for particles above and below the sphere top.
        """
        result, grid, config, state = self._run_sphere_drape(
            cols=15, rows=15, total_frames=90  # Longer run for settling
        )

        sphere_top_y = SPHERE_CENTER[1] + SPHERE_RADIUS
        sphere_mid_y = SPHERE_CENTER[1] + SPHERE_RADIUS * 0.5

        # Some particles should be near/above sphere mid-height (resting on top)
        near_top = np.sum(result.positions[:, 1] > sphere_mid_y)
        # Some particles should be below the sphere top (hanging off sides)
        below_top = np.sum(result.positions[:, 1] < sphere_top_y)

        assert near_top > 0, (
            f"No particles above sphere mid-height (y>{sphere_mid_y:.2f}). "
            "Cloth may have slid off entirely."
        )
        assert below_top > 0, (
            f"No particles below sphere top (y<{sphere_top_y:.2f}). "
            "Cloth may not be draping down."
        )

    def test_no_upward_crumpling(self):
        """
        No particle should be above its initial height.

        This catches the upward crumpling artifact that plagued the Vestra
        SDF approach. With analytical sphere collision, this should never
        happen — collision only pushes outward, never upward beyond the
        initial drop height.
        """
        result, grid, config, state = self._run_sphere_drape(
            cols=15, rows=15, total_frames=60
        )

        initial_y = 2.0  # Grid starts at y=2.0
        max_y = np.max(result.positions[:, 1])

        assert max_y <= initial_y + 0.05, (
            f"Upward crumpling: max_y={max_y:.3f}, initial_y={initial_y:.1f}. "
            f"Particles moved {max_y - initial_y:.3f}m above start."
        )

    def test_energy_decays(self):
        """
        With damping, mean velocity should be small after simulation.

        The cloth should settle onto the sphere, not oscillate indefinitely.
        """
        result, grid, config, state = self._run_sphere_drape(
            cols=15, rows=15, total_frames=90  # Longer run for settling
        )

        velocities = state.get_velocities_numpy()
        mean_speed = np.mean(np.linalg.norm(velocities, axis=1))

        assert mean_speed < 1.0, (
            f"Mean speed {mean_speed:.4f} m/s after {90} frames — "
            "simulation not settling"
        )

    def test_edge_lengths_preserved(self):
        """
        Distance constraints should still preserve edge lengths even with
        collision perturbation.

        Slightly relaxed threshold (10%) compared to Layer 2 (5%) since
        collision introduces additional position corrections that compete
        with the distance constraint projections.
        """
        result, grid, config, state = self._run_sphere_drape(
            cols=15, rows=15, total_frames=60,
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

        assert mean_stretch < 0.10, (
            f"Mean stretch {mean_stretch:.4%} exceeds 10% threshold"
        )
