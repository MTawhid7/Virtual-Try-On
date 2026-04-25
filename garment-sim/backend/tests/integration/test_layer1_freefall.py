import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid


class TestFreefall:
    """Validate that particles fall correctly under gravity with no constraints."""

    def test_freefall_positions(self):
        """
        Drop a 10×10 grid under pure gravity for 1 second.
        Verify final Y position matches analytical freefall within 1%.
        """
        config = SimConfig(
            total_frames=60,       # 1 second at 60fps
            substeps=6,
            solver_iterations=0,   # No constraints
            damping=1.0,           # No damping (pure physics)
            max_particles=200,
        )

        grid = generate_grid(cols=10, rows=10, center=(0, 2.0, 0))
        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        engine = SimulationEngine(config)
        result = engine.run(state)

        # Analytical: y = y0 + 0.5 * g * t^2
        t = config.total_frames * config.dt
        expected_y = 2.0 + 0.5 * config.gravity * t * t

        mean_y = np.mean(result.positions[:, 1])

        # Allow 5% tolerance due to substep discretization
        assert abs(mean_y - expected_y) < abs(expected_y - 2.0) * 0.05, (
            f"Freefall Y={mean_y:.4f}, expected={expected_y:.4f}"
        )

    def test_freefall_no_nan(self):
        """Verify no NaN values appear in positions after simulation."""
        config = SimConfig(
            total_frames=120,
            substeps=6,
            solver_iterations=0,
            damping=1.0,
            max_particles=200,
        )

        grid = generate_grid(cols=10, rows=10, center=(0, 5.0, 0))
        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        engine = SimulationEngine(config)
        result = engine.run(state)

        assert not np.any(np.isnan(result.positions)), "NaN detected in positions"
        assert not np.any(np.isinf(result.positions)), "Inf detected in positions"

    def test_freefall_uniform_motion(self):
        """
        All particles should fall identically (same Y) since there are
        no constraints or collisions to differentiate them.
        """
        config = SimConfig(
            total_frames=30,
            substeps=6,
            solver_iterations=0,
            damping=1.0,
            max_particles=200,
        )

        grid = generate_grid(cols=10, rows=10, center=(0, 2.0, 0))
        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        engine = SimulationEngine(config)
        result = engine.run(state)

        # All Y values should be identical (uniform gravity, no constraints)
        y_values = result.positions[:, 1]
        assert np.std(y_values) < 1e-5, (
            f"Y standard deviation too high: {np.std(y_values):.6f} "
            "(expected uniform fall)"
        )

    def test_freefall_xz_unchanged(self):
        """X and Z positions should not change during pure freefall."""
        config = SimConfig(
            total_frames=30,
            substeps=6,
            solver_iterations=0,
            damping=1.0,
            max_particles=200,
        )

        grid = generate_grid(cols=10, rows=10, center=(0, 2.0, 0))
        initial_xz = grid.positions[:, [0, 2]].copy()

        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        engine = SimulationEngine(config)
        result = engine.run(state)

        final_xz = result.positions[:, [0, 2]]
        np.testing.assert_allclose(
            final_xz, initial_xz, atol=1e-6,
            err_msg="X/Z positions changed during pure freefall"
        )

    def test_pinned_particles_stay(self):
        """Pinned particles (inv_mass=0) should not move."""
        config = SimConfig(
            total_frames=30,
            substeps=6,
            solver_iterations=0,
            damping=1.0,
            max_particles=200,
        )

        grid = generate_grid(cols=10, rows=10, center=(0, 2.0, 0))
        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        # Pin first and last particles
        pin_indices = [0, 9]
        original_positions = grid.positions[pin_indices].copy()
        for idx in pin_indices:
            state.pin_particle(idx)

        engine = SimulationEngine(config)
        result = engine.run(state)

        for i, idx in enumerate(pin_indices):
            np.testing.assert_allclose(
                result.positions[idx], original_positions[i], atol=1e-6,
                err_msg=f"Pinned particle {idx} moved"
            )

    def test_velocity_increases_under_gravity(self):
        """After falling, velocity should be approximately g*t in Y."""
        config = SimConfig(
            total_frames=30,
            substeps=6,
            solver_iterations=0,
            damping=1.0,
            max_particles=200,
        )

        grid = generate_grid(cols=5, rows=5, center=(0, 10.0, 0))
        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        engine = SimulationEngine(config)
        engine.run(state)

        # Check velocity after simulation (still in the state)
        velocities = state.get_velocities_numpy()
        mean_vy = np.mean(velocities[:, 1])

        # Expected: vy ≈ g * t = -9.81 * 0.5 = -4.905 m/s
        t = config.total_frames * config.dt
        expected_vy = config.gravity * t

        # Allow 5% tolerance
        assert abs(mean_vy - expected_vy) < abs(expected_vy) * 0.05, (
            f"Mean Vy={mean_vy:.4f}, expected={expected_vy:.4f}"
        )
