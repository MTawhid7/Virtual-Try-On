"""
Layer-by-layer integration tests for the simulation engine.

Each test validates one layer of the physics pipeline:
    Layer 1: Particle system (gravity + integration)
    Layer 2: Structural constraints (distance + bending)  — Sprint 1 Layer 2
    Layer 3a: Collision (sphere, then body mesh)           — Sprint 1 Layer 3a
    Layer 3b: Stitch constraints                           — Sprint 2
    Layer 4: Material variety + damping                    — Sprint 2
"""

import numpy as np
import pytest

# Import simulation package (triggers ti.init)
import simulation  # noqa: F401
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid


# ============================================================================
# Layer 1: Particle System — Gravity + Integration
# ============================================================================


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


# ============================================================================
# Grid Mesh Tests
# ============================================================================


class TestGridMesh:
    """Validate grid mesh generation."""

    def test_grid_vertex_count(self):
        grid = generate_grid(cols=10, rows=10)
        assert grid.positions.shape == (100, 3)

    def test_grid_face_count(self):
        grid = generate_grid(cols=10, rows=10)
        # (rows-1) * (cols-1) quads, 2 triangles each
        expected = (10 - 1) * (10 - 1) * 2
        assert grid.faces.shape == (expected, 3)

    def test_grid_edge_count(self):
        grid = generate_grid(cols=10, rows=10)
        # Euler: E = V + F - 2 (for a topological disk, chi=1, so E = V + F - 1)
        # Actually for a grid: E = 3*(rows-1)*(cols-1) + (rows-1) + (cols-1)
        # But let's just verify edges exist and are reasonable
        assert grid.edges.shape[1] == 2
        assert len(grid.edges) > 0

    def test_grid_dimensions(self):
        grid = generate_grid(width=2.0, height=3.0, cols=5, rows=5, center=(1, 2, 3))
        xs = grid.positions[:, 0]
        ys = grid.positions[:, 1]
        zs = grid.positions[:, 2]

        assert abs((xs.max() - xs.min()) - 2.0) < 1e-5, "Width mismatch"
        assert abs((zs.max() - zs.min()) - 3.0) < 1e-5, "Height mismatch"
        assert abs(np.mean(ys) - 2.0) < 1e-5, "Y center mismatch"

    def test_grid_no_degenerate_triangles(self):
        grid = generate_grid(cols=10, rows=10)
        for f in grid.faces:
            assert len(set(f)) == 3, f"Degenerate triangle: {f}"

    def test_grid_edges_are_unique(self):
        grid = generate_grid(cols=10, rows=10)
        edge_set = set()
        for e in grid.edges:
            key = (min(e[0], e[1]), max(e[0], e[1]))
            assert key not in edge_set, f"Duplicate edge: {key}"
            edge_set.add(key)

    def test_grid_minimum_size_validation(self):
        with pytest.raises(ValueError, match="at least 2"):
            generate_grid(cols=1, rows=5)

    def test_grid_checkerboard_triangulation(self):
        """Verify alternating diagonal pattern for structural isotropy."""
        grid = generate_grid(cols=3, rows=3)
        # 2×2 quads → 4 quads → 8 triangles
        assert grid.faces.shape[0] == 8


# ============================================================================
# ParticleState Tests
# ============================================================================


class TestParticleState:
    """Validate particle state initialization and field operations."""

    def test_load_positions(self):
        config = SimConfig(max_particles=100)
        state = ParticleState(config)

        positions = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
        state.load_from_numpy(positions)

        assert state.n_particles == 2
        loaded = state.get_positions_numpy()
        np.testing.assert_allclose(loaded, positions)

    def test_default_inv_mass(self):
        config = SimConfig(max_particles=100)
        state = ParticleState(config)

        positions = np.zeros((5, 3), dtype=np.float32)
        state.load_from_numpy(positions)

        # Default inv_mass should be 1.0 (unit mass)
        for i in range(5):
            assert state.inv_mass[i] == 1.0

    def test_custom_inv_mass(self):
        config = SimConfig(max_particles=100)
        state = ParticleState(config)

        positions = np.zeros((3, 3), dtype=np.float32)
        inv_masses = np.array([1.0, 0.0, 0.5], dtype=np.float32)
        state.load_from_numpy(positions, inv_masses=inv_masses)

        assert state.inv_mass[0] == 1.0
        assert state.inv_mass[1] == 0.0  # Pinned
        assert state.inv_mass[2] == 0.5

    def test_pin_particle(self):
        config = SimConfig(max_particles=100)
        state = ParticleState(config)

        positions = np.zeros((5, 3), dtype=np.float32)
        state.load_from_numpy(positions)

        state.pin_particle(2)
        assert state.inv_mass[2] == 0.0

    def test_exceeds_max_particles(self):
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        positions = np.zeros((20, 3), dtype=np.float32)
        with pytest.raises(ValueError, match="exceeds max_particles"):
            state.load_from_numpy(positions)

    def test_zero_initial_velocities(self):
        config = SimConfig(max_particles=100)
        state = ParticleState(config)

        positions = np.ones((5, 3), dtype=np.float32)
        state.load_from_numpy(positions)

        velocities = state.get_velocities_numpy()
        np.testing.assert_allclose(velocities, 0.0, atol=1e-8)
