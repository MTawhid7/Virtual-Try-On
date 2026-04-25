import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState


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
