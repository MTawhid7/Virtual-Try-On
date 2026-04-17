import numpy as np
import pytest

import simulation  # noqa: F401 — triggers Taichi init
from simulation.constraints.attachment import AttachmentConstraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState


class TestAttachmentConstraints:
    """Validate AttachmentConstraints XPBD projection behaviour."""

    def test_moves_toward_target(self):
        """Vertex should move toward its target position after projection iterations."""
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        # Particle starts far from target (btorso-like: Z=-0.1, target=+0.014)
        positions = np.array([[0.1, 1.0, -0.1], [0.0, 0.0, 0.0]], dtype=np.float32)
        state.load_from_numpy(positions)

        target = np.array([[0.1, 1.0, 0.014]], dtype=np.float32)
        ac = AttachmentConstraints(max_attachments=10)
        ac.initialize(
            vertex_indices=np.array([0], dtype=np.int32),
            target_positions=target,
        )

        dt = 1.0 / 60.0 / 4.0  # standard substep dt
        for _ in range(20):
            ac.reset_lambdas()
            ac.project(state.positions, state.inv_mass, ac.n_attachments, 1e-4, dt)

        result = state.get_positions_numpy()
        initial_dist = abs(-0.1 - 0.014)
        final_dist = abs(float(result[0, 2]) - 0.014)
        assert final_dist < initial_dist, (
            f"Vertex Z did not move toward target: initial_dist={initial_dist:.4f}, "
            f"final_dist={final_dist:.4f}"
        )

    def test_pinned_particle_unaffected(self):
        """Pinned particle (inv_mass == 0) must not be moved by attachment projection."""
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        positions = np.array([[0.0, 1.0, -0.2]], dtype=np.float32)
        state.load_from_numpy(positions)
        state.pin_particle(0)

        ac = AttachmentConstraints(max_attachments=10)
        ac.initialize(
            vertex_indices=np.array([0], dtype=np.int32),
            target_positions=np.array([[0.0, 1.0, 0.034]], dtype=np.float32),
        )

        dt = 1.0 / 240.0
        for _ in range(50):
            ac.reset_lambdas()
            ac.project(state.positions, state.inv_mass, ac.n_attachments, 1e-4, dt)

        result = state.get_positions_numpy()
        np.testing.assert_allclose(result[0], [0.0, 1.0, -0.2], atol=1e-6)

    def test_already_at_target_no_overshoot(self):
        """Vertex already at its target should stay there (no overshoot)."""
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        target = np.array([[0.0, 1.0, 0.034]], dtype=np.float32)
        state.load_from_numpy(target.copy())

        ac = AttachmentConstraints(max_attachments=10)
        ac.initialize(
            vertex_indices=np.array([0], dtype=np.int32),
            target_positions=target,
        )

        dt = 1.0 / 240.0
        for _ in range(10):
            ac.reset_lambdas()
            ac.project(state.positions, state.inv_mass, ac.n_attachments, 1e-4, dt)

        result = state.get_positions_numpy()
        np.testing.assert_allclose(result[0], target[0], atol=1e-5)
