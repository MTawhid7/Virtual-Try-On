import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.constraints.distance import DistanceConstraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState


class TestDistanceConstraint:
    """Validate distance constraint projection math."""

    def test_preserves_edge_length(self):
        """
        Create two particles connected by an edge. Stretch them apart.
        After projecting, the distance should be close to the rest length.
        """
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        # Two particles at distance 1.0
        positions = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32)
        state.load_from_numpy(positions)

        edges = np.array([[0, 1]], dtype=np.int32)

        dist_constraints = DistanceConstraints(max_edges=10)
        dist_constraints.initialize(edges, positions)

        # Stretch the particles apart to distance 2.0
        stretched = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
        state.positions.from_numpy(
            np.pad(stretched, ((0, config.max_particles - 2), (0, 0)))
        )

        # Project with zero compliance (rigid)
        dt = 1.0 / 60.0
        n_iterations = 10
        for _ in range(n_iterations):
            dist_constraints.project(
                state.positions, state.inv_mass, dist_constraints.n_edges,
                0.0,  # compliance = 0 (rigid)
                dt,
            )

        # Check distance is close to rest length (1.0)
        p = state.get_positions_numpy()
        dist = np.linalg.norm(p[1] - p[0])
        assert abs(dist - 1.0) < 0.01, f"Distance {dist:.4f}, expected ~1.0"

    def test_pinned_particle_stays(self):
        """
        One pinned, one free particle. Stretching should only move the free one.
        """
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        positions = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32)
        state.load_from_numpy(positions)
        state.pin_particle(0)  # Pin first particle

        edges = np.array([[0, 1]], dtype=np.int32)

        dist_constraints = DistanceConstraints(max_edges=10)
        dist_constraints.initialize(edges, positions)

        # Stretch particle 1 to x=2
        stretched = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
        state.positions.from_numpy(
            np.pad(stretched, ((0, config.max_particles - 2), (0, 0)))
        )

        dt = 1.0 / 60.0
        for _ in range(20):
            dist_constraints.project(
                state.positions, state.inv_mass, dist_constraints.n_edges,
                0.0, dt,
            )

        p = state.get_positions_numpy()
        # Pinned particle should not have moved
        np.testing.assert_allclose(p[0], [0, 0, 0], atol=1e-6)
        # Free particle should be at distance ~1.0 from pin
        dist = np.linalg.norm(p[1] - p[0])
        assert abs(dist - 1.0) < 0.01, f"Distance {dist:.4f}, expected ~1.0"

    def test_compliance_allows_stretch(self):
        """
        With non-zero compliance, edges should be allowed to stretch.
        High compliance should result in more stretch than zero compliance.
        """
        config = SimConfig(max_particles=10)

        edges = np.array([[0, 1]], dtype=np.int32)
        positions = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32)

        # Test with zero compliance (rigid)
        state_rigid = ParticleState(config)
        state_rigid.load_from_numpy(positions.copy())
        dist_rigid = DistanceConstraints(max_edges=10)
        dist_rigid.initialize(edges, positions)
        stretched = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
        state_rigid.positions.from_numpy(
            np.pad(stretched.copy(), ((0, config.max_particles - 2), (0, 0)))
        )
        dt = 1.0 / 60.0
        for _ in range(5):
            dist_rigid.project(
                state_rigid.positions, state_rigid.inv_mass, 1, 0.0, dt
            )

        # Test with high compliance (soft)
        state_soft = ParticleState(config)
        state_soft.load_from_numpy(positions.copy())
        dist_soft = DistanceConstraints(max_edges=10)
        dist_soft.initialize(edges, positions)
        state_soft.positions.from_numpy(
            np.pad(stretched.copy(), ((0, config.max_particles - 2), (0, 0)))
        )
        for _ in range(5):
            dist_soft.project(
                state_soft.positions, state_soft.inv_mass, 1, 1.0, dt
            )

        p_rigid = state_rigid.get_positions_numpy()
        p_soft = state_soft.get_positions_numpy()

        dist_after_rigid = np.linalg.norm(p_rigid[1] - p_rigid[0])
        dist_after_soft = np.linalg.norm(p_soft[1] - p_soft[0])

        # Soft should be more stretched than rigid
        assert dist_after_soft > dist_after_rigid, (
            f"Soft ({dist_after_soft:.4f}) should be longer than rigid ({dist_after_rigid:.4f})"
        )

    def test_rest_length_computed_correctly(self):
        """Rest lengths should match initial edge lengths."""
        positions = np.array([
            [0, 0, 0], [3, 0, 0], [0, 4, 0]
        ], dtype=np.float32)
        edges = np.array([[0, 1], [0, 2], [1, 2]], dtype=np.int32)

        dist = DistanceConstraints(max_edges=10)
        dist.initialize(edges, positions)

        assert abs(dist.rest_length[0] - 3.0) < 1e-5
        assert abs(dist.rest_length[1] - 4.0) < 1e-5
        assert abs(dist.rest_length[2] - 5.0) < 1e-5  # 3-4-5 triangle
