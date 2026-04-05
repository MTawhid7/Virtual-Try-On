"""
Tests for DistanceConstraints.apply_strain_limit() — the hard strain clamp.

Verifies that edges are clamped to [L0*(1-max_compress), L0*(1+max_stretch)]
after the pass, and that edges already within bounds are left untouched.
"""

import numpy as np

import simulation  # noqa: F401  (initialises Taichi before kernel import)
from simulation.constraints.distance import DistanceConstraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState


def _make_two_particle_state(p0_current, p1_current, rest_length=1.0, inv_mass=1.0):
    """
    Build a 2-particle state with a single edge.

    Constraints are initialised with the REST configuration (rest_length along X),
    then the state positions are set to the CURRENT (possibly deformed) positions.
    This separates rest-length initialisation from the deformed test configuration.
    """
    config = SimConfig(max_particles=10)
    state = ParticleState(config)

    # Current (deformed) positions loaded into state
    current_positions = np.array([p0_current, p1_current], dtype=np.float32)
    state.load_from_numpy(current_positions)

    # Rest positions: edge along X with the specified rest length
    rest_positions = np.array(
        [[0.0, 0.0, 0.0], [rest_length, 0.0, 0.0]], dtype=np.float32
    )
    dc = DistanceConstraints(max_edges=10)
    edges = np.array([[0, 1]], dtype=np.int32)
    dc.initialize(edges, rest_positions)  # rest length = rest_length

    inv_masses = np.full(10, inv_mass, dtype=np.float32)
    state.inv_mass.from_numpy(inv_masses)

    return state, dc


class TestStrainLimit:
    """Hard strain-clamp kernel correctness."""

    def test_overstretched_edge_gets_clamped(self):
        """
        An edge stretched to 2× rest length should be clamped to
        L0 × (1 + max_stretch).
        """
        p0 = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        p1 = np.array([2.0, 0.0, 0.0], dtype=np.float32)  # rest length = 1, current = 2
        state, dc = _make_two_particle_state(p0, p1)

        max_stretch = 0.05   # allow 5% stretch
        max_compress = 0.01  # allow 1% compression

        dc.apply_strain_limit(
            state.positions, state.inv_mass, dc.n_edges, max_stretch, max_compress
        )

        pos = state.get_positions_numpy()
        final_length = float(np.linalg.norm(pos[1] - pos[0]))
        expected_max = 1.0 * (1.0 + max_stretch)  # 1.05

        assert final_length <= expected_max + 1e-5, (
            f"Edge not clamped: final_length={final_length:.4f}, "
            f"expected ≤ {expected_max:.4f}"
        )

    def test_compressed_edge_gets_clamped(self):
        """
        An edge compressed to 0.5× rest length should be clamped to
        L0 × (1 - max_compress).
        """
        p0 = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        p1 = np.array([0.5, 0.0, 0.0], dtype=np.float32)  # rest = 1, current = 0.5
        state, dc = _make_two_particle_state(p0, p1)

        max_stretch = 0.05
        max_compress = 0.01  # allow only 1% compression

        dc.apply_strain_limit(
            state.positions, state.inv_mass, dc.n_edges, max_stretch, max_compress
        )

        pos = state.get_positions_numpy()
        final_length = float(np.linalg.norm(pos[1] - pos[0]))
        expected_min = 1.0 * (1.0 - max_compress)  # 0.99

        assert final_length >= expected_min - 1e-5, (
            f"Compressed edge not clamped: final_length={final_length:.4f}, "
            f"expected ≥ {expected_min:.4f}"
        )

    def test_edge_within_limits_unchanged(self):
        """
        An edge at rest length (ratio = 1.0) should not be moved at all.
        """
        p0 = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        p1 = np.array([1.0, 0.0, 0.0], dtype=np.float32)  # exactly at rest length
        state, dc = _make_two_particle_state(p0, p1)

        pos_before = state.get_positions_numpy().copy()

        dc.apply_strain_limit(
            state.positions, state.inv_mass, dc.n_edges, 0.05, 0.01
        )

        pos_after = state.get_positions_numpy()
        max_displacement = float(np.max(np.abs(pos_after - pos_before)))

        assert max_displacement < 1e-6, (
            f"Edge at rest length was moved: displacement={max_displacement:.2e}"
        )

    def test_strain_limit_respects_mass_ratio(self):
        """
        When one particle is heavier (lower inv_mass), the lighter particle
        should move more to satisfy the constraint.
        """
        # Stretched to 2× rest length
        p0 = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        p1 = np.array([2.0, 0.0, 0.0], dtype=np.float32)

        config = SimConfig(max_particles=10)
        state = ParticleState(config)
        state.load_from_numpy(np.array([p0, p1], dtype=np.float32))

        # Rest positions: 1 m apart (so current is 2× rest)
        rest_positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        dc = DistanceConstraints(max_edges=10)
        edges = np.array([[0, 1]], dtype=np.int32)
        dc.initialize(edges, rest_positions)

        # p0 is 4× heavier than p1 (inv_mass[0]=1, inv_mass[1]=4)
        inv_masses = np.zeros(10, dtype=np.float32)
        inv_masses[0] = 1.0   # heavier
        inv_masses[1] = 4.0   # lighter
        state.inv_mass.from_numpy(inv_masses)

        dc.apply_strain_limit(
            state.positions, state.inv_mass, dc.n_edges, 0.05, 0.01
        )

        pos = state.get_positions_numpy()
        p0_disp = float(abs(pos[0, 0] - 0.0))
        p1_disp = float(abs(pos[1, 0] - 2.0))

        # Lighter particle (higher inv_mass) moves more
        assert p1_disp > p0_disp, (
            f"Lighter particle should move more: p0_disp={p0_disp:.4f}, "
            f"p1_disp={p1_disp:.4f}"
        )

    def test_pinned_particle_not_moved(self):
        """A pinned particle (inv_mass=0) should not be moved by strain limiting."""
        p0 = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        p1 = np.array([2.0, 0.0, 0.0], dtype=np.float32)

        config = SimConfig(max_particles=10)
        state = ParticleState(config)
        positions = np.array([p0, p1], dtype=np.float32)
        state.load_from_numpy(positions)

        dc = DistanceConstraints(max_edges=10)
        edges = np.array([[0, 1]], dtype=np.int32)
        dc.initialize(edges, positions)

        inv_masses = np.zeros(10, dtype=np.float32)
        inv_masses[0] = 0.0   # pinned
        inv_masses[1] = 1.0
        state.inv_mass.from_numpy(inv_masses)

        pos_p0_before = state.get_positions_numpy()[0].copy()

        dc.apply_strain_limit(
            state.positions, state.inv_mass, dc.n_edges, 0.05, 0.01
        )

        pos_p0_after = state.get_positions_numpy()[0]
        assert np.allclose(pos_p0_before, pos_p0_after, atol=1e-6), (
            "Pinned particle was moved by strain limit"
        )
