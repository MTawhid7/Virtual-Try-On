"""Unit tests for constraints/stitch.py — zero rest-length stitch constraints."""

import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.constraints.stitch import StitchConstraints
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState


DT = 1.0 / 60.0 / 15.0  # substep dt at 15 substeps


def _make_state(positions: np.ndarray) -> ParticleState:
    """Helper: create a ParticleState from a numpy position array."""
    n = positions.shape[0]
    config = SimConfig(max_particles=n)
    state = ParticleState(config)
    state.load_from_numpy(positions.astype(np.float32))
    return state


class TestStitchConstraintBasic:
    """Basic correctness checks for stitch projection."""

    def test_closes_gap_after_iterations(self):
        """
        Two free particles 1m apart, stitched together.
        After 100 iterations the gap should be < 0.01m.
        """
        positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        state = _make_state(positions)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        for _ in range(100):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        gap = np.linalg.norm(pos[0] - pos[1])
        assert gap < 0.01, f"Gap {gap:.4f}m not closed after 100 iterations"

    def test_closes_gap_symmetrically(self):
        """
        Equal inv_mass — both particles move equal amounts toward each other.
        """
        positions = np.array([[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0]], dtype=np.float32)
        state = _make_state(positions)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        for _ in range(50):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        # With equal masses, midpoint stays near origin
        midpoint = (pos[0] + pos[1]) / 2.0
        assert np.linalg.norm(midpoint) < 0.05, \
            f"Midpoint drifted to {midpoint} — asymmetric correction"

    def test_no_nan_at_zero_gap(self):
        """
        Stitch pair with zero initial gap should not produce NaN.
        """
        positions = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        state = _make_state(positions)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        for _ in range(10):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        assert not np.any(np.isnan(pos)), "NaN produced with zero-gap stitch"

    def test_no_explosion_large_gap(self):
        """
        Large initial gap (0.5m — realistic seam opening at simulation start).
        Should converge without NaN or explosion.
        """
        positions = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]], dtype=np.float32)
        state = _make_state(positions)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        for _ in range(200):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        assert not np.any(np.isnan(pos)), "NaN with large initial gap"
        assert not np.any(np.isinf(pos)), "Inf with large initial gap"
        gap = np.linalg.norm(pos[0] - pos[1])
        assert gap < 0.01, f"Large gap not closed: {gap:.4f}m"


class TestStitchConstraintPinned:
    """Pinned particle behavior (inv_mass = 0)."""

    def test_pinned_particle_does_not_move(self):
        """
        Particle 0 is pinned (inv_mass=0). Only particle 1 should move.
        """
        positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        inv_masses = np.array([0.0, 1.0], dtype=np.float32)  # particle 0 pinned

        config = SimConfig(max_particles=2)
        state = ParticleState(config)
        state.load_from_numpy(positions, inv_masses=inv_masses)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        for _ in range(50):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        # Pinned particle must not have moved
        assert np.allclose(pos[0], [0.0, 0.0, 0.0], atol=1e-5), \
            f"Pinned particle moved to {pos[0]}"
        # Free particle should have moved toward origin
        assert pos[1, 0] < 0.5, f"Free particle didn't move: {pos[1]}"

    def test_both_pinned_no_movement(self):
        """Both pinned — neither moves."""
        positions = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        inv_masses = np.array([0.0, 0.0], dtype=np.float32)

        config = SimConfig(max_particles=2)
        state = ParticleState(config)
        state.load_from_numpy(positions, inv_masses=inv_masses)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        for _ in range(20):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        assert np.allclose(pos[0], [0.0, 0.0, 0.0], atol=1e-5)
        assert np.allclose(pos[1], [1.0, 0.0, 0.0], atol=1e-5)


class TestStitchConstraintMultiple:
    """Multiple stitch pairs."""

    def test_two_stitch_pairs(self):
        """
        Two independent stitch pairs both converge to < 0.01m gap.
        """
        positions = np.array([
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],  # pair 0
            [0.0, 1.0, 0.0], [0.0, 2.0, 0.0],  # pair 1
        ], dtype=np.float32)
        state = _make_state(positions)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1], [2, 3]], dtype=np.int32))

        assert stitch.n_stitches == 2

        for _ in range(100):
            stitch.reset_lambdas()
            stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)

        pos = state.positions.to_numpy()
        gap0 = np.linalg.norm(pos[0] - pos[1])
        gap1 = np.linalg.norm(pos[2] - pos[3])
        assert gap0 < 0.01, f"Pair 0 gap not closed: {gap0:.4f}m"
        assert gap1 < 0.01, f"Pair 1 gap not closed: {gap1:.4f}m"


class TestStitchConstraintInit:
    """Initialization edge cases."""

    def test_max_stitches_exceeded_raises(self):
        pairs = np.array([[0, 1], [2, 3]], dtype=np.int32)
        stitch = StitchConstraints(max_stitches=1)
        with pytest.raises(ValueError, match="exceeds max_stitches"):
            stitch.initialize(pairs)

    def test_reset_lambdas_clears_state(self):
        """reset_lambdas() should zero all accumulators between substeps."""
        positions = np.array([[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]], dtype=np.float32)
        state = _make_state(positions)

        stitch = StitchConstraints(max_stitches=10)
        stitch.initialize(np.array([[0, 1]], dtype=np.int32))

        # Run one round with reset
        stitch.reset_lambdas()
        stitch.project(state.positions, state.inv_mass, stitch.n_stitches, 1e-6, DT)
        lambdas_after = stitch.lambdas.to_numpy()[0]
        assert lambdas_after != 0.0, "Lambda should be non-zero after projection"

        # After reset, lambda should be 0
        stitch.reset_lambdas()
        lambdas_reset = stitch.lambdas.to_numpy()[0]
        assert lambdas_reset == 0.0, "Lambda not reset to 0"


class TestStitchIntegration:
    """Integration: build_constraints + XPBDSolver with stitch pairs."""

    def test_build_constraints_with_stitches(self):
        """build_constraints() with stitch_pairs creates a StitchConstraints object."""
        from simulation.constraints import build_constraints
        import numpy as np

        positions = np.array([
            [0.0, 0.0, 0.0], [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0], [1.0, 1.0, 0.0],
        ], dtype=np.float32)
        edges = np.array([[0, 1], [2, 3], [0, 2], [1, 3]], dtype=np.int32)
        stitch_pairs = np.array([[0, 2], [1, 3]], dtype=np.int32)

        cs = build_constraints(positions=positions, edges=edges, stitch_pairs=stitch_pairs)

        assert cs.stitch is not None
        assert cs.stitch.n_stitches == 2

    def test_solver_step_with_stitches(self):
        """XPBDSolver.step() runs without error when stitch constraints are present."""
        from simulation.constraints import build_constraints
        from simulation.solver.xpbd import XPBDSolver

        positions = np.array([[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]], dtype=np.float32)
        state = _make_state(positions)

        stitch_pairs = np.array([[0, 1]], dtype=np.int32)
        cs = build_constraints(positions=positions, stitch_pairs=stitch_pairs)

        solver = XPBDSolver(constraints=cs, stitch_compliance=1e-6)

        for _ in range(10):
            cs.reset_lambdas()
            solver.step(state, DT)

        pos = state.positions.to_numpy()
        assert not np.any(np.isnan(pos))
