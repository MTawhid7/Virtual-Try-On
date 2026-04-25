import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.constraints import build_constraints
from simulation.mesh.grid import generate_grid


class TestConstraintBuilder:
    """Validate the build_constraints() function."""

    def test_builds_both_constraint_types(self):
        """For a grid mesh, should create both distance and bending constraints."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
        )

        assert cs.distance is not None, "No distance constraints built"
        assert cs.bending is not None, "No bending constraints built"
        assert cs.distance.n_edges == len(grid.edges)
        assert cs.bending.n_hinges > 0

    def test_disable_distance(self):
        """Can disable distance constraints."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
            enable_distance=False,
        )

        assert cs.distance is None
        assert cs.bending is not None

    def test_disable_bending(self):
        """Can disable bending constraints."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
            enable_bending=False,
        )

        assert cs.distance is not None
        assert cs.bending is None

    def test_no_edges_no_constraints(self):
        """With no edges/faces, should return empty constraint set."""
        positions = np.zeros((5, 3), dtype=np.float32)
        cs = build_constraints(positions=positions)

        assert cs.distance is None
        assert cs.bending is None

    def test_reset_lambdas(self):
        """ConstraintSet.reset_lambdas() should not crash."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
        )

        # Should not raise
        cs.reset_lambdas()
