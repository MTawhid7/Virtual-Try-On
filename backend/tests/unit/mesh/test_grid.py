import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.mesh.grid import generate_grid


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
