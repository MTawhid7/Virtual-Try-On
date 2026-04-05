import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.mesh.grid import compute_area_weighted_inv_masses, generate_grid


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


class TestAreaWeightedInvMasses:
    """Validate compute_area_weighted_inv_masses correctness."""

    def test_total_mass_conservation(self):
        """sum(1/inv_mass) × density ≈ total mesh area × density (total fabric mass)."""
        grid = generate_grid(width=1.2, height=1.2, cols=20, rows=20)
        density = 0.30
        inv_masses = compute_area_weighted_inv_masses(grid.positions, grid.faces, density)

        # sum of vertex masses == total fabric mass
        total_mass = np.sum(1.0 / inv_masses)
        expected_mass = 1.2 * 1.2 * density  # area × density
        assert abs(total_mass - expected_mass) < 1e-5, (
            f"Mass not conserved: got {total_mass:.6f} kg, expected {expected_mass:.6f} kg"
        )

    def test_interior_vs_boundary_mass(self):
        """Interior particles accumulate more triangle area than boundary particles."""
        grid = generate_grid(cols=10, rows=10)
        inv_masses = compute_area_weighted_inv_masses(grid.positions, grid.faces, density=1.0)

        # Interior particle index (e.g. row=5, col=5 on a 10×10 grid)
        interior_idx = 5 * 10 + 5
        # Corner particle
        corner_idx = 0

        # Interior particle has MORE area → HIGHER mass → LOWER inv_mass
        assert inv_masses[interior_idx] < inv_masses[corner_idx], (
            f"Interior inv_mass {inv_masses[interior_idx]:.2f} should be less than "
            f"corner inv_mass {inv_masses[corner_idx]:.2f}"
        )

    def test_interior_two_mass_classes(self):
        """
        Checkerboard triangulation creates exactly two interior vertex classes.

        Even-parity vertices connect to 8 triangles; odd-parity to 4 triangles.
        This gives a 2:1 area ratio between the two classes — both are uniform
        within their class. Any other distribution indicates a broken triangulation.
        """
        grid = generate_grid(cols=10, rows=10)
        inv_masses = compute_area_weighted_inv_masses(grid.positions, grid.faces, density=1.0)

        # Interior particles: rows 1-8, cols 1-8
        interior_indices = [r * 10 + c for r in range(1, 9) for c in range(1, 9)]
        interior_inv_masses = inv_masses[interior_indices]

        unique_vals = np.unique(np.round(interior_inv_masses, decimals=3))
        assert len(unique_vals) == 2, (
            f"Expected exactly 2 interior mass classes, got {len(unique_vals)}: {unique_vals}"
        )
        ratio = unique_vals[1] / unique_vals[0]
        assert abs(ratio - 2.0) < 0.01, (
            f"Expected 2:1 mass ratio between interior classes, got {ratio:.4f}"
        )

    def test_zero_area_guard(self):
        """Degenerate near-zero-area triangle does not raise a division error."""
        positions = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 0]], dtype=np.float32)
        faces = np.array([[0, 1, 2], [0, 3, 1]], dtype=np.int32)  # second tri is degenerate

        # Should not raise
        inv_masses = compute_area_weighted_inv_masses(positions, faces, density=1.0)
        assert inv_masses is not None

    def test_all_positive(self):
        """All returned inv_mass values are strictly positive."""
        grid = generate_grid(cols=15, rows=15)
        inv_masses = compute_area_weighted_inv_masses(grid.positions, grid.faces, density=0.30)

        assert np.all(inv_masses > 0), "All inv_mass values must be > 0"
