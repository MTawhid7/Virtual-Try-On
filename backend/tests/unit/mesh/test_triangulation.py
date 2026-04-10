"""Unit tests for mesh/triangulation.py — pattern panel triangulation."""

import numpy as np
import pytest

import simulation  # noqa: F401 — initializes Taichi
from simulation.mesh.triangulation import triangulate_panel


RECTANGLE = [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]]

TRAPEZOID = [[0.0, 0.0], [0.5, 0.0], [0.4, 0.6], [0.1, 0.6]]

SQUARE = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]


class TestTriangulatePanelBasic:
    """Basic structural validity checks."""

    def test_returns_3d_positions(self):
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert panel.positions.ndim == 2
        assert panel.positions.shape[1] == 3
        assert panel.positions.dtype == np.float32

    def test_y_is_zero(self):
        """Positions are in the XZ plane — Y must be 0 for local panel space."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert np.all(panel.positions[:, 1] == 0.0), "Y should be 0 in local panel space"

    def test_faces_shape(self):
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert panel.faces.ndim == 2
        assert panel.faces.shape[1] == 3
        assert panel.faces.dtype == np.int32

    def test_edges_shape(self):
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert panel.edges.ndim == 2
        assert panel.edges.shape[1] == 2
        assert panel.edges.dtype == np.int32

    def test_uvs_in_unit_square(self):
        """UV coordinates must be in [0, 1]²."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert panel.uvs.shape == (panel.positions.shape[0], 2)
        assert panel.uvs.dtype == np.float32
        assert np.all(panel.uvs >= -1e-6), "UV coords must be >= 0"
        assert np.all(panel.uvs <= 1.0 + 1e-6), "UV coords must be <= 1"

    def test_boundary_indices(self):
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert len(panel.boundary_indices) >= len(RECTANGLE)
        assert len(panel.original_vertex_mapping) == len(RECTANGLE)
        assert np.all(panel.boundary_indices < panel.positions.shape[0])

    def test_face_indices_in_range(self):
        panel = triangulate_panel(RECTANGLE, resolution=10)
        n = panel.positions.shape[0]
        assert np.all(panel.faces >= 0)
        assert np.all(panel.faces < n), "Face indices must reference valid vertices"

    def test_edge_indices_in_range(self):
        panel = triangulate_panel(RECTANGLE, resolution=10)
        n = panel.positions.shape[0]
        assert np.all(panel.edges >= 0)
        assert np.all(panel.edges < n)

    def test_edges_are_sorted_pairs(self):
        """Each edge must have a[0] < a[1] (canonical form, no duplicates)."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert np.all(panel.edges[:, 0] < panel.edges[:, 1]), "Edges must be sorted (a < b)"

    def test_no_degenerate_faces(self):
        """No face should have two identical vertex indices."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        for face in panel.faces:
            assert len(set(face)) == 3, f"Degenerate face: {face}"

    def test_no_zero_area_faces(self):
        """All triangles must have positive area."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        pos2d = panel.positions[:, [0, 2]]  # XZ slice
        v0 = pos2d[panel.faces[:, 0]]
        v1 = pos2d[panel.faces[:, 1]]
        v2 = pos2d[panel.faces[:, 2]]
        cross = (v1[:, 0] - v0[:, 0]) * (v2[:, 1] - v0[:, 1]) - \
                (v1[:, 1] - v0[:, 1]) * (v2[:, 0] - v0[:, 0])
        assert np.all(np.abs(cross) > 1e-10), "Zero-area triangles found"

    def test_has_enough_particles(self):
        """resolution=10 should produce a reasonable number of particles (> 20)."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        assert panel.positions.shape[0] > 20, "Too few particles for resolution=10"


class TestTriangulatePanelGeometry:
    """Checks that vertices lie inside the polygon boundary."""

    def test_rectangle_vertices_inside(self):
        panel = triangulate_panel(RECTANGLE, resolution=15)
        xs = panel.positions[:, 0]
        zs = panel.positions[:, 2]
        assert xs.min() >= -1e-5 and xs.max() <= 0.4 + 1e-5, "X outside rectangle"
        assert zs.min() >= -1e-5 and zs.max() <= 0.7 + 1e-5, "Z outside rectangle"

    def test_trapezoid_vertices_inside(self):
        """No vertex should fall outside the trapezoid bounding box."""
        panel = triangulate_panel(TRAPEZOID, resolution=15)
        xs = panel.positions[:, 0]
        zs = panel.positions[:, 2]
        assert xs.min() >= -1e-5
        assert xs.max() <= 0.5 + 1e-5
        assert zs.min() >= -1e-5
        assert zs.max() <= 0.6 + 1e-5

    def test_boundary_vertices_match_polygon(self):
        """The mapped vertices must match the input polygon (lifted to 3D)."""
        poly = RECTANGLE
        panel = triangulate_panel(poly, resolution=10)
        for i, (px, pz) in enumerate(poly):
            mapped_idx = panel.original_vertex_mapping[i]
            # Since triangulate_panel outputs mapped_idx which corresponds to the first N items in positions...
            vx = panel.positions[mapped_idx, 0]
            vz = panel.positions[mapped_idx, 2]
            assert abs(vx - px) < 1e-5, f"Boundary vertex {i}: X mismatch {vx} vs {px}"
            assert abs(vz - pz) < 1e-5, f"Boundary vertex {i}: Z mismatch {vz} vs {pz}"
            assert abs(vz - pz) < 1e-5, f"Boundary vertex {i}: Z mismatch {vz} vs {pz}"

    def test_uv_boundary_corners(self):
        """Boundary corners of a rectangle should be at UV (0,0), (1,0), (1,1), (0,1)."""
        panel = triangulate_panel(RECTANGLE, resolution=10)
        expected_uvs = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
        for i in range(4):
            uv = panel.uvs[i]
            exp = expected_uvs[i]
            assert np.allclose(uv, exp, atol=1e-5), f"Corner UV {i}: got {uv}, expected {exp}"


class TestTriangulatePanelResolution:
    """Resolution parameter controls particle density."""

    def test_higher_resolution_more_particles(self):
        panel_lo = triangulate_panel(SQUARE, resolution=5)
        panel_hi = triangulate_panel(SQUARE, resolution=20)
        assert panel_hi.positions.shape[0] > panel_lo.positions.shape[0]

    def test_higher_resolution_more_faces(self):
        panel_lo = triangulate_panel(SQUARE, resolution=5)
        panel_hi = triangulate_panel(SQUARE, resolution=20)
        assert panel_hi.faces.shape[0] > panel_lo.faces.shape[0]

    def test_minimum_resolution(self):
        """resolution=2 should still produce a valid mesh (no crash)."""
        panel = triangulate_panel(RECTANGLE, resolution=2)
        assert panel.positions.shape[0] >= 4  # at least the boundary vertices
        assert panel.faces.shape[0] >= 1


class TestTriangulatePanelErrors:
    """Error handling."""

    def test_too_few_vertices(self):
        with pytest.raises(ValueError, match="at least 3"):
            triangulate_panel([[0, 0], [1, 0]], resolution=5)

    def test_degenerate_polygon(self):
        with pytest.raises(ValueError, match="Degenerate"):
            triangulate_panel([[0, 0], [1, 0], [2, 0]], resolution=5)  # collinear
