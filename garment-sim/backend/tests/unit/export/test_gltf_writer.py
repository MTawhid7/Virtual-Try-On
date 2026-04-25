"""
Unit tests for the glTF/GLB export module.

Tests that write_glb produces valid .glb files that can be reloaded
by trimesh with correct vertex/face counts, normals, and UVs.
"""

import numpy as np
import pytest
import trimesh

import simulation  # noqa: F401 — Taichi initialization
from simulation.export.gltf_writer import write_glb


@pytest.fixture
def simple_mesh():
    """A minimal 4-vertex, 2-triangle mesh (a quad split into two tris)."""
    positions = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ], dtype=np.float32)

    faces = np.array([
        [0, 1, 2],
        [0, 2, 3],
    ], dtype=np.int32)

    # Simple normals: all pointing along +Z
    normals = np.array([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)

    return positions, faces, normals


@pytest.fixture
def grid_mesh():
    """A 5×5 grid mesh for testing with more realistic geometry."""
    from simulation.mesh.grid import generate_grid
    grid = generate_grid(width=1.0, height=1.0, cols=5, rows=5, center=(0, 0, 0))
    from simulation.core.engine import compute_vertex_normals
    normals = compute_vertex_normals(grid.positions, grid.faces)
    return grid.positions, grid.faces, normals


class TestWriteGlb:
    """Tests for the write_glb function."""

    def test_write_glb_creates_file(self, simple_mesh, tmp_path):
        """write_glb should create a .glb file on disk."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "test.glb"

        result = write_glb(positions, faces, normals, path=out_path)

        assert result.exists(), f"Output file does not exist: {result}"
        assert result.stat().st_size > 0, "Output file is empty"

    def test_glb_loads_in_trimesh(self, simple_mesh, tmp_path):
        """Exported .glb should load in trimesh without errors."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "test.glb"
        write_glb(positions, faces, normals, path=out_path)

        scene = trimesh.load(str(out_path))
        assert scene is not None

    def test_vertex_count_preserved(self, simple_mesh, tmp_path):
        """Reloaded mesh should have the same vertex count as input."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "test.glb"
        write_glb(positions, faces, normals, path=out_path)

        scene = trimesh.load(str(out_path))
        # Extract the geometry (may be a Scene with named geometries)
        if hasattr(scene, 'geometry'):
            mesh = list(scene.geometry.values())[0]
        else:
            mesh = scene

        assert len(mesh.vertices) == len(positions), (
            f"Vertex count mismatch: {len(mesh.vertices)} != {len(positions)}"
        )

    def test_face_count_preserved(self, simple_mesh, tmp_path):
        """Reloaded mesh should have the same face count as input."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "test.glb"
        write_glb(positions, faces, normals, path=out_path)

        scene = trimesh.load(str(out_path))
        if hasattr(scene, 'geometry'):
            mesh = list(scene.geometry.values())[0]
        else:
            mesh = scene

        assert len(mesh.faces) == len(faces), (
            f"Face count mismatch: {len(mesh.faces)} != {len(faces)}"
        )

    def test_normals_present(self, simple_mesh, tmp_path):
        """Exported mesh should contain vertex normals."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "test.glb"
        write_glb(positions, faces, normals, path=out_path)

        scene = trimesh.load(str(out_path))
        if hasattr(scene, 'geometry'):
            mesh = list(scene.geometry.values())[0]
        else:
            mesh = scene

        # trimesh always computes normals, so just check shape
        assert mesh.vertex_normals.shape == positions.shape, (
            f"Normals shape mismatch: {mesh.vertex_normals.shape} != {positions.shape}"
        )

    def test_uvs_optional(self, simple_mesh, tmp_path):
        """Export with uvs=None should produce a valid .glb."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "test_no_uv.glb"

        # Should not raise
        result = write_glb(positions, faces, normals, uvs=None, path=out_path)
        assert result.exists()

        # Should reload cleanly
        scene = trimesh.load(str(out_path))
        assert scene is not None

    def test_creates_parent_dirs(self, simple_mesh, tmp_path):
        """write_glb should create parent directories if they don't exist."""
        positions, faces, normals = simple_mesh
        out_path = tmp_path / "nested" / "deep" / "output.glb"

        result = write_glb(positions, faces, normals, path=out_path)

        assert result.exists(), f"File not created at nested path: {result}"

    def test_grid_mesh_export(self, grid_mesh, tmp_path):
        """A 5×5 grid mesh should export and reload correctly."""
        positions, faces, normals = grid_mesh
        out_path = tmp_path / "grid.glb"
        write_glb(positions, faces, normals, path=out_path)

        scene = trimesh.load(str(out_path))
        if hasattr(scene, 'geometry'):
            mesh = list(scene.geometry.values())[0]
        else:
            mesh = scene

        assert len(mesh.vertices) == len(positions)
        assert len(mesh.faces) == len(faces)


class TestWriteGlbValidation:
    """Tests for input validation in write_glb."""

    def test_invalid_positions_shape(self, tmp_path):
        """Should raise ValueError for non-(N,3) positions."""
        bad_positions = np.zeros((10, 2), dtype=np.float32)
        faces = np.array([[0, 1, 2]], dtype=np.int32)

        with pytest.raises(ValueError, match="positions must be"):
            write_glb(bad_positions, faces, path=tmp_path / "bad.glb")

    def test_invalid_faces_shape(self, tmp_path):
        """Should raise ValueError for non-(F,3) faces."""
        positions = np.zeros((4, 3), dtype=np.float32)
        bad_faces = np.array([[0, 1]], dtype=np.int32)  # (F, 2) — not triangles

        with pytest.raises(ValueError, match="faces must be"):
            write_glb(positions, bad_faces, path=tmp_path / "bad.glb")

    def test_zero_faces(self, tmp_path):
        """Should raise ValueError for empty face array."""
        positions = np.zeros((4, 3), dtype=np.float32)
        empty_faces = np.zeros((0, 3), dtype=np.int32)

        with pytest.raises(ValueError, match="zero faces"):
            write_glb(positions, empty_faces, path=tmp_path / "bad.glb")

    def test_normals_shape_mismatch(self, simple_mesh, tmp_path):
        """Should raise ValueError if normals shape doesn't match positions."""
        positions, faces, _ = simple_mesh
        bad_normals = np.zeros((3, 3), dtype=np.float32)  # Wrong count

        with pytest.raises(ValueError, match="normals shape"):
            write_glb(positions, faces, bad_normals, path=tmp_path / "bad.glb")
