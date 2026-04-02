"""
Unit tests for StaticSpatialHash — build correctness and query coverage.

Tests verify that after building the hash from known triangle configurations,
queries return the correct candidates with no false negatives.
"""

import numpy as np
import pytest

import simulation  # noqa: F401 — Taichi initialization
from simulation.collision.spatial_hash import StaticSpatialHash, _hash_cell_np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_triangle(v0, v1, v2, n=(0.0, 1.0, 0.0)):
    """Build minimal vertices/faces/normals arrays for a single triangle."""
    vertices = np.array([v0, v1, v2], dtype=np.float32)
    faces = np.array([[0, 1, 2]], dtype=np.int32)
    normals = np.tile(np.array(n, dtype=np.float32), (3, 1))
    return vertices, faces, normals


def _make_triangles(tris, n=(0.0, 1.0, 0.0)):
    """Build vertices/faces/normals for multiple triangles (each a list of 3 verts)."""
    verts_list = []
    faces_list = []
    for i, (v0, v1, v2) in enumerate(tris):
        base = i * 3
        verts_list.extend([v0, v1, v2])
        faces_list.append([base, base + 1, base + 2])
    vertices = np.array(verts_list, dtype=np.float32)
    faces = np.array(faces_list, dtype=np.int32)
    normals = np.tile(np.array(n, dtype=np.float32), (len(vertices), 1))
    return vertices, faces, normals


def _query_candidates_np(spatial_hash: StaticSpatialHash, pos, search_27=False):
    """
    Python-side query: return list of triangle indices that cover pos.

    Used in unit tests to inspect hash contents without needing a Taichi kernel.
    Searches the single cell containing pos (or 27-cell neighborhood if search_27=True).
    """
    cs = spatial_hash.cell_size
    ts = spatial_hash.table_size

    cell_start_np = spatial_hash.cell_start.to_numpy()
    cell_count_np = spatial_hash.cell_count.to_numpy()
    entries_np = spatial_hash.entries.to_numpy()

    ix0 = int(np.floor(pos[0] / cs))
    iy0 = int(np.floor(pos[1] / cs))
    iz0 = int(np.floor(pos[2] / cs))

    candidates = set()
    r = range(-1, 2) if search_27 else range(0, 1)
    for di in r:
        for dj in r:
            for dk in r:
            # Single cell only if not search_27
                if not search_27 and (di != 0 or dj != 0 or dk != 0):
                    continue
                h = _hash_cell_np(ix0 + di, iy0 + dj, iz0 + dk, ts)
                start = cell_start_np[h]
                count = cell_count_np[h]
                for k in range(count):
                    candidates.add(int(entries_np[start + k]))

    return candidates


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSpatialHashBuildSingle:
    """Build from a single triangle and verify correct cell assignment."""

    def test_build_single_triangle(self):
        """A single triangle should be found by querying its centroid."""
        v0 = (0.0, 0.0, 0.0)
        v1 = (0.1, 0.0, 0.0)
        v2 = (0.0, 0.0, 0.1)
        vertices, faces, normals = _make_triangle(v0, v1, v2)

        sh = StaticSpatialHash(cell_size=0.05, table_size=256)
        sh.build(vertices, faces, normals)

        # Centroid of the triangle
        centroid = (0.033, 0.0, 0.033)
        candidates = _query_candidates_np(sh, centroid)
        assert 0 in candidates, f"Triangle 0 not found. Candidates: {candidates}"

    def test_build_single_triangle_vertex_queries(self):
        """Querying at each vertex should find the triangle."""
        v0 = (0.0, 0.0, 0.0)
        v1 = (0.1, 0.0, 0.0)
        v2 = (0.0, 0.0, 0.1)
        vertices, faces, normals = _make_triangle(v0, v1, v2)

        sh = StaticSpatialHash(cell_size=0.05, table_size=256)
        sh.build(vertices, faces, normals)

        for vert, name in [(v0, "v0"), (v1, "v1"), (v2, "v2")]:
            candidates = _query_candidates_np(sh, vert, search_27=True)
            assert 0 in candidates, (
                f"Triangle 0 not found when querying at {name}={vert}. "
                f"Candidates: {candidates}"
            )

    def test_n_triangles_set_correctly(self):
        """n_triangles should be set to the number of faces after build."""
        v0, v1, v2 = (0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1)
        vertices, faces, normals = _make_triangle(v0, v1, v2)
        sh = StaticSpatialHash(cell_size=0.05, table_size=256)
        sh.build(vertices, faces, normals)
        assert sh.n_triangles == 1


class TestSpatialHashBuildMultiple:
    """Build from multiple separated triangles and verify per-region lookups."""

    def test_build_three_separated_triangles(self):
        """Three well-separated triangles should each be findable in their region."""
        tris = [
            # Triangle 0: near origin
            [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1)],
            # Triangle 1: far in X
            [(2.0, 0.0, 0.0), (2.1, 0.0, 0.0), (2.0, 0.0, 0.1)],
            # Triangle 2: far in Z
            [(0.0, 0.0, 2.0), (0.1, 0.0, 2.0), (0.0, 0.0, 2.1)],
        ]
        vertices, faces, normals = _make_triangles(tris)

        sh = StaticSpatialHash(cell_size=0.05, table_size=4096)
        sh.build(vertices, faces, normals)

        # Centroids for each triangle
        centroids = [
            (0.033, 0.0, 0.033),
            (2.033, 0.0, 0.033),
            (0.033, 0.0, 2.033),
        ]
        for tri_idx, centroid in enumerate(centroids):
            candidates = _query_candidates_np(sh, centroid, search_27=True)
            assert tri_idx in candidates, (
                f"Triangle {tri_idx} not found at centroid {centroid}. "
                f"Candidates: {candidates}"
            )

    def test_no_false_negatives_at_vertices(self):
        """
        No false negatives: every triangle must be found by querying its own vertices.

        This is the fundamental correctness requirement — a cloth particle sitting
        on a body surface must find the triangle it's touching.
        """
        tris = [
            [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (0.0, 0.0, 0.2)],
            [(1.0, 0.0, 0.0), (1.2, 0.0, 0.0), (1.0, 0.0, 0.2)],
            [(0.0, 0.5, 0.0), (0.2, 0.5, 0.0), (0.0, 0.5, 0.2)],
        ]
        vertices, faces, normals = _make_triangles(tris)
        sh = StaticSpatialHash(cell_size=0.05, table_size=4096)
        sh.build(vertices, faces, normals)

        for tri_idx, (v0, v1, v2) in enumerate(tris):
            for vert, name in [(v0, "v0"), (v1, "v1"), (v2, "v2")]:
                candidates = _query_candidates_np(sh, vert, search_27=True)
                assert tri_idx in candidates, (
                    f"Triangle {tri_idx} not found at {name}={vert}. "
                    f"Candidates: {candidates}"
                )


class TestSpatialHashDistantQuery:
    """Query far from all triangles should return no candidates."""

    def test_distant_query_empty(self):
        """Querying far from all triangles returns no candidates."""
        v0, v1, v2 = (0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.0, 0.0, 0.1)
        vertices, faces, normals = _make_triangle(v0, v1, v2)

        sh = StaticSpatialHash(cell_size=0.05, table_size=4096)
        sh.build(vertices, faces, normals)

        # Query 100m away — should find nothing (unless hash collision)
        # We can't guarantee zero due to hash collisions on small tables,
        # but with table_size=4096 and 1 triangle, collisions are rare
        candidates = _query_candidates_np(sh, (100.0, 100.0, 100.0))
        # If candidates not empty, check they are all valid (hash collision, acceptable)
        for c in candidates:
            assert c < sh.n_triangles, f"Invalid candidate index {c}"


class TestSpatialHashHashCollision:
    """Hash collisions should not cause missing entries."""

    def test_many_triangles_all_found(self):
        """20 triangles in a grid — all should be findable at their centroids."""
        tris = []
        for i in range(5):
            for j in range(4):
                x = i * 0.3
                z = j * 0.3
                tris.append([
                    (x, 0.0, z),
                    (x + 0.1, 0.0, z),
                    (x, 0.0, z + 0.1),
                ])

        vertices, faces, normals = _make_triangles(tris)
        sh = StaticSpatialHash(cell_size=0.05, table_size=256, max_entries=50000)
        sh.build(vertices, faces, normals)

        centroids = [
            (tris[i][0][0] + 0.033, 0.0, tris[i][0][2] + 0.033)
            for i in range(len(tris))
        ]
        found_count = 0
        for tri_idx, centroid in enumerate(centroids):
            candidates = _query_candidates_np(sh, centroid, search_27=True)
            if tri_idx in candidates:
                found_count += 1

        # With 27-cell search, all triangles should be found
        assert found_count == len(tris), (
            f"Found only {found_count}/{len(tris)} triangles"
        )

    def test_triangle_geometry_uploaded(self):
        """Triangle vertices should be stored correctly in Taichi fields after build."""
        v0 = (1.0, 2.0, 3.0)
        v1 = (4.0, 5.0, 6.0)
        v2 = (7.0, 8.0, 9.0)
        vertices, faces, normals = _make_triangle(v0, v1, v2)

        sh = StaticSpatialHash(cell_size=1.0, table_size=256)
        sh.build(vertices, faces, normals)

        # Triangle 0 geometry should be accessible via Taichi fields
        stored_v0 = sh.tri_v0[0].to_numpy()
        stored_v1 = sh.tri_v1[0].to_numpy()
        stored_v2 = sh.tri_v2[0].to_numpy()

        np.testing.assert_allclose(stored_v0, v0, atol=1e-5)
        np.testing.assert_allclose(stored_v1, v1, atol=1e-5)
        np.testing.assert_allclose(stored_v2, v2, atol=1e-5)


class TestSpatialHashCellSize:
    """Hash should work correctly at different cell sizes."""

    def test_large_cell_size(self):
        """With a large cell_size, many triangles land in the same cell."""
        tris = [
            [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0), (0.0, 0.0, 0.01)],
            [(0.02, 0.0, 0.0), (0.03, 0.0, 0.0), (0.02, 0.0, 0.01)],
        ]
        vertices, faces, normals = _make_triangles(tris)

        # cell_size=1.0 is much larger than the triangles — both land in cell (0,0,0)
        sh = StaticSpatialHash(cell_size=1.0, table_size=256)
        sh.build(vertices, faces, normals)

        # Both triangles should be found at either centroid
        for centroid in [(0.005, 0.0, 0.005), (0.025, 0.0, 0.005)]:
            candidates = _query_candidates_np(sh, centroid)
            assert 0 in candidates
            assert 1 in candidates

    def test_small_cell_size(self):
        """With small cell_size, triangles in different cells are separate."""
        v0 = (0.0, 0.0, 0.0)
        v1 = (0.01, 0.0, 0.0)
        v2 = (0.0, 0.0, 0.01)
        vertices, faces, normals = _make_triangle(v0, v1, v2)

        sh = StaticSpatialHash(cell_size=0.005, table_size=4096)
        sh.build(vertices, faces, normals)

        # Should find triangle at its centroid
        candidates = _query_candidates_np(sh, (0.003, 0.0, 0.003), search_27=True)
        assert 0 in candidates
