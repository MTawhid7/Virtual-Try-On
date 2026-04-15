"""Unit tests for mesh/panel_builder.py — multi-panel garment mesh assembly."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from simulation.mesh.panel_builder import build_garment_mesh


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _write_pattern(tmp_path: Path, spec: dict) -> Path:
    """Write a pattern dict to a temporary JSON file and return the path."""
    p = tmp_path / "pattern.json"
    p.write_text(json.dumps(spec))
    return p


RECT_PANEL = {
    "id": "panel",
    "vertices_2d": [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]],
    "placement": {"position": [0.0, 1.0, 0.0], "rotation_y_deg": 0},
}

TANK_TOP_SPEC = {
    "name": "TankTop",
    "panels": [
        {
            "id": "front",
            "vertices_2d": [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]],
            "placement": {"position": [-0.2, 0.65, 0.12], "rotation_y_deg": 0},
        },
        {
            "id": "back",
            "vertices_2d": [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]],
            "placement": {"position": [-0.2, 0.65, -0.12], "rotation_y_deg": 180},
        },
    ],
    "stitches": [
        {"panel_a": "front", "edge_a": [0, 3], "panel_b": "back", "edge_b": [0, 3]},
        {"panel_a": "front", "edge_a": [1, 2], "panel_b": "back", "edge_b": [1, 2]},
    ],
    "fabric": "cotton",
}


# ---------------------------------------------------------------------------
# GarmentMesh structure
# ---------------------------------------------------------------------------

class TestGarmentMeshStructure:

    def test_single_panel_positions_shape(self, tmp_path):
        spec = {"name": "Test", "panels": [RECT_PANEL], "stitches": [], "fabric": "cotton"}
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=10)

        assert gm.positions.ndim == 2
        assert gm.positions.shape[1] == 3
        assert gm.positions.dtype == np.float32

    def test_single_panel_faces_shape(self, tmp_path):
        spec = {"name": "Test", "panels": [RECT_PANEL], "stitches": [], "fabric": "cotton"}
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=10)

        assert gm.faces.ndim == 2
        assert gm.faces.shape[1] == 3
        assert gm.faces.dtype == np.int32

    def test_single_panel_edges_shape(self, tmp_path):
        spec = {"name": "Test", "panels": [RECT_PANEL], "stitches": [], "fabric": "cotton"}
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=10)

        assert gm.edges.ndim == 2
        assert gm.edges.shape[1] == 2
        assert gm.edges.dtype == np.int32

    def test_uvs_shape_and_range(self, tmp_path):
        spec = {"name": "Test", "panels": [RECT_PANEL], "stitches": [], "fabric": "cotton"}
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=10)

        assert gm.uvs.shape == (gm.positions.shape[0], 2)
        assert np.all(gm.uvs >= -1e-5)
        assert np.all(gm.uvs <= 1.0 + 1e-5)

    def test_fabric_name_preserved(self, tmp_path):
        spec = {"name": "Test", "panels": [RECT_PANEL], "stitches": [], "fabric": "silk"}
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=10)
        assert gm.fabric == "silk"

    def test_panel_ids_preserved(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)
        assert gm.panel_ids == ["front", "back"]

    def test_face_indices_in_range(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)
        n = gm.positions.shape[0]
        assert np.all(gm.faces >= 0)
        assert np.all(gm.faces < n)

    def test_edge_indices_in_range(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)
        n = gm.positions.shape[0]
        assert np.all(gm.edges >= 0)
        assert np.all(gm.edges < n)


# ---------------------------------------------------------------------------
# Two-panel merging
# ---------------------------------------------------------------------------

_MERGE_TARGET_EDGE = 0.030  # explicit edge length so both build_garment_mesh and
                            # triangulate_panel use the same density


class TestTwoPanelMerge:

    def test_total_vertex_count(self, tmp_path):
        """Merged vertex count = sum of individual panel vertex counts."""
        from simulation.mesh.triangulation import triangulate_panel

        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10, target_edge=_MERGE_TARGET_EDGE)

        # Triangulate each panel independently to get expected counts.
        # Must use the same target_edge so both calls produce the same density.
        verts_2d = TANK_TOP_SPEC["panels"][0]["vertices_2d"]
        single = triangulate_panel(verts_2d, resolution=10, target_edge=_MERGE_TARGET_EDGE)
        expected = single.positions.shape[0] * 2  # two identical-shaped panels

        assert gm.positions.shape[0] == expected

    def test_panel_offsets(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10, target_edge=_MERGE_TARGET_EDGE)

        assert len(gm.panel_offsets) == 2
        assert gm.panel_offsets[0] == 0
        # Second panel starts after first panel's vertices
        from simulation.mesh.triangulation import triangulate_panel
        single = triangulate_panel(
            TANK_TOP_SPEC["panels"][0]["vertices_2d"],
            resolution=10,
            target_edge=_MERGE_TARGET_EDGE,
        )
        assert gm.panel_offsets[1] == single.positions.shape[0]

    def test_faces_reference_correct_panels(self, tmp_path):
        """Faces from the second panel must only reference indices >= panel_offsets[1]."""
        from simulation.mesh.triangulation import triangulate_panel
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10, target_edge=_MERGE_TARGET_EDGE)

        n_front = gm.panel_offsets[1]

        single = triangulate_panel(
            TANK_TOP_SPEC["panels"][0]["vertices_2d"],
            resolution=10,
            target_edge=_MERGE_TARGET_EDGE,
        )
        n_faces_per_panel = single.faces.shape[0]

        front_faces = gm.faces[:n_faces_per_panel]
        back_faces  = gm.faces[n_faces_per_panel:]

        # Front faces must only reference front vertices
        assert np.all(front_faces < n_front), "Front faces reference back vertices"
        # Back faces must only reference back vertices
        assert np.all(back_faces >= n_front), "Back faces reference front vertices"


# ---------------------------------------------------------------------------
# Placement transform
# ---------------------------------------------------------------------------

class TestPlacement:

    def test_translation_applied(self, tmp_path):
        """Panel center should be near placement.position after transform."""
        spec = {
            "name": "Test",
            "panels": [{
                "id": "p",
                "vertices_2d": [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]],
                "placement": {"position": [1.0, 2.0, 3.0], "rotation_y_deg": 0},
            }],
            "stitches": [],
            "fabric": "cotton",
        }
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=8)

        # Y should be near 2.0 (translated) for all vertices
        y_vals = gm.positions[:, 1]
        assert np.allclose(y_vals, 2.0, atol=1e-4), \
            f"Y not translated: range [{y_vals.min():.3f}, {y_vals.max():.3f}]"

    def test_rotation_180_flips_z(self, tmp_path):
        """rotation_y_deg=180 should flip the panel's X axis (back panel faces inward)."""
        spec_fwd = {
            "name": "Test",
            "panels": [{
                "id": "p",
                "vertices_2d": [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]],
                "placement": {"position": [0.0, 0.0, 0.0], "rotation_y_deg": 0},
            }],
            "stitches": [], "fabric": "cotton",
        }
        spec_back = {
            "name": "Test",
            "panels": [{
                "id": "p",
                "vertices_2d": [[0.0, 0.0], [0.4, 0.0], [0.4, 0.7], [0.0, 0.7]],
                "placement": {"position": [0.0, 0.0, 0.0], "rotation_y_deg": 180},
            }],
            "stitches": [], "fabric": "cotton",
        }
        fwd_dir  = tmp_path / "fwd";  fwd_dir.mkdir()
        back_dir = tmp_path / "back"; back_dir.mkdir()

        gm_fwd  = build_garment_mesh(_write_pattern(fwd_dir,  spec_fwd),  resolution=5)
        gm_back = build_garment_mesh(_write_pattern(back_dir, spec_back), resolution=5)

        x_fwd  = gm_fwd.positions[:, 0]
        x_back = gm_back.positions[:, 0]

        # Forward: X in [0, 0.4]; rotated 180: X in [-0.4, 0]
        assert x_fwd.min()  >= -0.01,  "Forward panel X should be >= 0"
        assert x_back.max() <=  0.01,  "Back panel X should be <= 0 after 180° rotation"

    def test_two_panels_separated_in_z(self, tmp_path):
        """Front at Z=+0.12, back at Z=-0.12 — no Z overlap after placement."""
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=8)

        n_front = gm.panel_offsets[1]
        z_front = gm.positions[:n_front, 2]
        z_back  = gm.positions[n_front:, 2]

        assert z_front.mean() > 0.0,  "Front panel should have positive Z"
        assert z_back.mean()  < 0.0,  "Back panel should have negative Z"


# ---------------------------------------------------------------------------
# Stitch pairs
# ---------------------------------------------------------------------------

class TestStitchPairs:

    def test_stitch_pairs_shape(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)

        assert gm.stitch_pairs.ndim == 2
        assert gm.stitch_pairs.shape[1] == 2
        assert gm.stitch_pairs.dtype == np.int32

    def test_stitch_pairs_nonempty(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)
        assert gm.stitch_pairs.shape[0] > 0, "Expected at least one stitch pair"

    def test_stitch_pairs_reference_valid_vertices(self, tmp_path):
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)
        n = gm.positions.shape[0]
        assert np.all(gm.stitch_pairs >= 0)
        assert np.all(gm.stitch_pairs < n)

    def test_stitch_pairs_cross_panels(self, tmp_path):
        """Each stitch pair must connect a front vertex to a back vertex."""
        path = _write_pattern(tmp_path, TANK_TOP_SPEC)
        gm = build_garment_mesh(path, resolution=10)
        n_front = gm.panel_offsets[1]

        for i, j in gm.stitch_pairs:
            on_front_i = i < n_front
            on_front_j = j < n_front
            assert on_front_i != on_front_j, \
                f"Stitch pair ({i},{j}) doesn't cross panels (n_front={n_front})"

    def test_no_stitches_when_not_defined(self, tmp_path):
        spec = {"name": "Test", "panels": [RECT_PANEL], "stitches": [], "fabric": "cotton"}
        path = _write_pattern(tmp_path, spec)
        gm = build_garment_mesh(path, resolution=8)
        assert gm.stitch_pairs.shape[0] == 0


# ---------------------------------------------------------------------------
# File handling
# ---------------------------------------------------------------------------

class TestFileHandling:

    def test_missing_file_raises(self):
        with pytest.raises(FileNotFoundError):
            build_garment_mesh("/nonexistent/pattern.json")

    def test_tank_top_json_loads(self):
        """The committed tank_top.json pattern loads without error."""
        data_path = Path(__file__).parent.parent.parent.parent / "data/patterns/tank_top.json"
        gm = build_garment_mesh(data_path, resolution=10)
        assert gm.positions.shape[0] > 0
        assert gm.fabric == "cotton"
        assert "front" in gm.panel_ids
        assert "back" in gm.panel_ids
