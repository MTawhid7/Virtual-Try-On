"""
Phase 2 verification test — validates the gc_mesh_adapter pipeline.

Tests:
1. Basic adapter: 2-panel (front+back) pattern → GarmentMesh
2. Stitch integrity: all stitch_pair indices are valid, panels match
3. Mesh quality: edges consistent with faces, no degenerate triangles
4. Coordinate scaling: cm→m conversion correct
5. Multi-stitch: 4-panel pattern with 4 stitches
6. Integration: GarmentMesh passes garment-sim validation checks
"""

import sys
import os
import json
import tempfile
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _make_spec(panels: dict, stitches: list, name: str = "test") -> str:
    """Create a temp GarmentCode spec JSON and return its path."""
    spec = {
        "pattern": {
            "panels": panels,
            "stitches": stitches,
        },
        "parameters": {},
        "parameter_order": [],
        "properties": {
            "curvature_coords": "relative",
            "normalize_panel_translation": False,
            "normalized_edge_loops": True,
            "units_in_meter": 100,
        },
    }
    f = tempfile.NamedTemporaryFile(
        mode="w", suffix="_specification.json", delete=False, prefix=name
    )
    json.dump(spec, f, indent=2)
    f.close()
    return f.name


def test_basic_adapter():
    """Test: 2-panel front+back → GarmentMesh with 1 stitch."""
    print("=== Test 1: Basic Adapter ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    panels = {
        "front": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, 5],
            "rotation": [0, 0, 0],
        },
        "back": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, -5],
            "rotation": [0, 0, 0],
        },
    }
    stitches = [
        [{"panel": "front", "edge": 1}, {"panel": "back", "edge": 3}],
    ]

    path = _make_spec(panels, stitches, "basic")
    try:
        gm = build_garment_mesh_gc(path, mesh_resolution=3.0)

        print(f"  Panels: {gm.panel_ids}")
        print(f"  Positions: {gm.positions.shape}")
        print(f"  Faces:     {gm.faces.shape}")
        print(f"  Edges:     {gm.edges.shape}")
        print(f"  UVs:       {gm.uvs.shape}")
        print(f"  Stitch pairs: {gm.stitch_pairs.shape}")
        print(f"  Panel offsets: {gm.panel_offsets}")
        print(f"  Fabric: {gm.fabric}")

        # Basic shape checks
        assert gm.positions.ndim == 2 and gm.positions.shape[1] == 3
        assert gm.faces.ndim == 2 and gm.faces.shape[1] == 3
        assert gm.edges.ndim == 2 and gm.edges.shape[1] == 2
        assert gm.uvs.ndim == 2 and gm.uvs.shape[1] == 2
        assert gm.stitch_pairs.ndim == 2 and gm.stitch_pairs.shape[1] == 2
        assert len(gm.panel_offsets) == 2
        assert len(gm.panel_ids) == 2
        assert gm.fabric == "cotton"

        # Should have at least some stitch pairs
        assert gm.stitch_pairs.shape[0] > 0, "No stitch pairs generated!"
        print(f"  ✅ Basic adapter passed ({gm.stitch_pairs.shape[0]} stitch pairs)")
    finally:
        os.unlink(path)
    print()


def test_coordinate_scaling():
    """Test: cm→m conversion is correct (0.01 factor)."""
    print("=== Test 2: Coordinate Scaling ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    # 20cm × 30cm panel at translation [0, 0, 5] (cm)
    panels = {
        "panel": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, 5],
            "rotation": [0, 0, 0],
        },
    }
    stitches = []

    path = _make_spec(panels, stitches, "scale")
    try:
        gm = build_garment_mesh_gc(path, mesh_resolution=5.0)

        # Panel dimension: 20cm = 0.2m, 30cm = 0.3m
        pos = gm.positions
        x_range = pos[:, 0].max() - pos[:, 0].min()
        y_range = pos[:, 1].max() - pos[:, 1].min()
        z_range = pos[:, 2].max() - pos[:, 2].min()

        # GarmentCode places vertices in XY plane for rotation=[0,0,0]
        # The 2D panel is in its local XY. After rot_trans, expected:
        #   X ∈ [0, 20]*0.01 = [0, 0.2]
        #   Y ∈ [0, 30]*0.01 = [0, 0.3]
        #   Z = translation_z * 0.01 = 0.05
        print(f"  X range: {x_range:.4f} m (expect ~0.2)")
        print(f"  Y range: {y_range:.4f} m (expect ~0.3)")
        print(f"  Z range: {z_range:.4f} m (expect ~0.0)")

        # Allow tolerance for Steiner points
        assert abs(x_range - 0.2) < 0.01, f"X range {x_range} not ~0.2"
        assert abs(y_range - 0.3) < 0.01, f"Y range {y_range} not ~0.3"
        assert z_range < 0.01, f"Z range {z_range} should be ~0 (flat panel)"

        # Translation check: Z should be ~0.05m
        z_center = pos[:, 2].mean()
        assert abs(z_center - 0.05) < 0.01, f"Z center {z_center} not ~0.05"

        print(f"  ✅ Coordinate scaling correct")
    finally:
        os.unlink(path)
    print()


def test_stitch_integrity():
    """Test: all stitch indices valid, pairs connect different panels."""
    print("=== Test 3: Stitch Integrity ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    panels = {
        "front": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, 5],
            "rotation": [0, 0, 0],
        },
        "back": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, -5],
            "rotation": [0, 0, 0],
        },
    }
    # Stitch front edge 1 (right side) to back edge 3 (left side)
    stitches = [
        [{"panel": "front", "edge": 1}, {"panel": "back", "edge": 3}],
    ]

    path = _make_spec(panels, stitches, "stitch")
    try:
        gm = build_garment_mesh_gc(path, mesh_resolution=3.0)

        n_verts = gm.positions.shape[0]
        n_stitch = gm.stitch_pairs.shape[0]

        # All stitch indices in valid range
        assert gm.stitch_pairs.min() >= 0, "Negative stitch index!"
        assert gm.stitch_pairs.max() < n_verts, (
            f"Stitch index {gm.stitch_pairs.max()} >= n_verts {n_verts}"
        )

        # Stitch pairs should connect different vertices
        for i in range(n_stitch):
            a, b = gm.stitch_pairs[i]
            assert a != b, f"Self-stitch at pair {i}: {a} == {b}"

        # Stitch pairs should connect vertices from panel "front" and "back"
        front_range = range(
            gm.panel_offsets[0],
            gm.panel_offsets[1] if len(gm.panel_offsets) > 1 else n_verts,
        )
        back_range = range(
            gm.panel_offsets[1] if len(gm.panel_offsets) > 1 else 0,
            n_verts,
        )

        cross_panel_count = 0
        for i in range(n_stitch):
            a, b = int(gm.stitch_pairs[i, 0]), int(gm.stitch_pairs[i, 1])
            if (a in front_range and b in back_range) or (a in back_range and b in front_range):
                cross_panel_count += 1

        print(f"  Total stitch pairs: {n_stitch}")
        print(f"  Cross-panel pairs:  {cross_panel_count}")
        assert cross_panel_count == n_stitch, (
            f"Expected all {n_stitch} stitches to be cross-panel, got {cross_panel_count}"
        )

        # Seam IDs should be present
        assert gm.stitch_seam_ids is not None
        assert len(gm.stitch_seam_ids) == n_stitch

        print(f"  ✅ Stitch integrity passed")
    finally:
        os.unlink(path)
    print()


def test_mesh_quality():
    """Test: edges are consistent with faces, no degenerates."""
    print("=== Test 4: Mesh Quality ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    panels = {
        "front": {
            "vertices": [[-15, 0], [15, 0], [10, 30], [-10, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, 5],
            "rotation": [0, 0, 0],
        },
    }

    path = _make_spec(panels, [], "quality")
    try:
        gm = build_garment_mesh_gc(path, mesh_resolution=3.0)

        # Every face edge should appear in the edges array
        face_edges = set()
        for f in gm.faces:
            for k in range(3):
                a, b = int(f[k]), int(f[(k + 1) % 3])
                face_edges.add((min(a, b), max(a, b)))

        edge_set = set(map(tuple, gm.edges.tolist()))
        missing = face_edges - edge_set
        assert len(missing) == 0, f"{len(missing)} face edges not in edge array"

        # No degenerate triangles (zero area)
        v = gm.positions
        for i, f in enumerate(gm.faces):
            v0, v1, v2 = v[f[0]], v[f[1]], v[f[2]]
            area = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0))
            assert area > 1e-10, f"Degenerate triangle {i}: area={area}"

        # Edge lengths in reasonable range (in metres)
        edge_lens = np.linalg.norm(
            gm.positions[gm.edges[:, 1]] - gm.positions[gm.edges[:, 0]], axis=1
        )
        print(f"  Edge lengths: min={edge_lens.min()*100:.2f}cm, "
              f"max={edge_lens.max()*100:.2f}cm, "
              f"mean={edge_lens.mean()*100:.2f}cm")

        # Min angle check
        min_angle = 180.0
        for f in gm.faces:
            v0, v1, v2 = v[f[0]], v[f[1]], v[f[2]]
            edges_tri = [v1 - v0, v2 - v1, v0 - v2]
            for j in range(3):
                e1 = -edges_tri[(j - 1) % 3]
                e2 = edges_tri[j]
                cos = np.dot(e1, e2) / (np.linalg.norm(e1) * np.linalg.norm(e2) + 1e-12)
                cos = np.clip(cos, -1, 1)
                angle = np.degrees(np.arccos(cos))
                min_angle = min(min_angle, angle)

        print(f"  Min triangle angle: {min_angle:.1f}° (target ≥ 20°)")
        assert min_angle >= 20.0, f"Min angle {min_angle}° too small!"

        print(f"  ✅ Mesh quality passed")
    finally:
        os.unlink(path)
    print()


def test_multi_stitch():
    """Test: 4-panel pattern with independent stitches."""
    print("=== Test 5: Multi-Stitch (4 panels) ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    # 4-panel layout: A, B, C, D arranged in a square
    # A top-right stitched to B top-left
    # C bottom-right stitched to D bottom-left
    panels = {
        "A": {
            "vertices": [[0, 0], [15, 0], [15, 20], [0, 20]],
            "edges": [
                {"endpoints": [0, 1]},  # 0: bottom
                {"endpoints": [1, 2]},  # 1: right
                {"endpoints": [2, 3]},  # 2: top
                {"endpoints": [3, 0]},  # 3: left
            ],
            "translation": [0, 0, 5],
            "rotation": [0, 0, 0],
        },
        "B": {
            "vertices": [[0, 0], [15, 0], [15, 20], [0, 20]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 0, -5],
            "rotation": [0, 0, 0],
        },
        "C": {
            "vertices": [[0, 0], [15, 0], [15, 20], [0, 20]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [20, 0, 5],
            "rotation": [0, 0, 0],
        },
        "D": {
            "vertices": [[0, 0], [15, 0], [15, 20], [0, 20]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [20, 0, -5],
            "rotation": [0, 0, 0],
        },
    }
    stitches = [
        [{"panel": "A", "edge": 1}, {"panel": "B", "edge": 3}],  # A right ↔ B left
        [{"panel": "C", "edge": 1}, {"panel": "D", "edge": 3}],  # C right ↔ D left
    ]

    path = _make_spec(panels, stitches, "multi")
    try:
        gm = build_garment_mesh_gc(path, mesh_resolution=3.0)

        n_stitch = gm.stitch_pairs.shape[0]
        print(f"  Panels: {gm.panel_ids}")
        print(f"  Total vertices: {gm.positions.shape[0]}")
        print(f"  Total faces:    {gm.faces.shape[0]}")
        print(f"  Total stitches: {n_stitch}")

        # Should have stitches for 2 seams
        assert n_stitch > 4, f"Expected many stitch pairs, got {n_stitch}"

        # Seam labels should be present
        assert gm.stitch_seam_ids is not None
        unique_seams = set(gm.stitch_seam_ids)
        print(f"  Unique seam labels: {unique_seams}")
        assert len(unique_seams) == 2, f"Expected 2 unique seam labels, got {len(unique_seams)}"

        # All stitch indices valid
        assert gm.stitch_pairs.min() >= 0
        assert gm.stitch_pairs.max() < gm.positions.shape[0]
        assert len(gm.panel_offsets) == 4, f"Expected 4 panel offsets, got {len(gm.panel_offsets)}"

        print(f"  ✅ Multi-stitch passed")
    finally:
        os.unlink(path)
    print()


def test_garment_sim_integration():
    """Test: GarmentMesh works with garment-sim's constraint builder."""
    print("=== Test 6: garment-sim Integration ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc
    from simulation.constraints import build_constraints
    from simulation.mesh.grid import compute_area_weighted_inv_masses
    from simulation.materials import FABRIC_PRESETS

    panels = {
        "front": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 50, 5],
            "rotation": [0, 0, 0],
        },
        "back": {
            "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
            "edges": [
                {"endpoints": [0, 1]},
                {"endpoints": [1, 2]},
                {"endpoints": [2, 3]},
                {"endpoints": [3, 0]},
            ],
            "translation": [0, 50, -5],
            "rotation": [0, 0, 0],
        },
    }
    stitches = [
        [{"panel": "front", "edge": 1}, {"panel": "back", "edge": 3}],
    ]

    path = _make_spec(panels, stitches, "integration")
    try:
        gm = build_garment_mesh_gc(path, mesh_resolution=3.0)

        # --- build_constraints should work ---
        fabric = FABRIC_PRESETS["cotton"]
        inv_masses = compute_area_weighted_inv_masses(
            gm.positions, gm.faces, fabric.density
        )

        constraints = build_constraints(
            positions=gm.positions,
            edges=gm.edges,
            faces=gm.faces,
            stitch_pairs=gm.stitch_pairs if gm.stitch_pairs.shape[0] > 0 else None,
            max_stitches=gm.stitch_pairs.shape[0] + 10,
        )

        print(f"  Distance constraints: {constraints.distance.n_edges if constraints.distance else 0}")
        print(f"  Bending constraints:  {constraints.bending.n_hinges if constraints.bending else 0}")
        print(f"  Stitch constraints:   {constraints.stitch.n_stitches if constraints.stitch else 0}")

        assert constraints.distance is not None, "No distance constraints!"
        assert constraints.distance.n_edges > 0, "Zero distance constraints!"
        assert constraints.bending is not None, "No bending constraints!"
        assert constraints.bending.n_hinges > 0, "Zero bending constraints!"
        assert constraints.stitch is not None, "No stitch constraints!"
        assert constraints.stitch.n_stitches > 0, "Zero stitch constraints!"

        # --- inv_masses should be valid ---
        assert inv_masses.shape[0] == gm.positions.shape[0]
        assert np.all(inv_masses > 0), "Some inv_masses are zero/negative"
        assert not np.any(np.isnan(inv_masses)), "NaN in inv_masses"

        # --- No NaN in positions ---
        assert not np.any(np.isnan(gm.positions)), "NaN in positions!"

        print(f"  ✅ garment-sim integration passed")
    finally:
        os.unlink(path)
    print()


if __name__ == "__main__":
    print("\n🔧 Phase 2: gc_mesh_adapter Verification\n")
    test_basic_adapter()
    test_coordinate_scaling()
    test_stitch_integrity()
    test_mesh_quality()
    test_multi_stitch()
    test_garment_sim_integration()
    print("🎉 All Phase 2 tests passed!\n")
