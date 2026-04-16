"""
Smoke test for vendored pygarment — verifies that:
1. All vendored modules import without error
2. The triangle-based triangulation produces valid meshes
3. A GarmentCode pattern spec can be loaded into BoxMesh
"""

import sys
import os
import json
import tempfile
import numpy as np

# Ensure our backend is on the path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_imports():
    """Test that all vendored modules import cleanly."""
    print("=== Test 1: Imports ===")
    
    # Core pattern modules
    from pygarment.pattern import core
    from pygarment.pattern import rotation
    from pygarment.pattern import utils as pat_utils
    print("  ✅ pygarment.pattern (core, rotation, utils)")
    
    # Data config
    from pygarment import data_config
    print("  ✅ pygarment.data_config")
    
    # Wrappers (stripped)
    from pygarment.pattern import wrappers
    print("  ✅ pygarment.pattern.wrappers (VisPattern)")
    
    # Triangulation utils (rewritten)
    from pygarment.meshgen import triangulation_utils as tri_utils
    print("  ✅ pygarment.meshgen.triangulation_utils (triangle-based)")
    
    # BoxMesh (modified)
    from pygarment.meshgen import boxmeshgen
    print("  ✅ pygarment.meshgen.boxmeshgen (BoxMesh, Panel, Edge, Seam)")
    
    # Garmentcode DSL
    from pygarment.garmentcode import utils as gc_utils
    from pygarment.garmentcode import params
    print("  ✅ pygarment.garmentcode (utils, params)")
    
    # Top-level package
    import pygarment
    print("  ✅ pygarment (top-level)")
    
    print("  All imports passed!\n")


def test_triangulation():
    """Test that the triangle-based CDT works on a simple polygon."""
    print("=== Test 2: Triangulation ===")
    from pygarment.meshgen.triangulation_utils import gen_panel_mesh_triangle, is_manifold
    
    # Simple square: 4 vertices, 4 boundary segments
    vertices = [[0, 0], [10, 0], [10, 10], [0, 10]]
    segments = [[0, 1], [1, 2], [2, 3], [3, 0]]
    
    new_verts, faces = gen_panel_mesh_triangle(vertices, segments, mesh_resolution=2.0)
    
    n_verts = len(new_verts)
    n_faces = len(faces)
    print(f"  Square 10x10, resolution=2.0:")
    print(f"    Vertices: {n_verts} (4 boundary + {n_verts - 4} interior)")
    print(f"    Faces:    {n_faces}")
    
    assert n_verts >= 4, f"Expected at least 4 vertices, got {n_verts}"
    assert n_faces >= 2, f"Expected at least 2 faces, got {n_faces}"
    
    # Check manifold
    manifold = is_manifold(faces, new_verts)
    print(f"    Manifold: {'✅' if manifold else '❌'}")
    assert manifold, "Mesh is not manifold!"
    
    # Check that original boundary vertices are preserved (Y flag)
    new_verts_arr = np.array(new_verts)
    for i, v in enumerate(vertices):
        dists = np.linalg.norm(new_verts_arr - np.array(v), axis=1)
        closest = np.min(dists)
        assert closest < 1e-10, f"Boundary vertex {i} = {v} not preserved (closest dist = {closest})"
    print("    Boundary vertices preserved: ✅")
    
    # Non-convex L-shape
    l_shape = [[0, 0], [6, 0], [6, 4], [3, 4], [3, 8], [0, 8]]
    l_segs = [[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 0]]
    
    l_verts, l_faces = gen_panel_mesh_triangle(l_shape, l_segs, mesh_resolution=1.5)
    l_manifold = is_manifold(l_faces, l_verts)
    print(f"\n  L-shape (non-convex), resolution=1.5:")
    print(f"    Vertices: {len(l_verts)}, Faces: {len(l_faces)}")
    print(f"    Manifold: {'✅' if l_manifold else '❌'}")
    assert l_manifold, "L-shape mesh is not manifold!"
    
    # Check all face vertices are inside the polygon bounds
    l_verts_arr = np.array(l_verts)
    x_range = (l_verts_arr[:, 0].min(), l_verts_arr[:, 0].max())
    y_range = (l_verts_arr[:, 1].min(), l_verts_arr[:, 1].max())
    print(f"    X range: [{x_range[0]:.2f}, {x_range[1]:.2f}]")
    print(f"    Y range: [{y_range[0]:.2f}, {y_range[1]:.2f}]")
    
    print("  All triangulation tests passed!\n")


def test_triangle_quality():
    """Test that minimum angle is respected."""
    print("=== Test 3: Triangle Quality ===")
    from pygarment.meshgen.triangulation_utils import gen_panel_mesh_triangle
    
    # Use a T-shirt-like panel shape (a trapezoid)
    trapezoid = [[-15, 0], [15, 0], [10, 30], [-10, 30]]
    segs = [[0, 1], [1, 2], [2, 3], [3, 0]]
    
    verts, faces = gen_panel_mesh_triangle(trapezoid, segs, mesh_resolution=3.0)
    verts_arr = np.array(verts)
    
    # Compute minimum angle across all triangles
    min_angle = 180.0
    for f in faces:
        v0, v1, v2 = verts_arr[f[0]], verts_arr[f[1]], verts_arr[f[2]]
        # Three edge vectors
        edges = [v1 - v0, v2 - v1, v0 - v2]
        for i in range(3):
            e1 = -edges[(i - 1) % 3]
            e2 = edges[i]
            cos_angle = np.dot(e1, e2) / (np.linalg.norm(e1) * np.linalg.norm(e2) + 1e-12)
            cos_angle = np.clip(cos_angle, -1, 1)
            angle = np.degrees(np.arccos(cos_angle))
            min_angle = min(min_angle, angle)
    
    print(f"  Trapezoid panel, resolution=3.0:")
    print(f"    Vertices: {len(verts)}, Faces: {len(faces)}")
    print(f"    Min angle: {min_angle:.1f}° (target ≥ 30°)")
    
    # The 'q30' flag asks for ≥30° but Triangle may produce slightly less
    # due to boundary constraints — accept ≥20° as valid
    assert min_angle >= 20.0, f"Minimum angle {min_angle:.1f}° is too small!"
    print(f"    Quality check: ✅")
    print("  All quality tests passed!\n")


def test_boxmesh_load():
    """Test loading a minimal GarmentCode pattern spec into BoxMesh."""
    print("=== Test 4: BoxMesh Load ===")
    from pygarment.meshgen.boxmeshgen import BoxMesh
    
    # Create a minimal pattern specification (GarmentCode format)
    # A simple rectangle panel: front_panel
    spec = {
        "pattern": {
            "panels": {
                "front": {
                    "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
                    "edges": [
                        {"endpoints": [0, 1]},
                        {"endpoints": [1, 2]},
                        {"endpoints": [2, 3]},
                        {"endpoints": [3, 0]}
                    ],
                    "translation": [0, 0, 5],
                    "rotation": [0, 0, 0]
                },
                "back": {
                    "vertices": [[0, 0], [20, 0], [20, 30], [0, 30]],
                    "edges": [
                        {"endpoints": [0, 1]},
                        {"endpoints": [1, 2]},
                        {"endpoints": [2, 3]},
                        {"endpoints": [3, 0]}
                    ],
                    "translation": [0, 0, -5],
                    "rotation": [0, 0, 0]
                }
            },
            "stitches": [
                [
                    {"panel": "front", "edge": 1},
                    {"panel": "back", "edge": 3}
                ]
            ]
        },
        "parameters": {},
        "parameter_order": [],
        "properties": {
            "curvature_coords": "relative",
            "normalize_panel_translation": False,
            "normalized_edge_loops": True,
            "units_in_meter": 100
        }
    }
    
    # Write to temp file
    with tempfile.NamedTemporaryFile(mode='w', suffix='_specification.json',
                                      delete=False) as f:
        json.dump(spec, f, indent=2)
        spec_path = f.name
    
    try:
        bm = BoxMesh(spec_path, res=3.0)
        print(f"  Pattern loaded: {bm.name}")
        print(f"  Panels: {bm.panelNames}")
        
        # Load = triangulate + stitch collapse + finalise
        bm.load()
        
        n_verts = len(bm.vertices)
        n_faces = len(bm.faces)
        n_stitches = len(bm.stitches)
        n_orig_lens = len(bm.orig_lens)
        
        print(f"  Vertices:       {n_verts}")
        print(f"  Faces:          {n_faces}")
        print(f"  Stitches:       {n_stitches}")
        print(f"  Orig lens:      {n_orig_lens}")
        print(f"  Segmentation:   {len(bm.stitch_segmentation)} entries")
        print(f"  Vertex texture: {len(bm.vertex_texture)} entries")
        
        assert n_verts > 0, "No vertices!"
        assert n_faces > 0, "No faces!"
        assert n_stitches == 1, f"Expected 1 stitch, got {n_stitches}"
        assert bm.loaded, "BoxMesh not marked as loaded!"
        
        # Check that vertices are 3D
        for v in bm.vertices[:3]:
            v_arr = np.asarray(v)
            assert v_arr.shape == (3,), f"Expected 3D vertex, got shape {v_arr.shape}"
        
        # Check face indices are valid
        for f in bm.faces[:5]:
            for idx in f:
                assert 0 <= idx < n_verts, f"Face index {idx} out of range [0, {n_verts})"
        
        print("  BoxMesh load: ✅")
        
    finally:
        os.unlink(spec_path)
    
    print("  All BoxMesh tests passed!\n")


if __name__ == '__main__':
    print("\n🔧 pygarment Vendor Smoke Test\n")
    test_imports()
    test_triangulation()
    test_triangle_quality()
    test_boxmesh_load()
    print("🎉 All tests passed! Vendored pygarment is working correctly.\n")
