"""
Pattern validation script to verify garment quality before simulation.
Checks particle density, edge length distribution, and stitch connectivity.
"""

import sys
import json
from pathlib import Path

# Add project root to sys.path to allow importing 'simulation' when run as a script
sys.path.append(str(Path(__file__).resolve().parent.parent))

import numpy as np

from simulation.mesh.panel_builder import build_garment_mesh

def validate_pattern(pattern_path: str, resolution: int = 20):
    print(f"--- Validating Pattern: {pattern_path} ---")
    
    path = Path(pattern_path)
    if not path.exists():
        print(f"Error: File {pattern_path} not found.")
        return

    try:
        garment = build_garment_mesh(pattern_path, resolution=resolution)
    except Exception as e:
        print(f"Error during mesh building: {e}")
        return

    n_particles = garment.positions.shape[0]
    n_faces = garment.faces.shape[0]
    n_edges = garment.edges.shape[0]
    n_stitches = garment.stitch_pairs.shape[0]

    print(f"  Summary:")
    print(f"    Particles: {n_particles}")
    print(f"    Triangles: {n_faces}")
    print(f"    Edges:     {n_edges}")
    print(f"    Stitches:  {n_stitches}")

    # 1. Edge Length Distribution (Phase 0 Check)
    v0 = garment.positions[garment.edges[:, 0]]
    v1 = garment.positions[garment.edges[:, 1]]
    lengths = np.linalg.norm(v1 - v0, axis=1)
    
    min_len = float(np.min(lengths))
    max_len = float(np.max(lengths))
    mean_len = float(np.mean(lengths))
    ratio = max_len / max(min_len, 1e-8)

    print(f"\n  Mesh Geometry (Phase 0):")
    print(f"    Min edge:  {min_len*1000:.2f}mm")
    print(f"    Max edge:  {max_len*1000:.2f}mm")
    print(f"    Mean edge: {mean_len*1000:.2f}mm")
    print(f"    Ratio:     {ratio:.2f}x")

    if min_len < 0.005:
        print(f"    [WARNING] Edge length below 5mm may cause solver instability.")
    elif ratio > 5.0:
        print(f"    [WARNING] High edge length ratio may cause 'rubber' or 'webbing' artifacts.")
    else:
        print(f"    [PASS] Edge distribution is healthy.")

    # 2. Stitch Density (Phase 1 Check)
    print(f"\n  Stitch Resolution (Phase 1):")
    with open(path) as f:
        spec = json.load(f)
    
    stitch_specs = spec.get("stitches", [])
    if not stitch_specs:
        print(f"    [INFO] No stitches defined in pattern.")
    else:
        # Match stitches to specs to check density
        # In build_garment_mesh, stitch_pairs are concatenated in order of stitch_specs
        offset = 0
        for i, sdef in enumerate(stitch_specs):
            comment = sdef.get("comment", f"Stitch {i}")
            # We don't easily know exactly how many pairs each spec produced without 
            # re-running the logic, but we can verify total count vs specs.
            # A healthy dense stitch should have many points.
            pass

        pairs_per_stitch = n_stitches / len(stitch_specs)
        print(f"    Avg pairs per stitch: {pairs_per_stitch:.1f}")
        if pairs_per_stitch < 5:
            print(f"    [WARNING] Low stitch density detected. Seams may have visible gaps.")
        else:
            print(f"    [PASS] Stitch density appears sufficient.")

    # 3. Placement Check
    bbox_min = np.min(garment.positions, axis=0)
    bbox_max = np.max(garment.positions, axis=0)
    print(f"\n  Placement Check:")
    print(f"    BBox Min: {bbox_min}")
    print(f"    BBox Max: {bbox_max}")
    
    # Mannequin is roughly at Y=0..1.75, X=-0.5..0.5, Z=-0.2..0.4
    if bbox_min[1] < 0.0 or bbox_max[1] > 2.0:
        print(f"    [WARNING] Garment extends outside normal body height range.")

    print("\n--- Validation Complete ---")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python scripts/validate_pattern.py <pattern_path>")
    else:
        validate_pattern(sys.argv[1])
