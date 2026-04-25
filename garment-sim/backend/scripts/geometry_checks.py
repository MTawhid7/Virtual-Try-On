import argparse
import sys
import json
import numpy as np
import trimesh

def check_geometry(mesh_path: str) -> dict:
    mesh = trimesh.load(mesh_path, force='mesh')
    
    issues = {
        'info': [],
        'warnings': [],
        'critical': [],
        'problematic_faces': [],
        'problematic_vertices': []
    }
    
    # Scale and units
    extents = mesh.extents
    max_extent = extents.max()
    if max_extent > 50:
        issues['warnings'].append(f"Mesh seems to be in centimeters (max extent: {max_extent:.2f}). Simulation expects meters.")
    elif max_extent < 0.5:
        issues['warnings'].append(f"Mesh seems unusually small (max extent: {max_extent:.2f}m).")
        
    aspect_ratio = extents[1] / max(extents[0], extents[2])
    if aspect_ratio < 1.0:
        issues['warnings'].append(f"Bounding box aspect ratio is unusual for a standing body (height is not the largest dimension).")
        
    # Mesh Integrity
    if not mesh.is_watertight:
        issues['info'].append("Mesh is not watertight (has holes or open boundaries). Tolerable for typical garment drops.")
        
    unref_verts = len(mesh.vertices) - len(np.unique(mesh.faces.flatten()))
    if unref_verts > 0:
        issues['warnings'].append(f"Found {unref_verts} unreferenced vertices.")
        
    # Triangle sizes
    areas = mesh.area_faces
    degenerate_mask = areas < 1e-10
    num_degenerate = degenerate_mask.sum()
    if num_degenerate > 0:
        issues['critical'].append(f"Found {num_degenerate} degenerate triangles (area < 1e-10).")
        issues['problematic_faces'].extend(np.where(degenerate_mask)[0].tolist())
        
    valid_areas = areas[~degenerate_mask]
    if len(valid_areas) > 0:
        mean_area = valid_areas.mean()
        
        tiny_triangles = (areas < mean_area * 0.05) & ~degenerate_mask
        huge_triangles = (areas > mean_area * 20)
        
        if tiny_triangles.sum() > 0:
             issues['info'].append(f"Found {tiny_triangles.sum()} unusually small triangles (< 5% of mean area).")
             issues['problematic_faces'].extend(np.where(tiny_triangles)[0].tolist())
             
        if huge_triangles.sum() > 0:
             issues['warnings'].append(f"Found {huge_triangles.sum()} unusually large triangles (> 20x mean area).")
             issues['problematic_faces'].extend(np.where(huge_triangles)[0].tolist())
             
    # Output structure
    issues['problematic_faces'] = list(set(issues['problematic_faces']))
    issues['problematic_vertices'] = list(set(issues['problematic_vertices']))
    
    return issues

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_path", help="Path to GLB/OBJ file")
    parser.add_argument("--json", action="store_true", help="Output details as JSON")
    args = parser.parse_args()
    
    issues = check_geometry(args.mesh_path)
    
    if args.json:
        print(json.dumps(issues, indent=2))
    else:
        print(f"--- Geometry Checks for {args.mesh_path} ---")
        for level in ['critical', 'warnings', 'info']:
            if issues[level]:
                for msg in issues[level]:
                    print(f"[{level.upper()}] {msg}")
        
        if sum(len(issues[lvl]) for lvl in ['critical', 'warnings', 'info']) == 0:
            print("No geometry issues found!")
        
        if issues['critical']:
            sys.exit(1)
