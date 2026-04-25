import argparse
import sys
import json
import numpy as np
import trimesh

def check_physics(mesh_path: str) -> dict:
    mesh = trimesh.load(mesh_path, force='mesh')
    
    issues = {
        'info': [],
        'warnings': [],
        'critical': [],
        'problematic_faces': [],
        'problematic_vertices': []
    }
    
    # Dihedral angles for sharp edges
    mesh.fix_normals()
    if not hasattr(mesh, 'face_adjacency_angles'):
        issues['info'].append("Could not compute adjacency angles.")
    else:
        # angles is between adjacent faces
        angles = np.degrees(mesh.face_adjacency_angles)
        sharp_edges_mask = angles > 80.0 # Sharp angle > 80 degrees 
        num_sharp = sharp_edges_mask.sum()
        if num_sharp > 0:
            issues['warnings'].append(f"Found {num_sharp} very sharp edges (dihedral angle > 80deg). Could snag particles.")
            problem_faces = mesh.face_adjacency[sharp_edges_mask].flatten()
            issues['problematic_faces'].extend(problem_faces.tolist())
            
    # Raycast for thin geometry (tunneling risks)
    # Sample points and raycast inwards along negative normal
    try:
        from trimesh.ray.ray_pyembree import RayMeshIntersector
        intersector = RayMeshIntersector(mesh)
    except ImportError:
        intersector = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
        
    num_samples = min(5000, len(mesh.faces))
    sample_face_indices = np.random.choice(len(mesh.faces), num_samples, replace=False)
    
    sample_points = mesh.triangles_center[sample_face_indices]
    sample_normals = mesh.face_normals[sample_face_indices]
    
    # Offset slightly to prevent self-intersection at t=0
    ray_origins = sample_points - sample_normals * 1e-4
    ray_directions = -sample_normals
    
    # Single hit raycast to see if ray immediately hits opposite wall
    locations, index_ray, index_tri = intersector.intersects_location(ray_origins, ray_directions, multiple_hits=False)
    
    if len(locations) > 0:
        distances = np.linalg.norm(locations - ray_origins[index_ray], axis=1)
        
        # Check if opposites are too close
        thin_threshold = 0.02
        if mesh.extents.max() > 50: # scale to cm if needed
            thin_threshold *= 100 
            
        thin_mask = distances < thin_threshold
        num_thin = thin_mask.sum()
        if num_thin > 0:
            issues['warnings'].append(f"Found {num_thin} regions with thin geometry (< {thin_threshold:.4f} units thick). Tunneling risk.")
            problem_faces = sample_face_indices[index_ray[thin_mask]]
            issues['problematic_faces'].extend(problem_faces.tolist())

    issues['problematic_faces'] = list(set(issues['problematic_faces']))
    issues['problematic_vertices'] = list(set(issues['problematic_vertices']))
    
    return issues

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_path", help="Path to GLB/OBJ file")
    parser.add_argument("--json", action="store_true", help="Output details as JSON")
    args = parser.parse_args()
    
    issues = check_physics(args.mesh_path)
    
    if args.json:
        print(json.dumps(issues, indent=2))
    else:
        print(f"--- Physics Checks for {args.mesh_path} ---")
        for level in ['critical', 'warnings', 'info']:
            if issues[level]:
                for msg in issues[level]:
                    print(f"[{level.upper()}] {msg}")
        
        if sum(len(issues[lvl]) for lvl in ['critical', 'warnings', 'info']) == 0:
            print("No physics issues found!")
        
        if issues['critical']:
            sys.exit(1)
