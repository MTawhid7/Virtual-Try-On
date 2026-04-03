import argparse
import sys
import json
import numpy as np
import trimesh
import pyglet

def visualize_mesh(mesh_path: str, issues_path: str = None, show_normals: bool = False):
    mesh = trimesh.load(mesh_path, force='mesh')
    
    # Ensure uniform base color, convert texture visuals if present
    try:
        mesh.visual = mesh.visual.to_color()
        if hasattr(mesh.visual, 'face_colors'):
            mesh.visual.face_colors = [200, 200, 200, 255]
    except Exception:
        # If the visual data is completely corrupted (IndexError on vertex colors)
        mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh)
        mesh.visual.face_colors = [200, 200, 200, 255]
        
    scene = trimesh.Scene([mesh])
    
    if issues_path:
        with open(issues_path, 'r') as f:
            issues = json.load(f)
            
        prob_faces = issues.get('problematic_faces', [])
        prob_verts = issues.get('problematic_vertices', [])
        
        if prob_faces:
            print(f"Highlighting {len(prob_faces)} problematic faces in red...")
            if hasattr(mesh.visual, 'face_colors'):
                colors = mesh.visual.face_colors.copy()
                colors[prob_faces] = [255, 0, 0, 255]
                mesh.visual.face_colors = colors

        if prob_verts:
            print(f"Highlighting {len(prob_verts)} problematic vertices with points...")
            points = mesh.vertices[prob_verts]
            pc = trimesh.points.PointCloud(points, colors=[255, 100, 0, 255])
            scene.add_geometry(pc)

    if show_normals:
        print("Adding face normals to scene...")
        origins = mesh.triangles_center
        ends = origins + mesh.face_normals * (mesh.extents.max() * 0.05)
        
        lines = np.hstack((origins, ends)).reshape(-1, 2, 3)
        path = trimesh.load_path(lines)
        path.colors = [[0, 0, 255, 255]] * len(path.entities)
        scene.add_geometry(path)
        
    print("Opening visualizer (close window to exit)...")
    scene.show(line_settings={'point_size': 5})

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_path", help="Path to GLB/OBJ file")
    parser.add_argument("--issues", help="Path to JSON file with issues (from geometry/physics checks)")
    parser.add_argument("--show-normals", action="store_true", help="Visualize face normals")
    args = parser.parse_args()
    
    visualize_mesh(args.mesh_path, args.issues, args.show_normals)
