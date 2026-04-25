"""
Visualization & verification of the Blender-exported tank top mesh.

Loads tank_top_mesh.obj, cleans debris, identifies panels and boundary
edges, and provides an interactive Taichi GGUI viewer.

Usage:
    python -m scripts.verify_blender_mesh

Controls:
    1 = body mesh  |  2 = front panel  |  3 = back panel
    4 = boundary edges  |  5 = stitch lines (side seam pairs)
    SPACE = wireframe  |  RMB + drag = rotate
"""

import os
import sys
import numpy as np
import trimesh
import taichi as ti
from collections import Counter

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)


def load_and_clean_mesh(obj_path: str):
    """Load OBJ, split into components, keep only the 2 large panels."""
    mesh = trimesh.load(obj_path, process=False, force="mesh")
    cc = mesh.split(only_watertight=False)

    # Keep only components with > 10 vertices (real panels)
    panels = sorted(
        [c for c in cc if len(c.vertices) > 10],
        key=lambda c: c.vertices[:, 2].mean(),  # sort by Z (back first, then front)
    )

    n_debris = len(cc) - len(panels)
    return panels, n_debris


def find_boundary_edges(mesh):
    """Find edges that belong to only 1 face (open boundaries)."""
    face_edge_list = []
    for face in mesh.faces:
        for i in range(3):
            e = tuple(sorted([face[i], face[(i + 1) % 3]]))
            face_edge_list.append(e)
    edge_counts = Counter(face_edge_list)
    return [e for e, c in edge_counts.items() if c == 1]


def classify_boundary_edges(mesh, boundary_edges):
    """Classify boundary edges into: left side, right side, top (armhole+neck+shoulder), bottom (hem)."""
    v = mesh.vertices
    classified = {"left": [], "right": [], "top": [], "bottom": []}

    x_min, x_max = v[:, 0].min(), v[:, 0].max()
    y_min, y_max = v[:, 1].min(), v[:, 1].max()
    x_mid = (x_min + x_max) / 2
    y_thresh_bottom = y_min + 0.02
    y_thresh_top = y_max - 0.02

    for a, b in boundary_edges:
        mid_x = (v[a, 0] + v[b, 0]) / 2
        mid_y = (v[a, 1] + v[b, 1]) / 2

        if mid_y < y_thresh_bottom:
            classified["bottom"].append((a, b))
        elif mid_y > y_thresh_top:
            classified["top"].append((a, b))
        elif mid_x < x_mid:
            classified["left"].append((a, b))
        else:
            classified["right"].append((a, b))

    return classified


def find_stitch_pairs(panel_a, panel_b, side: str):
    """Find matching boundary vertices between two panels on a given side."""
    va = panel_a.vertices
    vb = panel_b.vertices

    boundary_a = find_boundary_edges(panel_a)
    boundary_b = find_boundary_edges(panel_b)

    classified_a = classify_boundary_edges(panel_a, boundary_a)
    classified_b = classify_boundary_edges(panel_b, boundary_b)

    # Get unique boundary vertex indices for this side
    def get_side_verts(mesh, edges):
        verts = set()
        for a, b in edges:
            verts.add(a)
            verts.add(b)
        return sorted(verts, key=lambda i: mesh.vertices[i, 1])  # sort by Y

    verts_a = get_side_verts(panel_a, classified_a[side])
    verts_b = get_side_verts(panel_b, classified_b[side])

    # Match by closest Y position
    pairs = []
    n = min(len(verts_a), len(verts_b))
    if n == 0:
        return pairs

    # Subsample to match lengths
    if len(verts_a) != len(verts_b):
        idx_a = np.round(np.linspace(0, len(verts_a) - 1, n)).astype(int)
        idx_b = np.round(np.linspace(0, len(verts_b) - 1, n)).astype(int)
        verts_a = [verts_a[i] for i in idx_a]
        verts_b = [verts_b[i] for i in idx_b]

    for ia, ib in zip(verts_a, verts_b):
        pairs.append((ia, ib))

    return pairs


def main():
    obj_path = os.path.join(backend_dir, "data/patterns/tank_top_mesh.obj")
    body_path = os.path.join(backend_dir, "data/bodies/mannequin_physics.glb")

    print("═" * 60)
    print("  Blender Mesh Verification")
    print("═" * 60)

    # --- Load and clean ---
    panels, n_debris = load_and_clean_mesh(obj_path)
    print(f"\n  Loaded: {len(panels)} panels, discarded {n_debris} debris fragments")

    if len(panels) < 2:
        print("  ⚠ Expected 2 panels (front + back), found", len(panels))
        if len(panels) == 0:
            print("  FATAL: No usable panels found!")
            return

    # Identify front/back by Z position
    back_panel = panels[0]   # lower Z = back
    front_panel = panels[1]  # higher Z = front

    for name, panel in [("BACK", back_panel), ("FRONT", front_panel)]:
        v = panel.vertices
        boundary = find_boundary_edges(panel)
        classified = classify_boundary_edges(panel, boundary)

        print(f"\n  {name} Panel:")
        print(f"    Vertices: {len(v)}, Faces: {len(panel.faces)}")
        print(f"    X: [{v[:,0].min():.3f}, {v[:,0].max():.3f}] ({(v[:,0].max()-v[:,0].min())*100:.1f}cm)")
        print(f"    Y: [{v[:,1].min():.3f}, {v[:,1].max():.3f}] ({(v[:,1].max()-v[:,1].min())*100:.1f}cm)")
        print(f"    Z: {v[:,2].mean():.3f} (flat)")
        print(f"    Boundary edges: {len(boundary)}")
        print(f"      Left side:  {len(classified['left'])} edges")
        print(f"      Right side: {len(classified['right'])} edges")
        print(f"      Top:        {len(classified['top'])} edges")
        print(f"      Bottom:     {len(classified['bottom'])} edges")

        # Edge length stats
        everts = v[panel.edges_unique]
        elens = np.linalg.norm(everts[:, 0] - everts[:, 1], axis=1)
        print(f"    Edge lengths: mean={elens.mean()*100:.2f}cm, "
              f"min={elens.min()*100:.2f}cm, max={elens.max()*100:.2f}cm")

    # --- Find stitch pairs (side seams) ---
    print(f"\n  Stitch Analysis:")
    left_pairs = find_stitch_pairs(front_panel, back_panel, "left")
    right_pairs = find_stitch_pairs(front_panel, back_panel, "right")
    print(f"    Left side seam:  {len(left_pairs)} stitch pairs")
    print(f"    Right side seam: {len(right_pairs)} stitch pairs")

    # Measure stitch gaps
    for name, pairs in [("Left", left_pairs), ("Right", right_pairs)]:
        if pairs:
            gaps = []
            for ia, ib in pairs:
                pa = front_panel.vertices[ia]
                pb = back_panel.vertices[ib]
                gaps.append(np.linalg.norm(pa - pb))
            print(f"    {name} gap: mean={np.mean(gaps)*100:.1f}cm, "
                  f"min={np.min(gaps)*100:.1f}cm, max={np.max(gaps)*100:.1f}cm")

    # --- GGUI Visualization ---
    print(f"\n  Launching viewer...")
    print(f"    1=body  2=front  3=back  4=boundary  5=stitches  SPACE=wireframe")

    ti.init(arch=ti.cpu)
    window = ti.ui.Window("Blender Mesh Verification", (1280, 900))
    canvas = window.get_canvas()
    canvas.set_background_color((0.08, 0.08, 0.10))
    scene = window.get_scene()
    camera = ti.ui.Camera()
    camera.position(0.0, 1.10, 1.8)
    camera.lookat(0.0, 1.05, 0.16)

    # Load body mesh
    body = trimesh.load(body_path, force="mesh")
    body_v = ti.Vector.field(3, dtype=ti.f32, shape=len(body.vertices))
    body_v.from_numpy(body.vertices.astype(np.float32))
    body_f = ti.field(dtype=ti.i32, shape=body.faces.size)
    body_f.from_numpy(body.faces.flatten().astype(np.int32))

    # Front panel
    fv = ti.Vector.field(3, dtype=ti.f32, shape=len(front_panel.vertices))
    fv.from_numpy(front_panel.vertices.astype(np.float32))
    ff = ti.field(dtype=ti.i32, shape=front_panel.faces.size)
    ff.from_numpy(front_panel.faces.flatten().astype(np.int32))

    # Back panel
    bv = ti.Vector.field(3, dtype=ti.f32, shape=len(back_panel.vertices))
    bv.from_numpy(back_panel.vertices.astype(np.float32))
    bf = ti.field(dtype=ti.i32, shape=back_panel.faces.size)
    bf.from_numpy(back_panel.faces.flatten().astype(np.int32))

    # Boundary edge lines (both panels)
    all_boundary_pts = []
    for panel in [front_panel, back_panel]:
        bndy = find_boundary_edges(panel)
        classified = classify_boundary_edges(panel, bndy)
        colors_map = {
            "left": (1.0, 0.3, 0.3),    # red
            "right": (0.3, 1.0, 0.3),   # green
            "top": (0.3, 0.3, 1.0),     # blue
            "bottom": (1.0, 1.0, 0.3),  # yellow
        }
        for side, edges in classified.items():
            for a, b in edges:
                all_boundary_pts.append(panel.vertices[a])
                all_boundary_pts.append(panel.vertices[b])

    boundary_field = ti.Vector.field(3, dtype=ti.f32, shape=len(all_boundary_pts))
    boundary_field.from_numpy(np.array(all_boundary_pts, dtype=np.float32))

    # Stitch lines
    stitch_pts = []
    for ia, ib in left_pairs + right_pairs:
        stitch_pts.append(front_panel.vertices[ia])
        stitch_pts.append(back_panel.vertices[ib])
    if stitch_pts:
        stitch_field = ti.Vector.field(3, dtype=ti.f32, shape=len(stitch_pts))
        stitch_field.from_numpy(np.array(stitch_pts, dtype=np.float32))
        has_stitches = True
    else:
        has_stitches = False

    # Wireframe edges (front + back)
    wire_pts = []
    for panel in [front_panel, back_panel]:
        for a, b in panel.edges_unique:
            wire_pts.append(panel.vertices[a])
            wire_pts.append(panel.vertices[b])
    wire_field = ti.Vector.field(3, dtype=ti.f32, shape=len(wire_pts))
    wire_field.from_numpy(np.array(wire_pts, dtype=np.float32))

    show = {"body": True, "front": True, "back": True, "boundary": True, "stitch": True, "wire": False}

    while window.running:
        if window.get_event(ti.ui.PRESS):
            k = window.event.key
            if k == '1': show["body"] = not show["body"]; print(f"  Body: {'ON' if show['body'] else 'OFF'}")
            elif k == '2': show["front"] = not show["front"]; print(f"  Front: {'ON' if show['front'] else 'OFF'}")
            elif k == '3': show["back"] = not show["back"]; print(f"  Back: {'ON' if show['back'] else 'OFF'}")
            elif k == '4': show["boundary"] = not show["boundary"]; print(f"  Boundary: {'ON' if show['boundary'] else 'OFF'}")
            elif k == '5': show["stitch"] = not show["stitch"]; print(f"  Stitches: {'ON' if show['stitch'] else 'OFF'}")
            elif k == ti.ui.SPACE: show["wire"] = not show["wire"]; print(f"  Wireframe: {'ON' if show['wire'] else 'OFF'}")

        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        scene.ambient_light((0.35, 0.35, 0.35))
        scene.point_light(pos=(2, 5, 2), color=(1.0, 1.0, 1.0))
        scene.point_light(pos=(-2, 3, -1), color=(0.5, 0.5, 0.6))

        if show["body"]:
            scene.mesh(body_v, indices=body_f, color=(0.62, 0.52, 0.47), two_sided=True)
        if show["front"]:
            scene.mesh(fv, indices=ff, color=(0.3, 0.75, 0.85), two_sided=True)
        if show["back"]:
            scene.mesh(bv, indices=bf, color=(0.85, 0.5, 0.3), two_sided=True)
        if show["boundary"]:
            scene.lines(boundary_field, width=3.0, color=(1.0, 0.2, 0.2))
        if show["stitch"] and has_stitches:
            scene.lines(stitch_field, width=2.0, color=(0.2, 1.0, 0.3))
        if show["wire"]:
            scene.lines(wire_field, width=1.0, color=(1.0, 1.0, 1.0))

        canvas.scene(scene)
        window.show()

    print("\n  Viewer closed.")


if __name__ == "__main__":
    main()
