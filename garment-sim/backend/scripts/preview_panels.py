"""
Panel Preview v5 — Smooth Bézier panels with flat placement.

Generates panels, builds garment mesh, and visualizes with Taichi GGUI.
Includes verification diagnostics: panel outlines, body overlap check,
stitch gap measurements.

Usage:
    python -m scripts.preview_panels

Controls:
    1 = body  |  2 = panels  |  3 = boundary edges  |  4 = stitch lines
    5 = wireframe  |  RMB + drag = rotate  |  W/A/S/D = pan
"""

import os
import sys
import json
import numpy as np
import trimesh
import taichi as ti

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from simulation.mesh.pattern_generator import generate_tank_top
from simulation.mesh.panel_builder import build_garment_mesh


def preview_panels():
    profile_path = os.path.join(backend_dir, "data/bodies/mannequin_profile.json")
    body_glb = os.path.join(backend_dir, "data/bodies/mannequin_physics.glb")
    output_json = os.path.join(backend_dir, "data/patterns/tank_top_v2.json")

    print("═" * 60)
    print("  Panel Preview v5 — Smooth Bézier + Flat Placement")
    print("═" * 60)

    # --- Generate pattern ---
    pattern = generate_tank_top(profile_path, fit="relaxed")

    with open(output_json, "w") as f:
        json.dump(pattern, f, indent=2)
    print(f"\n  Saved: {output_json}")

    # --- Build garment mesh ---
    print("\n  Building garment mesh...")
    tmp_json = os.path.join(backend_dir, "data/patterns/_tmp_preview.json")
    with open(tmp_json, "w") as f:
        json.dump(pattern, f)

    try:
        garment = build_garment_mesh(tmp_json, resolution=25)
        print(f"  ✓ Garment: {len(garment.positions)} vertices, "
              f"{len(garment.faces)} triangles, "
              f"{len(garment.stitch_pairs)} stitch pairs")
    except Exception as e:
        print(f"  ✗ Build failed: {e}")
        import traceback
        traceback.print_exc()
        return
    finally:
        if os.path.exists(tmp_json):
            os.remove(tmp_json)

    # --- Verification diagnostics ---
    print("\n  ═══ Verification Checks ═══")

    # Check 1: Panel bounding box
    for i, pid in enumerate(garment.panel_ids):
        start = garment.panel_offsets[i]
        end = garment.panel_offsets[i + 1] if i + 1 < len(garment.panel_offsets) else len(garment.positions)
        pv = garment.positions[start:end]
        print(f"\n  Panel '{pid}':")
        print(f"    Vertices: {len(pv)}")
        print(f"    X: [{pv[:,0].min():.3f}, {pv[:,0].max():.3f}] ({(pv[:,0].max()-pv[:,0].min())*100:.1f}cm)")
        print(f"    Y: [{pv[:,1].min():.3f}, {pv[:,1].max():.3f}] ({(pv[:,1].max()-pv[:,1].min())*100:.1f}cm)")
        print(f"    Z: {pv[:,2].mean():.3f} (flat)")

    # Check 2: Stitch gaps
    if len(garment.stitch_pairs) > 0:
        gaps = []
        for a, b in garment.stitch_pairs:
            gap = np.linalg.norm(garment.positions[a] - garment.positions[b])
            gaps.append(gap)
        gaps = np.array(gaps)
        print(f"\n  Stitch gaps:")
        print(f"    Count:  {len(gaps)}")
        print(f"    Mean:   {gaps.mean()*100:.1f}cm")
        print(f"    Min:    {gaps.min()*100:.1f}cm")
        print(f"    Max:    {gaps.max()*100:.1f}cm")

    # Check 3: Body overlap check
    body = trimesh.load(body_glb, force="mesh")
    body_x_range = [body.vertices[:, 0].min(), body.vertices[:, 0].max()]
    body_y_range = [body.vertices[:, 1].min(), body.vertices[:, 1].max()]
    body_z_range = [body.vertices[:, 2].min(), body.vertices[:, 2].max()]
    print(f"\n  Body extent:")
    print(f"    X: [{body_x_range[0]:.3f}, {body_x_range[1]:.3f}]")
    print(f"    Y: [{body_y_range[0]:.3f}, {body_y_range[1]:.3f}]")
    print(f"    Z: [{body_z_range[0]:.3f}, {body_z_range[1]:.3f}]")

    front_z = garment.positions[:, 2].max()
    back_z = garment.positions[:, 2].min()
    print(f"    Front panel Z={front_z:.3f} vs body front Z={body_z_range[1]:.3f} "
          f"→ clearance: {(front_z - body_z_range[1])*100:.1f}cm {'✓' if front_z > body_z_range[1] else '✗ OVERLAP!'}")
    print(f"    Back panel Z={back_z:.3f} vs body back Z={body_z_range[0]:.3f} "
          f"→ clearance: {(body_z_range[0] - back_z)*100:.1f}cm {'✓' if back_z < body_z_range[0] else '✗ OVERLAP!'}")

    # --- GGUI Visualization ---
    print(f"\n  ═══ Launching Viewer ═══")
    print(f"  1=body  2=panels  3=boundary  4=stitches  5=wireframe  Q=quit")
    print()

    ti.init(arch=ti.cpu)
    window = ti.ui.Window("Panel Preview v5", (1280, 900))
    canvas = window.get_canvas()
    canvas.set_background_color((0.08, 0.08, 0.10))
    scene = window.get_scene()
    camera = ti.ui.Camera()
    camera.position(0.0, 1.10, 1.8)
    camera.lookat(0.0, 1.05, 0.16)

    # Body
    body_v = ti.Vector.field(3, dtype=ti.f32, shape=len(body.vertices))
    body_v.from_numpy(body.vertices.astype(np.float32))
    body_f = ti.field(dtype=ti.i32, shape=body.faces.size)
    body_f.from_numpy(body.faces.flatten().astype(np.int32))

    # Garment panels
    panel_v = ti.Vector.field(3, dtype=ti.f32, shape=len(garment.positions))
    panel_v.from_numpy(garment.positions)
    panel_f = ti.field(dtype=ti.i32, shape=garment.faces.size)
    panel_f.from_numpy(garment.faces.flatten().astype(np.int32))

    # Wireframe
    edge_set = set()
    for f in garment.faces:
        for i in range(3):
            e = tuple(sorted([f[i], f[(i + 1) % 3]]))
            edge_set.add(e)
    wire_pts = np.zeros((len(edge_set) * 2, 3), dtype=np.float32)
    for i, (a, b) in enumerate(edge_set):
        wire_pts[i * 2] = garment.positions[a]
        wire_pts[i * 2 + 1] = garment.positions[b]
    wire_field = ti.Vector.field(3, dtype=ti.f32, shape=len(wire_pts))
    wire_field.from_numpy(wire_pts)

    # Boundary edges (edges belonging to only 1 face)
    from collections import Counter
    face_edges = []
    for f in garment.faces:
        for i in range(3):
            e = tuple(sorted([f[i], f[(i + 1) % 3]]))
            face_edges.append(e)
    ec = Counter(face_edges)
    boundary = [e for e, c in ec.items() if c == 1]
    bound_pts = np.zeros((len(boundary) * 2, 3), dtype=np.float32)
    for i, (a, b) in enumerate(boundary):
        bound_pts[i * 2] = garment.positions[a]
        bound_pts[i * 2 + 1] = garment.positions[b]
    bound_field = ti.Vector.field(3, dtype=ti.f32, shape=max(len(bound_pts), 1))
    if len(bound_pts) > 0:
        bound_field.from_numpy(bound_pts)

    # Stitch lines
    has_stitches = len(garment.stitch_pairs) > 0
    if has_stitches:
        stitch_pts = np.zeros((len(garment.stitch_pairs) * 2, 3), dtype=np.float32)
        for i, (a, b) in enumerate(garment.stitch_pairs):
            stitch_pts[i * 2] = garment.positions[a]
            stitch_pts[i * 2 + 1] = garment.positions[b]
        stitch_field = ti.Vector.field(3, dtype=ti.f32, shape=len(stitch_pts))
        stitch_field.from_numpy(stitch_pts)

    show = {"body": True, "panels": True, "boundary": True, "stitch": True, "wire": False}

    while window.running:
        if window.get_event(ti.ui.PRESS):
            k = window.event.key
            if k == '1': show["body"] = not show["body"]
            elif k == '2': show["panels"] = not show["panels"]
            elif k == '3': show["boundary"] = not show["boundary"]
            elif k == '4': show["stitch"] = not show["stitch"]
            elif k == '5' or k == ti.ui.SPACE: show["wire"] = not show["wire"]

        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        scene.ambient_light((0.35, 0.35, 0.35))
        scene.point_light(pos=(2, 5, 2), color=(1.0, 1.0, 1.0))
        scene.point_light(pos=(-2, 3, -1), color=(0.5, 0.5, 0.6))

        if show["body"]:
            scene.mesh(body_v, indices=body_f, color=(0.62, 0.52, 0.47), two_sided=True)
        if show["panels"]:
            scene.mesh(panel_v, indices=panel_f, color=(0.3, 0.75, 0.85), two_sided=True)
        if show["boundary"] and len(bound_pts) > 0:
            scene.lines(bound_field, width=3.0, color=(1.0, 0.3, 0.3))
        if show["stitch"] and has_stitches:
            scene.lines(stitch_field, width=2.0, color=(0.2, 1.0, 0.3))
        if show["wire"]:
            scene.lines(wire_field, width=1.0, color=(1.0, 1.0, 1.0))

        canvas.scene(scene)
        window.show()


if __name__ == "__main__":
    preview_panels()
