"""
DXF Validation Preview — Validates imported CLO3D DXF patterns.

Loads imported pattern JSON, builds garment mesh, and visualizes with Taichi GGUI.
Provides verification diagnostics for flat panel placement and stitch gaps.

Usage:
    python -m scripts.verify_dxf_import

Controls:
    1 = body  |  2 = panels  |  3 = boundary edges  |  4 = stitch lines
    5 = wireframe  |  RMB + drag = rotate  |  W/A/S/D = pan
"""

import os
import sys
import numpy as np
import trimesh
import taichi as ti

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from simulation.mesh.panel_builder import build_garment_mesh

def verify_dxf(pattern_path: str):
    body_glb = os.path.join(backend_dir, "data/bodies/mannequin_physics.glb")

    print("═" * 60)
    print("  DXF Import Verification")
    print("═" * 60)

    # --- Build garment mesh ---
    print(f"\n  Loading pattern: {os.path.basename(pattern_path)}")

    try:
        garment = build_garment_mesh(pattern_path, resolution=25)
        print(f"  ✓ Garment: {len(garment.positions)} vertices, "
              f"{len(garment.faces)} triangles, "
              f"{len(garment.stitch_pairs)} stitch pairs")
    except Exception as e:
        print(f"  ✗ Build failed: {e}")
        import traceback
        traceback.print_exc()
        return

    # --- Verification diagnostics ---
    print("\n  ═══ Panel Summary ═══")
    
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

    body = trimesh.load(body_glb, force="mesh")

    # --- GGUI Visualization ---
    print(f"\n  ═══ Launching Viewer ═══")
    print(f"  1=body  2=panels  3=boundary  4=stitches  5=wireframe  Q=quit")
    print(f"  LMB drag = orbit around body  |  Scroll = zoom  |  RMB drag = pan")
    print()

    ti.init(arch=ti.cpu)
    window = ti.ui.Window("DXF Preview", (1280, 900))
    canvas = window.get_canvas()
    canvas.set_background_color((0.08, 0.08, 0.10))
    scene = window.get_scene()
    camera = ti.ui.Camera()

    # Orbit camera state — body centre is the fixed look-at target
    TARGET   = np.array([0.0, 1.05, 0.16], dtype=np.float64)
    orbit    = {"azim": 0.0, "elev": 10.0, "radius": 1.8}   # degrees, metres
    pan      = {"x": 0.0, "y": 0.0}                          # world-space pan
    prev_pos = {"x": None, "y": None, "btn": None}

    def _orbit_position():
        az  = np.radians(orbit["azim"])
        el  = np.radians(orbit["elev"])
        r   = orbit["radius"]
        px  = r * np.cos(el) * np.sin(az)
        py  = r * np.sin(el)
        pz  = r * np.cos(el) * np.cos(az)
        eye = TARGET + np.array([px, py, pz]) + np.array([pan["x"], pan["y"], 0.0])
        return eye.tolist()

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
    if edge_set:
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
        # --- Key events ---
        if window.get_event(ti.ui.PRESS):
            k = window.event.key
            if k == '1': show["body"] = not show["body"]
            elif k == '2': show["panels"] = not show["panels"]
            elif k == '3': show["boundary"] = not show["boundary"]
            elif k == '4': show["stitch"] = not show["stitch"]
            elif k == '5' or k == ti.ui.SPACE: show["wire"] = not show["wire"]

        # --- Orbit camera: LMB = rotate, RMB = pan, scroll = zoom ---
        mouse_x, mouse_y = window.get_cursor_pos()

        lmb = window.is_pressed(ti.ui.LMB)
        rmb = window.is_pressed(ti.ui.RMB)
        cur_btn = "lmb" if lmb else ("rmb" if rmb else None)

        if cur_btn and prev_pos["btn"] == cur_btn and prev_pos["x"] is not None:
            dx = mouse_x - prev_pos["x"]
            dy = mouse_y - prev_pos["y"]
            if cur_btn == "lmb":
                orbit["azim"] -= dx * 200.0
                orbit["elev"]  = float(np.clip(orbit["elev"] + dy * 100.0, -80, 80))
            else:  # RMB = pan
                pan["x"] -= dx * orbit["radius"] * 0.8
                pan["y"] += dy * orbit["radius"] * 0.8

        prev_pos["x"] = mouse_x if cur_btn else None
        prev_pos["y"] = mouse_y if cur_btn else None
        prev_pos["btn"] = cur_btn

        # Scroll = zoom
        if window.is_pressed("w"): orbit["radius"] = max(0.3, orbit["radius"] - 0.02)
        if window.is_pressed("s"): orbit["radius"] = min(5.0, orbit["radius"] + 0.02)

        eye = _orbit_position()
        target = (TARGET + np.array([pan["x"], pan["y"], 0.0])).tolist()
        camera.position(*eye)
        camera.lookat(*target)
        camera.up(0, 1, 0)

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
        if show["wire"] and edge_set:
            scene.lines(wire_field, width=1.0, color=(1.0, 1.0, 1.0))

        # --- Simple point-picker for manual ID inspection ---
        if window.is_pressed(ti.ui.LMB) and window.is_pressed(ti.ui.SHIFT):
            # Find closest vertex in 3D to central ray (approximation)
            cam_pos = np.array(eye)
            cam_look = np.array(target)
            fwd = cam_look - cam_pos
            fwd /= np.linalg.norm(fwd)
            
            # For simplicity, we just find the closest vertex to the camera target
            # since the user orbits around what they want to see.
            dists = np.linalg.norm(garment.positions - cam_look, axis=1)
            closest_idx = np.argmin(dists)
            if dists[closest_idx] < 0.1:
                # Find which panel this belongs to
                p_id = "unknown"
                l_idx = -1
                for i, pid in enumerate(garment.panel_ids):
                    start = garment.panel_offsets[i]
                    end = garment.panel_offsets[i+1] if i+1 < len(garment.panel_offsets) else len(garment.positions)
                    if start <= closest_idx < end:
                        p_id = pid
                        l_idx = closest_idx - start
                        break
                
                window.GUI.begin("Vertex Inspector", 0.05, 0.05, 0.2, 0.1)
                window.GUI.text(f"Panel: {p_id}")
                window.GUI.text(f"Global ID: {closest_idx}")
                window.GUI.text(f"Local ID: {l_idx}")
                window.GUI.end()

        canvas.scene(scene)
        window.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", type=str, default="data/patterns/tshirt.json")
    args = parser.parse_args()
    
    path = os.path.abspath(os.path.join(backend_dir, args.pattern))
    verify_dxf(path)
