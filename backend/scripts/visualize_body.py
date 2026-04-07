"""
Body Analysis Visualizer — Renders body measurements and landmarks on the mesh.

Shows:
  - Body mesh with semi-transparent rendering
  - Horizontal cross-section rings at key landmark heights
  - Torso boundary markers (colored dots) at each slice
  - Landmark labels at neck, shoulder, armpit, chest, waist, hip
  - Torso X-limits as vertical guidelines

Usage:
    python -m scripts.visualize_body

Controls:
    RMB + drag = rotate camera
    W/A/S/D    = pan camera
    1          = toggle body mesh
    2          = toggle cross-section rings
    3          = toggle all slices
"""

import json
from pathlib import Path

import numpy as np
import trimesh
import taichi as ti


BODY_GLB = "data/bodies/mannequin_physics.glb"
PROFILE_JSON = "data/bodies/mannequin_profile.json"

# Colors for landmarks
COLORS = {
    "neck":     (1.0, 0.2, 0.2),   # red
    "shoulder": (1.0, 0.6, 0.0),   # orange
    "armpit":   (1.0, 1.0, 0.0),   # yellow
    "chest":    (0.2, 1.0, 0.2),   # green
    "waist":    (0.2, 0.6, 1.0),   # blue
    "hip":      (0.6, 0.2, 1.0),   # purple
    "crotch":   (1.0, 0.2, 0.6),   # pink
}


def generate_ring_points(
    y: float,
    x_left: float, x_right: float,
    z_front: float, z_back: float,
    n_points: int = 60,
) -> np.ndarray:
    """Generate points forming an elliptical ring at height Y."""
    cx = (x_left + x_right) / 2
    cz = (z_front + z_back) / 2
    a = (x_right - x_left) / 2  # X semi-axis
    b = (z_front - z_back) / 2  # Z semi-axis
    
    if a < 0.005 or b < 0.005:
        return np.zeros((0, 3), dtype=np.float32)
    
    angles = np.linspace(0, 2 * np.pi, n_points, endpoint=False)
    pts = np.zeros((n_points, 3), dtype=np.float32)
    pts[:, 0] = cx + a * np.cos(angles)
    pts[:, 1] = y
    pts[:, 2] = cz + b * np.sin(angles)
    return pts


def ring_to_line_segments(pts: np.ndarray) -> np.ndarray:
    """Convert ring points to pairs for line rendering."""
    if len(pts) < 2:
        return np.zeros((0, 3), dtype=np.float32)
    n = len(pts)
    lines = np.zeros((n * 2, 3), dtype=np.float32)
    for i in range(n):
        lines[i * 2] = pts[i]
        lines[i * 2 + 1] = pts[(i + 1) % n]
    return lines


def main():
    import os
    
    # ─── Step 1: Run body analysis if profile doesn't exist ───
    profile_path = Path(PROFILE_JSON)
    if not profile_path.exists():
        print("  Profile not found, running analysis...")
        from scripts.analyze_body import analyze_body
        analyze_body(BODY_GLB, PROFILE_JSON)
    
    with open(profile_path) as f:
        profile = json.load(f)
    
    landmarks = profile["landmarks"]
    sections = profile["cross_sections"]
    
    print("═══════════════════════════════════════════════════")
    print("  Body Analysis Visualizer")
    print("═══════════════════════════════════════════════════\n")
    
    print("  Loaded landmarks:")
    for k, v in landmarks.items():
        if isinstance(v, float):
            print(f"    {k}: {v:.4f}m")
    
    # ─── Step 2: Build visualization geometry ───
    print("\n  Building visualization geometry...")
    
    # Landmark rings (one per landmark)
    landmark_names = ["neck", "shoulder", "armpit", "chest", "waist", "hip", "crotch"]
    landmark_rings = {}  # name -> line segments
    landmark_markers = {}  # name -> marker positions (corners of the cross-section)
    
    for name in landmark_names:
        ky = f"{name}_y"
        if ky not in landmarks:
            continue
        
        target_y = landmarks[ky]
        # Find closest cross-section
        closest = min(sections, key=lambda s: abs(s["y"] - target_y))
        t = closest["torso"]
        
        # Generate ring
        ring_pts = generate_ring_points(
            y=target_y,
            x_left=t["x_left"], x_right=t["x_right"],
            z_front=t["z_front"], z_back=t["z_back"],
            n_points=60,
        )
        landmark_rings[name] = ring_to_line_segments(ring_pts)
        
        # Generate corner markers (front, back, left, right surface points)
        markers = np.array([
            [t["center_x"], target_y, t["z_front"]],   # front
            [t["center_x"], target_y, t["z_back"]],    # back
            [t["x_left"], target_y, t["center_z"]],     # left
            [t["x_right"], target_y, t["center_z"]],    # right
        ], dtype=np.float32)
        landmark_markers[name] = markers
    
    # All cross-section rings (dim, for reference)
    all_rings_lines = []
    for s in sections:
        t = s["torso"]
        ring = generate_ring_points(
            y=s["y"],
            x_left=t["x_left"], x_right=t["x_right"],
            z_front=t["z_front"], z_back=t["z_back"],
            n_points=40,
        )
        if len(ring) > 0:
            all_rings_lines.append(ring_to_line_segments(ring))
    
    if all_rings_lines:
        all_rings_data = np.concatenate(all_rings_lines, axis=0)
    else:
        all_rings_data = np.zeros((2, 3), dtype=np.float32)
    
    # Measurement text lines — horizontal lines showing width at each landmark
    width_lines = []
    depth_lines = []
    for name in landmark_names:
        ky = f"{name}_y"
        if ky not in landmarks:
            continue
        target_y = landmarks[ky]
        closest = min(sections, key=lambda s: abs(s["y"] - target_y))
        t = closest["torso"]
        
        # Width line: left to right at z_center
        width_lines.append([t["x_left"], target_y, t["center_z"]])
        width_lines.append([t["x_right"], target_y, t["center_z"]])
        
        # Depth line: front to back at x_center
        depth_lines.append([t["center_x"], target_y, t["z_front"]])
        depth_lines.append([t["center_x"], target_y, t["z_back"]])
    
    width_lines = np.array(width_lines, dtype=np.float32) if width_lines else np.zeros((2, 3), dtype=np.float32)
    depth_lines = np.array(depth_lines, dtype=np.float32) if depth_lines else np.zeros((2, 3), dtype=np.float32)
    
    # ─── Step 3: Setup Taichi GGUI ───
    print("  Launching visualizer...")
    print("\n  Controls:")
    print("    RMB + drag = rotate camera")
    print("    W/A/S/D    = pan camera")
    print("    1          = toggle body mesh")
    print("    2          = toggle landmark rings")
    print("    3          = toggle all cross-section rings")
    print("    4          = toggle width/depth lines")
    print()
    print("  Legend:")
    for name, color in COLORS.items():
        ky = f"{name}_y"
        if ky in landmarks:
            print(f"    {name:<12s} Y={landmarks[ky]:.3f}m  ●")
    print()
    
    ti.init(arch=ti.cpu)
    
    # Resolve paths relative to backend/ directory
    import os
    backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    body_path = os.path.join(backend_dir, BODY_GLB)
    profile_path_abs = os.path.join(backend_dir, PROFILE_JSON)
    
    # Re-read profile with absolute path if needed
    if not Path(profile_path).exists():
        profile_path = Path(profile_path_abs)
        with open(profile_path) as f:
            profile = json.load(f)
        landmarks = profile["landmarks"]
        sections = profile["cross_sections"]
    
    window = ti.ui.Window("Body Analysis Visualizer", (1280, 900))
    canvas = window.get_canvas()
    canvas.set_background_color((0.10, 0.10, 0.13))
    scene = window.get_scene()
    camera = ti.ui.Camera()
    camera.position(0.0, 0.85, 2.2)
    camera.lookat(0.0, 0.85, 0.16)
    
    # Load body mesh
    body_mesh = trimesh.load(body_path, force="mesh")
    body_v = ti.Vector.field(3, dtype=ti.f32, shape=len(body_mesh.vertices))
    body_v.from_numpy(body_mesh.vertices.astype(np.float32))
    body_f = ti.field(dtype=ti.i32, shape=body_mesh.faces.size)
    body_f.from_numpy(body_mesh.faces.flatten().astype(np.int32))
    
    # Landmark rings as Taichi fields
    ring_fields = {}
    marker_fields = {}
    for name in landmark_names:
        if name in landmark_rings and len(landmark_rings[name]) > 0:
            data = landmark_rings[name]
            field = ti.Vector.field(3, dtype=ti.f32, shape=len(data))
            field.from_numpy(data)
            ring_fields[name] = field
        
        if name in landmark_markers:
            data = landmark_markers[name]
            field = ti.Vector.field(3, dtype=ti.f32, shape=len(data))
            field.from_numpy(data)
            marker_fields[name] = field
    
    # All rings
    all_rings_field = ti.Vector.field(3, dtype=ti.f32, shape=max(len(all_rings_data), 2))
    all_rings_field.from_numpy(all_rings_data[:max(len(all_rings_data), 2)])
    
    # Width/depth lines
    width_field = ti.Vector.field(3, dtype=ti.f32, shape=max(len(width_lines), 2))
    width_field.from_numpy(width_lines[:max(len(width_lines), 2)])
    depth_field = ti.Vector.field(3, dtype=ti.f32, shape=max(len(depth_lines), 2))
    depth_field.from_numpy(depth_lines[:max(len(depth_lines), 2)])
    
    show_body = True
    show_landmarks = True
    show_all_rings = False
    show_measurements = True
    
    print("  Visualizer running. Close window to exit.\n")
    
    while window.running:
        if window.get_event(ti.ui.PRESS):
            if window.event.key == '1':
                show_body = not show_body
                print(f"  Body: {'ON' if show_body else 'OFF'}")
            elif window.event.key == '2':
                show_landmarks = not show_landmarks
                print(f"  Landmarks: {'ON' if show_landmarks else 'OFF'}")
            elif window.event.key == '3':
                show_all_rings = not show_all_rings
                print(f"  All rings: {'ON' if show_all_rings else 'OFF'}")
            elif window.event.key == '4':
                show_measurements = not show_measurements
                print(f"  Width/depth lines: {'ON' if show_measurements else 'OFF'}")
        
        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)
        
        scene.ambient_light((0.3, 0.3, 0.3))
        scene.point_light(pos=(2, 5, 2), color=(0.9, 0.9, 0.9))
        scene.point_light(pos=(-2, 3, -1), color=(0.4, 0.4, 0.5))
        
        # Body mesh
        if show_body:
            scene.mesh(body_v, indices=body_f, color=(0.62, 0.52, 0.47), two_sided=True)
        
        # Landmark rings and markers
        if show_landmarks:
            for name in landmark_names:
                color = COLORS.get(name, (1, 1, 1))
                
                if name in ring_fields:
                    scene.lines(ring_fields[name], width=3.0, color=color)
                
                if name in marker_fields:
                    scene.particles(marker_fields[name], radius=0.008, color=color)
        
        # All cross-section rings (dim gray)
        if show_all_rings and len(all_rings_data) > 2:
            scene.lines(all_rings_field, width=1.0, color=(0.4, 0.4, 0.4))
        
        # Width and depth measurement lines
        if show_measurements:
            if len(width_lines) > 1:
                scene.lines(width_field, width=2.0, color=(1.0, 1.0, 0.3))  # yellow
            if len(depth_lines) > 1:
                scene.lines(depth_field, width=2.0, color=(0.3, 1.0, 1.0))  # cyan
        
        canvas.scene(scene)
        window.show()
    
    print("  Visualizer closed.")


if __name__ == "__main__":
    main()
