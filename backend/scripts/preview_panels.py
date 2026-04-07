"""
Panel Preview v3 — Correct tank top shape from validated body profile.

Key fixes from v2:
- Straps follow armhole edge (not fixed interior positions)
- More ease (6cm) and offset (3cm) for realistic loose fit
- Armhole starts below armpit, scoops curves inward naturally
- Higher grid resolution for smooth edges
- Hemline at hip level

Usage:
    python -m scripts.preview_panels

Controls:
    RMB + drag = rotate   |  W/A/S/D = pan
    1 = toggle body       |  2 = toggle panels
    SPACE = toggle wireframe
"""

import os
import numpy as np
import trimesh
import taichi as ti

from simulation.mesh.body_measurements import load_profile, wrap_point, BodyProfile


# ──────────────────────────────────────────────────────────
#  Tank Top Shape — Correct Pattern Geometry
# ──────────────────────────────────────────────────────────

def tank_top_shape(
    x: float, y: float,
    pw: float, side: str,
    lm: dict,
) -> bool:
    """
    Returns True if (x, y) is inside the tank top front/back panel.

    Pattern anatomy when laid flat:
    
        strap_L    neckline    strap_R
        ┌──┐   ╭──────────╮   ┌──┐
        │  │   │  (cutout) │   │  │
        │  ╰───╯           ╰───╯  │  ← armhole curves
        │         full width       │
        │         full width       │
        └──────────────────────────┘  ← hem

    - t=0: left side seam (under left arm)
    - t=0.5: body center (front or back)
    - t=1: right side seam (under right arm)
    
    Armhole cuts INWARD from edges. Straps = remaining strips at edges.
    Neckline scoops DOWN from center top.
    """
    shoulder_y = lm["shoulder_y"]
    armpit_y = lm["armpit_y"]

    # Normalize x to [0, 1]
    t = x / pw

    # Above shoulder: nothing
    if y > shoulder_y + 0.01:
        return False

    # ─── Full-width body zone (hem up to armhole start) ───
    # Armhole starts 4cm below the armpit for a natural scoop
    armhole_start_y = armpit_y - 0.04

    if y <= armhole_start_y:
        return True

    # ─── Upper panel (armhole + neckline zone) ───
    armhole_height = shoulder_y - armhole_start_y
    arm_p = min(max((y - armhole_start_y) / armhole_height, 0.0), 1.0)

    # Armhole curve: cuts inward from each edge.
    # Smooth scoop: starts gently, deepens toward shoulder.
    # At shoulder level, armhole has cut in ~35% from each side.
    # Using sin curve for natural armscye shape.
    cut = 0.35 * np.sin(arm_p * np.pi / 2)

    left_edge = cut            # left side moves right
    right_edge = 1.0 - cut     # right side moves left

    # Strap width: 7% of panel = ~3.7cm on a 53cm panel
    strap_w = 0.07

    # The armhole inner boundary (where armhole cutout ends, body panel begins)
    left_armhole_inner = left_edge + strap_w
    right_armhole_inner = right_edge - strap_w

    # ─── Zone check ───

    # Outside armhole boundary → cutout
    if t < left_edge or t > right_edge:
        return False

    # Inside strap zone (left)
    if t >= left_edge and t <= left_armhole_inner:
        return True

    # Inside strap zone (right)
    if t >= right_armhole_inner and t <= right_edge:
        return True

    # ─── Center zone: check neckline ───
    # Between the two straps is the body panel, which has a neckline scoop at top
    
    # Neckline depth (how far down the scoop goes from shoulder)
    if side == "front":
        neck_depth = armhole_height * 0.45  # front scoop: 45% of armhole height
    else:
        neck_depth = armhole_height * 0.20  # back: shallower

    neck_bottom_y = shoulder_y - neck_depth

    # Below neckline bottom: full center panel
    if y <= neck_bottom_y:
        return True

    # Above neckline bottom: sinusoidal scoop between strap inner edges
    neckline_span = right_armhole_inner - left_armhole_inner
    if neckline_span <= 0.001:
        return False

    center_t = (t - left_armhole_inner) / neckline_span
    # Sine curve: deepest at center (center_t=0.5), rising to shoulder at edges
    curve_y = neck_bottom_y + (shoulder_y - neck_bottom_y) * (1 - np.sin(center_t * np.pi))

    return y <= curve_y


def make_wrapped_grid(
    profile: BodyProfile,
    panel_width: float,
    side: str,
    y_bottom: float,
    y_top: float,
    landmarks: dict,
    cols: int = 40,
    rows: int = 50,
    offset: float = 0.025,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a grid mesh wrapped onto the body, masked by tank_top_shape."""
    xs = np.linspace(0, panel_width, cols)
    ys = np.linspace(y_bottom, y_top, rows)

    vertex_idx = np.full((rows, cols), -1, dtype=np.int32)
    positions = []
    idx = 0

    for r in range(rows):
        for c in range(cols):
            x_val, y_val = xs[c], ys[r]
            if not tank_top_shape(x_val, y_val, panel_width, side, landmarks):
                continue

            wx, wy, wz = wrap_point(profile, x_val, y_val, panel_width, side, offset)
            positions.append([wx, wy, wz])
            vertex_idx[r, c] = idx
            idx += 1

    if len(positions) == 0:
        return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.int32)

    positions = np.array(positions, dtype=np.float32)

    faces = []
    for r in range(rows - 1):
        for c in range(cols - 1):
            v00 = vertex_idx[r, c]
            v10 = vertex_idx[r + 1, c]
            v01 = vertex_idx[r, c + 1]
            v11 = vertex_idx[r + 1, c + 1]

            if v00 >= 0 and v10 >= 0 and v01 >= 0:
                faces.append([v00, v10, v01])
            if v10 >= 0 and v11 >= 0 and v01 >= 0:
                faces.append([v10, v11, v01])

    faces = np.array(faces, dtype=np.int32) if faces else np.zeros((0, 3), dtype=np.int32)
    return positions, faces


# ──────────────────────────────────────────────────────────
#  Main Preview
# ──────────────────────────────────────────────────────────

def preview_panels():
    backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    profile_path = os.path.join(backend_dir, "data/bodies/mannequin_profile.json")
    body_glb = os.path.join(backend_dir, "data/bodies/mannequin_physics.glb")

    print("═══════════════════════════════════════════════════")
    print("  Panel Preview v3 — Correct Shape")
    print("═══════════════════════════════════════════════════\n")

    profile = load_profile(profile_path)
    lm = profile.landmarks

    print(f"  Shoulder: Y={profile.shoulder_y:.3f}")
    print(f"  Armpit:   Y={profile.armpit_y:.3f}")
    print(f"  Chest:    Y={profile.chest_y:.3f}  circ={profile.chest_circumference:.3f}m")
    print(f"  Waist:    Y={profile.waist_y:.3f}")
    print(f"  Hip:      Y={profile.hip_y:.3f}")

    # ─── Panel dimensions ───
    ease = 0.06  # 6cm total ease for loose fit
    panel_width = profile.chest_circumference / 2 + ease
    y_bottom = profile.hip_y + 0.02  # hemline 2cm above hip landmark (avoid crotch)
    y_top = profile.shoulder_y + 0.01  # slight extension above shoulder
    offset = 0.030  # 3cm clearance — loose fit, no body contact

    print(f"\n  Panel width: {panel_width:.3f}m ({panel_width*100:.1f}cm)")
    print(f"  Panel height: {y_top - y_bottom:.3f}m (hip to shoulder)")
    print(f"  Ease: {ease*100:.0f}cm  |  Offset: {offset*100:.1f}cm")

    # ─── Generate panels (higher resolution) ───
    print("\n  Generating front panel...")
    front_pos, front_faces = make_wrapped_grid(
        profile, panel_width, "front", y_bottom, y_top, lm,
        cols=45, rows=55, offset=offset,
    )
    print(f"    {len(front_pos)} vertices, {len(front_faces)} triangles")

    print("  Generating back panel...")
    back_pos, back_faces = make_wrapped_grid(
        profile, panel_width, "back", y_bottom, y_top, lm,
        cols=45, rows=55, offset=offset,
    )
    print(f"    {len(back_pos)} vertices, {len(back_faces)} triangles")

    # ─── Seam edge diagnostics ───
    print("\n  ─── Seam Edge Analysis ───")
    for y_check in [y_bottom, profile.waist_y, profile.armpit_y]:
        fl = wrap_point(profile, 0, y_check, panel_width, "front", offset)
        fr = wrap_point(profile, panel_width, y_check, panel_width, "front", offset)
        bl = wrap_point(profile, 0, y_check, panel_width, "back", offset)
        br = wrap_point(profile, panel_width, y_check, panel_width, "back", offset)

        gap_left = np.sqrt(sum((a - b) ** 2 for a, b in zip(fl, br)))
        gap_right = np.sqrt(sum((a - b) ** 2 for a, b in zip(fr, bl)))

        y_name = "waist" if abs(y_check - profile.waist_y) < 0.01 else \
                 "armpit" if abs(y_check - profile.armpit_y) < 0.01 else "hem"
        print(f"    {y_name:>8s} (Y={y_check:.2f}): L-gap={gap_left*100:.1f}cm, R-gap={gap_right*100:.1f}cm")

    # ─── Taichi GGUI ───
    print("\n  Launching preview...")
    print("    1 = body  |  2 = panels  |  SPACE = wireframe")
    print()

    ti.init(arch=ti.cpu)

    window = ti.ui.Window("Panel Preview v3", (1280, 900))
    canvas = window.get_canvas()
    canvas.set_background_color((0.10, 0.10, 0.13))
    scene = window.get_scene()
    camera = ti.ui.Camera()
    camera.position(0.0, 1.10, 1.8)
    camera.lookat(0.0, 1.10, 0.16)

    # Body mesh
    body_mesh = trimesh.load(body_glb, force="mesh")
    body_v = ti.Vector.field(3, dtype=ti.f32, shape=len(body_mesh.vertices))
    body_v.from_numpy(body_mesh.vertices.astype(np.float32))
    body_f = ti.field(dtype=ti.i32, shape=body_mesh.faces.size)
    body_f.from_numpy(body_mesh.faces.flatten().astype(np.int32))

    # Panel mesh (combined front + back)
    n_front = len(front_pos)
    all_pos = np.concatenate([front_pos, back_pos], axis=0)
    all_faces_np = np.concatenate([front_faces, back_faces + n_front], axis=0)

    has_panels = len(all_pos) > 0 and len(all_faces_np) > 0
    if has_panels:
        panel_v = ti.Vector.field(3, dtype=ti.f32, shape=len(all_pos))
        panel_v.from_numpy(all_pos)
        panel_f = ti.field(dtype=ti.i32, shape=all_faces_np.size)
        panel_f.from_numpy(all_faces_np.flatten().astype(np.int32))

        # Wireframe: edges from triangles
        edges_set = set()
        for f in all_faces_np:
            for i in range(3):
                e = tuple(sorted([f[i], f[(i + 1) % 3]]))
                edges_set.add(e)
        edges = np.array(list(edges_set), dtype=np.int32)
        wire_pts = np.zeros((len(edges) * 2, 3), dtype=np.float32)
        for i, (a, b) in enumerate(edges):
            wire_pts[i * 2] = all_pos[a]
            wire_pts[i * 2 + 1] = all_pos[b]
        wire_field = ti.Vector.field(3, dtype=ti.f32, shape=len(wire_pts))
        wire_field.from_numpy(wire_pts)

    show_body = True
    show_panels = True
    show_wire = False

    print("  Running. Close window to exit.\n")

    while window.running:
        if window.get_event(ti.ui.PRESS):
            if window.event.key == '1':
                show_body = not show_body
                print(f"  Body: {'ON' if show_body else 'OFF'}")
            elif window.event.key == '2':
                show_panels = not show_panels
                print(f"  Panels: {'ON' if show_panels else 'OFF'}")
            elif window.event.key == ti.ui.SPACE:
                show_wire = not show_wire
                print(f"  Wireframe: {'ON' if show_wire else 'OFF'}")

        camera.track_user_inputs(window, movement_speed=0.03, hold_key=ti.ui.RMB)
        scene.set_camera(camera)

        scene.ambient_light((0.35, 0.35, 0.35))
        scene.point_light(pos=(2, 5, 2), color=(1.0, 1.0, 1.0))
        scene.point_light(pos=(-2, 3, -1), color=(0.5, 0.5, 0.6))

        if show_body:
            scene.mesh(body_v, indices=body_f, color=(0.62, 0.52, 0.47), two_sided=True)

        if show_panels and has_panels:
            scene.mesh(panel_v, indices=panel_f, color=(0.3, 0.75, 0.85), two_sided=True)

        if show_wire and has_panels:
            scene.lines(wire_field, width=1.0, color=(1.0, 1.0, 1.0))

        canvas.scene(scene)
        window.show()


if __name__ == "__main__":
    preview_panels()
