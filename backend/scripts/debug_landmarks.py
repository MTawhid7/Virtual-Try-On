"""
Debug script: print detected landmark indices and positions for each body panel.

Runs BEFORE and AFTER fixing get_top_landmarks() to confirm the bug and the fix.

Usage:
    python -m scripts.debug_landmarks
"""
import os
import sys
import numpy as np

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from scripts.import_dxf import (
    extract_pieces,
    normalize_piece,
    get_top_landmarks,
    get_sleeve_points,
    find_vertical_run,
)

dxf_path = os.path.join(backend_dir, "data/patterns/Polo-Shirt.dxf")

print("═" * 60)
print("  Landmark Detection Debug")
print("═" * 60)

raw = extract_pieces(dxf_path, ["Body_Front_1_M", "Body_Back_M", "Sleeves_M"])
for p in raw:
    normalize_piece(p)

pieces = {}
for p in raw:
    if "front" in p.name.lower():
        pieces["front_right"] = p
        p.name = "front_right"
    elif "back" in p.name.lower():
        pieces["back_right"] = p
        p.name = "back_right"
    elif "sleeve" in p.name.lower():
        pieces["sleeve_right"] = p
        p.name = "sleeve_right"

# --- Body panels ---
for panel_name, side in [("front_right", "right"), ("back_right", "left")]:
    if panel_name not in pieces:
        print(f"\n  ⚠ {panel_name} not found")
        continue

    verts = pieces[panel_name].vertices_2d
    pts = np.array(verts)
    n = len(verts)

    neck, inner_sh, outer_sh, underarm = get_top_landmarks(verts, side)

    pw = pts[:, 0].max()
    ph = pts[:, 1].max()

    print(f"\n  Panel: {panel_name}  ({n} verts, pw={pw*100:.1f}cm, ph={ph*100:.1f}cm)")
    print(f"    inner_shoulder  = v{inner_sh:3d}  X={pts[inner_sh,0]:.4f}  Y={pts[inner_sh,1]:.4f}")
    print(f"    outer_shoulder  = v{outer_sh:3d}  X={pts[outer_sh,0]:.4f}  Y={pts[outer_sh,1]:.4f}")
    print(f"    underarm        = v{underarm:3d}  X={pts[underarm,0]:.4f}  Y={pts[underarm,1]:.4f}")
    print(f"    neck            = v{neck:3d}  X={pts[neck,0]:.4f}  Y={pts[neck,1]:.4f}")

    # Sanity checks
    if side == "right":
        # outer_shoulder should be at high X and high Y (not neckline territory)
        ok_x = pts[outer_sh, 0] > pw * 0.7
        ok_y = pts[outer_sh, 1] > pts[underarm, 1]
        print(f"    ✓ outer_shoulder X > 70% width: {'✓' if ok_x else '✗ WRONG (too far left = neckline!)'}")
        print(f"    ✓ outer_shoulder Y > underarm Y: {'✓' if ok_y else '✗ WRONG'}")
    else:
        ok_x = pts[outer_sh, 0] < pw * 0.3
        ok_y = pts[outer_sh, 1] > pts[underarm, 1]
        print(f"    ✓ outer_shoulder X < 30% width: {'✓' if ok_x else '✗ WRONG (too far right = neckline!)'}")
        print(f"    ✓ outer_shoulder Y > underarm Y: {'✓' if ok_y else '✗ WRONG'}")

# --- Sleeve ---
if "sleeve_right" in pieces:
    verts = pieces["sleeve_right"].vertices_2d
    pts = np.array(verts)
    n = len(verts)

    cL, cR, uL, uR, notch = get_sleeve_points(verts)

    pw = pts[:, 0].max()
    ph = pts[:, 1].max()

    print(f"\n  Panel: sleeve_right  ({n} verts, pw={pw*100:.1f}cm, ph={ph*100:.1f}cm)")
    print(f"    cuff_left   = v{cL:3d}  X={pts[cL,0]:.4f}  Y={pts[cL,1]:.4f}")
    print(f"    cuff_right  = v{cR:3d}  X={pts[cR,0]:.4f}  Y={pts[cR,1]:.4f}")
    print(f"    underarm_L  = v{uL:3d}  X={pts[uL,0]:.4f}  Y={pts[uL,1]:.4f}")
    print(f"    underarm_R  = v{uR:3d}  X={pts[uR,0]:.4f}  Y={pts[uR,1]:.4f}")
    print(f"    notch       = v{notch:3d}  X={pts[notch,0]:.4f}  Y={pts[notch,1]:.4f}")

    # Sleeve cap (from uL to notch to uR) should be at high Y
    cap_y = pts[notch, 1]
    print(f"    ✓ notch (cap peak) Y={cap_y:.4f} is max: {'✓' if cap_y == pts[:, 1].max() else '✗ WRONG'}")


# --- Dump all back_right vertices for topology analysis ---
print("\n  ── back_right all vertices ──")
if "back_right" in pieces:
    verts = pieces["back_right"].vertices_2d
    pts = np.array(verts)
    pw = pts[:, 0].max()
    ph = pts[:, 1].max()
    print(f"  pw={pw*100:.1f}cm  ph={ph*100:.1f}cm  n={len(verts)}")
    for i, v in enumerate(verts):
        marker = ""
        neck, inner_sh, outer_sh, underarm = get_top_landmarks(verts, "left")
        if i == inner_sh: marker = " ← inner_shoulder"
        elif i == outer_sh: marker = " ← outer_shoulder"
        elif i == underarm: marker = " ← underarm"
        elif i == neck: marker = " ← neck"
        print(f"    v{i:3d}: X={v[0]:.4f}  Y={v[1]:.4f}{marker}")

print()
