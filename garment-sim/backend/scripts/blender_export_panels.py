"""
Blender export script — extracts panel Bézier curve outlines as pattern JSON.

Run from INSIDE Blender:
  1. Open Blender's Scripting workspace
  2. Open this file
  3. Click "Run Script"

Prerequisites:
  - Create Bézier curve objects named 'front_right' and 'back_right' in the scene
  - Draw panel outlines in the XZ plane (X = width toward side, Z = height upward)
  - The script auto-mirrors right panels to create left panels

Output:
  backend/data/patterns/tank_top_blender.json

Configuration:
  Edit the constants below to match your project paths and body measurements.
"""

import bpy
import json
import os

# ──────────────────────────────────────────────────────────
#  CONFIGURATION — Edit these to match your setup
# ──────────────────────────────────────────────────────────

# Output path  (absolute path to your project)
OUTPUT_PATH = "/Users/tawhid/Documents/garment-sim/backend/data/patterns/tank_top_blender.json"

# How many intermediate points to sample between each Bézier control point
# Higher = smoother edges, more triangles. 4 is good, 6-8 for ultra-smooth.
SAMPLES_PER_SEGMENT = 4

# Flat placement parameters (from mannequin_profile.json)
FRONT_Z = 0.32     # 4cm in front of body front surface (Z=0.279)
BACK_Z  = 0.00     # 3cm behind body back surface (Z=0.034)
HEM_Y   = 0.753    # Y world position of hemline (hip_y - 5cm)
CENTER_GAP = 0.01  # 1cm gap at center seam for stitch closure

# Panel names in the Blender scene
RIGHT_PANELS = {
    "front_right": {"object_name": "front_right", "face": "front"},
    "back_right":  {"object_name": "back_right",  "face": "back"},
}


# ──────────────────────────────────────────────────────────
#  CURVE SAMPLING
# ──────────────────────────────────────────────────────────

def sample_bezier_spline(spline, samples_per_seg=4):
    """
    Sample a Bézier spline into a list of (x, z) points.
    
    Iterates through control points, adding:
      - The control point itself
      - N intermediate samples between each consecutive pair
    
    For cyclic (closed) splines, the last-to-first segment is also sampled.
    """
    points = []
    bp_list = spline.bezier_points
    n = len(bp_list)
    
    if n == 0:
        return points
    
    segments = n if spline.use_cyclic_u else (n - 1)
    
    for i in range(segments):
        bp = bp_list[i]
        bp_next = bp_list[(i + 1) % n]
        
        # Add the control point
        points.append((round(bp.co.x, 5), round(bp.co.z, 5)))
        
        # Sample intermediate points along the Bézier segment
        p0 = bp.co
        p1 = bp.handle_right
        p2 = bp_next.handle_left
        p3 = bp_next.co
        
        for j in range(1, samples_per_seg):
            t = j / samples_per_seg
            u = 1 - t
            x = u**3*p0.x + 3*u**2*t*p1.x + 3*u*t**2*p2.x + t**3*p3.x
            z = u**3*p0.z + 3*u**2*t*p1.z + 3*u*t**2*p2.z + t**3*p3.z
            points.append((round(x, 5), round(z, 5)))
    
    return points


def mirror_panel_vertices(vertices, panel_width):
    """Mirror a right panel to create a left panel (negate X, reverse order)."""
    mirrored = [(round(panel_width - x, 5), z) for x, z in reversed(vertices)]
    return mirrored


# ──────────────────────────────────────────────────────────
#  MAIN EXPORT
# ──────────────────────────────────────────────────────────

def export_panels():
    pattern = {
        "name": "TankTop_Blender",
        "panels": [],
        "stitches": [],
        "fabric": "cotton",
    }
    
    for panel_id, pdef in RIGHT_PANELS.items():
        obj_name = pdef["object_name"]
        obj = bpy.data.objects.get(obj_name)
        
        if obj is None:
            print(f"⚠ Object '{obj_name}' not found in scene — skipping")
            continue
        
        if obj.type != 'CURVE':
            print(f"⚠ '{obj_name}' is not a Curve object (type={obj.type}) — skipping")
            continue
        
        if len(obj.data.splines) == 0:
            print(f"⚠ '{obj_name}' has no splines — skipping")
            continue
        
        spline = obj.data.splines[0]
        
        if spline.type != 'BEZIER':
            print(f"⚠ '{obj_name}' spline is not Bézier (type={spline.type}) — skipping")
            continue
        
        # Sample the curve
        vertices = sample_bezier_spline(spline, SAMPLES_PER_SEGMENT)
        
        if len(vertices) < 3:
            print(f"⚠ '{obj_name}' produced only {len(vertices)} vertices — skipping")
            continue
        
        # Panel width = max X coordinate
        pw = max(v[0] for v in vertices)
        
        # Determine Z placement
        is_front = pdef["face"] == "front"
        z_pos = FRONT_Z if is_front else BACK_Z
        
        # --- Right panel ---
        pattern["panels"].append({
            "id": panel_id,
            "vertices_2d": [list(v) for v in vertices],
            "placement": {
                "position": [CENTER_GAP, HEM_Y, z_pos],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            },
        })
        
        # --- Left panel (mirror) ---
        left_id = panel_id.replace("_right", "_left")
        left_verts = mirror_panel_vertices(vertices, pw)
        
        pattern["panels"].append({
            "id": left_id,
            "vertices_2d": [list(v) for v in left_verts],
            "placement": {
                "position": [-(pw + CENTER_GAP), HEM_Y, z_pos],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            },
        })
        
        print(f"✓ {panel_id}: {len(vertices)} vertices (pw={pw:.3f}m)")
        print(f"  → {left_id}: {len(left_verts)} vertices (mirrored)")
    
    # --- Print vertex index map for stitch definition ---
    print("\n" + "=" * 60)
    print("  VERTEX INDEX MAP")
    print("  Use these to define seam edges in the JSON 'stitches' array.")
    print("=" * 60)
    
    for p in pattern["panels"]:
        print(f"\n  Panel '{p['id']}':")
        for i, v in enumerate(p["vertices_2d"]):
            marker = ""
            if i == 0:
                marker = "  ← FIRST (hem center or side)"
            elif i == len(p["vertices_2d"]) - 1:
                marker = "  ← LAST"
            print(f"    [{i:3d}] ({v[0]:8.5f}, {v[1]:8.5f}){marker}")
    
    # Save
    os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)
    with open(OUTPUT_PATH, "w") as f:
        json.dump(pattern, f, indent=2)
    
    print(f"\n{'=' * 60}")
    print(f"  Saved: {OUTPUT_PATH}")
    print(f"  Panels: {len(pattern['panels'])}")
    print(f"\n  ⚠ NEXT STEP: Add 'stitches' array to the JSON file!")
    print(f"    See the vertex index map above to identify seam edges.")
    print(f"{'=' * 60}")


# Run
export_panels()
