"""
Stitch Debug — prints each stitch definition with arc step counts and vertex positions.

Shows exactly which boundary arc each stitch edge covers, making it easy to spot
wrong edges (e.g. side seam accidentally including the armhole arc).

Usage:
    python -m scripts.debug_stitches
"""
import os
import sys
import json
import numpy as np

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

JSON_PATH = os.path.join(backend_dir, "data/patterns/polo_shirt.json")

with open(JSON_PATH) as f:
    pattern = json.load(f)

# Build panel vertex lookup
panels = {p["id"]: np.array(p["vertices_2d"]) for p in pattern["panels"]}

def arc_steps(n, v_start, v_end):
    d_fwd = (v_end - v_start) % n
    d_bwd = (v_start - v_end) % n
    if d_fwd <= d_bwd:
        return d_fwd + 1, "forward"
    else:
        return d_bwd + 1, "backward"

print("═" * 70)
print("  Stitch Arc Diagnostics")
print("═" * 70)

for i, s in enumerate(pattern["stitches"]):
    pa, ea = s["panel_a"], s["edge_a"]
    pb, eb = s["panel_b"], s["edge_b"]
    comment = s.get("comment", f"stitch {i}")

    pts_a = panels[pa]
    pts_b = panels[pb]
    na, nb = len(pts_a), len(pts_b)

    steps_a, dir_a = arc_steps(na, ea[0], ea[1])
    steps_b, dir_b = arc_steps(nb, eb[0], eb[1])

    pa_start = pts_a[ea[0]]
    pa_end   = pts_a[ea[1]]
    pb_start = pts_b[eb[0]]
    pb_end   = pts_b[eb[1]]

    # Arc length in cm (sum of edge segments along the boundary walk)
    def arc_length_cm(pts, n, v_start, v_end):
        d_fwd = (v_end - v_start) % n
        d_bwd = (v_start - v_end) % n
        if d_fwd <= d_bwd:
            indices = [(v_start + k) % n for k in range(d_fwd + 1)]
        else:
            indices = [(v_start - k) % n for k in range(d_bwd + 1)]
        total = 0.0
        for j in range(len(indices) - 1):
            dx = pts[indices[j+1], 0] - pts[indices[j], 0]
            dy = pts[indices[j+1], 1] - pts[indices[j], 1]
            total += np.sqrt(dx*dx + dy*dy)
        return total * 100  # m → cm

    len_a = arc_length_cm(pts_a, na, ea[0], ea[1])
    len_b = arc_length_cm(pts_b, nb, eb[0], eb[1])

    print(f"\n  [{i}] {comment}")
    print(f"    {pa} edge {ea}  ({na} verts total)")
    print(f"         v{ea[0]}: X={pa_start[0]*100:.1f}cm  Y={pa_start[1]*100:.1f}cm")
    print(f"         v{ea[1]}: X={pa_end[0]*100:.1f}cm  Y={pa_end[1]*100:.1f}cm")
    print(f"         Arc: {steps_a} verts ({dir_a}), length={len_a:.1f}cm")
    print(f"    {pb} edge {eb}  ({nb} verts total)")
    print(f"         v{eb[0]}: X={pb_start[0]*100:.1f}cm  Y={pb_start[1]*100:.1f}cm")
    print(f"         v{eb[1]}: X={pb_end[0]*100:.1f}cm  Y={pb_end[1]*100:.1f}cm")
    print(f"         Arc: {steps_b} verts ({dir_b}), length={len_b:.1f}cm")

    # Sanity flags
    ratio = max(steps_a, steps_b) / max(min(steps_a, steps_b), 1)
    if ratio > 3:
        print(f"    ⚠ MISMATCH: {steps_a} vs {steps_b} verts (ratio {ratio:.1f}x) — will subsample heavily")
    if steps_a > 25 or steps_b > 25:
        print(f"    ⚠ LARGE ARC: check this isn't accidentally including armhole/neckline")

print()
