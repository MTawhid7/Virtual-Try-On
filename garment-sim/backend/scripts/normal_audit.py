"""
Face Winding / Normal Direction Audit (Root Cause G).

Checks whether the cylindrical wrap for the right sleeve reverses face
winding order, causing bending constraint normals to point inward.
If > 10% of right sleeve faces point inward while left sleeve is > 95%
outward, bending forces push the right sleeve concave where they should
push convex.

Run from backend/:
  python -m scripts.normal_audit
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.mesh.panel_builder import (
    _apply_placement,
    _cylindrical_wrap_sleeve,
    build_garment_mesh,
)
from simulation.mesh.triangulation import triangulate_panel

PATTERN = Path(__file__).resolve().parents[1] / "data/patterns/tshirt.json"
TARGET_EDGE = 0.020


def audit_sleeve_normals(spec, sleeve_id: str, arm_cx: float, arm_cz: float):
    panels = {p["id"]: p for p in spec["panels"]}
    pspec = panels[sleeve_id]

    panel = triangulate_panel(pspec["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
    world_pos = _apply_placement(panel.positions, pspec["placement"])
    world_pos_wrapped = _cylindrical_wrap_sleeve(world_pos, pspec["placement"])

    faces = panel.faces  # (F, 3)
    v0 = world_pos_wrapped[faces[:, 0]]
    v1 = world_pos_wrapped[faces[:, 1]]
    v2 = world_pos_wrapped[faces[:, 2]]

    # Face normals (CCW → right-hand rule)
    face_normals = np.cross(v1 - v0, v2 - v0)
    norms = np.linalg.norm(face_normals, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-9)
    face_normals = face_normals / norms  # unit normals

    # Face centroids
    centroids = (v0 + v1 + v2) / 3.0

    # Outward radial direction from arm center (in XZ plane)
    radial = centroids - np.array([arm_cx, 0.0, arm_cz])
    radial[:, 1] = 0.0  # project to XZ plane
    radial_norms = np.linalg.norm(radial, axis=1, keepdims=True)
    radial_norms = np.maximum(radial_norms, 1e-9)
    radial_unit = radial / radial_norms

    # dot(face_normal, radial) > 0 → outward-facing
    dots = np.sum(face_normals * radial_unit, axis=1)
    outward_mask = dots > 0
    n_outward = int(outward_mask.sum())
    n_total = len(faces)
    outward_frac = n_outward / n_total

    print(f"\n  {sleeve_id}  ({n_total} faces, arm_center=({arm_cx},{arm_cz}))")
    print(f"    Outward-facing: {n_outward}/{n_total} ({outward_frac*100:.1f}%)")
    print(f"    Inward-facing:  {n_total-n_outward}/{n_total} ({(1-outward_frac)*100:.1f}%)")
    print(f"    Mean dot(normal, radial): {dots.mean():.3f}  (ideal=+1.0)")

    if outward_frac < 0.90:
        print(f"    ⚠ FLAG: outward fraction {outward_frac*100:.1f}% < 90% (Root cause G)")
        print(f"    Fix: flip face winding after cylindrical wrap in build_garment_mesh():")
        print(f"         faces[:, [1, 2]] = faces[:, [2, 1]]  (swap columns 1 and 2)")
    else:
        print(f"    OK: {outward_frac*100:.1f}% outward → winding is correct")

    # Show a few inward-facing faces for inspection
    inward_indices = np.where(~outward_mask)[0]
    if len(inward_indices) > 0:
        print(f"    Sample inward faces (up to 5):")
        for fi in inward_indices[:5]:
            c = centroids[fi]
            print(f"      face {fi}: centroid=({c[0]:+.3f}, {c[1]:.3f}, {c[2]:+.3f})  "
                  f"dot={dots[fi]:.3f}")

    return outward_frac


def audit_flat_panel_normals(spec):
    """
    Also check front and back panel normals — the back panel has rotation_y=180°
    which can flip winding. Face normals should mostly point away from the body
    (front panel: +Z direction; back panel: -Z direction).
    """
    print("\n  --- Flat panel winding check ---")
    panels = {p["id"]: p for p in spec["panels"]}

    for panel_id, expected_sign, axis_label in [
        ("front", +1, "+Z"),
        ("back",  -1, "-Z"),
    ]:
        pspec = panels[panel_id]
        panel = triangulate_panel(pspec["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
        world_pos = _apply_placement(panel.positions, pspec["placement"])

        faces = panel.faces
        v0 = world_pos[faces[:, 0]]
        v1 = world_pos[faces[:, 1]]
        v2 = world_pos[faces[:, 2]]
        face_normals = np.cross(v1 - v0, v2 - v0)
        z_component = face_normals[:, 2]  # Z component of normals

        n_correct = int((z_component * expected_sign > 0).sum())
        n_total = len(faces)
        frac = n_correct / n_total

        flag = "  ⚠ FLAG <90%" if frac < 0.90 else "  OK"
        print(f"  {panel_id} panel: {n_correct}/{n_total} ({frac*100:.1f}%) normals toward {axis_label}{flag}")


def main():
    with open(PATTERN) as f:
        spec = json.load(f)

    print("\nFACE WINDING / NORMAL DIRECTION AUDIT")
    print(f"Pattern: {PATTERN.name}  target_edge={TARGET_EDGE}m")
    print("=" * 60)

    r_frac = audit_sleeve_normals(spec, "sleeve_right", arm_cx=+0.25, arm_cz=0.12)
    l_frac = audit_sleeve_normals(spec, "sleeve_left",  arm_cx=-0.25, arm_cz=0.12)

    audit_flat_panel_normals(spec)

    print()
    print("=" * 60)
    if r_frac < 0.90 and l_frac >= 0.90:
        print("⚠ Root cause G CONFIRMED: right sleeve winding is inverted vs left.")
        print("  Bending forces on right sleeve push concave rather than convex.")
    elif r_frac < 0.90 and l_frac < 0.90:
        print("⚠ Both sleeves have winding issues — may be a global formula problem.")
    else:
        print("✓ Both sleeves have correct outward-facing normals (>90%).")
        print("  Root cause G ruled out.")
    print("=" * 60)


if __name__ == "__main__":
    main()
