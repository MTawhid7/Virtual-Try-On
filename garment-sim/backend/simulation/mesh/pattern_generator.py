"""
Parametric pattern generator v2 — Blender-measurement-driven panels.

Creates 2 half-body panels (front + back) with:
  - Smooth Bézier armhole curves
  - Smooth Bézier neckline scoops
  - Straight side seams (stitch-compatible)
  - Generous ease for relaxed fit
  - Flat placement near body for simulation draping

Based on measurements extracted from the Blender mesh and the validated
mannequin_profile.json body analysis.

Usage:
    from simulation.mesh.pattern_generator import generate_tank_top
    pattern = generate_tank_top("data/bodies/mannequin_profile.json")
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


# ──────────────────────────────────────────────────────────
#  Bézier Helper
# ──────────────────────────────────────────────────────────

def _bezier3(P0, P1, P2, P3, n: int = 6):
    """Sample cubic Bézier at n points (excluding P0, including P3)."""
    pts = []
    for i in range(1, n + 1):
        t = i / n
        u = 1 - t
        x = u**3*P0[0] + 3*u**2*t*P1[0] + 3*u*t**2*P2[0] + t**3*P3[0]
        y = u**3*P0[1] + 3*u**2*t*P1[1] + 3*u*t**2*P2[1] + t**3*P3[1]
        pts.append([round(x, 5), round(y, 5)])
    return pts


# ──────────────────────────────────────────────────────────
#  Measurement Extraction
# ──────────────────────────────────────────────────────────

def _load_measurements(profile_path: str) -> dict:
    """Load body profile and compute all needed derived measurements."""
    with open(profile_path) as f:
        profile = json.load(f)
    lm = profile["landmarks"]

    return {
        "chest_circ": lm["chest_circumference"],
        "waist_circ": lm["waist_circumference"],
        "hip_circ": lm["hip_circumference"],
        "neck_circ": lm.get("neck_circumference", 0.506),
        "shoulder_width": lm["shoulder_torso_width"],
        "shoulder_y": lm["shoulder_y"],
        "armpit_y": lm["armpit_y"],
        "chest_y": lm["chest_y"],
        "waist_y": lm["waist_y"],
        "hip_y": lm["hip_y"],
        "neck_y": lm["neck_y"],
        "chest_z_front": lm.get("chest_z_front", 0.279),
        "chest_z_back": lm.get("chest_z_back", 0.034),
    }


# ──────────────────────────────────────────────────────────
#  Half-Body Panel Builder
# ──────────────────────────────────────────────────────────

def _build_half_panel(
    m: dict,
    face: str,
    ease_total: float = 0.14,
    hem_extension: float = 0.05,
) -> tuple[list[list[float]], dict[str, int]]:
    """
    Build a single half-body panel (covers full body width from side to side).

    The panel is CENTERED at x=0 in local space:
      x: [-hw, +hw] where hw = half-width
      z: [0, panel_height] (hemline to shoulder)

    Side seams are STRAIGHT vertical lines at x=±hw — this ensures
    _find_edge_particles can locate stitch vertices along the seam.

    Args:
        m: body measurements dict
        face: "front" or "back"
        ease_total: total circumferential ease in meters (default 14cm)
        hem_extension: how far below hip the hem extends

    Returns:
        (vertices_2d, landmarks) where vertices_2d is CCW polygon,
        landmarks maps names to vertex indices.
    """
    # --- Panel half-width (same at all heights for straight side seams) ---
    # Use chest as the widest measurement, add generous ease
    hw = (m["chest_circ"] + ease_total) / 4  # quarter-circ with ease
    # hw ≈ (1.021 + 0.14) / 4 ≈ 0.290m = 29cm per side

    # --- Heights relative to hemline (z=0) ---
    hem_y = m["hip_y"] - hem_extension
    z_underarm = m["armpit_y"] - hem_y
    z_shoulder = m["shoulder_y"] - hem_y

    # --- Neckline params ---
    neck_hw = m["neck_circ"] / 6       # ~8.4cm from center (half neckline width)
    if face == "front":
        neck_scoop = neck_hw + 0.03    # ~11.4cm deep scoop
    else:
        neck_scoop = 0.025             # 2.5cm shallow back

    z_neck = z_shoulder - neck_scoop

    # --- Shoulder strap ---
    strap_width = 0.045                # 4.5cm wide strap
    strap_inner = neck_hw              # inner edge at neckline width
    strap_outer = strap_inner + strap_width
    slope_drop = strap_width * np.tan(np.radians(3.0))  # 3° shoulder slope

    # --- Build polygon (counter-clockwise, centered) ---
    verts: list[list[float]] = []
    landmarks: dict[str, int] = {}

    def add(x, z, name=None):
        if name:
            landmarks[name] = len(verts)
        verts.append([round(float(x), 5), round(float(z), 5)])

    # ═══ 1. HEMLINE (bottom, left to right) ═══
    add(-hw, 0, "hem_left")
    add(+hw, 0, "hem_right")

    # ═══ 2. RIGHT SIDE SEAM (straight up to underarm) ═══
    add(+hw, z_underarm, "underarm_right")

    # ═══ 3. RIGHT ARMHOLE (Bézier curve, underarm → shoulder tip) ═══
    ah_height = z_shoulder - z_underarm
    P0 = (+hw, z_underarm)
    P3 = (+strap_outer, z_shoulder - slope_drop)
    # Control: leave underarm going straight up, arrive at shoulder going horizontal
    P1 = (+hw, z_underarm + ah_height * 0.55)
    P2 = (strap_outer + (hw - strap_outer) * 0.30, z_shoulder - slope_drop)
    for pt in _bezier3(P0, P1, P2, P3, n=7):
        verts.append(pt)
    add(+strap_outer, z_shoulder - slope_drop, "shoulder_right")

    # ═══ 4. RIGHT STRAP TOP ═══
    add(+strap_inner, z_shoulder, "strap_inner_right")

    # ═══ 5. FRONT NECKLINE (Bézier scoop, right strap → center) ═══
    P0 = (+strap_inner, z_shoulder)
    P3 = (0, z_neck)
    P1 = (+strap_inner, z_shoulder - neck_scoop * 0.55)
    P2 = (+strap_inner * 0.20, z_neck)
    for pt in _bezier3(P0, P1, P2, P3, n=5):
        verts.append(pt)
    add(0, z_neck, "neck_center")

    # ═══ 6. LEFT NECKLINE (mirror of right, center → left strap) ═══
    P0 = (0, z_neck)
    P3 = (-strap_inner, z_shoulder)
    P1 = (-strap_inner * 0.20, z_neck)
    P2 = (-strap_inner, z_shoulder - neck_scoop * 0.55)
    for pt in _bezier3(P0, P1, P2, P3, n=5):
        verts.append(pt)
    add(-strap_inner, z_shoulder, "strap_inner_left")

    # ═══ 7. LEFT STRAP TOP ═══
    add(-strap_outer, z_shoulder - slope_drop, "shoulder_left")

    # ═══ 8. LEFT ARMHOLE (mirror of right, shoulder → underarm) ═══
    P0 = (-strap_outer, z_shoulder - slope_drop)
    P3 = (-hw, z_underarm)
    P1 = (-(strap_outer + (hw - strap_outer) * 0.30), z_shoulder - slope_drop)
    P2 = (-hw, z_underarm + ah_height * 0.55)
    for pt in _bezier3(P0, P1, P2, P3, n=7):
        verts.append(pt)
    add(-hw, z_underarm, "underarm_left")

    # ═══ 9. LEFT SIDE SEAM (straight down, closing edge) ═══
    # The closing edge goes from underarm_left back to hem_left (implicit)
    # No intermediate vertices needed — it's a straight vertical line

    return verts, landmarks


# ──────────────────────────────────────────────────────────
#  Main Generator
# ──────────────────────────────────────────────────────────

def generate_tank_top(
    profile_path: str,
    fit: str = "relaxed",
    hem_extension: float = 0.05,
) -> dict[str, Any]:
    """
    Generate a complete tank top pattern from body measurements.

    Produces 2 half-body panels (front + back) with smooth Bézier
    armholes and necklines, flat placement, and stitch definitions
    for side seams + shoulder seams.

    Args:
        profile_path: Path to mannequin_profile.json
        fit: "fitted", "standard", or "relaxed"
        hem_extension: How far below hip the hemline extends (meters)

    Returns:
        Pattern dict compatible with panel_builder.build_garment_mesh()
    """
    ease_table = {
        "fitted":   0.06,   # 6cm total
        "standard": 0.10,   # 10cm total
        "relaxed":  0.14,   # 14cm total (generous)
    }
    ease = ease_table[fit]

    m = _load_measurements(profile_path)
    hem_y = m["hip_y"] - hem_extension

    # Build panels
    front_verts, front_lm = _build_half_panel(m, "front", ease, hem_extension)
    back_verts, back_lm = _build_half_panel(m, "back", ease, hem_extension)

    # Compute panel half-width for placement
    hw = max(v[0] for v in front_verts)

    # Flat placement positions — panels must be OUTSIDE full body extent
    # Body Z range: [0.031, 0.346] (includes nose). Generous clearance needed.
    front_z = 0.42    # ~7cm in front of body max Z (0.346)
    back_z = -0.05    # ~8cm behind body min Z (0.031)

    front_placement = {
        "position": [round(-hw, 5), round(hem_y, 5), round(front_z, 5)],
        "rotation_x_deg": -90,
        "rotation_y_deg": 0,
    }
    back_placement = {
        "position": [round(-hw, 5), round(hem_y, 5), round(back_z, 5)],
        "rotation_x_deg": -90,
        "rotation_y_deg": 0,
    }

    # Shift vertices so x goes from 0 to 2*hw (panel_builder expects non-negative x)
    front_shifted = [[round(v[0] + hw, 5), v[1]] for v in front_verts]
    back_shifted = [[round(v[0] + hw, 5), v[1]] for v in back_verts]

    # Stitch definitions
    stitches = [
        {
            "comment": "Right side seam (front right → back right)",
            "panel_a": "front",
            "edge_a": [front_lm["hem_right"], front_lm["underarm_right"]],
            "panel_b": "back",
            "edge_b": [back_lm["hem_right"], back_lm["underarm_right"]],
        },
        {
            "comment": "Left side seam (front left → back left)",
            "panel_a": "front",
            "edge_a": [front_lm["underarm_left"], front_lm["hem_left"]],
            "panel_b": "back",
            "edge_b": [back_lm["underarm_left"], back_lm["hem_left"]],
        },
        {
            "comment": "Right shoulder seam",
            "panel_a": "front",
            "edge_a": [front_lm["shoulder_right"], front_lm["strap_inner_right"]],
            "panel_b": "back",
            "edge_b": [back_lm["strap_inner_right"], back_lm["shoulder_right"]],
        },
        {
            "comment": "Left shoulder seam",
            "panel_a": "front",
            "edge_a": [front_lm["strap_inner_left"], front_lm["shoulder_left"]],
            "panel_b": "back",
            "edge_b": [back_lm["shoulder_left"], back_lm["strap_inner_left"]],
        },
    ]

    # Print dimensions for verification
    total_width = hw * 2
    panel_height = max(v[1] for v in front_verts)
    print(f"\n  ═══ Generated Tank Top Pattern ═══")
    print(f"  Panel width:      {total_width*100:.1f}cm  (half-width: {hw*100:.1f}cm)")
    print(f"  Panel height:     {panel_height*100:.1f}cm")
    print(f"  Total ease:       {ease*100:.0f}cm")
    print(f"  Both panels:      {total_width*2*100:.1f}cm vs body chest {m['chest_circ']*100:.1f}cm")
    print(f"  Front vertices:   {len(front_shifted)}")
    print(f"  Back vertices:    {len(back_shifted)}")
    print(f"  Stitches:         {len(stitches)}")
    print(f"  Front placement:  Z={front_z:.3f}")
    print(f"  Back placement:   Z={back_z:.3f}")
    print(f"  Stitch gap:       {(front_z - back_z)*100:.1f}cm")

    return {
        "name": "TankTop_v2",
        "panels": [
            {
                "id": "front",
                "vertices_2d": front_shifted,
                "placement": front_placement,
            },
            {
                "id": "back",
                "vertices_2d": back_shifted,
                "placement": back_placement,
            },
        ],
        "stitches": stitches,
        "fabric": "cotton",
        "metadata": {
            "generator": "pattern_generator_v2",
            "fit": fit,
            "ease_total_cm": ease * 100,
            "half_width_cm": hw * 100,
        },
    }
