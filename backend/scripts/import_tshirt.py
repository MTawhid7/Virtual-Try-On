"""
T-Shirt DXF Importer — handles CLO3D AAMA exports with full-width panels.

Extracts 4 panels from tshirt_new.dxf:
  Body_Front_M  — full-width front with V-neck  (~51cm × 70cm)
  Body_Back_M   — full-width back with round neck (~51cm × 73cm)
  Sleeves_M     — cap sleeve (right), ~41cm × 17cm
  Sleeves_401164_M — cap sleeve (left, mirrored)

Stitch topology:
  - Left/right side seams (front ↔ back)
  - Left/right shoulder seams (front ↔ back)
  - Left/right armhole (sleeve cap ↔ front + back armhole curves)
  - Left/right sleeve underarm seam (sleeve self-stitch)

Usage:
    python -m scripts.import_tshirt [--dxf PATH] [--output PATH]
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from scripts.import_dxf import (
    PanelPiece,
    extract_pieces,
    normalize_piece,
    mirror_piece,
)


# ──────────────────────────────────────────────────────────
#  Landmark detection for full-width panels
# ──────────────────────────────────────────────────────────

def body_landmarks(verts: list[list[float]]) -> dict[str, int]:
    """
    Detect seam-edge vertex indices for a full-width t-shirt body panel.

    The panel spans full garment width with:
    - Hem at Y≈0 (bottom)
    - Shoulders at Y≈max (top)
    - Side seams + armhole curves on the left (X≈0) and right (X≈pw) edges

    Returns dict with keys:
      left_hem, right_hem           — bottom corners
      left_underarm, right_underarm — side seam tops (extremal X in lower 75%)
      left_arm_corner, right_arm_corner — top of armhole curve (highest X≈edge vertex)
      left_shoulder, right_shoulder — highest Y on each side of centre
    """
    pts = np.array(verts)
    n = len(pts)
    pw, ph = pts[:, 0].max(), pts[:, 1].max()
    cx = pw / 2

    # Hem corners
    hem = [i for i in range(n) if pts[i, 1] < ph * 0.05]
    left_hem  = int(min(hem, key=lambda i: pts[i, 0]))
    right_hem = int(max(hem, key=lambda i: pts[i, 0]))

    # Underarm = extremal X in the lower 75% of panel height
    # (restricts to below the armhole curve which curves inward at the top)
    lower = [i for i in range(n) if pts[i, 1] < ph * 0.75]
    left_underarm  = int(min(lower, key=lambda i: pts[i, 0]))
    right_underarm = int(max(lower, key=lambda i: pts[i, 0]))

    # Armhole-to-shoulder corner: highest Y vertex near each lateral edge
    l_edge = [i for i in range(n) if pts[i, 0] < pw * 0.05]
    r_edge = [i for i in range(n) if pts[i, 0] > pw * 0.95]
    left_arm_corner  = int(max(l_edge, key=lambda i: pts[i, 1]))
    right_arm_corner = int(max(r_edge, key=lambda i: pts[i, 1]))

    # Shoulder peaks: highest Y on each side of the panel centre
    l_pts = [i for i in range(n) if pts[i, 0] < cx]
    r_pts = [i for i in range(n) if pts[i, 0] >= cx]
    left_shoulder  = int(max(l_pts, key=lambda i: pts[i, 1]))
    right_shoulder = int(max(r_pts, key=lambda i: pts[i, 1]))

    return {
        "left_hem": left_hem, "right_hem": right_hem,
        "left_underarm": left_underarm, "right_underarm": right_underarm,
        "left_arm_corner": left_arm_corner, "right_arm_corner": right_arm_corner,
        "left_shoulder": left_shoulder, "right_shoulder": right_shoulder,
    }


def sleeve_landmarks(verts: list[list[float]]) -> dict[str, int]:
    """
    Detect seam-edge vertex indices for a cap sleeve panel.

    The sleeve cap (highest Y) attaches to the armhole.
    The cuff (lowest Y) is the open hem.
    The left/right edges stitch together to form the underarm seam.

    Returns dict with keys:
      cap_crown   — highest Y (notch dividing front/back cap halves)
      underarm_L  — leftmost X  (cap left edge meets cuff left)
      underarm_R  — rightmost X (cap right edge meets cuff right)
      cuff_L      — lowest Y on the left side  (cuff left corner)
      cuff_R      — lowest Y on the right side (cuff right corner)
    """
    pts = np.array(verts)
    n = len(pts)
    pw, ph = pts[:, 0].max(), pts[:, 1].max()

    cap_crown  = int(np.argmax(pts[:, 1]))
    underarm_L = int(np.argmin(pts[:, 0]))
    underarm_R = int(np.argmax(pts[:, 0]))

    cuff = [i for i in range(n) if pts[i, 1] < ph * 0.02]
    cuff_L = int(min(cuff, key=lambda i: pts[i, 0]))
    cuff_R = int(max(cuff, key=lambda i: pts[i, 0]))

    return {
        "cap_crown": cap_crown,
        "underarm_L": underarm_L, "underarm_R": underarm_R,
        "cuff_L": cuff_L, "cuff_R": cuff_R,
    }


# ──────────────────────────────────────────────────────────
#  Placement
# ──────────────────────────────────────────────────────────

def compute_tshirt_placements(
    pieces: dict[str, PanelPiece],
    body_profile: dict,
) -> dict[str, dict]:
    """
    Compute flat 3D placement for each t-shirt panel.

    Front: centered at X=0, in front of body (Z=front_z), faces forward
    Back:  centered at X=0, behind body (Z=back_z), rotated 180° (faces inward)
    Sleeves: placed to the sides at shoulder height
    """
    # ─────────────── ADJUSTMENT GUIDE ───────────────
    # POSITION: [X, Y, Z] (meters).
    #   X+: Left (your right), Y+: UP, Z+: FORWARD.
    # ROTATION:
    #   rotation_x: Tilt (-90 = standing).
    #   rotation_y: Spin (0 = front, 180 = back).
    # ────────────────────────────────────────────────

    lm = body_profile["landmarks"]
    hem_y      = lm["hip_y"] - 0.05
    shoulder_y = lm["shoulder_y"]
    # ADJUST THESE for front/back panels
    front_z = 0.30    # Close the gaping void in front
    back_z  = 0.02    # Push back to avoid back-collision

    # ADJUST THESE for sleeve panels
    sleeve_x_dist   = 0.25   # How far out to the sides
    sleeve_z_base   = 0.12   # Depth: Matching body center (approx 0.16)
    sleeve_y_offset = 0.01   # Vertical lift relative to shoulder height

    placements: dict[str, dict] = {}

    for name, piece in pieces.items():
        pw = max(v[0] for v in piece.vertices_2d)
        ph = max(v[1] for v in piece.vertices_2d)

        if name == "front":
            placements[name] = {
                "position": [round(-pw / 2, 5), round(hem_y, 5), round(front_z, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }

        elif name == "back":
            placements[name] = {
                "position": [round(pw / 2, 5), round(hem_y, 5), round(back_z, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 180,
            }

        elif name == "sleeve_right":
            # sleeve_z_base is now the target center of the cylinder
            placements[name] = {
                "position": [round(sleeve_x_dist, 5), round(shoulder_y - ph + sleeve_y_offset, 5), round(sleeve_z_base, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }

        elif name == "sleeve_left":
            placements[name] = {
                "position": [round(-sleeve_x_dist, 5), round(shoulder_y - ph + sleeve_y_offset, 5), round(sleeve_z_base, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }

    return placements


# ──────────────────────────────────────────────────────────
#  Stitch definitions
# ──────────────────────────────────────────────────────────

def compute_tshirt_stitches(pieces: dict[str, PanelPiece]) -> list[dict]:
    """
    Compute all seam stitch definitions for the t-shirt.

    Seam map (world-space):
      Right side seam:    front local-right ↔ back local-left  (world right)
      Left side seam:     front local-left  ↔ back local-right (world left)
      Right shoulder:     front local-right ↔ back local-left  (world right)
      Left shoulder:      front local-left  ↔ back local-right (world left)
      Right sleeve cap:   sleeve_right front-half ↔ front right armhole
                          sleeve_right back-half  ↔ back local-left armhole
      Left sleeve cap:    sleeve_left  front-half ↔ front left armhole
                          sleeve_left  back-half  ↔ back local-right armhole
      Sleeve underarm:    sleeve left-edge ↔ sleeve right-edge (self-stitch)

    Note on back panel orientation:
      After rotation_y=180° the back panel's LOCAL left side appears on the
      WORLD RIGHT and vice versa.  So:
        world-right stitch → back local-left edge indices
        world-left stitch  → back local-right edge indices
    """
    fr = body_landmarks(pieces["front"].vertices_2d)
    br = body_landmarks(pieces["back"].vertices_2d)
    sr = sleeve_landmarks(pieces["sleeve_right"].vertices_2d)
    sl = sleeve_landmarks(pieces["sleeve_left"].vertices_2d)

    return [
        # ── Side seams ──────────────────────────────────────────
        {
            "comment": "Right side seam (front local-right ↔ back local-left)",
            "panel_a": "front", "edge_a": [fr["right_hem"],    fr["right_underarm"]],
            "panel_b": "back",  "edge_b": [br["left_hem"],     br["left_underarm"]],
        },
        {
            "comment": "Left side seam (front local-left ↔ back local-right)",
            "panel_a": "front", "edge_a": [fr["left_hem"],     fr["left_underarm"]],
            "panel_b": "back",  "edge_b": [br["right_hem"],    br["right_underarm"]],
        },

        # ── Shoulder seams ───────────────────────────────────────
        {
            "comment": "Right shoulder seam (front local-right ↔ back local-left)",
            "panel_a": "front", "edge_a": [fr["right_arm_corner"], fr["right_shoulder"]],
            "panel_b": "back",  "edge_b": [br["left_arm_corner"],  br["left_shoulder"]],
        },
        {
            "comment": "Left shoulder seam (front local-left ↔ back local-right)",
            "panel_a": "front", "edge_a": [fr["left_arm_corner"],  fr["left_shoulder"]],
            "panel_b": "back",  "edge_b": [br["right_arm_corner"], br["right_shoulder"]],
        },

        # ── Right sleeve (rotation_y=90°) ────────────────────────
        # With rotation_y=90°, local-x maps to -world_z:
        #   underarm_R (local x=pw) → world_z ≈ back_z = 0.04  → near BACK armhole
        #   underarm_L (local x=0)  → world_z ≈ front_z = 0.45 → near FRONT armhole
        # So the cap halves are SWAPPED vs the rotation_y=0 case.
        {
            "comment": "Right sleeve cap back-half → back local-left armhole",
            "panel_a": "sleeve_right", "edge_a": [sr["underarm_R"], sr["cap_crown"]],
            "panel_b": "back",         "edge_b": [br["left_underarm"], br["left_arm_corner"]],
        },
        {
            "comment": "Right sleeve cap front-half → front right armhole",
            "panel_a": "sleeve_right", "edge_a": [sr["cap_crown"],  sr["underarm_L"]],
            "panel_b": "front",        "edge_b": [fr["right_underarm"], fr["right_arm_corner"]],
        },
        {
            "comment": "Right sleeve underarm seam (self)",
            "panel_a": "sleeve_right", "edge_a": [sr["cuff_L"], sr["underarm_L"]],
            "panel_b": "sleeve_right", "edge_b": [sr["cuff_R"], sr["underarm_R"]],
        },

        # ── Left sleeve (rotation_y=0, same cylindrical wrap as right) ──────────
        # With rotation_y=0, cylindrical wrap uses angle = 2π*u_norm + π/2:
        #   underarm_L (u_norm=0) and underarm_R (u_norm=1) → inner armpit (Z≈0.05)
        #   u_norm=0.25 → Z = arm_cz + r (FRONT) ← underarm_L→cap_crown is FRONT half
        #   u_norm=0.75 → Z = arm_cz - r (BACK)  ← cap_crown→underarm_R is BACK half
        {
            "comment": "Left sleeve cap front-half → front left armhole",
            "panel_a": "sleeve_left", "edge_a": [sl["underarm_L"], sl["cap_crown"]],
            "panel_b": "front",       "edge_b": [fr["left_underarm"],  fr["left_arm_corner"]],
        },
        {
            "comment": "Left sleeve cap back-half → back local-right armhole",
            "panel_a": "sleeve_left", "edge_a": [sl["cap_crown"],  sl["underarm_R"]],
            "panel_b": "back",        "edge_b": [br["right_arm_corner"], br["right_underarm"]],
        },
        {
            "comment": "Left sleeve underarm seam (self)",
            "panel_a": "sleeve_left", "edge_a": [sl["cuff_L"], sl["underarm_L"]],
            "panel_b": "sleeve_left", "edge_b": [sl["cuff_R"], sl["underarm_R"]],
        },
    ]


# ──────────────────────────────────────────────────────────
#  Main pipeline
# ──────────────────────────────────────────────────────────

def import_tshirt(
    dxf_path: str,
    profile_path: str,
    output_path: str | None = None,
) -> dict:
    """
    Import a CLO3D t-shirt DXF and produce a pattern JSON.

    Block names expected in DXF:
      Body_Front_M, Body_Back_M, Sleeves_M, Sleeves_401164_M

    Returns a pattern dict compatible with build_garment_mesh().
    """
    print("═" * 60)
    print("  T-Shirt DXF → Pattern JSON Importer")
    print("═" * 60)
    print(f"\n  Source: {os.path.basename(dxf_path)}")

    # --- 1. Extract raw pieces ---
    raw_pieces = extract_pieces(dxf_path, [
        "Body_Front_M",
        "Body_Back_M",
        "Sleeves_M",
        "Sleeves_401164_M",
    ])

    print(f"  Extracted {len(raw_pieces)} pieces:")
    for p in raw_pieces:
        print(f"    • {p.name}: {p.width_cm:.1f}cm × {p.height_cm:.1f}cm "
              f"({len(p.outline_mm)} pts)")

    # --- 2. Normalize (mm → m, translate to origin) ---
    for p in raw_pieces:
        normalize_piece(p)

    # --- 3. Assign panel roles ---
    # The DXF has one front, one back, and two sleeve pieces.
    # We use Sleeves_M as the right sleeve and mirror it for the left.
    pieces: dict[str, PanelPiece] = {}
    sleeve_raw: PanelPiece | None = None

    for p in raw_pieces:
        name_lower = p.name.lower()
        if "front" in name_lower:
            p.name = "front"
            pieces["front"] = p
        elif "back" in name_lower:
            p.name = "back"
            pieces["back"] = p
        elif "sleeves_m" == (p.name.replace(" ", "_").lower()):
            # Primary sleeve → right side
            p.name = "sleeve_right"
            pieces["sleeve_right"] = p
            sleeve_raw = p
        # Sleeves_401164 is the duplicate; skip — we mirror from Sleeves_M

    if sleeve_raw is None:
        # Fallback: pick the first sleeve-named piece
        for p in raw_pieces:
            if "sleeve" in p.name.lower():
                p.name = "sleeve_right"
                pieces["sleeve_right"] = p
                sleeve_raw = p
                break

    if "sleeve_right" in pieces:
        left_sleeve = mirror_piece(pieces["sleeve_right"], "sleeve_left")
        pieces["sleeve_left"] = left_sleeve

    print(f"\n  After assignment: {len(pieces)} panels")
    for name, p in pieces.items():
        pts = np.array(p.vertices_2d)
        pw, ph = pts[:, 0].max(), pts[:, 1].max()
        print(f"    {name}: {pw*100:.1f}cm × {ph*100:.1f}cm ({len(p.vertices_2d)} verts)")

    # --- 4. Load body profile for placement ---
    with open(profile_path) as f:
        body_profile = json.load(f)

    # --- 5. Compute placements ---
    placements = compute_tshirt_placements(pieces, body_profile)

    # --- 6. Print landmark indices for verification ---
    print("\n  Landmarks:")
    fr = body_landmarks(pieces["front"].vertices_2d)
    br = body_landmarks(pieces["back"].vertices_2d)
    sr = sleeve_landmarks(pieces["sleeve_right"].vertices_2d)
    sl = sleeve_landmarks(pieces["sleeve_left"].vertices_2d)

    fpts = np.array(pieces["front"].vertices_2d)
    bpts = np.array(pieces["back"].vertices_2d)
    srpts = np.array(pieces["sleeve_right"].vertices_2d)
    slpts = np.array(pieces["sleeve_left"].vertices_2d)

    for lname, idx, pts in [
        ("front.left_hem",        fr["left_hem"],        fpts),
        ("front.right_hem",       fr["right_hem"],       fpts),
        ("front.left_underarm",   fr["left_underarm"],   fpts),
        ("front.right_underarm",  fr["right_underarm"],  fpts),
        ("front.left_arm_corner", fr["left_arm_corner"], fpts),
        ("front.right_arm_corner",fr["right_arm_corner"],fpts),
        ("front.left_shoulder",   fr["left_shoulder"],   fpts),
        ("front.right_shoulder",  fr["right_shoulder"],  fpts),
        ("back.left_hem",         br["left_hem"],        bpts),
        ("back.right_hem",        br["right_hem"],       bpts),
        ("back.left_underarm",    br["left_underarm"],   bpts),
        ("back.right_underarm",   br["right_underarm"],  bpts),
        ("back.left_arm_corner",  br["left_arm_corner"], bpts),
        ("back.right_arm_corner", br["right_arm_corner"],bpts),
        ("back.left_shoulder",    br["left_shoulder"],   bpts),
        ("back.right_shoulder",   br["right_shoulder"],  bpts),
        ("sleeve_r.cap_crown",    sr["cap_crown"],       srpts),
        ("sleeve_r.underarm_L",   sr["underarm_L"],      srpts),
        ("sleeve_r.underarm_R",   sr["underarm_R"],      srpts),
        ("sleeve_r.cuff_L",       sr["cuff_L"],          srpts),
        ("sleeve_r.cuff_R",       sr["cuff_R"],          srpts),
        ("sleeve_l.cap_crown",    sl["cap_crown"],       slpts),
        ("sleeve_l.underarm_L",   sl["underarm_L"],      slpts),
        ("sleeve_l.underarm_R",   sl["underarm_R"],      slpts),
    ]:
        p = pts[idx]
        print(f"    {lname:30s} v{idx:3d} [{p[0]:.4f}, {p[1]:.4f}]")

    # --- 7. Compute stitches ---
    stitches = compute_tshirt_stitches(pieces)
    print(f"\n  Stitches: {len(stitches)}")
    for s in stitches:
        print(f"    {s['comment']}")

    # --- 8. Assemble pattern JSON ---
    pattern = {
        "name": "TShirt_DXF",
        "panels": [],
        "stitches": stitches,
        "fabric": "cotton",
        "metadata": {
            "source": os.path.basename(dxf_path),
            "importer": "import_tshirt.py",
        },
    }

    panel_order = ["front", "back", "sleeve_right", "sleeve_left"]
    for name in panel_order:
        if name not in pieces:
            continue
        piece = pieces[name]
        pattern["panels"].append({
            "id": name,
            "vertices_2d": piece.vertices_2d,
            "placement": placements.get(name, {
                "position": [0, 0.75, 0.2],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }),
        })

    # --- 9. Save ---
    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w") as f:
            json.dump(pattern, f, indent=2)
        print(f"\n  Saved: {output_path}")

    return pattern


# ──────────────────────────────────────────────────────────
#  CLI
# ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Import CLO3D t-shirt DXF")
    parser.add_argument(
        "--dxf", default=os.path.join(backend_dir, "data/patterns/tshirt_new.dxf"),
        help="Path to the DXF file",
    )
    parser.add_argument(
        "--output", default=os.path.join(backend_dir, "data/patterns/tshirt.json"),
        help="Output JSON path",
    )
    parser.add_argument(
        "--profile",
        default=os.path.join(backend_dir, "data/bodies/mannequin_profile.json"),
        help="Path to mannequin_profile.json",
    )
    args = parser.parse_args()

    pattern = import_tshirt(args.dxf, args.profile, args.output)

    print("\n  ═══ Summary ═══")
    print(f"  Panels:   {len(pattern['panels'])}")
    for p in pattern["panels"]:
        print(f"    {p['id']}: {len(p['vertices_2d'])} verts, pos={p['placement']['position']}")
    print(f"  Stitches: {len(pattern['stitches'])}")
    print("\n  Next: python -m simulation --scene garment_drape "
          "--pattern data/patterns/tshirt.json")


if __name__ == "__main__":
    main()
