"""
DXF-to-Pattern-JSON Importer — parses CLO3D AAMA DXF exports.

Extracts 2D panel outlines from DXF blocks, normalizes coordinates,
applies mirroring for symmetric pieces, computes flat placement
positions, and outputs a pattern JSON for the simulation pipeline.

Usage:
    python -m scripts.import_dxf [--dxf PATH] [--output PATH] [--preview]

Supports:
    - CLO3D AAMA DXF exports (tested with CLO Standalone 6.x)
    - POLYLINE outlines (largest closed polyline per block = cutting line)
    - POINT entities (notch marks for stitch matching)
    - Automatic mirroring for left/right panels
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import Counter
from dataclasses import dataclass, field

import ezdxf
import numpy as np

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)


# ──────────────────────────────────────────────────────────
#  Data structures
# ──────────────────────────────────────────────────────────

@dataclass
class PanelPiece:
    """A single pattern piece extracted from DXF."""
    name: str
    outline_mm: np.ndarray       # (N, 2) in mm, as found in DXF
    seam_line_mm: np.ndarray | None  # (M, 2) sewing line, if found
    notches_mm: np.ndarray | None    # (K, 2) notch points
    vertices_2d: list[list[float]] = field(default_factory=list)  # normalized, in meters
    width_cm: float = 0.0
    height_cm: float = 0.0


# ──────────────────────────────────────────────────────────
#  DXF Parsing
# ──────────────────────────────────────────────────────────

def extract_pieces(dxf_path: str, piece_names: list[str]) -> list[PanelPiece]:
    """
    Extract named pattern pieces from a CLO3D AAMA DXF file.

    For each block, finds the largest closed POLYLINE (= cutting line outline),
    the second largest (= seam/sewing line), and POINT entities (= notches).
    """
    doc = ezdxf.readfile(dxf_path)
    pieces = []

    for piece_name in piece_names:
        block = doc.blocks.get(piece_name)
        if block is None:
            print(f"  ⚠ Block '{piece_name}' not found in DXF")
            continue

        entities = list(block)
        polylines = [e for e in entities if e.dxftype() == 'POLYLINE']
        point_ents = [e for e in entities if e.dxftype() == 'POINT']

        # Sort polylines by bounding box area (largest = cutting line)
        poly_data = []
        for pl in polylines:
            pts = list(pl.points())
            if len(pts) < 3:
                continue
            coords = np.array([(p.x, p.y) for p in pts])
            x_range = coords[:, 0].max() - coords[:, 0].min()
            y_range = coords[:, 1].max() - coords[:, 1].min()
            area = x_range * y_range
            poly_data.append((coords, area, pl.is_closed))

        poly_data.sort(key=lambda x: x[1], reverse=True)

        if not poly_data:
            print(f"  ⚠ No polylines in block '{piece_name}'")
            continue

        outline_mm = poly_data[0][0]  # largest = cutting line
        seam_mm = poly_data[1][0] if len(poly_data) > 1 else None

        # Notch points
        notches = None
        if point_ents:
            notch_coords = [(e.dxf.location.x, e.dxf.location.y) for e in point_ents]
            notches = np.array(notch_coords)

        piece = PanelPiece(
            name=piece_name.replace('_M', ''),
            outline_mm=outline_mm,
            seam_line_mm=seam_mm,
            notches_mm=notches,
        )
        piece.width_cm = (outline_mm[:, 0].max() - outline_mm[:, 0].min()) / 10
        piece.height_cm = (outline_mm[:, 1].max() - outline_mm[:, 1].min()) / 10

        pieces.append(piece)

    return pieces


def normalize_piece(piece: PanelPiece) -> None:
    """
    Normalize a piece's outline to local coordinates:
    - Translate so bounding box min corner is at origin
    - Convert mm to meters (÷ 1000)
    - Store as vertices_2d [x, z] where x=width, z=height
    """
    outline = piece.outline_mm.copy()

    # Translate to origin
    x_min = outline[:, 0].min()
    y_min = outline[:, 1].min()
    outline[:, 0] -= x_min
    outline[:, 1] -= y_min

    # Convert mm → meters
    outline_m = outline / 1000.0

    # Store as [x, z] list
    piece.vertices_2d = [[round(float(p[0]), 6), round(float(p[1]), 6)] for p in outline_m]


def mirror_piece(piece: PanelPiece, new_name: str) -> PanelPiece:
    """Mirror a piece horizontally (flip X) for the opposite side."""
    verts = piece.vertices_2d
    pw = max(v[0] for v in verts)

    # Mirror X and reverse order (maintain CCW winding)
    mirrored_verts = [[round(pw - v[0], 6), v[1]] for v in reversed(verts)]

    mirrored = PanelPiece(
        name=new_name,
        outline_mm=piece.outline_mm.copy(),
        seam_line_mm=piece.seam_line_mm,
        notches_mm=piece.notches_mm,
        vertices_2d=mirrored_verts,
        width_cm=piece.width_cm,
        height_cm=piece.height_cm,
    )
    return mirrored


# ──────────────────────────────────────────────────────────
#  Placement & Stitching
# ──────────────────────────────────────────────────────────

def compute_placements(pieces: dict[str, PanelPiece], body_profile: dict) -> dict[str, dict]:
    """
    Compute flat placement for each panel.

    Front panels: Z > body front surface + clearance
    Back panels:  Z < body back surface - clearance
    Sleeves:      to the sides, at appropriate height
    """
    lm = body_profile["landmarks"]
    hem_y = lm["hip_y"] - 0.05
    front_z = 0.35    # just in front of body front surface (~0.288m)
    back_z = 0.04     # just behind body back surface (~0.031m), avoids starting inside mesh
    collar_z = front_z - 0.10  # collar near neck, not behind body

    placements = {}

    for name, piece in pieces.items():
        pw = max(v[0] for v in piece.vertices_2d)

        if "front" in name.lower():
            if "left" in name.lower():
                pos_x = -(pw + 0.01)
            else:
                pos_x = 0.01
            placements[name] = {
                "position": [round(pos_x, 5), round(hem_y, 5), round(front_z, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }
        elif "back" in name.lower() and "placket" not in name.lower():
            if "left" in name.lower():
                pos_x = -0.01
            else:
                pos_x = pw + 0.01
            placements[name] = {
                "position": [round(pos_x, 5), round(hem_y, 5), round(back_z, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 180,
            }
        elif "sleeve" in name.lower():
            # Sleeves placed to the sides, at shoulder height
            shoulder_y = lm["shoulder_y"]
            sleeve_ph = max(v[1] for v in piece.vertices_2d)
            sleeve_pw = max(v[0] for v in piece.vertices_2d)
            body_z_mid = (lm.get("chest_z_front", 0.279) + lm.get("chest_z_back", 0.034)) / 2

            if "left" in name.lower():
                # Left sleeve: placed to the left of body
                placements[name] = {
                    "position": [round(-(0.30 + sleeve_pw), 5), round(shoulder_y - sleeve_ph, 5), round(body_z_mid, 5)],
                    "rotation_x_deg": -90,
                    "rotation_y_deg": 0,
                }
            else:
                # Right sleeve: placed to the right of body
                placements[name] = {
                    "position": [round(0.30, 5), round(shoulder_y - sleeve_ph, 5), round(body_z_mid, 5)],
                    "rotation_x_deg": -90,
                    "rotation_y_deg": 0,
                }
        elif "collar" in name.lower():
            placements[name] = {
                "position": [round(-pw/2, 5), round(lm["shoulder_y"] + 0.03, 5), round(collar_z, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }
        elif "placket" in name.lower():
            placements[name] = {
                "position": [round(-pw/2, 5), round(lm["chest_y"] + 0.05, 5), round(front_z + 0.02, 5)],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }

    return placements


def find_vertical_run(verts: list[list[float]], side: str) -> tuple[int, int]:
    """
    Find the start and end vertex indices of the longest straight vertical
    run on the specified side of the panel.
    """
    pw = max(v[0] for v in verts)
    threshold = pw * 0.2 if side == "left" else pw * 0.8

    side_verts = []
    for i, v in enumerate(verts):
        if (side == "left" and v[0] < threshold) or \
           (side == "right" and v[0] > threshold):
            side_verts.append((i, v[1]))

    if len(side_verts) < 2:
        return 0, 1

    side_verts.sort(key=lambda x: x[1])
    return side_verts[0][0], side_verts[-1][0]


def get_top_landmarks(verts: list[list[float]], side_seam_pos: str) -> tuple[int, int, int, int]:
    """
    Returns (neck, inner_shoulder, outer_shoulder, underarm).

    Geometry-correct approach:
    - inner_shoulder = highest Y vertex
    - underarm       = extreme X vertex on the side-seam side (rightmost for "right",
                       leftmost for "left") — the peak of the side seam / start of armhole
    - outer_shoulder = highest-Y vertex that is within 15% of the extreme X AND
                       above the underarm height (the corner where armhole meets
                       shoulder seam)
    - neck           = top of centre seam (unchanged)
    """
    pts = np.array(verts)
    n = len(pts)
    pw = pts[:, 0].max()
    ph = pts[:, 1].max()

    # inner_shoulder = highest Y vertex
    inner_shoulder = int(np.argmax(pts[:, 1]))

    # underarm = extreme X in the LOWER 70% of the panel.
    # Restricting to Y < 0.7*ph avoids the armhole scoop, which can extend further
    # toward the side than the side seam itself (the scoop is always in the upper panel).
    lower = [i for i in range(n) if pts[i, 1] < ph * 0.7]
    if side_seam_pos == "right":
        underarm = max(lower, key=lambda i: pts[i, 0])
    else:
        underarm = min(lower, key=lambda i: pts[i, 0])

    # neck = top of center seam
    center_side = "left" if side_seam_pos == "right" else "right"
    _, neck = find_vertical_run(verts, center_side)

    # outer_shoulder = highest-Y vertex near the side-seam extreme, above underarm
    underarm_x = pts[underarm, 0]
    underarm_y = pts[underarm, 1]

    if side_seam_pos == "right":
        threshold_x = underarm_x * 0.85
        candidates = [i for i in range(n)
                      if pts[i, 0] > threshold_x and pts[i, 1] > underarm_y and i != underarm]
    else:
        threshold_x = underarm_x + (pw - underarm_x) * 0.15
        candidates = [i for i in range(n)
                      if pts[i, 0] < threshold_x and pts[i, 1] > underarm_y and i != underarm]

    if candidates:
        outer_shoulder = candidates[int(np.argmax([pts[c, 1] for c in candidates]))]
    else:
        outer_shoulder = inner_shoulder  # fallback

    return neck, inner_shoulder, outer_shoulder, underarm


def get_sleeve_points(verts: list[list[float]]) -> tuple[int, int, int, int, int]:
    """Returns (cuff_left, cuff_right, underarm_left, underarm_right, notch)."""
    pw = max(v[0] for v in verts)
    bot_pts = [i for i, v in enumerate(verts) if v[1] < 0.05]
    cuff_left = min(bot_pts, key=lambda i: verts[i][0])
    cuff_right = max(bot_pts, key=lambda i: verts[i][0])

    left_edge = [i for i, v in enumerate(verts) if v[0] < pw * 0.15 and v[1] > 0.02]
    left_edge.sort(key=lambda i: verts[i][1])
    underarm_left = left_edge[-1] if left_edge else cuff_left

    right_edge = [i for i, v in enumerate(verts) if v[0] > pw * 0.85 and v[1] > 0.02]
    right_edge.sort(key=lambda i: verts[i][1])
    underarm_right = right_edge[-1] if right_edge else cuff_right

    notch = max(range(len(verts)), key=lambda i: verts[i][1])
    return cuff_left, cuff_right, underarm_left, underarm_right, notch


def compute_stitches(pieces: dict[str, PanelPiece]) -> list[dict]:
    """
    Compute stitch definitions based on panel edge analysis.

    For V1, finds the side seam edges (longest vertical edge on each panel)
    and connects front to back panels at corresponding sides.
    """
    stitches = []

    def find_side_edge(verts: list[list[float]], side: str) -> tuple[int, int]:
        """Find the longest vertical edge on the specified side (left or right)."""
        n = len(verts)
        pw = max(v[0] for v in verts)

        best_length = 0
        best_start = 0
        best_end = 0

        for i in range(n):
            j = (i + 1) % n
            x_avg = (verts[i][0] + verts[j][0]) / 2
            y_diff = abs(verts[j][1] - verts[i][1])

            if side == "right" and x_avg > pw * 0.8 and y_diff > best_length:
                best_length = y_diff
                best_start = i
                best_end = j
            elif side == "left" and x_avg < pw * 0.2 and y_diff > best_length:
                best_length = y_diff
                best_start = i
                best_end = j

        return best_start, best_end

    # --- Side seams (front ↔ back at matching sides) ---
    # The side seam runs from the hem corner up to the underarm only.
    # find_vertical_run's max-Y vertex is wrong here because the armhole arc
    # curves back into the "right-side" X-threshold, producing a top vertex near
    # the shoulder rather than the underarm. Use get_top_landmarks for underarm.
    if "front_right" in pieces and "back_right" in pieces:
        _, _, _, fr_underarm = get_top_landmarks(pieces["front_right"].vertices_2d, "right")
        _, _, _, br_underarm = get_top_landmarks(pieces["back_right"].vertices_2d, "left")
        fr_bot, _ = find_vertical_run(pieces["front_right"].vertices_2d, "right")
        br_bot, _ = find_vertical_run(pieces["back_right"].vertices_2d, "left")
        stitches.append({
            "comment": "Right side seam (front_right → back_right)",
            "panel_a": "front_right", "edge_a": [fr_bot, fr_underarm],
            "panel_b": "back_right",  "edge_b": [br_bot, br_underarm],
        })

    if "front_left" in pieces and "back_left" in pieces:
        _, _, _, fl_underarm = get_top_landmarks(pieces["front_left"].vertices_2d, "left")
        _, _, _, bl_underarm = get_top_landmarks(pieces["back_left"].vertices_2d, "right")
        fl_bot, _ = find_vertical_run(pieces["front_left"].vertices_2d, "left")
        bl_bot, _ = find_vertical_run(pieces["back_left"].vertices_2d, "right")
        stitches.append({
            "comment": "Left side seam (front_left → back_left)",
            "panel_a": "front_left", "edge_a": [fl_bot, fl_underarm],
            "panel_b": "back_left",  "edge_b": [bl_bot, bl_underarm],
        })

    # Center seams
    if "front_left" in pieces and "front_right" in pieces:
        fl_bot, fl_top = find_vertical_run(pieces["front_left"].vertices_2d, "right")
        fr_bot, fr_top = find_vertical_run(pieces["front_right"].vertices_2d, "left")
        stitches.append({
            "comment": "Center-front seam",
            "panel_a": "front_left", "edge_a": [fl_bot, fl_top],
            "panel_b": "front_right", "edge_b": [fr_bot, fr_top],
        })

    if "back_left" in pieces and "back_right" in pieces:
        bl_bot, bl_top = find_vertical_run(pieces["back_left"].vertices_2d, "left")
        br_bot, br_top = find_vertical_run(pieces["back_right"].vertices_2d, "right")
        stitches.append({
            "comment": "Center-back seam",
            "panel_a": "back_left", "edge_a": [bl_bot, bl_top],
            "panel_b": "back_right", "edge_b": [br_bot, br_top],
        })

    # --- Shoulder seams ---
    if "front_right" in pieces and "back_right" in pieces:
        fr_neck, fr_ishoulder, fr_oshoulder, _ = get_top_landmarks(pieces["front_right"].vertices_2d, "right")
        br_neck, br_ishoulder, br_oshoulder, _ = get_top_landmarks(pieces["back_right"].vertices_2d, "left")
        stitches.append({
            "comment": "Right shoulder seam",
            "panel_a": "front_right", "edge_a": [fr_oshoulder, fr_ishoulder],
            "panel_b": "back_right",  "edge_b": [br_oshoulder, br_ishoulder]
        })

    if "front_left" in pieces and "back_left" in pieces:
        fl_neck, fl_ishoulder, fl_oshoulder, _ = get_top_landmarks(pieces["front_left"].vertices_2d, "left")
        bl_neck, bl_ishoulder, bl_oshoulder, _ = get_top_landmarks(pieces["back_left"].vertices_2d, "right")
        stitches.append({
            "comment": "Left shoulder seam",
            "panel_a": "front_left", "edge_a": [fl_oshoulder, fl_ishoulder],
            "panel_b": "back_left",  "edge_b": [bl_oshoulder, bl_ishoulder]
        })

    # --- Sleeves ---
    if "sleeve_right" in pieces and "front_right" in pieces and "back_right" in pieces:
        v_sleeve = pieces["sleeve_right"].vertices_2d
        cL, cR, uL, uR, notch = get_sleeve_points(v_sleeve)
        _, _, fr_oshoulder, fr_u = get_top_landmarks(pieces["front_right"].vertices_2d, "right")
        _, _, br_oshoulder, br_u = get_top_landmarks(pieces["back_right"].vertices_2d, "left")

        # Right Sleeve to Armholes
        stitches.append({
            "comment": "Right sleeve -> front armhole",
            "panel_a": "sleeve_right", "edge_a": [uL, notch],
            "panel_b": "front_right",  "edge_b": [fr_u, fr_oshoulder]
        })
        stitches.append({
            "comment": "Right sleeve -> back armhole",
            "panel_a": "sleeve_right", "edge_a": [notch, uR],
            "panel_b": "back_right",   "edge_b": [br_oshoulder, br_u]
        })
        # Right underarm internal seam
        stitches.append({
            "comment": "Right sleeve underarm seam",
            "panel_a": "sleeve_right", "edge_a": [cL, uL],
            "panel_b": "sleeve_right", "edge_b": [cR, uR]
        })

    if "sleeve_left" in pieces and "front_left" in pieces and "back_left" in pieces:
        v_sleeve = pieces["sleeve_left"].vertices_2d
        cL, cR, uL, uR, notch = get_sleeve_points(v_sleeve)
        _, _, fl_oshoulder, fl_u = get_top_landmarks(pieces["front_left"].vertices_2d, "left")
        _, _, bl_oshoulder, bl_u = get_top_landmarks(pieces["back_left"].vertices_2d, "right")

        # Left Sleeve to Armholes
        stitches.append({
            "comment": "Left sleeve -> back armhole",
            "panel_a": "sleeve_left", "edge_a": [uL, notch],
            "panel_b": "back_left",   "edge_b": [bl_u, bl_oshoulder]
        })
        stitches.append({
            "comment": "Left sleeve -> front armhole",
            "panel_a": "sleeve_left", "edge_a": [notch, uR],
            "panel_b": "front_left",  "edge_b": [fl_oshoulder, fl_u]
        })
        # Left underarm internal seam
        stitches.append({
            "comment": "Left sleeve underarm seam",
            "panel_a": "sleeve_left", "edge_a": [cL, uL],
            "panel_b": "sleeve_left", "edge_b": [cR, uR]
        })

    return stitches


# ──────────────────────────────────────────────────────────
#  Main Pipeline
# ──────────────────────────────────────────────────────────

def import_dxf(
    dxf_path: str,
    profile_path: str,
    output_path: str | None = None,
) -> dict:
    """
    Import a CLO3D DXF and produce a pattern JSON.

    Args:
        dxf_path: Path to the DXF file
        profile_path: Path to mannequin_profile.json
        output_path: Where to save the JSON (optional)

    Returns:
        Pattern dict compatible with panel_builder.build_garment_mesh()
    """
    print("═" * 60)
    print("  DXF → Pattern JSON Importer")
    print("═" * 60)

    # --- 1. Extract pieces ---
    print(f"\n  Source: {os.path.basename(dxf_path)}")

    raw_pieces = extract_pieces(dxf_path, [
        "Body_Front_1_M",
        "Body_Back_M",
        "Sleeves_M",
        "Collar_M",
        "Body_Front_Placket_M",
    ])

    print(f"  Extracted {len(raw_pieces)} pieces:")
    for p in raw_pieces:
        print(f"    • {p.name}: {p.width_cm:.1f}cm × {p.height_cm:.1f}cm "
              f"({len(p.outline_mm)} points)")

    # --- 2. Normalize coordinates (translate to origin, mm → m) ---
    for p in raw_pieces:
        normalize_piece(p)

    # --- 3. Create mirrored panels ---
    pieces: dict[str, PanelPiece] = {}

    for p in raw_pieces:
        if "front" in p.name.lower() and "placket" not in p.name.lower():
            pieces["front_right"] = p
            p.name = "front_right"
            mirrored = mirror_piece(p, "front_left")
            pieces["front_left"] = mirrored
        elif "back" in p.name.lower() and "sleeve" not in p.name.lower():
            pieces["back_right"] = p
            p.name = "back_right"
            mirrored = mirror_piece(p, "back_left")
            pieces["back_left"] = mirrored
        elif "sleeve" in p.name.lower():
            pieces["sleeve_right"] = p
            p.name = "sleeve_right"
            mirrored = mirror_piece(p, "sleeve_left")
            pieces["sleeve_left"] = mirrored
        elif "collar" in p.name.lower():
            pieces["collar"] = p
            p.name = "collar"
        elif "placket" in p.name.lower():
            pieces["placket"] = p
            p.name = "placket"

    print(f"\n  After mirroring: {len(pieces)} panels")
    for name, p in pieces.items():
        pw = max(v[0] for v in p.vertices_2d)
        ph = max(v[1] for v in p.vertices_2d)
        print(f"    {name}: {pw*100:.1f}cm × {ph*100:.1f}cm ({len(p.vertices_2d)} verts)")

    # --- 4. Load body profile for placement ---
    with open(profile_path) as f:
        body_profile = json.load(f)

    # --- 5. Compute placements ---
    placements = compute_placements(pieces, body_profile)

    # --- 6. Compute stitches ---
    stitches = compute_stitches(pieces)
    print(f"\n  Stitches: {len(stitches)}")
    for s in stitches:
        print(f"    {s['comment']}")

    # --- 7. Assemble pattern JSON ---
    pattern = {
        "name": "PoloShirt_DXF",
        "panels": [],
        "stitches": stitches,
        "fabric": "cotton",
        "metadata": {
            "source": os.path.basename(dxf_path),
            "importer": "import_dxf.py",
        },
    }

    for name, piece in pieces.items():
        panel = {
            "id": name,
            "vertices_2d": piece.vertices_2d,
            "placement": placements.get(name, {
                "position": [0, 0.75, 0.2],
                "rotation_x_deg": -90,
                "rotation_y_deg": 0,
            }),
        }
        pattern["panels"].append(panel)

    # --- 8. Save ---
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
    dxf_path = os.path.join(backend_dir, "data/patterns/Polo-Shirt.dxf")
    profile_path = os.path.join(backend_dir, "data/bodies/mannequin_profile.json")
    output_path = os.path.join(backend_dir, "data/patterns/polo_shirt.json")

    pattern = import_dxf(dxf_path, profile_path, output_path)

    print(f"\n  ═══ Summary ═══")
    print(f"  Panels: {len(pattern['panels'])}")
    for p in pattern["panels"]:
        print(f"    {p['id']}: {len(p['vertices_2d'])} verts, pos={p['placement']['position']}")
    print(f"  Stitches: {len(pattern['stitches'])}")
    print(f"\n  Next: python -m scripts.verify_dxf_import")


if __name__ == "__main__":
    main()
