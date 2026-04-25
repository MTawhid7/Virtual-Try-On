"""
Stitch Crossing & Force-Angle Detector (Root Cause B).

For each sleeve seam, maps stitch pairs to normalized arc-parameter space
and detects:
  - Crossings: pair k and k+1 traverse the two edges in opposite directions
  - Force angle divergence: adjacent stitch force vectors point in very
    different directions (> 30°), indicating erratic force field

Run from backend/:
  python -m scripts.detect_stitch_crossings
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
    _find_edge_particles,
    build_garment_mesh,
)
from simulation.mesh.triangulation import triangulate_panel

PATTERN = Path(__file__).resolve().parents[1] / "data/patterns/tshirt.json"
TARGET_EDGE = 0.020


def _cumulative_arc(positions_3d: np.ndarray, particle_indices: np.ndarray) -> np.ndarray:
    """
    Cumulative arc length along a particle chain, normalized to [0, 1].
    Returns array of length len(particle_indices).
    """
    pts = positions_3d[particle_indices]
    diffs = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(diffs)])
    total = cum[-1]
    if total < 1e-9:
        return np.linspace(0.0, 1.0, len(particle_indices))
    return cum / total


def _world_positions(spec, panel_id):
    """Return 3D world positions (post cylindrical wrap) for a panel."""
    panels = {p["id"]: p for p in spec["panels"]}
    pspec = panels[panel_id]
    panel = triangulate_panel(pspec["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
    world_pos = _apply_placement(panel.positions, pspec["placement"])
    if "sleeve" in panel_id.lower():
        world_pos = _cylindrical_wrap_sleeve(world_pos, pspec["placement"])
    return panel, world_pos


def analyze_seam(
    label: str,
    spec,
    panel_id_a: str, va_s: int, va_e: int,
    panel_id_b: str, vb_s: int, vb_e: int,
    angle_threshold_deg: float = 30.0,
) -> dict:
    panels = {p["id"]: p for p in spec["panels"]}

    panel_a, pos_a_world = _world_positions(spec, panel_id_a)
    panel_b, pos_b_world = _world_positions(spec, panel_id_b)

    local_a = _find_edge_particles(panel_a, va_s, va_e)
    local_b = _find_edge_particles(panel_b, vb_s, vb_e)

    n_pts = max(len(local_a), len(local_b))
    idx_a = np.linspace(0, len(local_a) - 1, n_pts, dtype=int)
    idx_b = np.linspace(0, len(local_b) - 1, n_pts, dtype=int)

    sampled_a = local_a[idx_a]
    sampled_b = local_b[idx_b]

    # Arc parameters for the selected particles
    t_a = _cumulative_arc(pos_a_world, local_a)[idx_a]
    t_b = _cumulative_arc(pos_b_world, local_b)[idx_b]

    # Crossing detection
    crossings = 0
    crossing_pairs = []
    for k in range(n_pts - 1):
        da = t_a[k + 1] - t_a[k]
        db = t_b[k + 1] - t_b[k]
        if da * db < 0 and abs(da) > 1e-6 and abs(db) > 1e-6:
            crossings += 1
            crossing_pairs.append(k)

    # Force angle divergence
    pa3d = pos_a_world[sampled_a]  # (n_pts, 3)
    pb3d = pos_b_world[sampled_b]  # (n_pts, 3)
    force_vectors = pa3d - pb3d    # direction a→b for each pair
    norms = np.linalg.norm(force_vectors, axis=1, keepdims=True)
    norms = np.maximum(norms, 1e-9)
    unit_forces = force_vectors / norms

    angles = []
    worst_angle = 0.0
    worst_pair = -1
    for k in range(n_pts - 1):
        cos_theta = np.clip(np.dot(unit_forces[k], unit_forces[k + 1]), -1.0, 1.0)
        angle_deg = np.degrees(np.arccos(cos_theta))
        angles.append(angle_deg)
        if angle_deg > worst_angle:
            worst_angle = angle_deg
            worst_pair = k

    angles = np.array(angles)
    n_large_angle = int((angles > angle_threshold_deg).sum())

    # Print report
    print(f"\n  {label}  ({n_pts} pairs, sleeve N={len(local_a)}, panel N={len(local_b)})")
    flag_cross = "  ⚠ FLAG >0 (Root cause B — 3D crossings!)" if crossings > 0 else "  OK"
    print(f"    Crossings: {crossings}{flag_cross}")
    if crossing_pairs:
        for k in crossing_pairs[:5]:
            print(f"      pair {k}: t_a={t_a[k]:.3f}→{t_a[k+1]:.3f}  t_b={t_b[k]:.3f}→{t_b[k+1]:.3f}")

    flag_angle = f"  ⚠ FLAG >{angle_threshold_deg:.0f}°" if worst_angle > angle_threshold_deg else "  OK"
    print(f"    Max adjacent-pair force angle: {worst_angle:.1f}°{flag_angle}")
    print(f"    Pairs with angle > {angle_threshold_deg:.0f}°: {n_large_angle}/{n_pts-1}")
    if worst_angle > angle_threshold_deg and worst_pair >= 0:
        print(f"    Worst pair {worst_pair}: pos_a={pa3d[worst_pair].round(3)}  pos_b={pb3d[worst_pair].round(3)}")
        print(f"             initial gap={norms[worst_pair, 0]*100:.1f}cm")

    # Initial gap distribution
    gaps = norms[:, 0] * 100  # cm
    print(f"    Initial gap: min={gaps.min():.1f}cm  mean={gaps.mean():.1f}cm  max={gaps.max():.1f}cm")

    return {
        "crossings": crossings,
        "max_angle": worst_angle,
        "n_large_angle": n_large_angle,
        "n_pts": n_pts,
    }


def main():
    with open(PATTERN) as f:
        spec = json.load(f)

    print("\nSTITCH CROSSING & FORCE-ANGLE DETECTION")
    print(f"Pattern: {PATTERN.name}  target_edge={TARGET_EDGE}m")
    print("=" * 60)

    seams = [
        ("right_cap_back  [4→25]  ↔ back[6→29]",
         "sleeve_right", 4,  25, "back",  6,  29),
        ("right_cap_front [43→25] ↔ front[48→68]",
         "sleeve_right", 43, 25, "front", 48, 68),
        ("right_underarm  [47→43] ↔ [0→4]",
         "sleeve_right", 47, 43, "sleeve_right", 0, 4),
        ("left_cap_front  [52→31] ↔ front[36→16]",
         "sleeve_left",  52, 31, "front", 36, 16),
        ("left_cap_back   [31→13] ↔ back[69→92]",
         "sleeve_left",  31, 13, "back",  69, 92),
        ("left_underarm   [56→52] ↔ [9→13]",
         "sleeve_left",  56, 52, "sleeve_left", 9, 13),
    ]

    total_crossings = 0
    for args in seams:
        label = args[0]
        result = analyze_seam(label, spec, *args[1:])
        total_crossings += result["crossings"]

    print()
    print("=" * 60)
    if total_crossings > 0:
        print(f"⚠ TOTAL CROSSINGS: {total_crossings} — Root cause B confirmed.")
        print("  Fix: in build_garment_mesh(), after _find_edge_particles(), add")
        print("  arc-parameter check and reverse shorter edge if t_b is decreasing")
        print("  while t_a is increasing.")
    else:
        print("✓ No crossings detected — Root cause B ruled out.")
    print("=" * 60)


if __name__ == "__main__":
    main()
