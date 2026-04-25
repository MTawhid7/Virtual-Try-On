"""
Sleeve Symmetry Audit — right vs left sleeve geometric comparison.

Diagnoses root causes A–F for right sleeve uneven surface:
  A. Asymmetric stitch pair count (front-half vs back-half)
  B. (see detect_stitch_crossings.py)
  C. Cap split not at geometric midpoint
  D. Stitch clustering (linspace repeats vertices)
  E. Mass asymmetry at seam vertices
  F. Underarm azimuthal twist

Run from backend/:
  python -m scripts.sleeve_symmetry_audit
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

# ── project imports ──────────────────────────────────────────────────────────
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from simulation.mesh.panel_builder import (
    _apply_placement,
    _cylindrical_wrap_sleeve,
    _find_edge_particles,
    build_garment_mesh,
)
from simulation.mesh.triangulation import triangulate_panel
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.materials import FABRIC_PRESETS

PATTERN = Path(__file__).resolve().parents[1] / "data/patterns/tshirt.json"
TARGET_EDGE = 0.020


def _load_pattern():
    with open(PATTERN) as f:
        return json.load(f)


def _poly_arc_length(verts_2d: list[list[float]], v_start: int, v_end: int) -> float:
    """Arc length of polygon boundary from v_start to v_end (shorter direction)."""
    n = len(verts_2d)
    pts = np.array(verts_2d, dtype=np.float64)

    def arc(start, end):
        length = 0.0
        idx = start
        while idx != end:
            nxt = (idx + 1) % n
            length += np.linalg.norm(pts[nxt] - pts[idx])
            idx = nxt
        return length

    fwd = arc(v_start, v_end)
    bwd = arc(v_end, v_start)
    return min(fwd, bwd)


def _cap_split_arc_fraction(verts_2d: list, cap_start: int, split: int, cap_end: int) -> float:
    """
    Fraction of total cap arc where the split vertex falls.
    cap_start → split → cap_end (all in one direction around polygon).
    Returns split_arc / total_arc.
    """
    n = len(verts_2d)
    pts = np.array(verts_2d, dtype=np.float64)

    def arc_fwd(a, b):
        length, idx = 0.0, a
        while idx != b:
            nxt = (idx + 1) % n
            length += np.linalg.norm(pts[nxt] - pts[idx])
            idx = nxt
        return length

    split_arc = arc_fwd(cap_start, split)
    total_arc = arc_fwd(cap_start, cap_end)
    if total_arc < 1e-9:
        return 0.5
    return split_arc / total_arc


# ── Section A: 2D arc lengths ─────────────────────────────────────────────────
def section_a(spec):
    print("=" * 60)
    print("SECTION A — 2D Polygon Arc Lengths (cm)")
    print("=" * 60)

    r_verts = spec["panels"][2]["vertices_2d"]  # sleeve_right
    l_verts = spec["panels"][3]["vertices_2d"]  # sleeve_left

    seams = {
        "right_back_half  [4→25]":  (r_verts, 4,  25),
        "right_front_half [43→25]": (r_verts, 43, 25),
        "right_underarm_a [47→43]": (r_verts, 47, 43),
        "right_underarm_b [0→4]  ": (r_verts, 0,  4),
        "left_front_half  [52→31]": (l_verts, 52, 31),
        "left_back_half   [31→13]": (l_verts, 31, 13),
        "left_underarm_a  [56→52]": (l_verts, 56, 52),
        "left_underarm_b  [9→13] ": (l_verts, 9,  13),
    }

    arcs = {}
    for label, (verts, a, b) in seams.items():
        arc = _poly_arc_length(verts, a, b) * 100  # metres → cm
        arcs[label] = arc
        print(f"  {label}: {arc:.1f}cm")

    print()
    r_back  = arcs["right_back_half  [4→25]"]
    r_front = arcs["right_front_half [43→25]"]
    l_front = arcs["left_front_half  [52→31]"]
    l_back  = arcs["left_back_half   [31→13]"]

    ratio_r = r_back / r_front if r_front > 0 else 0
    ratio_l = l_back / l_front if l_front > 0 else 0
    print(f"  right back/front ratio: {ratio_r:.3f}  {'⚠ FLAG >1.10' if ratio_r > 1.10 else 'OK'}")
    print(f"  left  back/front ratio: {ratio_l:.3f}  {'⚠ FLAG >1.10' if ratio_l > 1.10 else 'OK'}")

    # Cap split fraction (C)
    r_frac = _cap_split_arc_fraction(r_verts, 4,  25, 43)
    l_frac = _cap_split_arc_fraction(l_verts, 13, 31, 52)
    print()
    print(f"  right cap split fraction (v25): {r_frac:.4f}  (ideal=0.50)")
    print(f"  left  cap split fraction (v31): {l_frac:.4f}  (ideal=0.50)")
    diff = abs(r_frac - l_frac)
    print(f"  right−left difference: {diff*100:.2f}%  {'⚠ FLAG >2%' if diff > 0.02 else 'OK'}")
    if abs(r_frac - 0.5) > 0.02:
        print(f"  ⚠ RIGHT cap split deviates from midpoint by {abs(r_frac-0.5)*100:.1f}%")

    return arcs


# ── Section B: Triangulated particle counts ───────────────────────────────────
def section_b(spec):
    print()
    print("=" * 60)
    print("SECTION B — Triangulated Particle Counts per Arc")
    print("=" * 60)

    panels = {p["id"]: p for p in spec["panels"]}
    results = {}

    def arc_particles(panel_id, v_start, v_end, label):
        pspec = panels[panel_id]
        panel = triangulate_panel(pspec["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
        particles = _find_edge_particles(panel, v_start, v_end)
        results[label] = len(particles)
        return panel, particles

    _, rp_back  = arc_particles("sleeve_right", 4,  25, "r_back")
    _, rp_front = arc_particles("sleeve_right", 43, 25, "r_front")
    _, rp_u_a   = arc_particles("sleeve_right", 47, 43, "r_ua")
    _, rp_u_b   = arc_particles("sleeve_right", 0,  4,  "r_ub")
    _, lp_front = arc_particles("sleeve_left",  52, 31, "l_front")
    _, lp_back  = arc_particles("sleeve_left",  31, 13, "l_back")
    _, lp_u_a   = arc_particles("sleeve_left",  56, 52, "l_ua")
    _, lp_u_b   = arc_particles("sleeve_left",  9,  13, "l_ub")

    # Stitch pair counts (max(len_a, len_b) per build_garment_mesh)
    pair_r_back  = max(results["r_back"],  _armhole_count(spec, "back",  6,  29))
    pair_r_front = max(results["r_front"], _armhole_count(spec, "front", 48, 68))
    pair_l_front = max(results["l_front"], _armhole_count(spec, "front", 36, 16))
    pair_l_back  = max(results["l_back"],  _armhole_count(spec, "back",  69, 92))
    pair_r_under = max(results["r_ua"], results["r_ub"])
    pair_l_under = max(results["l_ua"], results["l_ub"])

    print(f"  right back-half  sleeve N={results['r_back']:3d}  armhole N={_armhole_count(spec,'back',6,29):3d}  → pairs={pair_r_back}")
    print(f"  right front-half sleeve N={results['r_front']:3d}  armhole N={_armhole_count(spec,'front',48,68):3d}  → pairs={pair_r_front}")
    print(f"  left  front-half sleeve N={results['l_front']:3d}  armhole N={_armhole_count(spec,'front',36,16):3d}  → pairs={pair_l_front}")
    print(f"  left  back-half  sleeve N={results['l_back']:3d}  armhole N={_armhole_count(spec,'back',69,92):3d}  → pairs={pair_l_back}")
    print(f"  right underarm  [{results['r_ua']} vs {results['r_ub']}] → pairs={pair_r_under}")
    print(f"  left  underarm  [{results['l_ua']} vs {results['l_ub']}] → pairs={pair_l_under}")

    print()
    # Root cause A checks
    ab_diff = abs(pair_r_back - pair_r_front)
    lr_front_diff = abs(pair_r_front - pair_l_front)
    lr_back_diff  = abs(pair_r_back  - pair_l_back)
    print(f"  right back vs front pair diff:  {ab_diff}  {'⚠ FLAG >2 (Root cause A)' if ab_diff > 2 else 'OK'}")
    print(f"  right vs left front-half diff:  {lr_front_diff}  {'⚠ FLAG >2' if lr_front_diff > 2 else 'OK'}")
    print(f"  right vs left back-half diff:   {lr_back_diff}  {'⚠ FLAG >2' if lr_back_diff > 2 else 'OK'}")

    return results, {"r_back": pair_r_back, "r_front": pair_r_front,
                     "l_front": pair_l_front, "l_back": pair_l_back}


def _armhole_count(spec, panel_id, v_start, v_end):
    """Number of boundary particles on an armhole arc."""
    panels = {p["id"]: p for p in spec["panels"]}
    pspec = panels[panel_id]
    panel = triangulate_panel(pspec["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
    return len(_find_edge_particles(panel, v_start, v_end))


# ── Section C: Stitch clustering ──────────────────────────────────────────────
def section_c(spec):
    print()
    print("=" * 60)
    print("SECTION C — Stitch Clustering (linspace repeated vertices)")
    print("=" * 60)

    panels = {p["id"]: p for p in spec["panels"]}

    def cluster_report(label, panel_id_a, va_s, va_e, panel_id_b, vb_s, vb_e):
        pa = triangulate_panel(panels[panel_id_a]["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
        pb = triangulate_panel(panels[panel_id_b]["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
        local_a = _find_edge_particles(pa, va_s, va_e)
        local_b = _find_edge_particles(pb, vb_s, vb_e)

        n_pts = max(len(local_a), len(local_b))
        idx_a = np.linspace(0, len(local_a) - 1, n_pts, dtype=int)
        idx_b = np.linspace(0, len(local_b) - 1, n_pts, dtype=int)

        sampled_a = local_a[idx_a]
        sampled_b = local_b[idx_b]

        def cluster_stats(sampled, n_unique_total, side):
            from collections import Counter
            counts = Counter(sampled.tolist())
            repeated = sum(1 for v, c in counts.items() if c > 1)
            max_reps = max(counts.values())
            frac = repeated / len(counts) if counts else 0
            return repeated, max_reps, frac

        rep_a, max_a, frac_a = cluster_stats(sampled_a, len(local_a), "sleeve")
        rep_b, max_b, frac_b = cluster_stats(sampled_b, len(local_b), "panel")

        flag_a = " ⚠ FLAG >20%" if frac_a > 0.20 else ""
        flag_b = " ⚠ FLAG >20%" if frac_b > 0.20 else ""
        flag_ma = " ⚠ FLAG >2" if max_a > 2 else ""
        flag_mb = " ⚠ FLAG >2" if max_b > 2 else ""
        print(f"  {label}:")
        print(f"    sleeve side: {len(local_a):3d} unique → {len(np.unique(sampled_a)):3d} sampled, "
              f"repeated={rep_a} ({frac_a*100:.0f}%), max_reps={max_a}{flag_a}{flag_ma}")
        print(f"    panel  side: {len(local_b):3d} unique → {len(np.unique(sampled_b)):3d} sampled, "
              f"repeated={rep_b} ({frac_b*100:.0f}%), max_reps={max_b}{flag_b}{flag_mb}")

    cluster_report("right_cap_back ", "sleeve_right", 4,  25, "back",  6,  29)
    cluster_report("right_cap_front", "sleeve_right", 43, 25, "front", 48, 68)
    cluster_report("left_cap_front ", "sleeve_left",  52, 31, "front", 36, 16)
    cluster_report("left_cap_back  ", "sleeve_left",  31, 13, "back",  69, 92)
    cluster_report("right_underarm ", "sleeve_right", 47, 43, "sleeve_right", 0, 4)
    cluster_report("left_underarm  ", "sleeve_left",  56, 52, "sleeve_left",  9, 13)


# ── Section D: Cap split azimuthal position ───────────────────────────────────
def section_d(spec):
    print()
    print("=" * 60)
    print("SECTION D — Cap Split 3D Azimuthal Position")
    print("=" * 60)

    panels = {p["id"]: p for p in spec["panels"]}

    for sleeve_id, split_vi, arm_cx, arm_cz in [
        ("sleeve_right", 25, +0.25, 0.12),
        ("sleeve_left",  31, -0.25, 0.12),
    ]:
        pspec = panels[sleeve_id]
        verts_2d = pspec["vertices_2d"]
        placement = pspec["placement"]

        # 3D position after placement + cylindrical wrap
        panel = triangulate_panel(verts_2d, resolution=20, target_edge=TARGET_EDGE)
        world_pos = _apply_placement(panel.positions, placement)
        world_pos_wrapped = _cylindrical_wrap_sleeve(world_pos, placement)

        # Map original polygon vertex → mesh index
        split_mesh_idx = int(panel.original_vertex_mapping[split_vi])
        split_3d = world_pos_wrapped[split_mesh_idx]

        dx = float(split_3d[0]) - arm_cx
        dz = float(split_3d[2]) - arm_cz
        theta_deg = np.degrees(np.arctan2(dx, -dz))  # +Z front = 0°, +X outer = +90°

        print(f"  {sleeve_id}: v{split_vi}")
        print(f"    2D coords:  ({verts_2d[split_vi][0]:.4f}, {verts_2d[split_vi][1]:.4f})")
        print(f"    3D wrapped: X={split_3d[0]:+.4f}  Y={split_3d[1]:.4f}  Z={split_3d[2]:+.4f}")
        print(f"    Azimuthal angle from arm center: {theta_deg:+.1f}°")
        print(f"    (0°=front +Z, +90°=outer +X, ±180°=back -Z)")

        # Flag if not within ±5° of the outer (+X) direction = 90°
        expected = 90.0 if arm_cx > 0 else -90.0
        deviation = abs(theta_deg - expected)
        if deviation > 10.0:
            print(f"    ⚠ FLAG: cap split is {deviation:.1f}° from expected outer direction ({expected:.0f}°)")
        else:
            print(f"    OK: within {deviation:.1f}° of expected outer direction")


# ── Section E: Mass asymmetry ─────────────────────────────────────────────────
def section_e():
    print()
    print("=" * 60)
    print("SECTION E — Mass Asymmetry at Seam Vertices")
    print("=" * 60)

    fabric = FABRIC_PRESETS["cotton"]
    garment = build_garment_mesh(str(PATTERN), resolution=20, target_edge=TARGET_EDGE)
    inv_masses = compute_area_weighted_inv_masses(garment.positions, garment.faces, fabric.density)

    if garment.stitch_seam_ids is None:
        print("  (stitch_seam_ids not available — skipping)")
        return

    from collections import defaultdict
    seam_pairs = defaultdict(list)
    for i, (a, b) in enumerate(garment.stitch_pairs):
        label = garment.stitch_seam_ids[i]
        seam_pairs[label].append((int(a), int(b)))

    sleeve_seams = [k for k in seam_pairs if "sleeve cap" in k.lower() or "sleeve_cap" in k.lower()
                    or "cap" in k.lower()]

    for label, pairs in seam_pairs.items():
        if not any(kw in label.lower() for kw in ["cap", "shoulder", "side"]):
            continue
        a_masses = [inv_masses[a] for a, b in pairs]
        b_masses = [inv_masses[b] for a, b in pairs]
        a_mean, b_mean = np.mean(a_masses), np.mean(b_masses)
        ratio = a_mean / b_mean if b_mean > 1e-9 else 0

        flag = ""
        if ratio > 1.5 or ratio < 0.67:
            flag = f"  ⚠ FLAG (ratio {'>' if ratio > 1 else '<'} threshold)"

        short_label = label[:52]
        print(f"  {short_label}")
        print(f"    side-A inv_mass: mean={a_mean:.1f}  min={min(a_masses):.1f}  max={max(a_masses):.1f}")
        print(f"    side-B inv_mass: mean={b_mean:.1f}  min={min(b_masses):.1f}  max={max(b_masses):.1f}")
        print(f"    A/B ratio: {ratio:.3f}{flag}")


# ── Section F: Underarm azimuthal twist ───────────────────────────────────────
def section_f(spec):
    print()
    print("=" * 60)
    print("SECTION F — Underarm Azimuthal Twist")
    print("=" * 60)

    panels = {p["id"]: p for p in spec["panels"]}

    for sleeve_id, ua_s, ua_e, ub_s, ub_e, arm_cx, arm_cz in [
        ("sleeve_right", 47, 43, 0, 4,   +0.25, 0.12),
        ("sleeve_left",  56, 52, 9, 13,  -0.25, 0.12),
    ]:
        pspec = panels[sleeve_id]
        placement = pspec["placement"]

        panel = triangulate_panel(pspec["vertices_2d"], resolution=20, target_edge=TARGET_EDGE)
        world_pos = _apply_placement(panel.positions, placement)
        world_pos_wrapped = _cylindrical_wrap_sleeve(world_pos, placement)

        local_a = _find_edge_particles(panel, ua_s, ua_e)
        local_b = _find_edge_particles(panel, ub_s, ub_e)

        n_pts = max(len(local_a), len(local_b))
        idx_a = np.linspace(0, len(local_a) - 1, n_pts, dtype=int)
        idx_b = np.linspace(0, len(local_b) - 1, n_pts, dtype=int)

        pos_a = world_pos_wrapped[local_a[idx_a]]
        pos_b = world_pos_wrapped[local_b[idx_b]]

        # Azimuthal angle relative to arm center
        def azimuth(pts):
            dx = pts[:, 0] - arm_cx
            dz = pts[:, 2] - arm_cz
            return np.degrees(np.arctan2(dx, -dz))

        theta_a = azimuth(pos_a)
        theta_b = azimuth(pos_b)
        delta_theta = theta_a - theta_b

        print(f"  {sleeve_id} underarm seam ({n_pts} pairs):")
        print(f"    Δθ at cuff (k=0):     {delta_theta[0]:+.1f}°")
        print(f"    Δθ at armpit (k=-1):  {delta_theta[-1]:+.1f}°")
        print(f"    Δθ range:             [{delta_theta.min():+.1f}°, {delta_theta.max():+.1f}°]")
        twist_grad = abs(delta_theta.max() - delta_theta.min())
        flag = "  ⚠ FLAG >5° (Root cause F — twist)" if twist_grad > 5.0 else "  OK"
        print(f"    Twist gradient: {twist_grad:.1f}°{flag}")


# ── main ──────────────────────────────────────────────────────────────────────
def main():
    spec = _load_pattern()
    print("\nSLEEVE SYMMETRY AUDIT")
    print(f"Pattern: {PATTERN.name}  target_edge={TARGET_EDGE}m\n")

    section_a(spec)
    section_b(spec)
    section_c(spec)
    section_d(spec)
    section_e()
    section_f(spec)

    print()
    print("=" * 60)
    print("Audit complete. Review ⚠ FLAG lines for anomalies.")
    print("=" * 60)


if __name__ == "__main__":
    main()
