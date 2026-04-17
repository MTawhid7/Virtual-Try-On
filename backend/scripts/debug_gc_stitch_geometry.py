"""
Analyze GarmentCode stitch geometry before simulation.

For each seam this script prints:
  - Which panels are connected
  - Number of stitch pairs and whether they reach the minimum density target
  - Initial 3D gap distribution (min / mean / max)
  - Arc length coverage: how far the pairs span along each seam edge
  - Stitch pair clustering: are pairs evenly spread or piled near one endpoint?

With --verbose it also prints the 3D position of every individual vertex pair.

This script is the first stop when debugging sleeve-cap gaps, asymmetric
seam closure, or "stitches barely move" complaints.  It runs with no
simulation — fast enough for iterative mesh_resolution tuning.

Usage (run from backend/ with virtualenv active):
    python -m scripts.debug_gc_stitch_geometry
    python -m scripts.debug_gc_stitch_geometry --pattern data/patterns/garmentcode/hoody_mean.json
    python -m scripts.debug_gc_stitch_geometry --res 1.0 --verbose
    python -m scripts.debug_gc_stitch_geometry --z-offset 0.0   # inspect pre-offset positions
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def run_analysis(
    pattern_path: str = "data/patterns/garmentcode/shirt_mean.json",
    mesh_resolution: float = 1.5,
    body_z_offset: float = 0.131,
    verbose: bool = False,
) -> None:
    import collections

    import numpy as np
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    print(f"\n{'='*72}")
    print("GC Stitch Geometry Diagnostics")
    print(f"{'='*72}")
    print(f"  Pattern:     {pattern_path}")
    print(f"  Mesh res:    {mesh_resolution} cm")
    print(f"  Z offset:    {body_z_offset:+.4f} m")

    gm = build_garment_mesh_gc(
        pattern_path,
        mesh_resolution=mesh_resolution,
        body_z_offset=body_z_offset,
    )

    n = gm.positions.shape[0]
    offsets = gm.panel_offsets + [n]
    pos = gm.positions  # (N, 3) float32

    # -----------------------------------------------------------------------
    # Panel summary table
    # -----------------------------------------------------------------------
    print(f"\n  Panels ({len(gm.panel_ids)}):")
    print(f"  {'Panel':<26} {'N verts':>7}  {'Centroid (X, Y, Z)':>28}  {'Z range':>16}")
    print(f"  {'─'*26} {'─'*7}  {'─'*28}  {'─'*16}")
    for k, pid in enumerate(gm.panel_ids):
        ps = pos[offsets[k]:offsets[k + 1]]
        cx, cy, cz = ps.mean(axis=0).tolist()
        zmin, zmax = float(ps[:, 2].min()), float(ps[:, 2].max())
        print(
            f"  {pid:<26} {len(ps):>7}  ({cx:+.3f}, {cy:.3f}, {cz:+.3f})"
            f"  [{zmin:+.3f}, {zmax:+.3f}]"
        )

    # -----------------------------------------------------------------------
    # Panel lookup helper
    # -----------------------------------------------------------------------
    def panel_of(v: int) -> str:
        for k, pid in enumerate(gm.panel_ids):
            if offsets[k] <= v < offsets[k + 1]:
                return pid
        return "unknown"

    # -----------------------------------------------------------------------
    # Per-seam analysis
    # -----------------------------------------------------------------------
    print(f"\n  Total stitch pairs: {gm.stitch_pairs.shape[0]}")

    if gm.stitch_seam_ids is None:
        # No labels — treat all as one group
        seam_to_indices: dict = {"all": list(range(gm.stitch_pairs.shape[0]))}
    else:
        seam_to_indices = collections.OrderedDict()
        for pair_idx, label in enumerate(gm.stitch_seam_ids):
            seam_to_indices.setdefault(label, []).append(pair_idx)

    print(f"  Seams: {len(seam_to_indices)}\n")

    _MIN_PAIRS_TARGET = 12  # matches gc_mesh_adapter._MIN_PAIRS_PER_SEAM

    for seam_label, indices in seam_to_indices.items():
        pairs = gm.stitch_pairs[indices]  # (n_pairs, 2)
        pa = pos[pairs[:, 0]]             # (n_pairs, 3) — vertex positions on panel A
        pb = pos[pairs[:, 1]]             # (n_pairs, 3) — vertex positions on panel B
        gaps = np.linalg.norm(pa - pb, axis=1)

        panel_a = panel_of(int(pairs[0, 0]))
        panel_b = panel_of(int(pairs[0, 1]))
        n_pairs = len(pairs)
        max_gap = float(gaps.max())
        mean_gap = float(gaps.mean())
        min_gap = float(gaps.min())

        gap_status = "✅" if max_gap < 0.05 else "❌"
        density_status = "✅" if n_pairs >= _MIN_PAIRS_TARGET else "⚠️ sparse"

        print(f"  {'─'*70}")
        print(f"  [{gap_status} gap] [{density_status}]  {seam_label}")
        print(f"    Panels:  {panel_a} ↔ {panel_b}")
        print(f"    Pairs:   {n_pairs}  (target ≥ {_MIN_PAIRS_TARGET})")
        print(
            f"    Gap:     min={min_gap*100:.1f}cm  mean={mean_gap*100:.1f}cm  "
            f"max={max_gap*100:.1f}cm"
        )

        # Arc length of path traced by panel-A stitch vertices
        # Measures how far along the seam edge the pairs actually span.
        if n_pairs > 1:
            seg_lens_a = np.linalg.norm(np.diff(pa, axis=0), axis=1)
            arc_len_a = float(seg_lens_a.sum())
            seg_lens_b = np.linalg.norm(np.diff(pb, axis=0), axis=1)
            arc_len_b = float(seg_lens_b.sum())
            # Max pair-to-pair step — detects gaps in coverage
            max_step_a = float(seg_lens_a.max()) if len(seg_lens_a) > 0 else 0.0
            print(
                f"    Arc length: {panel_a}={arc_len_a*100:.1f}cm  "
                f"{panel_b}={arc_len_b*100:.1f}cm  "
                f"(max inter-pair step on A: {max_step_a*100:.1f}cm)"
            )

            # Clustering check: are most pairs within 20% of one endpoint?
            arc_cumsum = np.concatenate([[0], np.cumsum(seg_lens_a)])
            if arc_len_a > 1e-6:
                norm_pos = arc_cumsum / arc_len_a  # [0..1]
                in_first_20 = int(np.sum(norm_pos <= 0.2))
                in_last_20 = int(np.sum(norm_pos >= 0.8))
                if in_first_20 > n_pairs * 0.6 or in_last_20 > n_pairs * 0.6:
                    print(
                        f"    ⚠️  Pairs clustered: {in_first_20}/{n_pairs} near start, "
                        f"{in_last_20}/{n_pairs} near end — densification may be stuck"
                    )

        # Duplicate pair check
        pair_set = set(map(tuple, pairs.tolist()))
        if len(pair_set) < n_pairs:
            print(f"    ⚠️  Duplicate pairs: {n_pairs - len(pair_set)} duplicates detected")

        # Body interior check: how many vertices end up inside body Z range
        body_z_back, body_z_front = 0.034, 0.279
        inside_a = int(np.sum((pa[:, 2] > body_z_back) & (pa[:, 2] < body_z_front)))
        inside_b = int(np.sum((pb[:, 2] > body_z_back) & (pb[:, 2] < body_z_front)))
        if inside_a > 0 or inside_b > 0:
            print(
                f"    ℹ️  Verts inside body Z=[{body_z_back},{body_z_front}]: "
                f"{panel_a}={inside_a}/{n_pairs}  {panel_b}={inside_b}/{n_pairs}"
            )

        if verbose:
            print(f"\n    Vertex pairs (v_A → v_B | gap):")
            for k_p, (va_idx, vb_idx) in enumerate(pairs):
                p_a = pos[va_idx]
                p_b = pos[vb_idx]
                g = float(np.linalg.norm(p_a - p_b))
                print(
                    f"      [{k_p:3d}] v{int(va_idx):5d}"
                    f"({p_a[0]:+.3f},{p_a[1]:.3f},{p_a[2]:+.3f})"
                    f" → v{int(vb_idx):5d}"
                    f"({p_b[0]:+.3f},{p_b[1]:.3f},{p_b[2]:+.3f})"
                    f"  gap={g*100:.1f}cm"
                )

    # -----------------------------------------------------------------------
    # Overall summary
    # -----------------------------------------------------------------------
    print(f"\n  {'─'*70}")
    all_gaps = np.linalg.norm(
        pos[gm.stitch_pairs[:, 0]] - pos[gm.stitch_pairs[:, 1]], axis=1
    )
    print("\n  Overall stitch gaps (pre-simulation):")
    print(f"    Min:    {all_gaps.min()*100:.2f} cm")
    print(f"    P25:    {np.percentile(all_gaps, 25)*100:.2f} cm")
    print(f"    Median: {np.median(all_gaps)*100:.2f} cm")
    print(f"    Mean:   {all_gaps.mean()*100:.2f} cm")
    print(f"    P90:    {np.percentile(all_gaps, 90)*100:.2f} cm")
    print(f"    Max:    {all_gaps.max()*100:.2f} cm")
    print(f"    > 5cm:  {int((all_gaps > 0.05).sum())} pairs ({100*(all_gaps>0.05).mean():.0f}%)")
    print(f"    >10cm:  {int((all_gaps > 0.10).sum())} pairs ({100*(all_gaps>0.10).mean():.0f}%)")

    # Seams that will fail (max gap > 5cm)
    failing = [
        (label, max(np.linalg.norm(
            pos[gm.stitch_pairs[idx, 0]] - pos[gm.stitch_pairs[idx, 1]], axis=1
        ).max() for idx in [seam_to_indices[label]])
        )
        for label in seam_to_indices
        if np.linalg.norm(
            pos[gm.stitch_pairs[seam_to_indices[label], 0]] -
            pos[gm.stitch_pairs[seam_to_indices[label], 1]], axis=1
        ).max() > 0.05
    ]
    if failing:
        print(f"\n  Seams likely to fail (<5cm closure threshold):")
        for label, mg in sorted(failing, key=lambda x: -x[1]):
            n_p = len(seam_to_indices[label])
            print(f"    ❌  {label:<48}  max_gap={mg*100:.1f}cm  n={n_p}")
    else:
        print("\n  ✅ All seams have initial gaps < 5cm — sew phase should close cleanly.")

    print()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern", default="data/patterns/garmentcode/shirt_mean.json",
        help="GarmentCode spec JSON",
    )
    parser.add_argument(
        "--z-offset", type=float, default=0.131, dest="z_offset",
        help="Body Z offset in metres (default: 0.131)",
    )
    parser.add_argument(
        "--res", type=float, default=1.5,
        help="Mesh resolution in cm (default: 1.5)",
    )
    parser.add_argument(
        "--verbose", action="store_true",
        help="Print 3D position of every stitch vertex pair",
    )
    args = parser.parse_args()

    # numpy import deferred so Taichi init prints stay out of argparse output
    run_analysis(
        pattern_path=args.pattern,
        mesh_resolution=args.res,
        body_z_offset=args.z_offset,
        verbose=args.verbose,
    )


if __name__ == "__main__":
    main()
