"""
Verify that GarmentCode panels are correctly positioned relative to the
mannequin body mesh after applying the Z offset.

For each panel in the pattern this script prints:
  - The panel's Y and Z bounding box in world space
  - The body surface bounds (z_front / z_back) at that panel's median Y
  - A PASS/FAIL indicator:
      * Front panels: mean Z should be > body z_back (panel wraps around front)
      * Back panels:  mean Z should be < body z_front (panel wraps around back)
      * Sleeve panels: skipped (rotated panels, Z interpretation differs)

Usage (run from backend/ with virtualenv active):
    python -m scripts.verify_gc_alignment
    python -m scripts.verify_gc_alignment --pattern data/patterns/garmentcode/shirt_mean.json
    python -m scripts.verify_gc_alignment --z-offset 0.0   # check without offset
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def run_verify(
    pattern_path: str = "data/patterns/garmentcode/shirt_mean.json",
    mesh_resolution: float = 2.0,
    body_z_offset: float = 0.131,
    profile_path: str = "data/bodies/mannequin_profile.json",
) -> bool:
    """
    Load pattern, apply Z offset, and check each panel against body bounds.

    Returns True if all checked panels pass alignment, False otherwise.
    """
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc
    from simulation.mesh.body_measurements import load_profile

    print(f"\n{'='*60}")
    print(f"GC Alignment Verification")
    print(f"{'='*60}")
    print(f"  Pattern:      {pattern_path}")
    print(f"  Body Z offset: {body_z_offset:+.4f} m")
    print(f"  Mesh res:     {mesh_resolution} cm")

    # --- Load body profile for surface reference bounds ---
    try:
        profile = load_profile(profile_path)
        profile_loaded = True
    except FileNotFoundError:
        print(f"\n  Warning: profile not found at {profile_path} — body bounds unavailable")
        profile_loaded = False

    # --- Build mesh with given Z offset ---
    gm = build_garment_mesh_gc(
        pattern_path,
        mesh_resolution=mesh_resolution,
        body_z_offset=body_z_offset,
    )
    n = gm.positions.shape[0]
    offsets = gm.panel_offsets + [n]

    print(f"\n  Total particles: {n}")
    print(f"  Total panels:    {len(gm.panel_ids)}")
    print(f"  Total stitches:  {gm.stitch_pairs.shape[0]}")

    print(f"\n{'─'*60}")
    print(f"  {'Panel':<26} {'Y range':>16}  {'Z range':>16}  {'Body Z ref':>18}  Status")
    print(f"{'─'*60}")

    all_pass = True
    for k, pid in enumerate(gm.panel_ids):
        ps = gm.positions[offsets[k]:offsets[k + 1]]
        y_min, y_max = float(ps[:, 1].min()), float(ps[:, 1].max())
        z_min, z_max = float(ps[:, 2].min()), float(ps[:, 2].max())
        z_mean = float(ps[:, 2].mean())
        y_mid = (y_min + y_max) / 2

        # Look up body surface bounds at mid-panel height
        if profile_loaded:
            body = profile.at_y(y_mid)
            z_back_ref = body.z_back
            z_front_ref = body.z_front
            body_ref = f"[{z_back_ref:.3f}, {z_front_ref:.3f}]"
        else:
            z_back_ref = 0.034
            z_front_ref = 0.279
            body_ref = "[0.034, 0.279] (est)"

        # Determine expected side from panel name
        pid_lower = pid.lower()
        is_sleeve = "sleeve" in pid_lower
        is_front = "ftorso" in pid_lower or "front" in pid_lower
        is_back = "btorso" in pid_lower or "back" in pid_lower
        is_hood = "hood" in pid_lower

        if is_sleeve or is_hood:
            # Rotated panels — Z interpretation is different, skip geometry check
            status = "SKIP (rotated)"
        elif is_front:
            # Front panels should be outside the body front face or nearby
            ok = z_mean > z_back_ref
            status = "PASS ✅" if ok else "FAIL ❌ (inside body?)"
            all_pass = all_pass and ok
        elif is_back:
            # Back panels should be on the back side of the body
            ok = z_mean < z_front_ref
            status = "PASS ✅" if ok else "FAIL ❌ (inside body?)"
            all_pass = all_pass and ok
        else:
            status = "SKIP (unknown)"

        print(
            f"  {pid:<26} "
            f"Y=[{y_min:.3f},{y_max:.3f}]  "
            f"Z=[{z_min:.3f},{z_max:.3f}]  "
            f"{body_ref:>18}  {status}"
        )

    print(f"{'─'*60}")

    # Summary
    print(f"\n  Overall: {'ALL PANELS PASS ✅' if all_pass else 'SOME PANELS FAILED ❌'}")
    if not all_pass:
        print(f"\n  Tip: try adjusting --z-offset (current: {body_z_offset:.4f})")
        print(f"       The correct offset = mannequin_center_z - gc_center_z")
        print(f"       mannequin centre ≈ (chest_z_front + chest_z_back) / 2")
        print(f"       gc centre ≈ (front_panel_z + back_panel_z) / 2 [after cm→m]")

    # Extra: print stitch gap in rest pose (before simulation)
    if gm.stitch_pairs.shape[0] > 0:
        pa = gm.positions[gm.stitch_pairs[:, 0]]
        pb = gm.positions[gm.stitch_pairs[:, 1]]
        gaps = np.linalg.norm(pa - pb, axis=1)
        print(f"\n  Stitch gaps at rest (pre-simulation):")
        print(f"    Min:  {gaps.min()*100:.2f} cm")
        print(f"    Mean: {gaps.mean()*100:.2f} cm")
        print(f"    Max:  {gaps.max()*100:.2f} cm")
        print(f"    (The sew phase must close these within 240 frames)")

    print()
    return all_pass


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pattern",
        default="data/patterns/garmentcode/shirt_mean.json",
        help="GarmentCode spec JSON path (default: shirt_mean.json)",
    )
    parser.add_argument(
        "--z-offset",
        type=float,
        default=0.131,
        dest="z_offset",
        help="Body Z offset in metres (default: 0.131)",
    )
    parser.add_argument(
        "--res",
        type=float,
        default=2.0,
        help="Mesh resolution in cm (default: 2.0)",
    )
    parser.add_argument(
        "--profile",
        default="data/bodies/mannequin_profile.json",
        help="Body profile JSON path",
    )
    args = parser.parse_args()

    # numpy must be imported after argparse in case taichi init prints
    global np
    import numpy as np

    success = run_verify(
        pattern_path=args.pattern,
        mesh_resolution=args.res,
        body_z_offset=args.z_offset,
        profile_path=args.profile,
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
