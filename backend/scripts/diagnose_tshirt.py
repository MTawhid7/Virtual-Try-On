"""
Pre-simulation mesh diagnostic for tshirt.json.

Builds the garment mesh with the same parameters used in garment_drape
and reports:
  1. Per-panel placement bounds vs body surface reference
  2. Per-seam stitch count and initial gap statistics
  3. Overall edge quality (min/max length, ratio)

Usage:
    cd backend
    source .venv/bin/activate
    python -m scripts.diagnose_tshirt

No simulation is run — this is a pure geometry check.
"""

import numpy as np
from simulation.mesh.panel_builder import build_garment_mesh

_PATTERN = "data/patterns/tshirt.json"

# Body surface reference bounds (measured from mannequin_physics.glb, see CLAUDE.md)
BODY_FRONT_Z_MAX = 0.288   # front surface — panel must start at Z > this
BODY_BACK_Z_MIN  = 0.031   # back surface  — panel must start at Z < this


def main() -> None:
    print("=" * 60)
    print("Garment Mesh Diagnostics — tshirt.json, target_edge=0.020m")
    print("=" * 60)

    garment = build_garment_mesh(_PATTERN, resolution=20, global_scale=1.0, target_edge=0.020)

    n_total    = garment.positions.shape[0]
    n_tri      = garment.faces.shape[0]
    n_edges    = garment.edges.shape[0]
    n_stitches = garment.stitch_pairs.shape[0]

    print(f"\nTotal particles:  {n_total}")
    print(f"Total triangles:  {n_tri}")
    print(f"Total edges:      {n_edges}")
    print(f"Total stitch pairs: {n_stitches}")

    # --- Panel bounds ---
    print("\n--- Panel Placement Bounds ---")
    print(f"{'Panel':<16} {'N':>5}  {'X_min':>7} {'X_max':>7}  {'Y_min':>7} {'Y_max':>7}  {'Z_min':>7} {'Z_max':>7}")
    offsets = garment.panel_offsets + [n_total]
    for i, pid in enumerate(garment.panel_ids):
        start = offsets[i]
        end   = offsets[i + 1]
        pts   = garment.positions[start:end]
        n     = end - start
        print(
            f"  {pid:<14} {n:>5}  "
            f"{pts[:,0].min():>7.3f} {pts[:,0].max():>7.3f}  "
            f"{pts[:,1].min():>7.3f} {pts[:,1].max():>7.3f}  "
            f"{pts[:,2].min():>7.3f} {pts[:,2].max():>7.3f}"
        )

    # --- Panel placement check vs body surface ---
    print("\n--- Panel Placement vs Body Surface ---")
    print(f"  Body front surface Z_max = {BODY_FRONT_Z_MAX:.3f}m  (front panel must start OUTSIDE: Z > {BODY_FRONT_Z_MAX:.3f})")
    print(f"  Body back  surface Z_min = {BODY_BACK_Z_MIN:.3f}m  (back panel must start OUTSIDE: Z < {BODY_BACK_Z_MIN:.3f})")

    for i, pid in enumerate(garment.panel_ids):
        start = offsets[i]
        end   = offsets[i + 1]
        pts   = garment.positions[start:end]
        z_min = float(pts[:, 2].min())
        z_max = float(pts[:, 2].max())

        if "front" in pid:
            ok = z_min > BODY_FRONT_Z_MAX
            status = "PASS" if ok else "FAIL — panel overlaps front body surface!"
            print(f"  {pid}: Z_min={z_min:.3f}m  →  {status}")
        elif "back" in pid:
            ok = z_max < BODY_BACK_Z_MIN
            status = "PASS" if ok else "FAIL — panel overlaps back body surface!"
            print(f"  {pid}: Z_max={z_max:.3f}m  →  {status}")
        elif "sleeve" in pid:
            # Sleeves wrap around arm; just report their Z range
            print(f"  {pid}: Z=[{z_min:.3f}, {z_max:.3f}]m  (cylindrical wrap — no simple pass/fail)")

    # --- Per-seam stitch stats ---
    print("\n--- Per-Seam Stitch Diagnostics (initial gaps) ---")
    seam_ids = garment.stitch_seam_ids

    if seam_ids is None:
        print("  [no seam label metadata — stitch_seam_ids is None]")
    else:
        import collections
        # Group stitch pair indices by seam label (preserving order of first appearance)
        seam_to_indices: dict[str, list[int]] = collections.OrderedDict()
        for pair_idx, label in enumerate(seam_ids):
            seam_to_indices.setdefault(label, []).append(pair_idx)

        hdr = f"  {'Seam':<48} {'N':>4}  {'Gap_min':>8} {'Gap_mean':>9} {'Gap_max':>8}"
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))

        all_pass = True
        for label, indices in seam_to_indices.items():
            pa = garment.positions[garment.stitch_pairs[indices, 0]]
            pb = garment.positions[garment.stitch_pairs[indices, 1]]
            gaps = np.linalg.norm(pa - pb, axis=1)
            n    = len(indices)
            gmin = gaps.min() * 100
            gmean= gaps.mean() * 100
            gmax = gaps.max() * 100
            warn = " ← LARGE" if gmax > 35.0 else ""
            print(f"  {label:<48} {n:>4}  {gmin:>7.1f}cm {gmean:>8.1f}cm {gmax:>7.1f}cm{warn}")
            if gmax > 35.0:
                all_pass = False

        print()
        if all_pass:
            print("  All seam initial gaps ≤ 35cm — within sew-phase closure range.")
        else:
            print("  WARNING: Some seams have initial gaps > 35cm.")
            print("  These may not fully close in 150 sew frames.")

    # --- Edge quality ---
    print("\n--- Edge Quality ---")
    edge_lengths = np.linalg.norm(
        garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]], axis=1
    )
    e_min   = edge_lengths.min() * 100
    e_max   = edge_lengths.max() * 100
    e_mean  = edge_lengths.mean() * 100
    e_ratio = edge_lengths.max() / max(edge_lengths.min(), 1e-8)
    ratio_ok = e_ratio < 10.0
    print(f"  min={e_min:.2f}cm  mean={e_mean:.2f}cm  max={e_max:.2f}cm  ratio={e_ratio:.1f}x  {'PASS' if ratio_ok else 'WARN — high ratio, may cause instability'}")

    print("\n" + "=" * 60)
    print("Share the output above, then run the static simulation:")
    print("  python -m simulation --scene garment_drape")
    print("=" * 60)


if __name__ == "__main__":
    main()
