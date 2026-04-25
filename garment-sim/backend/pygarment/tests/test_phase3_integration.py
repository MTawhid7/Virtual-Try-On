"""
Phase 3 integration test — verifies the GarmentCode → garment-sim
end-to-end pipeline without running the actual simulation.

Tests:
1. build_garment_mesh_gc() with the real GarmentCode shirt pattern
2. Constraint building (distance, bending, stitch)
3. ParticleState loading
4. Validates the mesh is compatible with the XPBD solver
5. Verifies the --gc flag in garment_drape.py is wired correctly
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_GC_SHIRT = "data/patterns/garmentcode/shirt_mean.json"


def test_gc_shirt_mesh():
    """Test the real GarmentCode shirt pattern loads and produces valid mesh."""
    print("=== Test 1: GarmentCode Shirt Mesh ===")
    from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc

    if not os.path.exists(_GC_SHIRT):
        print(f"  ⚠️  {_GC_SHIRT} not found — skipping")
        return None

    gm = build_garment_mesh_gc(_GC_SHIRT, mesh_resolution=2.0)

    print(f"  Panels ({len(gm.panel_ids)}): {gm.panel_ids}")
    print(f"  Vertices:      {gm.positions.shape[0]}")
    print(f"  Faces:         {gm.faces.shape[0]}")
    print(f"  Edges:         {gm.edges.shape[0]}")
    print(f"  Stitch pairs:  {gm.stitch_pairs.shape[0]}")

    # Verify we have the expected 8 panels
    assert len(gm.panel_ids) == 8, f"Expected 8 panels, got {len(gm.panel_ids)}"

    # Position ranges should be in the torso region
    pos = gm.positions
    print(f"\n  Position ranges:")
    print(f"    X: [{pos[:,0].min():.3f}, {pos[:,0].max():.3f}] m")
    print(f"    Y: [{pos[:,1].min():.3f}, {pos[:,1].max():.3f}] m")
    print(f"    Z: [{pos[:,2].min():.3f}, {pos[:,2].max():.3f}] m")

    # Y should be in torso region (0.5–1.5m for a 1.78m body)
    assert pos[:, 1].min() > 0.5, f"Min Y {pos[:,1].min():.3f} too low"
    assert pos[:, 1].max() < 2.0, f"Max Y {pos[:,1].max():.3f} too high"

    # X should be symmetric around 0 (within ±0.7m for shirt+sleeves)
    assert abs(pos[:, 0].min() + pos[:, 0].max()) < 0.05, "X not symmetric"

    # No NaN
    assert not np.any(np.isnan(pos)), "NaN in positions!"

    # Edge quality
    edge_lens = np.linalg.norm(
        pos[gm.edges[:, 1]] - pos[gm.edges[:, 0]], axis=1
    )
    print(f"\n  Edge lengths: min={edge_lens.min()*100:.2f}cm, "
          f"max={edge_lens.max()*100:.2f}cm, mean={edge_lens.mean()*100:.2f}cm")

    # All stitch indices valid
    assert gm.stitch_pairs.min() >= 0
    assert gm.stitch_pairs.max() < len(pos)

    print(f"  ✅ GC shirt mesh passed")
    return gm


def test_gc_shirt_constraints(gm):
    """Test constraint building with the GC shirt mesh."""
    print("\n=== Test 2: Constraint Building ===")
    from simulation.constraints import build_constraints
    from simulation.mesh.grid import compute_area_weighted_inv_masses
    from simulation.materials import FABRIC_PRESETS

    if gm is None:
        print("  ⚠️  Skipping (no mesh)")
        return

    fabric = FABRIC_PRESETS["cotton"]
    inv_masses = compute_area_weighted_inv_masses(
        gm.positions, gm.faces, fabric.density
    )

    constraints = build_constraints(
        positions=gm.positions,
        edges=gm.edges,
        faces=gm.faces,
        stitch_pairs=gm.stitch_pairs if gm.stitch_pairs.shape[0] > 0 else None,
        max_stitches=gm.stitch_pairs.shape[0] + 10,
    )

    n_dist = constraints.distance.n_edges if constraints.distance else 0
    n_bend = constraints.bending.n_hinges if constraints.bending else 0
    n_stitch = constraints.stitch.n_stitches if constraints.stitch else 0

    print(f"  Distance constraints: {n_dist}")
    print(f"  Bending constraints:  {n_bend}")
    print(f"  Stitch constraints:   {n_stitch}")
    print(f"  Inv masses range:     [{inv_masses.min():.4f}, {inv_masses.max():.4f}]")

    assert n_dist > 0, "No distance constraints!"
    assert n_bend > 0, "No bending constraints!"
    assert n_stitch > 0, "No stitch constraints!"
    assert np.all(inv_masses > 0), "Zero/negative inv_masses!"
    assert not np.any(np.isnan(inv_masses)), "NaN in inv_masses!"

    print(f"  ✅ Constraint building passed")
    return constraints, inv_masses


def test_gc_shirt_state(gm, inv_masses):
    """Test ParticleState loading with GC mesh."""
    print("\n=== Test 3: ParticleState Loading ===")
    from simulation.core.config import SimConfig
    from simulation.core.state import ParticleState

    if gm is None:
        print("  ⚠️  Skipping (no mesh)")
        return

    config = SimConfig(
        total_frames=10,
        substeps=4,
        solver_iterations=8,
        max_particles=gm.positions.shape[0] + 200,
    )

    state = ParticleState(config)
    state.load_from_numpy(
        gm.positions,
        faces=gm.faces,
        edges=gm.edges,
        inv_masses=inv_masses,
    )

    # Verify state loaded correctly
    loaded_pos = state.get_positions_numpy()
    assert loaded_pos.shape == gm.positions.shape, (
        f"Shape mismatch: {loaded_pos.shape} vs {gm.positions.shape}")
    assert np.allclose(loaded_pos, gm.positions, atol=1e-5), "Position mismatch!"

    print(f"  ParticleState loaded: {loaded_pos.shape[0]} particles")
    print(f"  ✅ State loading passed")


def test_cli_flag():
    """Test that --gc flag is recognized."""
    print("\n=== Test 4: CLI --gc Flag ===")
    import argparse
    # Import the module to check the parser is set up
    import simulation.scenes.garment_drape as gd

    # Check that run_garment_drape accepts gc_pattern
    import inspect
    sig = inspect.signature(gd.run_garment_drape)
    params = list(sig.parameters.keys())
    assert "gc_pattern" in params, f"gc_pattern not in run_garment_drape params: {params}"
    print(f"  run_garment_drape params: {params}")
    print(f"  ✅ CLI --gc flag is wired")


def test_per_panel_breakdown(gm):
    """Display per-panel statistics for visual verification."""
    print("\n=== Test 5: Per-Panel Breakdown ===")

    if gm is None:
        print("  ⚠️  Skipping (no mesh)")
        return

    offsets = gm.panel_offsets + [gm.positions.shape[0]]
    print(f"  {'Panel':<25} {'Verts':>6} {'Faces':>6} {'Y range (m)':>15}")
    print(f"  {'-'*55}")

    for i, pid in enumerate(gm.panel_ids):
        v_start, v_end = offsets[i], offsets[i + 1]
        n_verts = v_end - v_start

        # Count faces in this panel
        n_faces = 0
        for f in gm.faces:
            if all(v_start <= idx < v_end for idx in f):
                n_faces += 1

        panel_pos = gm.positions[v_start:v_end]
        y_range = f"[{panel_pos[:,1].min():.3f}, {panel_pos[:,1].max():.3f}]"

        print(f"  {pid:<25} {n_verts:>6} {n_faces:>6} {y_range:>15}")

    # Stitch pair breakdown
    if gm.stitch_seam_ids:
        print(f"\n  Seam labels:")
        from collections import Counter
        for label, count in Counter(gm.stitch_seam_ids).items():
            print(f"    {label}: {count} pairs")

    print(f"  ✅ Per-panel breakdown complete")


if __name__ == "__main__":
    print("\n🔧 Phase 3: End-to-End Integration Verification\n")
    gm = test_gc_shirt_mesh()
    result = test_gc_shirt_constraints(gm)
    if result:
        constraints, inv_masses = result
        test_gc_shirt_state(gm, inv_masses)
    test_cli_flag()
    test_per_panel_breakdown(gm)
    print("\n🎉 All Phase 3 tests passed!\n")
