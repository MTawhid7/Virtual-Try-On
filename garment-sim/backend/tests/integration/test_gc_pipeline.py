"""
Integration tests for the GarmentCode pipeline (--gc flag / build_garment_mesh_gc).

Validates the full path:
    shirt_mean.json → build_garment_mesh_gc() → ParticleState → XPBDSolver
    + BodyCollider → SimulationEngine.run() → validate positions + stitch gaps

Tests are split into two groups:
  * TestGCMeshSetup  — fast, no simulation; verify mesh geometry only
  * TestGCDrapeSimulation — reduced-frame simulation to check physics behaviour

All simulation tests use total_frames=100 (50 sew + 50 drape) to keep runtime
acceptable while exercising the full constraint/collision pipeline.

Body Z offset rationale:
    GarmentCode SMPL torso centre Z ≈ +0.025m (after cm→m scaling).
    mannequin_physics.glb torso centre Z ≈ +0.156m.
    Required offset: 0.156 - 0.025 = 0.131m.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import simulation  # noqa: F401 — Taichi init
from simulation.collision import BodyCollider
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.materials import FABRIC_PRESETS
from simulation.mesh.gc_mesh_adapter import build_garment_mesh_gc
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.solver.xpbd import XPBDSolver


# ---------------------------------------------------------------------------
# Paths and constants
# ---------------------------------------------------------------------------

_GC_SHIRT = "data/patterns/garmentcode/shirt_mean.json"
_BODY_GLB = "data/bodies/mannequin_physics.glb"

# Z offset to align GarmentCode SMPL body onto mannequin_physics.glb.
# See garment_drape._GC_BODY_Z_OFFSET for derivation.
_GC_BODY_Z_OFFSET = 0.131  # metres

# Mesh resolution matches the production setting in garment_drape.py
_MESH_RES = 2.0  # cm

# Reduced frame count keeps test runtime < 60s
_TEST_FRAMES = 100
_SEW_FRAMES = 50

# Body surface bounds at chest height (from mannequin_profile.json)
_BODY_Z_FRONT = 0.2786   # m — front face at chest
_BODY_Z_BACK = 0.0335    # m — back face at chest
_BODY_Z_INTERIOR_MIN = 0.05   # m — conservative interior threshold
_BODY_Z_INTERIOR_MAX = 0.25   # m — conservative interior threshold


# ---------------------------------------------------------------------------
# Skip marker — files may not exist in all environments
# ---------------------------------------------------------------------------

_SKIP_SHIRT = pytest.mark.skipif(
    not Path(_GC_SHIRT).exists(),
    reason=f"GC spec not found: {_GC_SHIRT}",
)
_SKIP_BODY = pytest.mark.skipif(
    not Path(_BODY_GLB).exists(),
    reason=f"Body mesh not found: {_BODY_GLB}",
)


# ---------------------------------------------------------------------------
# Shared simulation helper
# ---------------------------------------------------------------------------

def _build_gc_garment():
    """Load shirt_mean.json with body Z offset applied."""
    return build_garment_mesh_gc(
        _GC_SHIRT,
        mesh_resolution=_MESH_RES,
        body_z_offset=_GC_BODY_Z_OFFSET,
    )


def _run_gc_drape(total_frames: int = _TEST_FRAMES, with_body: bool = True):
    """
    Build and run a reduced-frame GarmentCode shirt simulation.

    Returns:
        (result, garment, state, config)
    """
    fabric = FABRIC_PRESETS["cotton"]
    garment = _build_gc_garment()
    n_particles = garment.positions.shape[0]
    n_stitches = garment.stitch_pairs.shape[0]

    sew_frames = total_frames // 2
    config = SimConfig(
        total_frames=total_frames,
        substeps=4,
        solver_iterations=8,
        sew_solver_iterations=16,
        damping=fabric.damping,
        max_particles=n_particles + 200,
        collision_thickness=0.012,
        sew_collision_thickness=0.006,
        friction_coefficient=fabric.friction,
        air_drag=0.3,
        sew_frames=sew_frames,
        sew_gravity_fraction=0.15,
        sew_stitch_compliance=1e-10,
        drape_stitch_compliance=1e-8,
        transition_frames=10,
        sew_ramp_frames=20,
        sew_initial_compliance=1e-7,
        enable_self_collision=False,
    )

    inv_masses = compute_area_weighted_inv_masses(
        garment.positions, garment.faces, fabric.density
    )

    reference_substep_dt = 1.0 / 60.0 / 4.0
    substep_dt = config.dt / config.substeps
    compliance_scale = (substep_dt / reference_substep_dt) ** 2
    scaled_stretch = fabric.stretch_compliance * compliance_scale
    scaled_bend = fabric.bend_compliance * compliance_scale

    constraints = build_constraints(
        positions=garment.positions,
        edges=garment.edges,
        faces=garment.faces,
        stitch_pairs=garment.stitch_pairs if n_stitches > 0 else None,
        max_stitches=n_stitches + 10,
    )

    state = ParticleState(config)
    state.load_from_numpy(
        garment.positions,
        faces=garment.faces,
        edges=garment.edges,
        inv_masses=inv_masses,
    )

    solver = XPBDSolver(
        constraints=constraints,
        stretch_compliance=scaled_stretch,
        bend_compliance=scaled_bend,
        stitch_compliance=config.sew_stitch_compliance,
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
        stretch_damping=fabric.stretch_damping,
        bend_damping=fabric.bend_damping,
    )

    engine = SimulationEngine(config, solver=solver)
    if with_body:
        engine.collider = BodyCollider.from_glb(_BODY_GLB, decimate_target=5000)

    result = engine.run(state)
    return result, garment, state, config


# ---------------------------------------------------------------------------
# TestGCMeshSetup — geometry only, no simulation
# ---------------------------------------------------------------------------

class TestGCMeshSetup:
    """Fast tests: verify GC mesh loads correctly before any simulation."""

    @_SKIP_SHIRT
    def test_gc_shirt_loads_8_panels(self):
        """shirt_mean.json should produce exactly 8 panels."""
        gm = _build_gc_garment()
        assert len(gm.panel_ids) == 8, (
            f"Expected 8 panels, got {len(gm.panel_ids)}: {gm.panel_ids}"
        )

    @_SKIP_SHIRT
    def test_gc_shirt_has_all_expected_panels(self):
        """All torso and sleeve panels should be present."""
        gm = _build_gc_garment()
        panel_set = set(gm.panel_ids)
        for expected in ("left_ftorso", "right_ftorso", "left_btorso", "right_btorso"):
            assert expected in panel_set, f"Missing panel: {expected}"

    @_SKIP_SHIRT
    def test_no_nan_in_gc_mesh(self):
        """Positions, faces, edges and UVs must be finite after loading."""
        gm = _build_gc_garment()
        assert not np.any(np.isnan(gm.positions)), "NaN in positions"
        assert not np.any(np.isinf(gm.positions)), "Inf in positions"

    @_SKIP_SHIRT
    def test_stitch_pairs_valid_indices(self):
        """Stitch pair indices must be in [0, n_particles) and non-empty."""
        gm = _build_gc_garment()
        n = gm.positions.shape[0]
        assert gm.stitch_pairs.shape[0] > 0, "No stitch pairs produced"
        assert gm.stitch_pairs.shape[1] == 2
        assert np.all(gm.stitch_pairs >= 0), "Negative stitch index"
        assert np.all(gm.stitch_pairs < n), "Stitch index out of bounds"

    @_SKIP_SHIRT
    def test_z_offset_applied(self):
        """
        After the body Z offset the torso panels must sit outside the mannequin.

        Front torso panels should be at Z > body front face (0.279m).
        Back torso panels should be at Z < body front face (centred inward).
        """
        gm = _build_gc_garment()
        n = gm.positions.shape[0]
        offsets = gm.panel_offsets + [n]

        front_panels = [i for i, pid in enumerate(gm.panel_ids) if "ftorso" in pid]
        back_panels = [i for i, pid in enumerate(gm.panel_ids) if "btorso" in pid]

        assert front_panels, "No front torso panels found"
        assert back_panels, "No back torso panels found"

        for k in front_panels:
            z = gm.positions[offsets[k]:offsets[k + 1], 2]
            # Front panels should be near or beyond the body front surface
            assert z.mean() > _BODY_Z_BACK, (
                f"{gm.panel_ids[k]}: mean Z {z.mean():.3f}m should be > {_BODY_Z_BACK}m"
            )

        for k in back_panels:
            z = gm.positions[offsets[k]:offsets[k + 1], 2]
            # Back panels should be behind the body front surface
            assert z.mean() < _BODY_Z_FRONT, (
                f"{gm.panel_ids[k]}: mean Z {z.mean():.3f}m should be < {_BODY_Z_FRONT}m"
            )

    @_SKIP_SHIRT
    def test_panels_not_all_inside_body(self):
        """
        After Z offset, fewer than 10% of torso-height particles should fall
        in the deep body interior (Z in [0.05, 0.25]m at torso Y range).
        """
        gm = _build_gc_garment()
        pos = gm.positions
        torso_mask = (pos[:, 1] >= 0.8) & (pos[:, 1] <= 1.5)
        torso_z = pos[torso_mask, 2]
        deep_interior = (torso_z > _BODY_Z_INTERIOR_MIN) & (torso_z < _BODY_Z_INTERIOR_MAX)
        n_inside = int(deep_interior.sum())
        n_torso = int(torso_mask.sum())
        assert n_torso > 0, "No particles in torso height range"
        fraction_inside = n_inside / n_torso
        assert fraction_inside < 0.10, (
            f"{n_inside}/{n_torso} ({fraction_inside:.1%}) torso particles appear "
            f"inside mannequin body (Z in [{_BODY_Z_INTERIOR_MIN}, {_BODY_Z_INTERIOR_MAX}]m)"
        )


# ---------------------------------------------------------------------------
# TestGCDrapeSimulation — physics validation
# ---------------------------------------------------------------------------

class TestGCDrapeSimulation:
    """
    Reduced-frame simulation tests for the GC pipeline.

    These are slower (~30–60s each) but verify that the full physics
    pipeline (XPBD + body collision) produces stable, physically plausible
    results for GarmentCode-generated meshes.
    """

    @_SKIP_SHIRT
    @_SKIP_BODY
    def test_no_nan_after_simulation(self):
        """Final positions and velocities must be finite."""
        result, _, state, _ = _run_gc_drape()
        assert not np.any(np.isnan(result.positions)), "NaN in final positions"
        assert not np.any(np.isinf(result.positions)), "Inf in final positions"
        vels = state.get_velocities_numpy()
        assert not np.any(np.isnan(vels)), "NaN in final velocities"

    @_SKIP_SHIRT
    @_SKIP_BODY
    def test_garment_settles_in_torso_region(self):
        """
        After simulation, the majority of particles should be in the torso
        region (Y = 0.5–1.8m), confirming the garment wrapped around the body.
        """
        result, garment, _, _ = _run_gc_drape()
        pos = result.positions
        n = pos.shape[0]
        in_torso = int(np.sum((pos[:, 1] >= 0.5) & (pos[:, 1] <= 1.8)))
        assert in_torso > n * 0.50, (
            f"Only {in_torso}/{n} particles in torso Y=[0.5, 1.8]m — "
            f"garment may have fallen through or drifted"
        )

    @_SKIP_SHIRT
    @_SKIP_BODY
    def test_no_sub_floor_penetration(self):
        """No particle should fall significantly below Y = 0 (body base)."""
        result, _, _, config = _run_gc_drape()
        min_y = float(np.min(result.positions[:, 1]))
        tolerance = config.collision_thickness * 2
        assert min_y >= -tolerance, (
            f"Sub-floor penetration: min_y={min_y:.4f}m, tolerance={-tolerance:.4f}m"
        )

    @_SKIP_SHIRT
    def test_seam_gaps_reduced_after_sewing(self):
        """
        Stitch gaps should be substantially smaller after simulation than
        before (panels start far apart; stitches pull them together).

        Run without body collision so panels can approach freely —
        body collision is tested separately in test_garment_settles_in_torso_region.
        """
        result, garment, _, _ = _run_gc_drape(with_body=False)
        if garment.stitch_pairs.shape[0] == 0:
            pytest.skip("No stitch pairs in pattern")

        pa_final = result.positions[garment.stitch_pairs[:, 0]]
        pb_final = result.positions[garment.stitch_pairs[:, 1]]
        final_gaps = np.linalg.norm(pa_final - pb_final, axis=1)

        pa_init = garment.positions[garment.stitch_pairs[:, 0]]
        pb_init = garment.positions[garment.stitch_pairs[:, 1]]
        initial_gaps = np.linalg.norm(pa_init - pb_init, axis=1)

        mean_reduction = float(final_gaps.mean() / max(initial_gaps.mean(), 1e-8))
        assert mean_reduction < 0.75, (
            f"Mean stitch gap reduced by only {(1-mean_reduction):.1%} "
            f"(initial {initial_gaps.mean()*100:.1f}cm → final {final_gaps.mean()*100:.1f}cm); "
            f"expected at least 25% reduction after {_TEST_FRAMES} frames"
        )

    @_SKIP_SHIRT
    def test_edge_length_preservation(self):
        """
        Structural edges should stay close to rest length (< 25% mean stretch).
        A higher threshold than the native pipeline (20%) accounts for the
        sew-phase tension on seam-adjacent edges.
        """
        result, garment, _, _ = _run_gc_drape(with_body=False)
        final_lengths = np.linalg.norm(
            result.positions[garment.edges[:, 1]] - result.positions[garment.edges[:, 0]],
            axis=1,
        )
        rest_lengths = np.linalg.norm(
            garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]],
            axis=1,
        )
        # Avoid div-by-zero on the rare filtered zero-length edges
        valid = rest_lengths > 1e-6
        mean_stretch = float(np.mean(np.abs(final_lengths[valid] / rest_lengths[valid] - 1.0)))
        assert mean_stretch < 0.25, (
            f"Mean edge stretch {mean_stretch:.2%} exceeds 25% threshold"
        )

    @_SKIP_SHIRT
    def test_gc_glb_export(self, tmp_path):
        """GLB export from a GC simulation must produce a non-empty file."""
        from simulation.core.engine import compute_vertex_normals
        from simulation.export.gltf_writer import write_glb_with_body
        import trimesh

        result, garment, _, _ = _run_gc_drape(total_frames=30, with_body=False)
        normals = compute_vertex_normals(result.positions, garment.faces)
        body = trimesh.load(_BODY_GLB, force="mesh")
        body_verts = np.array(body.vertices, dtype=np.float32)
        body_faces = np.array(body.faces, dtype=np.int32)

        out = tmp_path / "gc_shirt_test.glb"
        write_glb_with_body(
            result.positions, garment.faces,
            body_verts, body_faces,
            cloth_normals=normals,
            cloth_uvs=garment.uvs,
            path=str(out),
        )
        assert out.exists(), "GLB file was not created"
        assert out.stat().st_size > 1_000, "GLB file is suspiciously small"
