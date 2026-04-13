"""
Integration tests for Sprint 2 Layer 3b-Extended Phase 4: Garment Drape.

Validates the full garment pipeline:
    pattern JSON → build_garment_mesh() → ParticleState → XPBDSolver (with stitches)
    → BodyCollider → SimulationEngine.run() → results

Uses a small resolution (resolution=10) and reduced frame count (60 frames) to
keep test runtime acceptable while exercising the complete pipeline.
"""

import numpy as np
import pytest

import simulation  # noqa: F401 — Taichi initialization
from simulation.collision import BodyCollider
from simulation.constraints import build_constraints
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine
from simulation.core.state import ParticleState
from simulation.materials import FABRIC_PRESETS
from simulation.mesh.grid import compute_area_weighted_inv_masses
from simulation.mesh.panel_builder import build_garment_mesh
from simulation.solver.xpbd import XPBDSolver


_BODY_GLB = "data/bodies/mannequin_physics.glb"
_TANK_TOP_JSON = "data/patterns/tank_top.json"

# Reduced parameters for test speed
_TEST_RESOLUTION = 10   # ~60 particles
_TEST_FRAMES = 150
_TEST_SUBSTEPS = 12
_TEST_ITERATIONS = 8


def _run_garment_drape(
    total_frames: int = _TEST_FRAMES,
    resolution: int = _TEST_RESOLUTION,
    with_body: bool = True,
    stitch_compliance: float = 1e-6,
):
    """
    Shared helper: build garment mesh, wire up solver + collider, run engine.

    Returns:
        (final_positions, garment, state, config)
    """
    fabric = FABRIC_PRESETS["cotton"]

    garment = build_garment_mesh(_TANK_TOP_JSON, resolution=resolution)
    n_particles = garment.positions.shape[0]

    config = SimConfig(
        total_frames=total_frames,
        substeps=_TEST_SUBSTEPS,
        solver_iterations=_TEST_ITERATIONS,
        damping=fabric.damping,
        max_particles=n_particles + 50,
        collision_thickness=0.008,
        friction_coefficient=fabric.friction,
        air_drag=0.3,
        sew_frames=total_frames // 2,
        sew_stitch_compliance=1e-9,
        drape_stitch_compliance=stitch_compliance,
    )

    inv_masses = compute_area_weighted_inv_masses(
        garment.positions, garment.faces, fabric.density
    )

    n_stitches = garment.stitch_pairs.shape[0]
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
        stretch_compliance=fabric.stretch_compliance,
        bend_compliance=fabric.bend_compliance,
        stitch_compliance=stitch_compliance,
        max_stretch=fabric.max_stretch,
        max_compress=fabric.max_compress,
    )

    engine = SimulationEngine(config, solver=solver)
    if with_body:
        engine.collider = BodyCollider.from_glb(_BODY_GLB, decimate_target=5000)

    result = engine.run(state)
    return result, garment, state, config


class TestGarmentMeshSetup:
    """Verify build_garment_mesh produces a valid mesh before simulation."""

    def test_garment_mesh_loads(self):
        """build_garment_mesh() succeeds and returns correct panel count."""
        garment = build_garment_mesh(_TANK_TOP_JSON, resolution=_TEST_RESOLUTION)
        assert len(garment.panel_ids) == 2
        assert "front" in garment.panel_ids
        assert "back" in garment.panel_ids

    def test_garment_has_stitch_pairs(self):
        """tank_top.json should produce stitch pairs for left and right seams."""
        garment = build_garment_mesh(_TANK_TOP_JSON, resolution=_TEST_RESOLUTION)
        assert garment.stitch_pairs.shape[0] > 0, "Expected stitch pairs from side seams"
        assert garment.stitch_pairs.shape[1] == 2

    def test_garment_particles_in_expected_bbox(self):
        """
        Front panel at Z=+0.30, back at Z=+0.02, Y range 0.85–1.55m.
        All particles should be within an expanded bounding box.
        """
        garment = build_garment_mesh(_TANK_TOP_JSON, resolution=_TEST_RESOLUTION)
        pos = garment.positions

        assert np.all(pos[:, 1] >= 0.80), "Particle below expected Y min"
        assert np.all(pos[:, 1] <= 1.60), "Particle above expected Y max"
        # Z spread: front at +0.30, back at +0.02 — both in positive Z
        # (mannequin body is entirely in positive Z, not centered at Z=0)
        assert np.min(pos[:, 2]) >= -0.05, "Particle below expected Z min"
        assert np.max(pos[:, 2]) > 0.0, "Front panel should have positive Z"

    def test_stitch_pairs_reference_valid_indices(self):
        """All stitch pair vertex indices must be within [0, n_particles)."""
        garment = build_garment_mesh(_TANK_TOP_JSON, resolution=_TEST_RESOLUTION)
        n = garment.positions.shape[0]
        assert np.all(garment.stitch_pairs >= 0)
        assert np.all(garment.stitch_pairs < n)


class TestGarmentDrapeSimulation:
    """Full pipeline integration: garment drapes onto body mesh."""

    def test_no_nan_after_simulation(self):
        """Final positions must not contain NaN or Inf."""
        result, _, _, _ = _run_garment_drape()
        assert not np.any(np.isnan(result.positions)), "NaN in positions"
        assert not np.any(np.isinf(result.positions)), "Inf in positions"

    def test_no_sub_floor_penetration(self):
        """No particle should fall significantly below Y=0 (body base)."""
        result, _, _, config = _run_garment_drape()
        min_y = float(np.min(result.positions[:, 1]))
        tolerance = config.collision_thickness * 2
        assert min_y >= -tolerance, (
            f"Sub-floor penetration: min_y={min_y:.4f}m, tolerance=-{tolerance:.4f}m"
        )

    def test_garment_settles_on_body(self):
        """
        After simulation, a meaningful fraction of particles should be in the
        torso region (Y=0.8–1.6m) where the body mesh provides support.
        """
        result, garment, _, _ = _run_garment_drape(total_frames=_TEST_FRAMES)
        pos = result.positions
        n = pos.shape[0]
        in_torso = int(np.sum((pos[:, 1] >= 0.8) & (pos[:, 1] <= 1.6)))
        assert in_torso > n * 0.20, (
            f"Only {in_torso}/{n} particles in torso region — garment may have fallen through"
        )

    def test_stitches_closed_after_settling(self):
        """
        Side seam stitch pairs should converge to near-zero gap after 60 frames
        of stitch constraint projection.
        """
        result, garment, _, _ = _run_garment_drape(total_frames=_TEST_FRAMES)
        if garment.stitch_pairs.shape[0] == 0:
            pytest.skip("No stitch pairs in pattern")

        pa = result.positions[garment.stitch_pairs[:, 0]]
        pb = result.positions[garment.stitch_pairs[:, 1]]
        gaps = np.linalg.norm(pa - pb, axis=1)
        max_gap = float(np.max(gaps))
        # 5cm threshold — conservative for only 60 frames at lower resolution
        assert max_gap < 0.05, (
            f"Stitch seam not closed: max gap = {max_gap * 100:.2f}cm (threshold 5cm)"
        )

    def test_energy_decay(self):
        """Mean particle speed should be low after settling."""
        result, _, state, _ = _run_garment_drape(total_frames=_TEST_FRAMES)
        velocities = state.get_velocities_numpy()
        mean_speed = float(np.mean(np.linalg.norm(velocities, axis=1)))
        assert mean_speed < 2.0, (
            f"Mean speed {mean_speed:.4f} m/s — garment not settling"
        )

    def test_edge_length_preservation(self):
        """Distance constraints should keep fabric near its rest shape (< 20% mean stretch)."""
        result, garment, _, _ = _run_garment_drape()
        pos = result.positions
        edge_lengths = np.linalg.norm(
            pos[garment.edges[:, 1]] - pos[garment.edges[:, 0]], axis=1
        )
        rest_lengths = np.linalg.norm(
            garment.positions[garment.edges[:, 1]] - garment.positions[garment.edges[:, 0]],
            axis=1,
        )
        mean_stretch = float(np.mean(np.abs(edge_lengths / rest_lengths - 1.0)))
        assert mean_stretch < 0.20, (
            f"Mean stretch {mean_stretch:.4%} exceeds 20% threshold"
        )

    def test_simulation_without_body_collider(self):
        """Scene should run and produce valid output even without body collision."""
        result, garment, _, _ = _run_garment_drape(total_frames=30, with_body=False)
        assert not np.any(np.isnan(result.positions))
        # Without body, garment falls under gravity — all particles should move down
        delta_y = np.mean(result.positions[:, 1]) - np.mean(garment.positions[:, 1])
        assert delta_y < 0, "Garment should fall under gravity"


class TestGarmentDrapeScene:
    """Test the run_garment_drape() scene function directly."""

    def test_run_garment_drape_exports_glb(self, tmp_path):
        """run_garment_drape() completes and writes a GLB file."""
        from simulation.scenes.garment_drape import run_garment_drape

        out = tmp_path / "garment_drape_test.glb"
        # Use very short run to keep test fast
        import unittest.mock as mock
        with mock.patch(
            "simulation.scenes.garment_drape.SimConfig",
            wraps=lambda **kwargs: SimConfig(**{**kwargs, "total_frames": 10}),
        ):
            run_garment_drape(output_path=str(out))

        assert out.exists(), "GLB file not created"
        assert out.stat().st_size > 0, "GLB file is empty"
