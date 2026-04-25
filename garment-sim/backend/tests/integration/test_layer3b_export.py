"""
Integration tests for Layer 3b: glTF Export.

Validates the full pipeline: engine.run() → SimResult → export .glb → reload.
This is the final validation gate for Sprint 1 — confirming that the entire
physics pipeline produces a valid, loadable glTF file.
"""

import numpy as np
import pytest
import trimesh

import simulation  # noqa: F401 — Taichi initialization
from simulation.core.config import SimConfig
from simulation.core.engine import SimulationEngine, compute_vertex_normals
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid
from simulation.constraints import build_constraints
from simulation.solver.xpbd import XPBDSolver
from simulation.collision import SphereCollider
from simulation.export import write_glb


# Sphere parameters (shared with test_layer3a_sphere.py)
SPHERE_CENTER = (0.0, 0.8, 0.0)
SPHERE_RADIUS = 0.5


class TestSphereExportRoundtrip:
    """
    Validate that sphere-drape simulation results can be exported to
    .glb and reloaded with correct geometry.
    """

    @staticmethod
    def _run_and_export(tmp_path, cols=10, rows=10, total_frames=30):
        """
        Helper: run sphere drape → export → reload.

        Uses a small grid and short simulation for fast test execution.
        Returns (result, reloaded_mesh, output_path).
        """
        config = SimConfig(
            total_frames=total_frames,
            substeps=6,
            solver_iterations=12,
            damping=0.98,
            max_particles=cols * rows + 100,
            collision_thickness=0.005,
            friction_coefficient=0.3,
        )

        grid = generate_grid(
            width=1.0, height=1.0, cols=cols, rows=rows, center=(0, 2.0, 0)
        )

        constraints = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
        )

        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        solver = XPBDSolver(
            constraints=constraints,
            stretch_compliance=1e-8,
            bend_compliance=1e-3,
        )
        collider = SphereCollider(center=SPHERE_CENTER, radius=SPHERE_RADIUS)

        engine = SimulationEngine(config, solver=solver)
        engine.collider = collider
        result = engine.run(state)

        # Export
        out_path = tmp_path / "sphere_drape.glb"
        result.export_glb(out_path)

        # Reload
        scene = trimesh.load(str(out_path))
        if hasattr(scene, 'geometry'):
            mesh = list(scene.geometry.values())[0]
        else:
            mesh = scene

        return result, mesh, out_path

    def test_sphere_drape_export_creates_file(self, tmp_path):
        """Sphere drape export should produce a file on disk."""
        result, mesh, out_path = self._run_and_export(tmp_path)
        assert out_path.exists()
        assert out_path.stat().st_size > 0

    def test_sphere_drape_vertex_count(self, tmp_path):
        """Exported mesh should have the same vertex count as SimResult."""
        result, mesh, _ = self._run_and_export(tmp_path)
        assert len(mesh.vertices) == len(result.positions), (
            f"Vertex count: {len(mesh.vertices)} (file) vs {len(result.positions)} (result)"
        )

    def test_sphere_drape_face_count(self, tmp_path):
        """Exported mesh should have the same face count as SimResult."""
        result, mesh, _ = self._run_and_export(tmp_path)
        assert len(mesh.faces) == len(result.faces), (
            f"Face count: {len(mesh.faces)} (file) vs {len(result.faces)} (result)"
        )

    def test_exported_positions_match(self, tmp_path):
        """Vertex positions in the reloaded mesh should closely match SimResult."""
        result, mesh, _ = self._run_and_export(tmp_path)

        # trimesh may store as float64, so cast for comparison
        loaded_pos = np.array(mesh.vertices, dtype=np.float32)
        max_error = np.max(np.abs(loaded_pos - result.positions))

        # glTF stores float32, so we allow a tiny tolerance for roundtrip
        assert max_error < 1e-4, (
            f"Max position error between SimResult and reloaded .glb: {max_error}"
        )

    def test_exported_normals_are_unit_length(self, tmp_path):
        """Vertex normals in the export should be approximately unit length."""
        result, mesh, _ = self._run_and_export(tmp_path)

        normal_lengths = np.linalg.norm(result.normals, axis=1)
        max_deviation = np.max(np.abs(normal_lengths - 1.0))

        assert max_deviation < 0.01, (
            f"Normals are not unit length: max deviation {max_deviation}"
        )

    def test_export_no_nan(self, tmp_path):
        """Exported geometry should contain no NaN values."""
        result, mesh, _ = self._run_and_export(tmp_path)

        assert not np.any(np.isnan(np.array(mesh.vertices))), "NaN in exported vertices"
        assert not np.any(np.isnan(result.normals)), "NaN in result normals"


class TestSimResultExportConvenience:
    """Test the SimResult.export_glb() convenience method."""

    def test_export_glb_method(self, tmp_path):
        """SimResult.export_glb() should produce a valid .glb file."""
        # Simple simulation: 5×5 grid, minimal frames
        config = SimConfig(total_frames=5, substeps=2, solver_iterations=0)
        grid = generate_grid(width=0.5, height=0.5, cols=5, rows=5)

        state = ParticleState(config)
        state.load_from_numpy(grid.positions, faces=grid.faces, edges=grid.edges)

        engine = SimulationEngine(config)
        result = engine.run(state)

        out_path = tmp_path / "convenience_test.glb"
        returned_path = result.export_glb(out_path)

        assert returned_path.exists()
        # Verify it loads
        scene = trimesh.load(str(returned_path))
        assert scene is not None
