import numpy as np
import pytest

import simulation  # noqa: F401
from simulation.constraints.bending import BendingConstraints, find_adjacent_triangle_pairs
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.mesh.grid import generate_grid


class TestBendingConstraint:
    """Validate bending constraint projection math."""

    def test_flat_mesh_zero_constraint(self):
        """
        A flat grid should have rest angles of 0 (or π depending on
        convention). The constraint violation should be zero — no correction
        applied to a mesh that starts in its rest configuration.
        """
        grid = generate_grid(cols=5, rows=5, center=(0, 0, 0))

        bend = BendingConstraints(max_hinges=100)
        bend.initialize(grid.faces, grid.positions)

        # Should find hinges (internal edges)
        assert bend.n_hinges > 0, "No hinges found in 5×5 grid"

        # Rest angles for a flat grid — the dihedral angle between
        # coplanar triangles should be 0 (or ±π for some edge orientations).
        # Either way, constraint violation should be zero.
        config = SimConfig(max_particles=100)
        state = ParticleState(config)
        state.load_from_numpy(grid.positions.copy(), faces=grid.faces)

        # Store original positions
        original = state.get_positions_numpy().copy()

        # Project bending — should not move vertices since mesh is at rest
        dt = 1.0 / 60.0
        bend.project(
            state.positions, state.inv_mass, bend.n_hinges,
            1e-3, dt,
        )

        updated = state.get_positions_numpy()
        max_displacement = np.max(np.abs(updated - original))

        # Flat mesh at rest should have negligible corrections
        assert max_displacement < 0.01, (
            f"Flat mesh at rest had displacement {max_displacement:.6f}"
        )

    def test_hinge_count_for_grid(self):
        """A 5×5 grid should have a known number of internal edge hinges."""
        grid = generate_grid(cols=5, rows=5)
        hinges = find_adjacent_triangle_pairs(grid.faces)

        # For a grid with alternating diagonals:
        # Interior edges shared by 2 triangles form the hinges
        # Should be > 0 and reasonable
        assert len(hinges) > 0, "No hinges found"
        # Each hinge has 4 vertices
        assert hinges.shape[1] == 4

    def test_bent_mesh_gets_corrected(self):
        """
        If we run a full constrained simulation on a folded strip,
        the bending constraint should resist the fold relative to
        a simulation without bending.

        We compare: distance-only vs distance+bending, checking
        that bending changes the drape shape.
        """
        from simulation.constraints import build_constraints
        from simulation.solver.xpbd import XPBDSolver
        from simulation.core.engine import SimulationEngine

        cols, rows = 5, 5
        config = SimConfig(
            total_frames=30,
            substeps=6,
            solver_iterations=12,
            damping=0.98,
            max_particles=100,
        )

        grid = generate_grid(width=0.5, height=0.5, cols=cols, rows=rows, center=(0, 2.0, 0))

        # Run with bending enabled (stiff bending)
        constraints_with = build_constraints(
            positions=grid.positions, edges=grid.edges, faces=grid.faces,
            enable_bending=True,
        )
        state_with = ParticleState(config)
        state_with.load_from_numpy(grid.positions.copy(), faces=grid.faces, edges=grid.edges)
        state_with.pin_particle(0)
        state_with.pin_particle(cols - 1)

        solver_with = XPBDSolver(
            constraints=constraints_with,
            stretch_compliance=1e-8,
            bend_compliance=1e-5,  # Very stiff bending
        )
        engine_with = SimulationEngine(config, solver=solver_with)
        result_with = engine_with.run(state_with)

        # Run without bending
        constraints_without = build_constraints(
            positions=grid.positions, edges=grid.edges, faces=grid.faces,
            enable_bending=False,
        )
        state_without = ParticleState(config)
        state_without.load_from_numpy(grid.positions.copy(), faces=grid.faces, edges=grid.edges)
        state_without.pin_particle(0)
        state_without.pin_particle(cols - 1)

        solver_without = XPBDSolver(
            constraints=constraints_without,
            stretch_compliance=1e-8,
            bend_compliance=1e-3,
        )
        engine_without = SimulationEngine(config, solver=solver_without)
        result_without = engine_without.run(state_without)

        # The two should produce different shapes
        diff = np.linalg.norm(result_with.positions - result_without.positions, axis=1)
        max_diff = np.max(diff)

        assert max_diff > 1e-4, (
            f"Bending constraint had no effect: max position diff = {max_diff:.6f}"
        )


class TestAdjacentTrianglePairs:
    """Validate the hinge-finding algorithm."""

    def test_two_triangles_one_hinge(self):
        """Two triangles sharing one edge should produce one hinge."""
        faces = np.array([
            [0, 1, 2],
            [1, 3, 2],
        ], dtype=np.int32)

        hinges = find_adjacent_triangle_pairs(faces)
        assert len(hinges) == 1, f"Expected 1 hinge, got {len(hinges)}"

    def test_single_triangle_no_hinges(self):
        """A single triangle has no internal edges → no hinges."""
        faces = np.array([[0, 1, 2]], dtype=np.int32)
        hinges = find_adjacent_triangle_pairs(faces)
        assert len(hinges) == 0

    def test_hinge_vertices_are_valid(self):
        """All vertex indices in hinges should reference valid vertices."""
        grid = generate_grid(cols=5, rows=5)
        hinges = find_adjacent_triangle_pairs(grid.faces)

        n_verts = grid.positions.shape[0]
        for h in hinges:
            for v in h:
                assert 0 <= v < n_verts, f"Invalid vertex index {v} in hinge"
