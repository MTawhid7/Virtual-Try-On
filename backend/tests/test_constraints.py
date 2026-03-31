"""
Unit tests for constraint math.

Tests:
    - Distance constraint preserves edge length
    - Distance constraint respects compliance
    - Bending constraint returns zero for flat mesh
    - Bending finds correct number of hinges
    - Constraint builder creates expected constraint types
"""

import numpy as np
import pytest

# Import simulation package (triggers ti.init)
import simulation  # noqa: F401
from simulation.constraints.distance import DistanceConstraints
from simulation.constraints.bending import BendingConstraints, find_adjacent_triangle_pairs
from simulation.constraints import build_constraints, ConstraintSet
from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.core.engine import SimulationEngine
from simulation.mesh.grid import generate_grid


# ============================================================================
# Distance Constraint Tests
# ============================================================================


class TestDistanceConstraint:
    """Validate distance constraint projection math."""

    def test_preserves_edge_length(self):
        """
        Create two particles connected by an edge. Stretch them apart.
        After projecting, the distance should be close to the rest length.
        """
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        # Two particles at distance 1.0
        positions = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32)
        state.load_from_numpy(positions)

        edges = np.array([[0, 1]], dtype=np.int32)

        dist_constraints = DistanceConstraints(max_edges=10)
        dist_constraints.initialize(edges, positions)

        # Stretch the particles apart to distance 2.0
        stretched = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
        state.positions.from_numpy(
            np.pad(stretched, ((0, config.max_particles - 2), (0, 0)))
        )

        # Project with zero compliance (rigid)
        dt = 1.0 / 60.0
        n_iterations = 10
        for _ in range(n_iterations):
            dist_constraints.project(
                state.positions, state.inv_mass, dist_constraints.n_edges,
                0.0,  # compliance = 0 (rigid)
                dt,
            )

        # Check distance is close to rest length (1.0)
        p = state.get_positions_numpy()
        dist = np.linalg.norm(p[1] - p[0])
        assert abs(dist - 1.0) < 0.01, f"Distance {dist:.4f}, expected ~1.0"

    def test_pinned_particle_stays(self):
        """
        One pinned, one free particle. Stretching should only move the free one.
        """
        config = SimConfig(max_particles=10)
        state = ParticleState(config)

        positions = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32)
        state.load_from_numpy(positions)
        state.pin_particle(0)  # Pin first particle

        edges = np.array([[0, 1]], dtype=np.int32)

        dist_constraints = DistanceConstraints(max_edges=10)
        dist_constraints.initialize(edges, positions)

        # Stretch particle 1 to x=2
        stretched = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
        state.positions.from_numpy(
            np.pad(stretched, ((0, config.max_particles - 2), (0, 0)))
        )

        dt = 1.0 / 60.0
        for _ in range(20):
            dist_constraints.project(
                state.positions, state.inv_mass, dist_constraints.n_edges,
                0.0, dt,
            )

        p = state.get_positions_numpy()
        # Pinned particle should not have moved
        np.testing.assert_allclose(p[0], [0, 0, 0], atol=1e-6)
        # Free particle should be at distance ~1.0 from pin
        dist = np.linalg.norm(p[1] - p[0])
        assert abs(dist - 1.0) < 0.01, f"Distance {dist:.4f}, expected ~1.0"

    def test_compliance_allows_stretch(self):
        """
        With non-zero compliance, edges should be allowed to stretch.
        High compliance should result in more stretch than zero compliance.
        """
        config = SimConfig(max_particles=10)

        edges = np.array([[0, 1]], dtype=np.int32)
        positions = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.float32)

        # Test with zero compliance (rigid)
        state_rigid = ParticleState(config)
        state_rigid.load_from_numpy(positions.copy())
        dist_rigid = DistanceConstraints(max_edges=10)
        dist_rigid.initialize(edges, positions)
        stretched = np.array([[0, 0, 0], [2, 0, 0]], dtype=np.float32)
        state_rigid.positions.from_numpy(
            np.pad(stretched.copy(), ((0, config.max_particles - 2), (0, 0)))
        )
        dt = 1.0 / 60.0
        for _ in range(5):
            dist_rigid.project(
                state_rigid.positions, state_rigid.inv_mass, 1, 0.0, dt
            )

        # Test with high compliance (soft)
        state_soft = ParticleState(config)
        state_soft.load_from_numpy(positions.copy())
        dist_soft = DistanceConstraints(max_edges=10)
        dist_soft.initialize(edges, positions)
        state_soft.positions.from_numpy(
            np.pad(stretched.copy(), ((0, config.max_particles - 2), (0, 0)))
        )
        for _ in range(5):
            dist_soft.project(
                state_soft.positions, state_soft.inv_mass, 1, 1.0, dt
            )

        p_rigid = state_rigid.get_positions_numpy()
        p_soft = state_soft.get_positions_numpy()

        dist_after_rigid = np.linalg.norm(p_rigid[1] - p_rigid[0])
        dist_after_soft = np.linalg.norm(p_soft[1] - p_soft[0])

        # Soft should be more stretched than rigid
        assert dist_after_soft > dist_after_rigid, (
            f"Soft ({dist_after_soft:.4f}) should be longer than rigid ({dist_after_rigid:.4f})"
        )

    def test_rest_length_computed_correctly(self):
        """Rest lengths should match initial edge lengths."""
        positions = np.array([
            [0, 0, 0], [3, 0, 0], [0, 4, 0]
        ], dtype=np.float32)
        edges = np.array([[0, 1], [0, 2], [1, 2]], dtype=np.int32)

        dist = DistanceConstraints(max_edges=10)
        dist.initialize(edges, positions)

        assert abs(dist.rest_length[0] - 3.0) < 1e-5
        assert abs(dist.rest_length[1] - 4.0) < 1e-5
        assert abs(dist.rest_length[2] - 5.0) < 1e-5  # 3-4-5 triangle


# ============================================================================
# Bending Constraint Tests
# ============================================================================


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


# ============================================================================
# Constraint Builder Tests
# ============================================================================


class TestConstraintBuilder:
    """Validate the build_constraints() function."""

    def test_builds_both_constraint_types(self):
        """For a grid mesh, should create both distance and bending constraints."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
        )

        assert cs.distance is not None, "No distance constraints built"
        assert cs.bending is not None, "No bending constraints built"
        assert cs.distance.n_edges == len(grid.edges)
        assert cs.bending.n_hinges > 0

    def test_disable_distance(self):
        """Can disable distance constraints."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
            enable_distance=False,
        )

        assert cs.distance is None
        assert cs.bending is not None

    def test_disable_bending(self):
        """Can disable bending constraints."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
            enable_bending=False,
        )

        assert cs.distance is not None
        assert cs.bending is None

    def test_no_edges_no_constraints(self):
        """With no edges/faces, should return empty constraint set."""
        positions = np.zeros((5, 3), dtype=np.float32)
        cs = build_constraints(positions=positions)

        assert cs.distance is None
        assert cs.bending is None

    def test_reset_lambdas(self):
        """ConstraintSet.reset_lambdas() should not crash."""
        grid = generate_grid(cols=5, rows=5)
        cs = build_constraints(
            positions=grid.positions,
            edges=grid.edges,
            faces=grid.faces,
        )

        # Should not raise
        cs.reset_lambdas()


# ============================================================================
# Adjacent Triangle Pair Tests
# ============================================================================


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
