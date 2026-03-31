"""
XPBD (Extended Position-Based Dynamics) solver — Phase 1 implementation.

Iterates over constraint groups (distance → bending → stitch) each solver
iteration, projecting corrections using per-constraint Lagrange multipliers.

The solver is stateless between substeps — Lagrange multipliers are reset
at the start of each substep (no warm starting). From Vestra: "Lagrange
multiplier accumulation injected energy." Stateless is safer.

Implements the SolverStrategy Protocol for swappability with PD (Phase 2).
"""

from __future__ import annotations

from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.constraints import ConstraintSet


class XPBDSolver:
    """
    XPBD Gauss-Seidel constraint solver.

    Projects constraints in order: distance → bending → (stitch in Sprint 2).
    Called `solver_iterations` times per substep by the engine.

    This implements the SolverStrategy Protocol:
        initialize(state, config): Build constraints, store compliance
        step(state, dt):           One iteration of constraint projection
    """

    def __init__(
        self,
        constraints: ConstraintSet,
        stretch_compliance: float = 1e-8,
        bend_compliance: float = 1e-3,
    ) -> None:
        """
        Args:
            constraints: Pre-built ConstraintSet from build_constraints().
            stretch_compliance: XPBD compliance for distance constraints.
                Lower = stiffer. 0.0 = rigid (from Vestra: "0.0 compliance
                combined with 10 iterations").
            bend_compliance: XPBD compliance for bending constraints.
                Higher = more drapey. Controls fold sharpness.
        """
        self.constraints = constraints
        self.stretch_compliance = stretch_compliance
        self.bend_compliance = bend_compliance
        self._initialized = False

    def initialize(self, state: ParticleState, config: SimConfig) -> None:
        """
        Prepare for simulation. Constraints are already built externally.

        This is called once before the simulation loop begins.
        """
        self._initialized = True

    def reset_lambdas(self) -> None:
        """Reset all Lagrange multipliers — called at start of each substep."""
        self.constraints.reset_lambdas()

    def step(self, state: ParticleState, dt: float) -> None:
        """
        Perform one solver iteration: project distance → bending constraints.

        Called `solver_iterations` times per substep.
        """
        # Distance constraints (edge length preservation)
        if self.constraints.distance is not None:
            self.constraints.distance.project(
                state.positions,
                state.inv_mass,
                self.constraints.distance.n_edges,
                self.stretch_compliance,
                dt,
            )

        # Bending constraints (dihedral angle preservation)
        if self.constraints.bending is not None:
            self.constraints.bending.project(
                state.positions,
                state.inv_mass,
                self.constraints.bending.n_hinges,
                self.bend_compliance,
                dt,
            )
