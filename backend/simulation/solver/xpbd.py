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

    Projects constraints in order: distance → bending → stitch.
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
        stitch_compliance: float = 1e-6,
        max_stretch: float | None = None,
        max_compress: float | None = None,
        stretch_damping: float = 0.0,
        bend_damping: float = 0.0,
    ) -> None:
        """
        Args:
            constraints: Pre-built ConstraintSet from build_constraints().
            stretch_compliance: XPBD compliance for distance constraints.
                Lower = stiffer. 0.0 = rigid (from Vestra: "0.0 compliance
                combined with 10 iterations").
            bend_compliance: XPBD compliance for bending constraints.
                Higher = more drapey. Controls fold sharpness.
            stitch_compliance: XPBD compliance for stitch constraints.
                1e-6 closes a 0.24m gap in ~60–80 frames (Sprint 2 default).
            max_stretch: Hard upper strain limit as a fraction of rest length
                (e.g. 0.03 = 3%).  None = disabled.
            max_compress: Hard lower strain limit as a fraction of rest length
                (e.g. 0.01 = 1%).  None = disabled.
            stretch_damping: Constraint-velocity damping along edges (0 = off,
                1 = critically damped). Applied once per substep after velocity
                update — damps stretch oscillations without affecting global damping.
            bend_damping: Constraint-velocity damping along hinge gradient
                (0 = off, 1 = critically damped). Requires Track A analytical
                gradients to be in place.
        """
        self.constraints = constraints
        self.stretch_compliance = stretch_compliance
        self.bend_compliance = bend_compliance
        self._stitch_compliance = float(stitch_compliance)
        self._max_stretch = float(max_stretch) if max_stretch is not None else None
        self._max_compress = float(max_compress) if max_compress is not None else None
        self._stretch_damping = float(stretch_damping)
        self._bend_damping = float(bend_damping)
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
        Perform one solver iteration: distance → bending → stitch → strain limit.

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

        # Bending constraints (analytical cotangent gradients)
        if self.constraints.bending is not None:
            self.constraints.bending.project(
                state.positions,
                state.inv_mass,
                self.constraints.bending.n_hinges,
                self.bend_compliance,
                dt,
            )

        # Stitch constraints (zero rest-length, pulls seam vertices together)
        if self.constraints.stitch is not None:
            self.constraints.stitch.project(
                state.positions,
                state.inv_mass,
                self.constraints.stitch.n_stitches,
                self._stitch_compliance,
                dt,
            )

        # Hard strain limit — clamp edges to [1-max_compress, 1+max_stretch] × L₀
        if (
            self.constraints.distance is not None
            and self._max_stretch is not None
            and self._max_compress is not None
        ):
            self.constraints.distance.apply_strain_limit(
                state.positions,
                state.inv_mass,
                self.constraints.distance.n_edges,
                self._max_stretch,
                self._max_compress,
            )

    def apply_damping(self, state: ParticleState) -> None:
        """
        Apply constraint-velocity damping once per substep.

        Called by the engine AFTER integrator.update() has computed fresh
        velocities from the XPBD position deltas. The damped velocities feed
        into the next substep's predict step.

        Track D — constraint-based damping (Sprint 2 Fabric Realism).
        """
        if self._stretch_damping > 0.0 and self.constraints.distance is not None:
            self.constraints.distance.apply_stretch_damping(
                state.positions,
                state.velocities,
                state.inv_mass,
                self.constraints.distance.n_edges,
                self._stretch_damping,
            )

        if self._bend_damping > 0.0 and self.constraints.bending is not None:
            self.constraints.bending.apply_bend_damping(
                state.positions,
                state.velocities,
                state.inv_mass,
                self.constraints.bending.n_hinges,
                self._bend_damping,
            )
