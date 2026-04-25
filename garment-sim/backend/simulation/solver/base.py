"""
Solver strategy base — defines the Protocol that all solvers must implement.

This abstraction allows swapping XPBD (Phase 1) for PD (Phase 2) without
touching collision, mesh, pattern, or export code.
"""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from simulation.core.state import ParticleState
from simulation.core.config import SimConfig


@runtime_checkable
class SolverStrategy(Protocol):
    """
    Protocol for simulation solvers.

    Any solver must implement:
        initialize(): Set up internal data structures (constraint indexing, etc.)
        step(): Perform one iteration of constraint solving within a substep.
    """

    def initialize(
        self,
        state: ParticleState,
        config: SimConfig,
    ) -> None:
        """
        Initialize the solver with the current particle state and config.

        Called once before the simulation loop begins. Use this to build
        constraint data structures, prefactor matrices (PD), etc.
        """
        ...

    def step(
        self,
        state: ParticleState,
        dt: float,
    ) -> None:
        """
        Perform one solver iteration.

        For XPBD: project distance → bending → stitch constraints.
        For PD: local step (per-element projection) → global step (backsubstitution).

        Called `solver_iterations` times per substep.
        """
        ...
