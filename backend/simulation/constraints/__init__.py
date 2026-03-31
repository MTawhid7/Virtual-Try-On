"""
Constraint module — distance, bending, and stitch constraints.

The ConstraintSet groups all active constraints together. The builder function
extracts topology from the mesh and creates the appropriate constraint objects.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from simulation.constraints.distance import DistanceConstraints
from simulation.constraints.bending import BendingConstraints


@dataclass
class ConstraintSet:
    """All active constraint groups for the simulation."""

    distance: DistanceConstraints | None = None
    bending: BendingConstraints | None = None
    # stitch: StitchConstraints | None = None  # Sprint 2

    def reset_lambdas(self) -> None:
        """Reset all Lagrange multipliers at the start of each substep."""
        if self.distance is not None:
            self.distance.reset_lambdas()
        if self.bending is not None:
            self.bending.reset_lambdas()


def build_constraints(
    positions: NDArray[np.float32],
    edges: NDArray[np.int32] | None = None,
    faces: NDArray[np.int32] | None = None,
    max_edges: int = 100_000,
    max_hinges: int = 100_000,
    enable_distance: bool = True,
    enable_bending: bool = True,
) -> ConstraintSet:
    """
    Build all constraints from mesh topology and initial positions.

    Args:
        positions: (N, 3) initial vertex positions.
        edges: (E, 2) edge vertex pairs (for distance constraints).
        faces: (F, 3) triangle vertex indices (for bending constraints).
        max_edges: Pre-allocation size for distance constraint fields.
        max_hinges: Pre-allocation size for bending constraint fields.
        enable_distance: Whether to create distance constraints.
        enable_bending: Whether to create bending constraints.

    Returns:
        ConstraintSet with initialized constraint objects.
    """
    constraint_set = ConstraintSet()

    # Distance constraints from edges
    if enable_distance and edges is not None and len(edges) > 0:
        dist = DistanceConstraints(max_edges=max_edges)
        dist.initialize(edges, positions)
        constraint_set.distance = dist

    # Bending constraints from faces (finds internal edge hinges)
    if enable_bending and faces is not None and len(faces) > 0:
        bend = BendingConstraints(max_hinges=max_hinges)
        bend.initialize(faces, positions)
        if bend.n_hinges > 0:
            constraint_set.bending = bend
        else:
            constraint_set.bending = None

    return constraint_set
