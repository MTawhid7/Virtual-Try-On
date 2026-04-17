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
from simulation.constraints.stitch import StitchConstraints
from simulation.constraints.attachment import AttachmentConstraints


@dataclass
class ConstraintSet:
    """All active constraint groups for the simulation."""

    distance: DistanceConstraints | None = None
    bending: BendingConstraints | None = None
    stitch: StitchConstraints | None = None
    attachment: AttachmentConstraints | None = None

    def reset_lambdas(self) -> None:
        """Reset all Lagrange multipliers at the start of each substep."""
        if self.distance is not None:
            self.distance.reset_lambdas()
        if self.bending is not None:
            self.bending.reset_lambdas()
        if self.stitch is not None:
            self.stitch.reset_lambdas()
        if self.attachment is not None:
            self.attachment.reset_lambdas()


def build_constraints(
    positions: NDArray[np.float32],
    edges: NDArray[np.int32] | None = None,
    faces: NDArray[np.int32] | None = None,
    stitch_pairs: NDArray[np.int32] | None = None,
    max_edges: int = 100_000,
    max_hinges: int = 100_000,
    max_stitches: int = 10_000,
    enable_distance: bool = True,
    enable_bending: bool = True,
) -> ConstraintSet:
    """
    Build all constraints from mesh topology and initial positions.

    Args:
        positions: (N, 3) initial vertex positions.
        edges: (E, 2) edge vertex pairs (for distance constraints).
        faces: (F, 3) triangle vertex indices (for bending constraints).
        stitch_pairs: (S, 2) global vertex index pairs to stitch together
                      with zero rest-length constraints (Sprint 2).
        max_edges: Pre-allocation size for distance constraint fields.
        max_hinges: Pre-allocation size for bending constraint fields.
        max_stitches: Pre-allocation size for stitch constraint fields.
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

    # Stitch constraints from panel seam vertex pairs
    if stitch_pairs is not None and len(stitch_pairs) > 0:
        stitch = StitchConstraints(max_stitches=max_stitches)
        stitch.initialize(stitch_pairs)
        constraint_set.stitch = stitch

    return constraint_set
