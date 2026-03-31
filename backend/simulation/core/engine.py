"""
Simulation engine — top-level orchestrator.

Wires together: state → integrator → solver → collision → export.
This is the entry point for running a simulation programmatically.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.solver.base import SolverStrategy
from simulation.solver.integrator import Integrator


@dataclass
class SimResult:
    """Output of a simulation run."""

    positions: NDArray[np.float32]   # (N, 3) final vertex positions
    faces: NDArray[np.int32]         # (F, 3) triangle indices
    normals: NDArray[np.float32]     # (N, 3) vertex normals
    uvs: NDArray[np.float32] | None  # (N, 2) UV coordinates


def compute_vertex_normals(
    positions: NDArray[np.float32],
    faces: NDArray[np.int32],
) -> NDArray[np.float32]:
    """
    Compute area-weighted vertex normals from triangle faces.

    From Vistio: area-weighted lumped normals produce better shading
    than uniform averaging.
    """
    normals = np.zeros_like(positions)

    v0 = positions[faces[:, 0]]
    v1 = positions[faces[:, 1]]
    v2 = positions[faces[:, 2]]

    # Face normals (area-weighted — cross product magnitude = 2× triangle area)
    face_normals = np.cross(v1 - v0, v2 - v0)

    # Accumulate to vertices
    for i in range(3):
        np.add.at(normals, faces[:, i], face_normals)

    # Normalize
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    lengths = np.maximum(lengths, 1e-8)  # Avoid division by zero
    normals /= lengths

    return normals.astype(np.float32)


class SimulationEngine:
    """
    Top-level simulation engine.

    Usage:
        engine = SimulationEngine(config)
        state = ParticleState(config)
        state.load_from_numpy(positions, faces=faces, edges=edges)
        result = engine.run(state)
    """

    def __init__(self, config: SimConfig, solver: SolverStrategy | None = None) -> None:
        self.config = config
        self.solver = solver
        # Collider will be set in Sprint 2
        self.collider = None

    def run(self, state: ParticleState, progress_callback=None) -> SimResult:
        """
        Run the full simulation loop.

        Args:
            state: Initialized ParticleState with positions loaded.
            progress_callback: Optional callable(frame, total_frames) for progress.

        Returns:
            SimResult with final positions, faces, normals, UVs.
        """
        config = self.config
        integrator = Integrator(
            dt=config.substep_dt,
            gravity=config.gravity,
            damping=config.damping,
            max_displacement=config.max_displacement,
        )

        # Initialize solver if present
        if self.solver is not None:
            self.solver.initialize(state, config)

        # --- Main simulation loop ---
        for frame in range(config.total_frames):
            for _substep in range(config.substeps):

                # 1. Predict: apply gravity, compute predicted positions
                integrator.predict(state)

                # 2. Solve constraints (XPBD iterations)
                if self.solver is not None:
                    for _iteration in range(config.solver_iterations):
                        self.solver.step(state, config.substep_dt)

                        # 3. Collision (interleaved inside solver loop)
                        # Will be added in Sprint 1 Layer 3a / Sprint 2
                        if self.collider is not None:
                            self.collider.resolve(state, config.collision_thickness)

                # 4. Update velocities from position delta + damping
                integrator.update(state)

            if progress_callback:
                progress_callback(frame + 1, config.total_frames)

        # --- Build result ---
        final_positions = state.get_positions_numpy()
        faces = state.faces if state.faces is not None else np.zeros((0, 3), dtype=np.int32)
        normals = compute_vertex_normals(final_positions, faces) if len(faces) > 0 else np.zeros_like(final_positions)

        return SimResult(
            positions=final_positions,
            faces=faces,
            normals=normals,
            uvs=state.uvs,
        )
