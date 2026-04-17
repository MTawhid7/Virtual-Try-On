"""
Simulation engine — top-level orchestrator.

Wires together: state → integrator → solver → collision → export.
This is the entry point for running a simulation programmatically.

Implements a 2-stage simulation loop:
  Stage 1 (Sew): Reduced gravity, very stiff stitches, no self-collision.
                  Panels slide together and seams close.
  Stage 2 (Drape): Full gravity, normal stitch compliance, optional self-collision.
                   Fabric drapes naturally on the body.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace as dc_replace
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from simulation.core.config import SimConfig
from simulation.core.state import ParticleState
from simulation.solver.base import SolverStrategy
from simulation.solver.integrator import Integrator


@dataclass
class SimResult:
    """Output of a simulation run."""

    positions: NDArray[np.float32]              # (N, 3) final vertex positions
    faces: NDArray[np.int32]                    # (F, 3) triangle indices
    normals: NDArray[np.float32]                # (N, 3) vertex normals
    uvs: NDArray[np.float32] | None             # (N, 2) UV coordinates
    frame_positions: list[NDArray[np.float32]] | None = None  # per-keyframe snapshots

    def export_glb(self, path: str | Path) -> Path:
        """
        Export this result to a binary glTF (.glb) file.

        Convenience wrapper around simulation.export.write_glb.
        Lazy-imports to avoid circular deps and keep trimesh out of
        the engine hot path.

        Args:
            path: Output file path (.glb).

        Returns:
            Resolved Path to the written file.
        """
        from simulation.export.gltf_writer import write_glb

        return write_glb(
            positions=self.positions,
            faces=self.faces,
            normals=self.normals,
            uvs=self.uvs,
            path=path,
        )


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
    Top-level simulation engine with 2-stage sew-then-drape loop.

    Usage:
        engine = SimulationEngine(config)
        state = ParticleState(config)
        state.load_from_numpy(positions, faces=faces, edges=edges)
        result = engine.run(state)
    """

    def __init__(self, config: SimConfig, solver: SolverStrategy | None = None) -> None:
        self.config = config
        self.solver = solver
        self.collider = None       # Body / sphere collider (set externally)
        self.self_collider = None  # Cloth self-collider (set externally)

    def step_frame(self, state: ParticleState, frame: int = -1) -> None:
        """
        Run one full frame of simulation (executes multiple substeps).

        Args:
            frame: Current frame number. Used to determine sew vs drape phase.
                   -1 means drape phase (default for backward compatibility).
        """
        config = self.config

        # --- Phase determination (sew / transition / drape) ---
        sew_end   = config.sew_frames
        trans_end = sew_end + config.transition_frames

        in_sew_phase = (0 <= frame < sew_end)
        in_trans     = (config.transition_frames > 0) and (sew_end <= frame < trans_end)

        if in_sew_phase:
            gravity_scale     = config.sew_gravity_fraction
            stitch_compliance = config.sew_stitch_compliance
            enable_strain_limit = False

            # Optional compliance ramp: start soft, tighten over sew_ramp_frames
            if config.sew_ramp_frames > 0 and 0 <= frame < config.sew_ramp_frames:
                t = frame / config.sew_ramp_frames
                log_i = math.log10(config.sew_initial_compliance)
                log_t = math.log10(config.sew_stitch_compliance)
                stitch_compliance = 10.0 ** (log_i + t * (log_t - log_i))

        elif in_trans:
            # Smooth ramp: gravity 15%→100%, compliance 1e-10→1e-8 (log-space)
            t = (frame - sew_end) / config.transition_frames
            gravity_scale = config.sew_gravity_fraction + t * (1.0 - config.sew_gravity_fraction)
            log_s = math.log10(config.sew_stitch_compliance)
            log_d = math.log10(config.drape_stitch_compliance)
            stitch_compliance = 10.0 ** (log_s + t * (log_d - log_s))
            enable_strain_limit = False  # keep off during transition

        else:
            gravity_scale     = 1.0
            stitch_compliance = config.drape_stitch_compliance
            enable_strain_limit = True

        enable_self_collision = (
            config.enable_self_collision
            and not in_sew_phase
            and not in_trans
            and self.self_collider is not None
        )

        # Number of solver iterations — higher during sew for better gap closure
        n_iters = config.sew_solver_iterations if in_sew_phase else config.solver_iterations

        # Update stitch compliance on solver
        if self.solver is not None and hasattr(self.solver, '_stitch_compliance'):
            self.solver._stitch_compliance = stitch_compliance

        # Effective config for collision — optionally thinner shell during sew phase
        if in_sew_phase and config.sew_collision_thickness is not None:
            effective_config = dc_replace(config, collision_thickness=config.sew_collision_thickness)
        else:
            effective_config = config

        # Integrator is stateless, so instantiating here is extremely lightweight
        integrator = Integrator(
            dt=config.substep_dt,
            gravity=config.gravity * gravity_scale,
            damping=config.damping,
            max_displacement=config.max_displacement,
            air_drag=config.air_drag,
        )

        for _substep in range(config.substeps):
            # 1. Predict: apply gravity, compute predicted positions
            integrator.predict(state)

            # 2. Reset Lagrange multipliers (no warm starting — Vestra lesson)
            if self.solver is not None and hasattr(self.solver, 'reset_lambdas'):
                self.solver.reset_lambdas()

            # 3. Self-collision (only during drape phase, if enabled)
            if enable_self_collision:
                self.self_collider.update_hash(state)
                self.self_collider.resolve(state)

            # 4. Solve constraints (XPBD iterations, body collision interleaved)
            if self.solver is not None:
                for _iteration in range(n_iters):
                    self.solver.step(
                        state,
                        config.substep_dt,
                        rest_length_scale=1.0,
                        enable_strain_limit=enable_strain_limit,
                        enable_attachment=in_sew_phase,
                    )

                    # Body collision interleaved — static geometry, safe to repeat
                    if self.collider is not None:
                        self.collider.resolve(state, effective_config)

            # 5. Update velocities from position delta + damping
            integrator.update(state)

            # 6. Constraint-based velocity damping (once per substep, after velocity update)
            if self.solver is not None and hasattr(self.solver, 'apply_damping'):
                self.solver.apply_damping(state)

    def run(
        self,
        state: ParticleState,
        progress_callback=None,
        record_every_n_frames: int = 0,
    ) -> SimResult:
        """
        Run the full simulation loop (sew phase → drape phase).

        Args:
            state: Initialized ParticleState with positions loaded.
            progress_callback: Optional callable(frame, total_frames) for progress.
            record_every_n_frames: If > 0, snapshot positions every N frames.
                Frame 0 (initial panel layout) is always included as the first
                snapshot. Use 5 for ~64 keyframes from a 320-frame sim.
                Snapshots are returned in SimResult.frame_positions.

        Returns:
            SimResult with final positions, faces, normals, UVs, and optionally
            frame_positions (list of per-keyframe (N,3) position arrays).
        """
        config = self.config

        # Initialize solver if present
        if self.solver is not None:
            self.solver.initialize(state, config)

        # Capture the initial panel layout as frame 0 (flat panels, pre-physics)
        frame_positions: list[NDArray[np.float32]] = []
        if record_every_n_frames > 0:
            frame_positions.append(state.get_positions_numpy().copy())

        # --- Main simulation loop ---
        for frame in range(config.total_frames):
            self.step_frame(state, frame=frame)

            if record_every_n_frames > 0 and (frame + 1) % record_every_n_frames == 0:
                frame_positions.append(state.get_positions_numpy().copy())

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
            frame_positions=frame_positions if frame_positions else None,
        )
