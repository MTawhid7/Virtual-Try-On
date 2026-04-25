"""
Particle state — SoA (Structure of Arrays) layout using Taichi fields.

This is the central mutable state of the simulation. All solvers, constraints,
and collision systems read from and write to these fields.

Design note: Taichi fields are pre-allocated to max_particles. The actual
particle count is tracked by `n_particles`. Only indices [0, n_particles)
contain valid data.
"""

from __future__ import annotations

import taichi as ti
import numpy as np
from numpy.typing import NDArray

from simulation.core.config import SimConfig


@ti.data_oriented
class ParticleState:
    """
    SoA particle state for cloth simulation.

    Fields:
        positions:  Current particle positions (vec3)
        predicted:  Predicted positions after semi-implicit Euler (vec3)
        velocities: Particle velocities (vec3)
        inv_mass:   Inverse mass per particle (scalar, 0 = pinned)
    """

    def __init__(self, config: SimConfig) -> None:
        self.max_particles = config.max_particles
        self.n_particles: int = 0

        # --- Taichi fields (pre-allocated to max size) ---
        self.positions = ti.Vector.field(3, dtype=ti.f32, shape=self.max_particles)
        self.predicted = ti.Vector.field(3, dtype=ti.f32, shape=self.max_particles)
        self.velocities = ti.Vector.field(3, dtype=ti.f32, shape=self.max_particles)
        self.inv_mass = ti.field(dtype=ti.f32, shape=self.max_particles)

        # --- Topology (stored as numpy, not needed in Taichi kernels) ---
        self.faces: NDArray[np.int32] | None = None     # (F, 3) triangle indices
        self.edges: NDArray[np.int32] | None = None      # (E, 2) edge vertex pairs
        self.uvs: NDArray[np.float32] | None = None      # (N, 2) UV coordinates

    def load_from_numpy(
        self,
        positions: NDArray[np.float32],
        faces: NDArray[np.int32] | None = None,
        edges: NDArray[np.int32] | None = None,
        uvs: NDArray[np.float32] | None = None,
        inv_masses: NDArray[np.float32] | None = None,
    ) -> None:
        """
        Load particle data from numpy arrays into Taichi fields.

        Args:
            positions: (N, 3) particle positions.
            faces: (F, 3) triangle vertex indices.
            edges: (E, 2) edge vertex pairs.
            uvs: (N, 2) UV coordinates.
            inv_masses: (N,) inverse masses. If None, defaults to 1.0 (unit mass).
        """
        n = positions.shape[0]
        if n > self.max_particles:
            raise ValueError(
                f"Particle count {n} exceeds max_particles {self.max_particles}. "
                f"Increase SimConfig.max_particles."
            )

        self.n_particles = n

        # Load positions into Taichi field
        self.positions.from_numpy(
            np.pad(positions, ((0, self.max_particles - n), (0, 0)))
        )

        # Zero out velocities and predicted
        self.velocities.from_numpy(np.zeros((self.max_particles, 3), dtype=np.float32))
        self.predicted.from_numpy(
            np.pad(positions, ((0, self.max_particles - n), (0, 0)))
        )

        # Inverse masses: default 1.0 (unit mass), 0.0 = pinned
        if inv_masses is not None:
            padded = np.zeros(self.max_particles, dtype=np.float32)
            padded[:n] = inv_masses
            self.inv_mass.from_numpy(padded)
        else:
            masses = np.zeros(self.max_particles, dtype=np.float32)
            masses[:n] = 1.0
            self.inv_mass.from_numpy(masses)

        # Topology (stays in numpy)
        self.faces = faces
        self.edges = edges
        self.uvs = uvs

    def get_positions_numpy(self) -> NDArray[np.float32]:
        """Extract current positions as (N, 3) numpy array."""
        return self.positions.to_numpy()[:self.n_particles]

    def get_velocities_numpy(self) -> NDArray[np.float32]:
        """Extract current velocities as (N, 3) numpy array."""
        return self.velocities.to_numpy()[:self.n_particles]

    def pin_particle(self, index: int) -> None:
        """Pin a particle in place by setting its inverse mass to 0."""
        self.inv_mass[index] = 0.0

    def unpin_particle(self, index: int, mass: float = 1.0) -> None:
        """Unpin a particle by restoring its inverse mass."""
        if mass <= 0:
            raise ValueError("Mass must be positive to unpin.")
        self.inv_mass[index] = 1.0 / mass
