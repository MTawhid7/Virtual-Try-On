"""
Simulation configuration.

All simulation parameters in a single dataclass.
Default values are the "Goldilocks" from Vestra: 6 substeps × 12 iterations.
"""

from dataclasses import dataclass, field


@dataclass
class SimConfig:
    """Immutable configuration for a simulation run."""

    # --- Time ---
    dt: float = 1.0 / 60.0          # Frame timestep (seconds)
    substeps: int = 6                # Substeps per frame
    solver_iterations: int = 12      # XPBD constraint iterations per substep
    total_frames: int = 120          # Total simulation frames

    # --- Physics ---
    gravity: float = -9.81           # m/s² (Y-down)
    damping: float = 0.98            # Velocity damping per substep (0–1)

    # --- Collision (from Vestra tuning) ---
    collision_thickness: float = 0.005   # 5mm push-out margin
    friction_coefficient: float = 0.3    # Tangential friction
    max_displacement: float = 0.05       # 5cm per substep — tunneling guard

    # --- Mesh ---
    max_particles: int = 50_000      # Upper bound for Taichi field allocation

    @property
    def substep_dt(self) -> float:
        """Timestep for a single substep."""
        return self.dt / self.substeps
