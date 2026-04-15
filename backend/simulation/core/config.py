"""
Simulation configuration.

All simulation parameters in a single dataclass.
Tuned for real-time 30fps visualization with the Taichi GGUI viewer.
"""

from dataclasses import dataclass


@dataclass
class SimConfig:
    """Immutable configuration for a simulation run."""

    # --- Time ---
    dt: float = 1.0 / 60.0          # Frame timestep (seconds)
    substeps: int = 4                # Substeps per frame (reduced from 15 for 30fps)
    solver_iterations: int = 8       # XPBD constraint iterations per substep (drape phase)
    sew_solver_iterations: int = 8   # XPBD iterations during sew phase; set higher to close gaps faster
    total_frames: int = 300          # Total simulation frames

    # --- Physics ---
    gravity: float = -9.81           # m/s² (Y-down)
    damping: float = 0.98            # Velocity damping per substep (0–1)

    # --- Sew-then-Drape (replaces shrink_frames/initial_scale) ---
    sew_frames: int = 0              # Frames for sewing phase (reduced gravity, stiff stitches)
    sew_gravity_fraction: float = 0.05  # Gravity multiplier during sewing (5% = very gentle)
    sew_stitch_compliance: float = 1e-9  # Very stiff stitches during sewing phase (target)
    drape_stitch_compliance: float = 1e-7  # Normal stitch compliance during draping
    transition_frames: int = 0       # Frames to ramp gravity+compliance at sew→drape boundary
    sew_ramp_frames: int = 0         # Frames at start of sew to ramp compliance from soft→stiff
    sew_initial_compliance: float = 1e-7  # Starting (soft) compliance at frame 0 of sew ramp

    # --- Collision (from Vestra tuning) ---
    collision_thickness: float = 0.005       # 5mm body push-out margin
    sew_collision_thickness: float | None = None  # if set, overrides collision_thickness during sew phase
    friction_coefficient: float = 0.3        # Tangential friction (body + self)
    max_displacement: float = 0.05           # 5cm per substep — tunneling guard
    air_drag: float = 0.0                    # Exponential velocity drag coefficient per substep (0 = disabled)
    self_collision_thickness: float = 0.004  # 4mm cloth self-collision push-out

    # --- Self-collision control ---
    enable_self_collision: bool = False      # Disabled by default for performance (GPU→CPU sync)

    # --- Mesh ---
    max_particles: int = 50_000      # Upper bound for Taichi field allocation

    @property
    def substep_dt(self) -> float:
        """Timestep for a single substep."""
        return self.dt / self.substeps
