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
    # --- Contact energy algorithms ---
    contact_damping: float = 0.007           # Per-iteration velocity decay in the 3×thickness contact band.
                                             # With 16 solver iters: (1-0.007)^16 ≈ 0.90 → ~10%/substep.
    contact_speed_limit: float = 1.5        # Fallback max m/s toward body; engine overrides with
                                             # physics-derived value: collision_thickness * 0.9 / substep_dt.
    collision_stiffness: float = 0.75       # Fractional push-out for the contact zone (sd ≥ 0).
                                             # Full stiffness (1.0) applied when sd < 0 (truly penetrating).
                                             # Soft contact prevents kick-back explosion (vestra pattern).

    # --- Self-collision control ---
    enable_self_collision: bool = False      # Disabled by default for performance (GPU→CPU sync)

    # --- Unconditional stability backstops ---
    max_speed: float = 20.0                 # Hard per-substep velocity ceiling (m/s). Catches any explosion
                                             # that slips through layers 1–6. Real cloth at 20 m/s tears.
    max_inv_mass: float = 5000.0            # Per-vertex inv_mass cap. Prevents 0.3mm CDT boundary triangles
                                             # from producing XPBD corrections of ~gap magnitude per iteration.
    sew_max_stretch: float = 5.0            # Relaxed fabric strain limit during sew + transition phases
                                             # (500% = 5× rest length). Bounds stitch-induced stretching
                                             # without blocking seam closure (stitch pairs unaffected).

    # --- Mesh ---
    max_particles: int = 50_000      # Upper bound for Taichi field allocation

    @property
    def substep_dt(self) -> float:
        """Timestep for a single substep."""
        return self.dt / self.substeps
