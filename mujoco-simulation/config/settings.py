import dataclasses
from pathlib import Path

@dataclasses.dataclass(frozen=True)
class SimConfig:
    """Immutable configuration for the simulation."""
    # Physics
    timestep: float = 0.002  # 2ms
    gravity: tuple[float, float, float] = (0.0, 0.0, -9.81)

    # Cloth Parameters
    cloth_grid_size: tuple[int, int] = (20, 20)
    cloth_spacing: float = 0.05
    cloth_mass: float = 1.0

    # Rendering
    window_width: int = 1280
    window_height: int = 720
    render_fps: int = 60

    # Logging
    log_frequency_steps: int = 10  # Log every N steps
    log_flush_interval: int = 1000 # Flush to disk every N logs
    output_dir: Path = Path("assets")

    @property
    def render_interval(self) -> float:
        return 1.0 / self.render_fps