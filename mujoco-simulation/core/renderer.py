import mujoco
import mujoco.viewer
from config.settings import SimConfig

class MuJoCoRenderer:
    """
    Wraps the mujoco.viewer.launch_passive for M1-compatible rendering.
    """
    def __init__(self, model, data, config: SimConfig):
        self.model = model
        self.data = data
        self.config = config
        self.viewer = None

    def start(self):
        """Launches the passive viewer."""
        # launch_passive is non-blocking and works well on macOS main thread loops
        # It allows us to keep control of the simulation loop in Python.
        self.viewer = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=True,
            show_right_ui=True
        )
        # Initial sync to show the first frame
        self.viewer.sync()

    def update(self):
        """Syncs the viewer with the physics state."""
        if self.viewer and self.viewer.is_running():
            self.viewer.sync()

    def is_running(self) -> bool:
        return self.viewer is not None and self.viewer.is_running()

    def close(self):
        if self.viewer:
            self.viewer.close()