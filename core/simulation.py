import mujoco
import numpy as np
from config.settings import SimConfig
from utils.logger import PhysicsLogger

class SimulationManager:
    """
    Owns the Physics Model and Data. Handles the stepping logic and error catching.
    """
    def __init__(self, xml_path: str, config: SimConfig, logger: PhysicsLogger):
        self.cfg = config
        self.logger = logger

        try:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
            self.data = mujoco.MjData(self.model)
            self.logger.log_console("MuJoCo Model loaded successfully.")
        except Exception as e:
            self.logger.log_console(f"Failed to load model: {e}", level="FATAL")
            raise e

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.logger.log_console("Simulation reset.")

    def step(self) -> bool:
        """
        Advances physics by one timestep.
        Returns False if a fatal error occurs, True otherwise.
        """
        try:
            # The core physics step
            mujoco.mj_step(self.model, self.data)

            # Check for numerical instability (NaNs)
            # qacc (acceleration) is usually the first thing to explode
            if np.isnan(self.data.qacc).any():
                raise ArithmeticError("NaN detected in acceleration (qacc).")

            return True

        except Exception as e:
            self.logger.log_console(f"Physics Error at time {self.data.time:.4f}: {e}", level="ERROR")
            return False

    def get_diagnostics(self) -> dict:
        """Extracts physics metrics for logging."""

        # 1. Force energy computation (mj_step doesn't do this by default)
        mujoco.mj_energyPos(self.model, self.data)
        mujoco.mj_energyVel(self.model, self.data)

        # 2. FIX: Sum the '.number' field of the warning structs
        # data.warning is an array of MjWarningStat objects
        warning_count = sum(w.number for w in self.data.warning)

        return {
            "time": self.data.time,
            # energy[0] is Potential, energy[1] is Kinetic
            "energy_total": self.data.energy[0] + self.data.energy[1],
            "energy_kinetic": self.data.energy[1],
            "ncon": self.data.ncon, # Number of active contacts
            "warning_count": warning_count
        }