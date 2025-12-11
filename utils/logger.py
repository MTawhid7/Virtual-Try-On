import csv
import time
import numpy as np
from pathlib import Path
from typing import List, Dict, Any

class PhysicsLogger:
    """
    Handles logging of simulation data.
    Supports console output and buffered CSV file writing to minimize I/O latency.
    """
    def __init__(self, output_dir: Path, buffer_size: int = 1000):
        self.output_dir = output_dir
        self.buffer_size = buffer_size
        self.buffer: List[Dict[str, Any]] = []

        # Create a unique filename with timestamp
        self.csv_file = self.output_dir / f"physics_log_{int(time.time())}.csv"
        self.headers_written = False

        # Ensure directory exists
        self.output_dir.mkdir(parents=True, exist_ok=True)
        print(f"[Logger] Logging to: {self.csv_file}")

    def log_console(self, message: str, level: str = "INFO"):
        """Print formatted message to console."""
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")

    def log_step(self, data: Dict[str, Any]):
        """Add a step of data to the buffer."""
        self.buffer.append(data)
        if len(self.buffer) >= self.buffer_size:
            self.flush()

    def flush(self):
        """Write buffered data to disk."""
        if not self.buffer:
            return

        keys = self.buffer[0].keys()

        mode = 'a' if self.headers_written else 'w'
        try:
            with open(self.csv_file, mode, newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                if not self.headers_written:
                    writer.writeheader()
                    self.headers_written = True
                writer.writerows(self.buffer)
        except Exception as e:
            print(f"[Logger] Failed to write to CSV: {e}")

        self.buffer.clear()

    def close(self):
        """Flush remaining data."""
        self.flush()
        self.log_console("Log file closed.")