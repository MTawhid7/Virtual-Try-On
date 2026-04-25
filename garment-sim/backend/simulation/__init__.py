"""
Garment Simulation Engine

A modular cloth simulation engine using XPBD with Taichi kernels.
Designed with a SolverStrategy abstraction to allow future PD upgrade.
"""

import taichi as ti

# Initialize Taichi — CPU for initial development, switch to Metal/GPU later.
# This must happen before any @ti.kernel or ti.field usage.
ti.init(arch=ti.cpu, default_fp=ti.f32, debug=False)

__version__ = "0.1.0"
