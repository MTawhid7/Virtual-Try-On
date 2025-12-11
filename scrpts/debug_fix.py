import mujoco
import os

print(f"MuJoCo Version: {mujoco.__version__}")

# Correct MuJoCo 3.4+ Syntax for Cloth
# 1. Use <flexcomp> with type="grid"
# 2. Use <edge> for structural stiffness (equality="true")
# 3. Use <elasticity> CHILD ELEMENT (not attribute) for material properties (Young's modulus)
xml_fix = """
<mujoco>
  <option timestep="0.005"/>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <!-- dim="2" implies a 2D sheet (cloth) -->
      <flexcomp type="grid" count="5 5 1" spacing="0.05 0.05 0.05" radius="0.01"
                name="cloth" rgba="0.8 0.2 0.2 1" mass="1.0" dim="2">

          <!-- Structural constraints (keeps nodes at fixed distance) -->
          <edge equality="true" damping="1.0"/>

          <!-- Continuum mechanics (bending/stretching resistance) -->
          <!-- This is a child element, NOT an attribute -->
          <elasticity young="5000" poisson="0.3" damping="0.1"/>

      </flexcomp>
    </body>
  </worldbody>
</mujoco>
"""

print("\n[Test] Attempting Corrected Native Syntax...")
try:
    m = mujoco.MjModel.from_xml_string(xml_fix)
    print(">>> SUCCESS: Model loaded! This is the correct syntax.")
except Exception as e:
    print(f">>> FAILED: {e}")