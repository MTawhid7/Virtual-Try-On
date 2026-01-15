import mujoco

print(f"MuJoCo Version: {mujoco.__version__}")

# Corrected XML:
# 1. Removed <freejoint/> from the parent body.
# 2. Removed 'dim' attribute (inferred from count).
# 3. Kept <elasticity> as a child element.
xml_fix = """
<mujoco>
  <option timestep="0.005"/>
  <worldbody>
    <!-- Parent body is static (no freejoint). It acts as the origin for the cloth. -->
    <body name="holder" pos="0 0 1">

      <!-- Flexcomp creates the dynamic cloth particles -->
      <flexcomp type="grid" count="5 5 1" spacing="0.05 0.05 0.05" radius="0.01"
                name="cloth" rgba="0.8 0.2 0.2 1" mass="1.0">

          <!-- Structural constraints -->
          <edge equality="true" damping="1.0"/>

          <!-- Material properties -->
          <elasticity young="5000" poisson="0.3" damping="0.1"/>

      </flexcomp>
    </body>
  </worldbody>
</mujoco>
"""

print("\n[Test] Attempting Final Fix...")
try:
    m = mujoco.MjModel.from_xml_string(xml_fix)
    print(">>> SUCCESS: Model loaded! Physics is valid.")
except Exception as e:
    print(f">>> FAILED: {e}")