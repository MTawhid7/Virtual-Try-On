import mujoco

print(f"MuJoCo Version: {mujoco.__version__}")

# Corrected XML: Added name="holder" to the parent body
xml_fix = """
<mujoco>
  <option timestep="0.005"/>
  <worldbody>
    <!-- FIXED: Added name="holder" (Required for flexcomp parents) -->
    <body name="holder" pos="0 0 1">
      <freejoint/>

      <!-- Flexcomp with child elements for edge/elasticity -->
      <flexcomp type="grid" count="5 5 1" spacing="0.05 0.05 0.05" radius="0.01"
                name="cloth" rgba="0.8 0.2 0.2 1" mass="1.0" dim="2">

          <!-- Structural constraints -->
          <edge equality="true" damping="1.0"/>

          <!-- Material properties (Child element, NOT attribute) -->
          <elasticity young="5000" poisson="0.3" damping="0.1"/>

      </flexcomp>
    </body>
  </worldbody>
</mujoco>
"""

print("\n[Test] Attempting Corrected Native Syntax...")
try:
    m = mujoco.MjModel.from_xml_string(xml_fix)
    print(">>> SUCCESS: Model loaded! The syntax is correct.")
except Exception as e:
    print(f">>> FAILED: {e}")