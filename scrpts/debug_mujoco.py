import mujoco
import os

print(f"--- MuJoCo Diagnostics ---")
print(f"MuJoCo Version: {mujoco.__version__}")
print(f"Installation Dir: {os.path.dirname(mujoco.__file__)}")

# 1. Test New Native Syntax (MuJoCo 3.3+)
# The shell plugin is replaced by native 'elastic2d' attributes on the flexcomp element.
xml_native = """
<mujoco>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <!-- New Syntax: Attributes directly on flexcomp, no <plugin> tag -->
      <flexcomp type="grid" count="5 5 1" spacing="0.05 0.05 0.05" radius="0.01"
                name="cloth" rgba="0.8 0.2 0.2 1"
                mass="1.0"
                elasticity="true" elastic2d="true" young="5000" poisson="0.3" thickness="0.005">
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
"""

print("\n[Test 1] Attempting Native Syntax (No Plugin)...")
try:
    m = mujoco.MjModel.from_xml_string(xml_native)
    print(">>> SUCCESS: Native syntax works! Use this.")
except Exception as e:
    print(f">>> FAILED: {e}")

# 2. Test Legacy Plugin Syntax
xml_plugin = """
<mujoco>
  <extension>
      <plugin plugin="mujoco.elasticity.shell"/>
  </extension>
  <worldbody>
    <body pos="0 0 1">
      <freejoint/>
      <flexcomp type="grid" count="5 5 1" spacing="0.05 0.05 0.05" radius="0.01" name="cloth">
          <plugin plugin="mujoco.elasticity.shell">
             <config key="thickness" value="0.005"/>
          </plugin>
      </flexcomp>
    </body>
  </worldbody>
</mujoco>
"""

print("\n[Test 2] Attempting Legacy Plugin Syntax...")
try:
    m = mujoco.MjModel.from_xml_string(xml_plugin)
    print(">>> SUCCESS: Plugin syntax works (You are on an older version).")
except Exception as e:
    print(f">>> FAILED: {e}")