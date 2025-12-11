import os
from config.settings import SimConfig

class ModelBuilder:
    """
    Programmatically generates MuJoCo MJCF (XML) strings.
    Updated for MuJoCo 3.4+:
    - Uses native <flexcomp> with <elasticity> child elements.
    - Removes <freejoint> from parent body (cloth particles are naturally dynamic).
    """
    def __init__(self, config: SimConfig):
        self.cfg = config

    def build_xml(self) -> str:
        count_x, count_y = self.cfg.cloth_grid_size
        spacing = self.cfg.cloth_spacing

        xml = f"""
<mujoco model="cloth_simulation">
    <option timestep="{self.cfg.timestep}" gravity="{' '.join(map(str, self.cfg.gravity))}" />

    <visual>
        <rgba haze="0.15 0.25 0.35 1"/>
        <global azimuth="120" elevation="-20"/>
    </visual>

    <asset>
        <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
        <material name="grid" texture="grid" texrepeat="1 1" texuniform="true" reflectance=".2"/>
    </asset>

    <worldbody>
        <!-- Floor -->
        <geom name="floor" size="0 0 .05" type="plane" material="grid" condim="3"/>

        <!-- Collision Primitives -->
        <body name="collider_sphere" pos="0 0 0.5">
            <geom type="sphere" size="0.3" rgba="0.8 0.2 0.2 1" friction="0.5"/>
        </body>
        <body name="collider_box" pos="0.8 0.5 0.4">
            <geom type="box" size="0.2 0.2 0.4" rgba="0.2 0.8 0.2 1"/>
        </body>

        <!-- Cloth via Flexcomp (Native MuJoCo 3.4+ Syntax) -->
        <!-- Parent body is static container. Particles generated inside are dynamic. -->
        <body name="cloth_root" pos="0 0 1.5">
            <flexcomp type="grid"
                      count="{count_x} {count_y} 1"
                      spacing="{spacing} {spacing} {spacing}"
                      mass="{self.cfg.cloth_mass}"
                      radius="0.015"
                      name="cloth"
                      rgba="0.7 0.7 0.9 1">

                <!-- 1. Structural Constraints -->
                <edge equality="true" damping="0.5"/>

                <!-- 2. Continuum Elasticity (Child Element) -->
                <elasticity young="5000" poisson="0.3" damping="0.1"/>

            </flexcomp>
        </body>
    </worldbody>
</mujoco>
"""
        return xml

    def save_xml(self, filename: str = "scene.xml") -> str:
        xml_content = self.build_xml()
        os.makedirs(self.cfg.output_dir, exist_ok=True)
        path = self.cfg.output_dir / filename
        with open(path, "w") as f:
            f.write(xml_content)
        return str(path)