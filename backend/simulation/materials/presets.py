"""Fabric presets — compliance-based material definitions for XPBD.

Values are calibrated for substep_dt ≈ 0.00278s (dt=1/60, substeps=6).
α̃ = compliance / substep_dt² determines per-iteration correction strength.

Lower compliance = stiffer (more correction per iteration).
Higher compliance = softer (less correction per iteration, more give).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class FabricPreset:
    name: str
    stretch_compliance: float  # XPBD distance compliance
    bend_compliance: float     # XPBD bending compliance
    density: float             # kg/m² — used for area-weighted inv_mass via compute_area_weighted_inv_masses()
    damping: float             # global velocity damping per substep (applied each substep)
    friction: float            # collision friction coefficient (0 = frictionless, 1 = no slip)
    max_stretch: float         # hard strain limit above rest length (e.g. 0.03 = 3%)
    max_compress: float        # hard strain limit below rest length (e.g. 0.01 = 1%)
    stretch_damping: float     # constraint-velocity damping along edges (0 = off, 1 = critically damped)
    bend_damping: float        # constraint-velocity damping along hinge gradient (0 = off, 1 = critically damped)


# α̃ values at substep_dt=0.00111s (dt=1/60, substeps=15, dt²=1.23e-6):
# Cotton is calibrated for 15 substeps with area-weighted mass. Other fabrics use
# their original values and should be re-calibrated when their scenes are updated.
#
#   cotton  stretch α̃=0.081  bend α̃=5994  → moderate folds, conforms to body
#   silk    stretch α̃=0.407  bend α̃=16260 → large sweeping folds, settles slowly
#   denim   stretch α̃=0.0008 bend α̃=407   → near-rigid, few wide folds
#   jersey  stretch α̃=0.813  bend α̃=40650 → stretchy, conforms tightly to body
#   chiffon stretch α̃=0.163  bend α̃=40650 → lightweight, billowy, barely grips surface
FABRIC_PRESETS: dict[str, FabricPreset] = {
    "cotton":  FabricPreset("cotton",  stretch_compliance=1e-7, bend_compliance=7.4e-3, density=0.30, damping=0.990, friction=0.35, max_stretch=0.03, max_compress=0.01, stretch_damping=0.20, bend_damping=0.10),
    "silk":    FabricPreset("silk",    stretch_compliance=5e-7, bend_compliance=2e-2,   density=0.12, damping=0.995, friction=0.20, max_stretch=0.05, max_compress=0.02, stretch_damping=0.10, bend_damping=0.05),
    "denim":   FabricPreset("denim",   stretch_compliance=1e-9, bend_compliance=5e-4,   density=0.60, damping=0.980, friction=0.45, max_stretch=0.01, max_compress=0.005, stretch_damping=0.50, bend_damping=0.30),
    "jersey":  FabricPreset("jersey",  stretch_compliance=1e-6, bend_compliance=5e-2,   density=0.25, damping=0.990, friction=0.30, max_stretch=0.15, max_compress=0.05, stretch_damping=0.15, bend_damping=0.05),
    "chiffon": FabricPreset("chiffon", stretch_compliance=2e-7, bend_compliance=5e-2,   density=0.08, damping=0.995, friction=0.15, max_stretch=0.08, max_compress=0.03, stretch_damping=0.05, bend_damping=0.02),
}
