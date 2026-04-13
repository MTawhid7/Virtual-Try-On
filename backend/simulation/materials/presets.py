"""Fabric presets — compliance-based material definitions for XPBD.

Values are calibrated for substep_dt ≈ 0.00417s (dt=1/60, substeps=4).
α̃ = compliance / substep_dt² determines per-iteration correction strength.

Lower compliance = stiffer (more correction per iteration).
Higher compliance = softer (less correction per iteration, more give).

Calibration note: compliance values were recalibrated from the original
15-substep values by scaling by (new_dt/old_dt)² = (0.00417/0.00111)² ≈ 14.1
to preserve the same effective α̃ (correction strength per iteration).
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


# Compliance recalibration for 4 substeps (substep_dt = 0.00417s):
#   scale_factor = (0.00417 / 0.00111)² ≈ 14.1
#
# α̃ at substep_dt=0.00417s (dt=1/60, substeps=4):
#   cotton  stretch α̃=0.082  bend α̃=5120  → moderate folds, conforms to body
#   silk    stretch α̃=0.406  bend α̃=11703 → large sweeping folds
#   denim   stretch α̃=0.0008 bend α̃=293   → near-rigid
#   jersey  stretch α̃=0.812  bend α̃=29258 → stretchy, conforms tightly
#   chiffon stretch α̃=0.162  bend α̃=29258 → lightweight, billowy
FABRIC_PRESETS: dict[str, FabricPreset] = {
    "cotton":  FabricPreset("cotton",  stretch_compliance=1.4e-6, bend_compliance=2.0e-1, density=0.30, damping=0.990, friction=0.35, max_stretch=0.05, max_compress=0.15, stretch_damping=0.20, bend_damping=0.10),
    "silk":    FabricPreset("silk",    stretch_compliance=7.1e-6, bend_compliance=4.2e-1, density=0.12, damping=0.995, friction=0.20, max_stretch=0.08, max_compress=0.20, stretch_damping=0.10, bend_damping=0.05),
    "denim":   FabricPreset("denim",   stretch_compliance=1.4e-8, bend_compliance=7.1e-3, density=0.60, damping=0.980, friction=0.45, max_stretch=0.02, max_compress=0.01, stretch_damping=0.50, bend_damping=0.30),
    "jersey":  FabricPreset("jersey",  stretch_compliance=1.4e-5, bend_compliance=7.1e-1, density=0.25, damping=0.990, friction=0.30, max_stretch=0.20, max_compress=0.10, stretch_damping=0.15, bend_damping=0.05),
    "chiffon": FabricPreset("chiffon", stretch_compliance=2.8e-6, bend_compliance=7.1e-1, density=0.08, damping=0.995, friction=0.15, max_stretch=0.10, max_compress=0.05, stretch_damping=0.05, bend_damping=0.02),
}
