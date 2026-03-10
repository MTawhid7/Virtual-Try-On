# Vistio Worklog — Volume 2 (2026-03-10)

This volume documents the ongoing technical analysis and stabilization transition of the Vistio simulation engine.

## 2026-03-10 (Current State Analysis)

### 1. Implementation Summary (Current Engine State)

The Vistio engine has reached a milestone of **numerical stability**. After resolving the catastrophic 10^6m explosions, the current architecture consists of:

- **Solver**: Projective Dynamics (PD) with a constant-matrix implicit global step and ARAP local step.
- **Contact Model**: Incremental Potential Contact (IPC) using log-barriers and Augmented Lagrangian multipliers.
- **Stability Mechanisms**: Directional CCD clamping for analytical colliders and topologically safe Augmented Lagrangian warm-starts (`prev_x` initialization).
- **Architecture**: Modularized codebase with dedicated crates for math, contact, and solver logic.

---

### 2. Observed Abnormal Behaviors & Technical Analysis

While the simulation no longer "explodes," it currently exhibits several non-physical artifacts caused by current parameter scaling and structural limitations.

#### A. Sphere Drape Test: The "Force Field" Gap

- **Observed Behavior**: The cloth maintains a large (approx. 10cm) visible gap above the sphere and floor.
- **Technical Observation**: The `barrier_d_hat` parameter is set to `0.01`. In IPC, the activation distance is $\sqrt{d_{hat}}$, which yields 0.1m.
- **Cause**: To prevent the high-velocity free-fall (~2.6 m/s) from overshooting the barrier zone in a single frame ($dt=1/60 \approx 4.3cm$), the activation zone was widened. This creates a strong repulsive force far before the cloth reaches the surface, preventing natural "flush" contact.

#### B. Cusick Drape Test: Rigidity & Lack of Folds

- **Observed Behavior**: The fabric rests as a rigid, flat disc on the cylinder top, failing to drape or form folds around the perimeter.
- **Technical Observation**:
  - **Topology**: The circular grid resolution was reduced ($32 \times 10$) to maintain CPU performance.
  - **Bending Model**: Discrete Shells with high stiffness weights.
- **Cause**: The combination of coarse triangle resolution and high bending resistance creates a "rigid plate" behavior. The gravitational potential energy of the circular disc is insufficient to overcome the bending energy required to initiate the first "buckling" fold around the cylinder's sharp edge.

#### C. Self-Fold Test: Geometry Penetration

- **Observed Behavior**: The ribbon passes through the floor instead of stacking or folding.
- **Technical Observation**: The scenario configuration explicitly has `ipc_enabled: false`.
- **Cause**: Self-collision and stacking stability have been deferred to Tier 5 (GPU Compute). Without IPC active for this scenario, the analytical barriers are skipped or failing due to high approach velocity, causing the geometry to tunnel through the ground.

---

### 3. Special Case: Cantilever Bending Test Profile

**Purpose**:
The Cantilever Bending test is a standard simulation of the **ASTM D1388** "Peirce" stiffness test. It is used to measure the **flexural rigidity** and "hand" of a fabric.

**Physical Properties Measured**:

- **Bending Stiffness**: The fabric's resistance to forming a curve.
- **Gravity-to-Stiffness Ratio**: It validates that the fabric mass correctly interacts with the internal bending stencils.

**Expected Behavior (Correct Implementation)**:
A correct implementation should see the cloth strip pinned at one end (on a horizontal shelf or box) and immediately drooping downwards under its own weight, forming a smooth **parabolic curve**.

**Visual Indicators of Success**:

- **Smooth Deflection**: The strip should exhibit a continuous curve, not a sharp "kink."
- **Equilibrium Hand**: The strip should hang at an angle proportional to its material density and stiffness (stiff Denim should hang more horizontally; Silk should droop almost vertically).

---

### 4. Planned Debugging & Development Steps

1. **Substepping for Gap Reduction**: Re-implement substeps (maintaining matrix sync) to allow shrinking `d_hat` to 1e-4 (~1cm gap) or 1e-6 (~1mm gap) for realistic visual contact.
2. **Bending Normalization**: Audit the weight scaling in `crates/vistio-solver/src/constraints/bending.rs` and `discrete_shells.rs`. The area-stiffness weights must be tuned to allow gravity-induced buckling in low-resolution meshes.
3. **GPU Compute Port (Tier 5)**: Port the contact detection and BVH traversal to WGSL shaders to handle the 1000x increase in collision pairs required for realistic folding and stacking.
4. **Cusick Grid Refinement**: Experiment with higher-resolution grids and lower bending stiffness to validate the "draping" threshold.
