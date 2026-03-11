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

---

### 5. Technical Post-Mortem & Deep Insights

This section preserves key technical discoveries made during the intensive debugging phase (March 5–10). These observations are critical for long-term engine maintenance.

#### I. The "Cfl-Desync" Phenomenon

- **Discovery**: We initially attempted to fix 1.2M meter explosions by adding "Adaptive Substepping" in the Augmented Lagrangian loop. When CCD detected a potential overshoot, it would shrink the timestep `sub_dt = dt / n`.
- **The Bug**: While `sub_dt` was used to calculate the explicit forces on the RHS of the equation, the **Implicit System Matrix (A)** was pre-factorized at `dt = 1/60`.
- **Observation**: In Projective Dynamics, the matrix diagonal is dominated by $M/dt^2$. By shrinking `sub_dt` while keeping the matrix constant, we effectively decoupled the force magnitudes from the system's inertia. A 0.5x reduction in `sub_dt` caused a 4x mismatch in force scaling, injecting massive artificial kinetic energy that manifested as "vibrational explosions."
- **Rule**: All time-steps used in force assembly **must** strictly match the time-step used for the cached Cholesky factorization.

#### II. Analytical CCD "Blind Spots"

- **Discovery**: Analytical colliders (Sphere, Cylinder) were found to have a "blind spot" inside their padding zone.
- **The Bug**: The logic `if distance < padding { continue }` intended to allow penetrating vertices to move freely toward the surface. However, this also allowed vertices *approaching* from the surface to tunnel deep into the core if their starting position was within the epsilon threshold.
- **Insight**: CCD must never be unconditionally skipped. The fix was implementing **Directional Clamping**: only skip CCD if the vertex velocity dot-product indicates it is moving *away* from the center.

#### III. The "Chewing Gum" Constraint Stall

- **Observation**: When a single vertex became "stuck" in a CCD deadlock (Time-of-Impact $\approx 1e-6$), the entire mesh would stop moving at that vertex's anchor, while the rest of the cloth continued under gravity. This caused the cloth to stretch like thin chewing gum.
- **Resolution**: CCD must be "leaky" for already-penetrating nodes. If a vertex is already deep inside, CCD should allow it to move toward a more legal state rather than stalling it at the penetration point.

#### IV. Mass Matrix Unification

- **Discovery**: We found a subtle discrepancy where the Solver was using **Area-Weighted Lumped Masses** ($M/3$ per vertex) while the Prediction step was using **Uniform Vertex Masses** ($M/N$).
- **Observation**: This 2x discrepancy meant the "predicted inertia" was twice as heavy as the "solved stiffness," making the cloth behave like heavy lead weights connected by weak rubber bands. Unifying the mass initialization via `solver.lumped_masses()` was the "silent fix" that finally stopped the high-frequency jitter.

#### V. Multiplier Accumulation vs Penalty Stability

- **Insight**: Relying purely on the Augmented Lagrangian penalty parameter $\mu$ leads to "stiff" explosions. Accumulating the Lagrange Multiplier $\lambda$ allows the solver to "learn" the counter-force of gravity.
- **The "Levitation" Bug**: If $\lambda$ is preserved for vertices that are no longer in contact, they will literally float upwards (levitate) as the stored force is still being applied. $\lambda$ must be strictly zeroed out for any vertex that exits the barrier zone.

## 2026-03-11 (Physical Stiffness Tuning & Deformation Post-Mortem)

### 1. Implementation Progress & Fixes Completed

A comprehensive audit of the physics engine was completed today to address the lack of natural draping and non-physical rigid behaviors across all three main scenarios (Sphere, Cusick, Cantilever).

The following structural improvements were successfully made and locally verified via solver unit tests:

- **Discrete Shells Normalization Fixed**: Removed the L2 normalization of the cotangent stencil. The simulation now properly retains the raw geometric coefficients needed to derive physical curvature without resolution destruction.
- **Bending Weight Energy Corrected**: Replaced the primitive `stiffness * edge_len` formula with the Grinspun et al. 2003 area-normalized formulation: `k_b * 3 * edge_len² / combined_area`. Both `discrete_shells.rs` and the dihedral `bending.rs` now share consistent energy scales.
- **Geometry & Scenario Alignments**:
  - Cantilever fabric was repositioned to correctly hang 60% over the box ledge with isolated root pinning, preventing it from resting directly on top of the collision boundary.
  - Sphere drape initialization now properly receives the `cotton_twill` default material profile, allowing it to bypass Tier 1 fallback logic and correctly run the material projection solvers.
- **Viewer Material Synchronization**: The visualizer app `lib.rs` was updated to accurately probe `properties.is_anisotropic()` so it properly delegates to the `AnisotropicCoRotationalModel` rather than defaulting to isotropic.
- **Contact Gradient Clamping**: Asymmetric barrier gradient limiters (which previously clamped positive displacement to 50 but allowed -500 negative) in both the sphere colliders and IPC response were modified to a symmetric `[-100, 100]` threshold to stop the generation of unbalanced upward contact forces.

### 2. The "Rigid Board" Discovery: Physical Parameter Scaling

The most significant discovery during this audit was the root cause of the **extreme fabric rigidity** separating the cloth from natural gravity.

- **Observation**: The Projective Dynamics matrix assembly was consuming the raw Kawabata (KES) bending parameters (`bending_stiffness_warp/weft`).
- **The Bug**: These values are normalized between `[0.0, 1.0]`. When fed directly into the spatial solver, they produced a physical constraint weight around ~1.0. Given that the typical vertex mass is very low (yielding a gravitational $F_g$ of only $\approx 0.0001N$), the mathematical stiffness was orders of magnitude too strong.
- **The Result**: The bending elements enforced absolute rigidity like thick cardboard because gravity was 10,000x too weak to successfully buckle a low-resolution edge.
- **The Mitigation**: Applied an experimental `1e-4` physical scaling multiplier to all bending properties in the solver mapping step.

### 3. Current State & Remaining Anomalies

Despite successfully modifying the underlying algorithms and artificially scaling the KES values down by $10^{-4}$ (which successfully passes simulated solver unit tests showing actual downward Y displacement), the visual outputs in the viewer remain distinctly broken:

1. **Sphere Drape "Hat" Shape**: Instead of the fabric conforming sequentially to the sphere, the absolute center vertex locks onto the sphere's apex, creating a sharp, tent-like "spike." The rest of the fabric remains completely planar and horizontal relative to the ground.
2. **Cusick Disc Rigidity**: The fabric sits on the test cylinder as a perfectly flat, horizontal plate with no drooping down the sides.
3. **Cantilever Straightness**: Despite moving the pinning off the ledge and applying the stiffness scaler, the fabric refuses to sag under gravity and sticks straight out over the drop-off.

### 4. Plans for Future Work

The current state strongly indicates that while the internal energy formulation is mathematically sound, either the system matrix is overly stiffened globally, or the **membrane (stretch) stiffness** is artificially locking the bending degrees of freedom.

**Next Immediate Steps to Investigate:**

- **Membrane Scaling**: Audit `element.rs` for the isotropic stretch stiffness mappings (e.g., `avg_stretch_stiffness() * 10.0`). The stretch weights are likely dominating the system just as the bending weights were, preventing the local projection from breaking the planar shape constraints.
- **Damping & Augmented Lagrangian Penalties**: Inspect if the global $\mu$ multiplier in the IPC pipeline is so large that the AL step enforces absolute stiffness instead of allowing the cloth to yield naturally under gravitational pull.
- **Solver Constraint Ordering**: Check if solving membrane strings *after* bending limits curvature, maintaining the horizontal sheets seen in the screenshots.
