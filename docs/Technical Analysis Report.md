# Vistio Engine — Comprehensive Technical Analysis Report

> **Objective:** Diagnose the current simulation engine's limitations, identify gaps relative to state-of-the-art systems, and provide a detailed roadmap for achieving production-grade, best-in-class realism in cloth simulation.

---

## 1. Diagnosis of Current Engine Limitations

After exhaustive analysis of every module in the Vistio codebase, the following architectural, numerical, and algorithmic weaknesses have been identified as the root causes of the persistent oscillation and non-settling behavior observed in the sphere drape test.

### 1.1 Frozen Barrier Gradients in the Inner PD Loop (Critical)

**Location:** [pd_solver.rs:596-599](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/pd_solver.rs#L596-L599) (outer AL loop) vs. [pd_solver.rs:700-708](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/pd_solver.rs#L700-L708) (inner PD loop)

**Problem:** The barrier gradient is computed **once** at the start of each AL outer iteration and then held constant across all inner PD iterations. As the cloth moves during the PD solve, positions change, but the barrier forces don't update to reflect the new distances. This causes:

- **Overshooting:** The solver pushes the cloth based on stale distance information, creating force/position mismatch.
- **Oscillation:** The next AL iteration re-detects contacts at the overshot position and pushes back, creating a perpetual bounce between "too close" and "too far."

In the original IPC paper (Li et al. 2020), the barrier gradient is recomputed at **every** Newton iteration, ensuring geometric consistency between the contact forces and the current positions.

> [!CAUTION]
> This is the single most impactful bug. The inner PD loop is effectively solving a different optimization problem than the one defined by the current contact state.

### 1.2 Missing Barrier Hessian in System Matrix (Critical)

**Location:** System matrix is prefactored once in [pd_solver.rs:246](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/pd_solver.rs#L246) and never updated.

**Problem:** The system matrix `A = M/h² + Σ S_iᵀ S_i` contains only the inertia and elastic stiffness terms. It does **not** include the barrier Hessian contribution `κ · ∂²b/∂x²`. This means:

- The global solve has no knowledge of contact stiffness.
- When a vertex is near a collider, the barrier gradient added to the RHS creates a force, but the LHS (system matrix) has no corresponding stiffness term to properly resolve it.
- This mismatch causes the PD iteration to take very small effective steps near contacts, leading to poor convergence and residual oscillation.

In proper IPC implementations, the barrier Hessian is either:
1. Added to the system matrix and refactored (full Newton approach), or
2. Approximated via a proxy stiffness that doesn't require refactoring (e.g., lagged Hessian approximation).

### 1.3 Gradient Clamping at ±1e4 (Significant)

**Location:** [barrier.rs:70](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-contact/src/barrier.rs#L70)

```rust
(kappa * raw).clamp(-1e4, 1e4)
```

**Problem:** Hard-clamping the barrier gradient caps the maximum repulsive force, preventing the barrier from generating arbitrarily strong repulsion near zero distance. While this prevents numerical overflow, it:

- Breaks the mathematical guarantee of IPC (the barrier must go to ∞ as d → 0).
- Creates a "ceiling" on repulsive forces, allowing slow penetration under sustained load (e.g., the weight of the cloth pressing against the sphere).
- The clamped gradient means the solver "gives up" enforcing the barrier when the cloth is very close, leading to the persistent gap rather than clean contact.

### 1.4 Dual Ground Plane Enforcement (Significant)

**Location:** Multiple competing ground enforcement mechanisms:
1. [state.rs:162](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/state.rs#L162) — `GROUND_SURFACE_OFFSET = 0.005` (5mm hard offset)
2. [state.rs:169-178](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/state.rs#L169-L178) — `enforce_ground()` clamps positions to `ground_height + 0.005`
3. [state.rs:184-201](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/state.rs#L184-L201) — `enforce_ground_velocities()` zeros downward velocity + applies 50% tangential friction
4. [ground_plane.rs:68-106](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-contact/src/ground_plane.rs#L68-L106) — IPC barrier gradient for ground
5. [ground_plane.rs:26-65](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-contact/src/ground_plane.rs#L26-L65) — Hard position projection (called post-solve in viewer)
6. [collision_pipeline.rs:260-272](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-contact/src/collision_pipeline.rs#L260-L272) — IPC contact detection for ground

**Problem:** There are **six separate mechanisms** controlling ground contact behavior. These interact destructively:

- The 5mm `GROUND_SURFACE_OFFSET` creates a visible gap between the cloth and the floor.
- The hard position clamping (enforce_ground) fights against the barrier gradient, creating a discontinuity in the energy landscape.
- The post-solve position projection in the viewer (line 345-347) overrides the barrier-computed positions.
- The velocity zeroing in `enforce_ground_velocities()` applies 50% tangential friction blanket-style, which doesn't coordinate with the IPC velocity filter.

### 1.5 Post-Solve Position Correction Overriding IPC (Significant)

**Location:** [lib.rs:341-348](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-viewer/src/lib.rs#L341-L348)

```rust
// Post-solve: enforce analytical colliders as a safety net.
if let Some(ref mut pipeline) = collision {
    if let Some(ref sphere) = pipeline.sphere {
        sphere.resolve(state);  // Hard position projection!
    }
    if let Some(ref ground) = pipeline.ground {
        ground.resolve(state);  // Hard position projection!
    }
}
```

**Problem:** After the IPC solver carefully computes barrier-consistent positions, the viewer **hard-projects** any remaining penetrations. This creates:
- Velocity artifacts: The position jump creates spurious velocity in the next frame.
- Energy injection: Moving vertices without accounting for elastic energy creates artificial energy that manifests as bouncing.
- Defeat of the barrier purpose: If hard projection is the fallback, the barrier solve's work is partially wasted.

### 1.6 Weak Augmented Lagrangian Configuration (Moderate)

**Location:** [config.rs:129-134](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/config.rs#L129-L134)

```rust
al_max_iterations: 3,
al_mu_initial: 1.0,
al_mu_growth: 2.0,
al_tolerance: 1e-4,
```

**Problem:**
- `al_mu_initial: 1.0` is far too low. The IPC paper recommends initial μ on the order of the elastic stiffness (typically 1e3–1e6 for cloth-scale simulations).
- Only 3 AL outer iterations provides insufficient convergence. The original IPC paper typically uses 5–20 outer iterations.
- `al_mu_growth: 2.0` is conservative; the standard is 10× growth per unsuccessful iteration.
- These settings mean the contact constraints are severely under-enforced, producing the "soft" contact feel and persistent gap.

### 1.7 Inconsistent Velocity Filter (Moderate)

**Location:** [pd_solver.rs:820-882](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/pd_solver.rs#L820-L882)

**Problems:**
1. The velocity filter uses per-vertex contact normals derived from `barrier gradient direction` rather than pure geometric normals. For non-convex contact regions, the gradient can point in unexpected directions.
2. The filter applies `contact_damping = 0.3` uniformly to **all three velocity components** of contacting vertices (line 879-881), including the tangential component. This conflicts with the Coulomb friction model computed just above it.
3. The filter runs **after** `update_velocities()`, meaning it modifies velocities derived from positions that may have been hard-clamped by `enforce_ground()`. The sequence creates a feedback loop: clamped position → small velocity → small inertial prediction → barrier pushes up → overshoot → repeat.

### 1.8 f32 Precision for Barrier Computations (Moderate)

**Location:** Throughout [barrier.rs](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-contact/src/barrier.rs), [ipc_response.rs](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-contact/src/ipc_response.rs)

**Problem:** Barrier function evaluation uses `f32` precision. The log-barrier `-(d - d̂)² · ln(d/d̂)` involves subtraction of nearly-equal values when `d ≈ d̂`, causing catastrophic cancellation. At `d_hat = 1e-3`, vertices at distance 0.03m from the surface produce `d² = 9e-4`, and `d² - 1e-3 = -1e-4`, which has only ~3 significant digits in f32. This creates noisy, oscillating gradients near the barrier boundary.

### 1.9 CCD Step Size Application (Minor)

**Location:** [pd_solver.rs:744-749](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-solver/src/pd_solver.rs#L744-L749)

```rust
if max_alpha < 1.0 { max_alpha *= 0.8; } // Safe margin
```

**Problem:** The CCD step is applied to the raw `sol - pos` direction, but this direction was computed by the global solve using stale barrier gradients (issue 1.1). The CCD is therefore limiting steps based on a direction that may not be the actual trajectory. Additionally, the 0.8 safety factor means the PD solver never takes a full step when CCD triggers, persistently under-shooting the contact surface.

### 1.10 Bench Runner Double-Steps Collision (Minor)

**Location:** [runner.rs:205](file:///Users/tawhid/Documents/Codes/vistio/crates/vistio-bench/src/runner.rs#L205)

```rust
let _ = pipeline.step(&mut state)?;  // Full pipeline.step AFTER IPC solve
```

**Problem:** The bench runner calls `pipeline.step()` after the IPC solver has already resolved contacts. This runs the full collision pipeline including hard position projections, ground resolution, and non-IPC self-collision response — undoing the IPC solver's work and injecting energy.

---

## 2. Gaps Between Current Implementation and State-of-the-Art

| Aspect | Vistio (Current) | State-of-the-Art | Gap Severity |
|---|---|---|---|
| **Barrier gradient update** | Frozen per AL iteration | Updated every Newton/PD iteration | 🔴 Critical |
| **System matrix** | Constant (no barrier Hessian) | Barrier Hessian included or approximated | 🔴 Critical |
| **Contact model** | Rigid analytical colliders only | Deformable-deformable + compliant bodies | 🟠 Significant |
| **Ground/collider approach** | Hybrid barrier + hard projection | Pure barrier-based (no projection) | 🟠 Significant |
| **AL parameters** | μ₀=1.0, 3 iters, 2× growth | μ₀~1e4, 5–20 iters, 10× growth | 🟡 Moderate |
| **Precision** | f32 barriers | f64 barriers with f32 positions | 🟡 Moderate |
| **Friction model** | Post-solve velocity filter | Implicit friction (lagged, inside Newton loop) | 🟡 Moderate |
| **Self-collision** | BVH + vertex-triangle only | VT + edge-edge with mollification | 🟡 Moderate |
| **Line search** | CCD step only | Armijo + CCD combined | 🟡 Moderate |
| **Solver architecture** | PD + external AL | Unified Newton with warm-starting | 🟡 Moderate |
| **Adaptive κ** | Static initial estimate | Dynamic κ per AL iteration (IPC recipe) | 🟡 Moderate |
| **Mesh resolution** | Fixed 20×20 | Adaptive remeshing | ⬜ Future |
| **GPU acceleration** | CPU-only | GPU global solve + local step | ⬜ Future |

---

## 3. Advanced Algorithms and Techniques to Adopt

### 3.1 Unified Barrier-Augmented Lagrangian (BAL) Method

**Source:** "Barrier-Augmented Lagrangian for GPU-based Elastodynamic Contact" (2024)

The BAL method introduces a **slack variable** into the IPC formulation that improves system conditioning. Instead of the raw barrier:

```
min E(x) + κ · Σ b(d_i(x), d̂)
```

BAL reformulates as:

```
min E(x) + κ · Σ b(s_i, d̂)
s.t. s_i = d_i(x)
```

The slack variables `s_i` decouple the barrier stiffness from the geometric solve, producing a better-conditioned system. This directly addresses Vistio's barrier Hessian problem (issue 1.2) without requiring refactoring the system matrix.

### 3.2 Compliant Contact Modeling

**Concept:** The user's intuition about modeling collision bodies as deformable/compliant is well-founded. In practice, this is achieved through:

1. **Soft barrier thickness:** Instead of `d̂ → 0` (infinitely hard), use `d̂ = fabric_thickness ≈ 1mm`. This creates a compliant "cushion" where the barrier provides gentle, graduated pushback rather than a hard wall.

2. **Contact compliance parameter:** Introduce a compliance `α` such that the effective barrier stiffness is `κ_eff = κ / (1 + α)`. Larger α → softer contact → smoother settling.

3. **Deformable body meshes:** For mannequin/body simulation, represent the body as a low-stiffness FEM solid. When cloth presses against it, the body yields slightly, creating a natural contact patch rather than a knife-edge contact line. Style3D and CLO3D both implement this for tight-fitting garments.

### 3.3 Implicit Friction via Lagged Iteration

**Source:** Li et al. 2020 (IPC), Bridson et al. 2003

Current approach: friction is a post-solve velocity filter, completely outside the optimization. This means friction forces don't influence the position solve, creating a disconnect where the cloth slides to a position without friction, then friction is applied retroactively.

**Recommended approach:** Lagged implicit friction:
1. At the start of each AL iteration, compute the friction force from the previous iteration's contact data.
2. Add the friction force to the RHS during the PD solve.
3. Update the friction model after the solve.

This converges to the correct frictional contact solution over AL iterations, rather than applying friction as a post-hoc correction.

### 3.4 IPC κ Update Schedule

**Source:** Li et al. 2020, Section 6

The IPC paper specifies an adaptive κ schedule:

```
κ_{k+1} = κ_k * max(1, min(κ_max / κ_k, ε_d / max_violation))
```

Where `ε_d` is the target distance tolerance. This ensures κ grows only when needed and doesn't explode. Vistio currently uses a static κ computed once at initialization.

### 3.5 Non-Distance Barriers

**Source:** "Efficient GPU Cloth Simulation with Non-distance Barriers" (SIGGRAPH Asia 2024)

This replaces the distance-based barrier with a **lifespan-based** barrier that tracks how long a collision has been active. Key advantages:
- No per-iteration CCD (CCD only for line search)
- Naturally handles resting contact (long-lived contacts get stable, high barriers)
- GPU-native computation

This is the most promising medium-term upgrade path for Vistio's contact system.

### 3.6 Domain-Decomposed PD with Additive Schwarz

**Source:** SIGGRAPH 2025

For production-grade performance on CPU, this approach:
1. Partitions the mesh into overlapping subdomains
2. Prefactors each subdomain independently
3. Uses additive Schwarz iteration for the global solve

This enables near-linear scaling with core count and matches GPU performance on modern multi-core CPUs.

---

## 4. Architectural Upgrades Required

### 4.1 Unified Contact Resolution (Remove Dual System)

**Current state:** There are two parallel collision systems:
- The **IPC barrier** system (gradients → RHS → solver)
- The **hard projection** system (position correction after solve)

**Required change:** Remove all hard position projection for colliders that have IPC barriers. The IPC barrier must be the **sole** mechanism for contact enforcement. The `enforce_ground()`, `enforce_ground_velocities()`, `sphere.resolve()`, and `ground.resolve()` calls must be eliminated from the IPC codepath.

### 4.2 Per-Iteration Barrier Update

**Current state:** Barrier gradients computed once per AL outer iteration.

**Required change:** Move `handler.detect_contacts()` inside the inner PD loop, so barrier gradients are recomputed at every PD iteration. This is the single most impactful architectural change.

### 4.3 Barrier Hessian Proxy

**Current state:** System matrix is constant (no barrier contribution).

**Required change:** Add a diagonal barrier Hessian approximation to the system matrix. For each vertex in contact:
```
A[i,i] += κ · ∂²b/∂d² · (∂d/∂x)²
```

Since this changes per-iteration, use a **lagged diagonal**: compute the barrier Hessian at the start of each AL iteration and add it as a diagonal modification. This doesn't require refactoring the Cholesky — it can be handled by adding a diagonal perturbation to the RHS computation:
```
rhs[i] -= H_barrier[i] * (x[i] - x_prev[i])
```

### 4.4 Precision Upgrade for Barriers

**Required change:** Compute barrier energy, gradient, and Hessian in `f64`, then cast results to `f32` for the solver. This eliminates the catastrophic cancellation near the barrier boundary.

---

## 5. Collision and Contact Model Improvements

### 5.1 Priority 1: Fix the Existing System (Immediate)

These changes address the root causes of the current oscillation without requiring architectural redesign:

| Change | Impact | Effort |
|---|---|---|
| Move `detect_contacts` inside inner PD loop | Eliminates stale gradient bounce | Low |
| Remove all hard projection from IPC codepath | Eliminates energy injection | Low |
| Remove `GROUND_SURFACE_OFFSET` | Eliminates 5mm gap | Trivial |
| Increase AL params: μ₀=1e4, iters=10, growth=10 | Tighter contact enforcement | Trivial |
| Remove gradient clamping (use f64 barriers) | Restores IPC guarantee | Low |
| Remove post-solve `sphere.resolve()` / `ground.resolve()` in viewer | Eliminates double-enforcement | Trivial |

### 5.2 Priority 2: Proper Friction and Damping (Short-term)

| Change | Impact | Effort |
|---|---|---|
| Lagged implicit friction in RHS | Physical friction during solve | Moderate |
| IPC adaptive κ schedule | Proper barrier scaling | Low |
| Barrier Hessian diagonal proxy | Better convergence near contact | Moderate |
| Contact-aware Rayleigh damping (higher near contacts) | Faster settling | Low |

### 5.3 Priority 3: Compliant Contact (Medium-term)

| Change | Impact | Effort |
|---|---|---|
| Thickness-aware C-IPC barriers | Physical cloth thickness | Moderate |
| Soft body collision mesh for mannequin | Realistic garment fit | High |
| Non-distance barriers (SIGGRAPH Asia 2024) | GPU-ready resting contact | High |

---

## 6. Solver Improvements and Stability Strategies

### 6.1 Chebyshev Acceleration (Already Configured, Not Working)

The config has `chebyshev_acceleration: bool` but the solver doesn't implement it. Chebyshev acceleration with a known spectral radius ρ accelerates PD convergence by a factor of ~√(1/ρ). For ρ ≈ 0.95 (typical for cloth), this provides ~4.5× faster convergence.

**Implementation:**
```
ω₁ = 1
ω₂ = 2 / (2 - ρ²)
ωₖ = 4 / (4 - ρ² · ω_{k-1})

x_k = ωₖ · (x_solve - x_{k-1}) + x_{k-1}   // Accelerated update
```

### 6.2 Warm-Starting AL from Previous Frame

**Current state:** λ is initialized to zero every frame. This means the solver must re-discover the contact force from scratch.

**Required change:** Store λ between frames and use the previous frame's converged λ as the initial guess. For resting contact, the converged λ barely changes frame-to-frame, so warm-starting provides near-instant AL convergence (often 1 iteration vs. 5–10).

### 6.3 Energy-Based Convergence Check

**Current state:** Convergence is measured by relative position change `||Δx|| / ||x||`.

**Problem:** This metric doesn't distinguish between "not converged because the solver is still iterating" and "not converged because the barrier is fighting the elastic forces." The latter case can never converge below the tolerance because there's genuine force imbalance.

**Required change:** Use an energy-based convergence criterion:
```
converged = |E(x_k) - E(x_{k-1})| / |E(x_k)| < tolerance
```

Where `E = E_elastic + E_barrier + E_inertia`. This correctly identifies the minimum of the combined objective.

### 6.4 Line Search (Armijo Backtracking)

**Current state:** CCD provides a maximum step size, but there's no energy-based line search.

**Required change:** After CCD limits the step, apply Armijo backtracking:
```
while E(x + α·Δx) > E(x) + c·α·∇E·Δx:
    α *= 0.5
```

This guarantees monotonic energy decrease and prevents the solver from overshooting into high-energy states.

---

## 7. Roadmap: Tier 4 → Production-Grade

### Phase 1: Critical Bug Fixes (1–2 weeks)

**Goal:** Eliminate oscillation and achieve stable resting contact.

1. ✏️ Move barrier gradient computation inside the inner PD loop
2. ✏️ Remove all hard position projection from IPC codepath
3. ✏️ Remove `GROUND_SURFACE_OFFSET`
4. ✏️ Upgrade AL parameters (μ₀=1e4, max_iters=10, growth=10)
5. ✏️ Switch barrier computations to f64
6. ✏️ Remove gradient clamping
7. ✏️ Remove post-solve `sphere.resolve()` / `ground.resolve()` calls in viewer and bench runner
8. ✏️ Implement warm-starting of λ between frames

**Expected outcome:** Cloth settles naturally against sphere and floor with zero visible gap and zero oscillation.

### Phase 2: Solver Quality (2–3 weeks)

**Goal:** Production-grade convergence and physical accuracy.

1. ✏️ Implement barrier Hessian diagonal proxy
2. ✏️ Implement IPC adaptive κ schedule
3. ✏️ Implement lagged implicit friction in RHS
4. ✏️ Implement Chebyshev acceleration for PD iterations
5. ✏️ Implement Armijo line search
6. ✏️ Implement energy-based convergence criterion
7. ✏️ Add edge-edge collision primitives with mollification

**Expected outcome:** Robust convergence for all scenarios including self-fold. Physical Coulomb friction behavior during draping.

### Phase 3: Compliant Contact and Realism (3–4 weeks)

**Goal:** State-of-the-art contact behavior comparable to Style3D/CLO3D.

1. ✏️ Implement C-IPC thickness-aware barriers
2. ✏️ Implement compliant contact model (soft d̂)
3. ✏️ Implement contact-aware adaptive damping
4. ✏️ Research and prototype non-distance barriers

**Expected outcome:** Visually indistinguishable from real fabric contact behavior. Cloth settles with natural thickness separation and proper fold stacking.

### Phase 4: GPU Acceleration (Tier 5) (6–8 weeks)

**Goal:** 10–50× performance improvement.

1. Port local step to wgpu compute shaders
2. Implement GPU BVH traversal
3. Implement GPU barrier evaluation
4. Implement domain-decomposed PD for multi-core CPU

### Phase 5: Adaptive Remeshing (Tier 6) (4–6 weeks)

**Goal:** Wrinkle-resolution detail.

1. Port ARCSim's anisotropic remeshing
2. Implement sizing field from curvature + velocity gradient
3. Implement system matrix refactoring on topology change

---

## Summary: Root Cause Analysis

The oscillation and non-settling behavior stem from a single fundamental issue that manifests in multiple ways:

> **The IPC barrier system and the legacy hard-projection system are fighting each other.**

The IPC barrier computes smooth, gradient-based forces to keep the cloth at a distance. The hard projections (ground enforcement, sphere resolution, velocity zeroing) then move vertices discontinuously, injecting energy. The next frame, the IPC barrier must correct for these perturbations, creating a cycle that never converges.

**The fix is architectural purity:** IPC must be the **sole** contact handler. All legacy position projections, velocity modifications, and artificial offsets must be removed from the IPC codepath. Combined with proper gradient refresh per PD iteration and stronger AL parameters, this should produce stable, realistic resting contact behavior.

**The compliant contact direction (deformable/soft collision bodies) is the correct long-term path** for achieving the highest realism tier — but the immediate oscillation issue is caused by implementation bugs rather than a fundamental limitation of the rigid-collider IPC approach.
