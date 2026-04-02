# Garment Simulation Engine — Worklog

This document organizes progress history, encountered issues, structural adjustments, and future plans to serve as a reliable reference point across development sprints.

### 📅 April 2, 2026: Sprint 1 Layer 3a Finalization - Analytic Collisions and Visual Pipelines

**Status:** Completed
**Focus:** Implementing the core analytical sphere collider mechanism into the solver alongside debugging and GUI hooks.

#### Completed Work & Implemented Features
- **Core XPBD Engine (Layer 1 & Layer 2):** Distance constraints and Isometric Bending constraints are fully processed efficiently via finite difference abstractions. 
- **Analytical Sphere Collider (Layer 3a):** We established swift penetration bounds checking, normal push-out vectorization, and tangent friction integrations. Collision resolves are executed directly interleaved within the XPBD iterative solver hook ensuring stable resolutions against large timesteps natively avoiding "overshooting".
- **Realtime Diagnostic Viewer Expansion:** Wrote a cross-platform live viewer running off Taichi's GGUI (`window.get_scene()`) initialized by dropping `--visualize / -v` onto standard CLI executions (`freefall`, `constrained_fall`, `sphere_drape`), drastically increasing feedback density while preserving raw computational performance.
- **Project Structure Refactor:** To ensure immediate scalable capacity over scaling scene scripts as complexity rises, decoupled all heavy testing environments out from root `__main__.py` into their respective `backend/simulation/scenes/*.py` namespaces properly abstracting user inputs from heavy scene boilerplate.
- **Organized Outputs:** All physics exports have been securely bound to dump out to `backend/outputs/` directory dynamically creating themselves, eliminating noise out of the repo root.

#### Issues Encountered & Diagnostics
- **Taichi Compilation Strictness (`ti.template()` Error):** 
  - *Cause:* Found a severe breakdown over parsing generic template bounds while tracking object attributes if `from __future__ import annotations` was universally declared inside the scope.
  - *Solution:* Dropped future annotation requirements inside Taichi-heavy logic modules (`sphere_collider.py`) limiting usage to isolated global methods where `ti.template()` could parse securely directly into native Python ast.
- **"Motionless" Output Confusion:**
  - *Cause:* Standard `obj` execution natively renders statically, and initial tests failed to artificially stitch the analytical sphere collision points directly into the returned export geometry. 
  - *Solution:* Hardcoded programmatic `trimesh.icosphere` integration into final frame output geometries. We now inject both meshes offset intelligently so viewing agents/teams see exactly what collided. Appended realtime `-v` functionality for deeper diagnostics instead.

#### Observations & Lessons Learned
- **Decoupled Architecture Scalability:** By retaining extremely strict barriers isolating tests mathematically apart from structural mesh execution strings (e.g. `tests/integration`), validation and isolating regressions executes instantaneously. The XPBD structural abstraction allows for effortless modifications downstream.
- **Physical Bounciness is Kinematic:** Pure iterations absent damping create elastic "rubber sheet" responses under harsh analytical collision parameters. Real cloth is deadened, validating our future pivot toward enforcing strict velocity caps inside the Sprint 2 mesh upgrades.

#### Future Plans & Next Steps
- **Layer 3b (Exporting):** Translate OBJ scripts robustly handling continuous data buffers baking simulations into `glTF` static sequences usable directly inside frontend canvases.
- **Sprint 2 Architecture Shift:** Substitute out analytical math constraints targeting real-world arbitrary body meshes parsing actual geometries mapped from rigid bounds (`trimesh`).
- **Post-Collision Damping Hooks:** Explicitly introduce and profile strict velocity clamps combating unnatural bounce thresholds natively inside the main loop iteration solver as complexity mounts.
