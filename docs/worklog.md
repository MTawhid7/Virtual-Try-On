# Garment Simulation Engine — Worklog

This document organizes progress history, encountered issues, structural adjustments, and future plans to serve as a reliable reference point across development sprints.

### 📅 April 2, 2026 (PM): Sprint 1 Complete — Layer 3b glTF Export

**Status:** ✅ Sprint 1 Fully Complete
**Focus:** Implementing glTF/GLB export and closing out Sprint 1 validation.

#### Completed Work
- **`simulation/export/gltf_writer.py`:** Created stateless `write_glb()` function using `trimesh` to produce binary `.glb` files. Handles vertex positions, faces, pre-computed normals, and optional UVs. Input validation, automatic parent directory creation, `process=False` to preserve vertex indexing.
- **`SimResult.export_glb()`:** Added convenience method to the engine result dataclass. Lazy-imports trimesh to keep it out of the engine hot path.
- **CLI `--output`/`-o` flag:** Default output path is `storage/{scene}.glb`. All three scenes (freefall, constrained_fall, sphere_drape) now export `.glb` instead of `.obj`.
- **Output directory:** Standardized on `storage/` at project root (matches architecture diagram), replacing the ad-hoc `backend/outputs/` OBJ dumps.
- **Test suite expanded to 74 tests:**
  - 12 new unit tests: file creation, trimesh roundtrip, vertex/face/normal preservation, UV optionality, parent dir creation, input validation
  - 6 new integration tests: sphere drape export roundtrip, position matching, unit-length normals, NaN checks, `SimResult.export_glb()` convenience method

#### Sprint 1 Validation Checklist — Final Results
| # | Check | Result |
|---|-------|--------|
| 1 | Particles under gravity: y-acceleration ≈ -9.81 m/s² | ✅ |
| 2 | Pinned cloth hangs naturally (distance + bending) | ✅ |
| 3 | No NaN in any field after 120 frames | ✅ |
| 4 | Cloth does not penetrate sphere | ✅ |
| 5 | Cloth does not oscillate indefinitely (KE → 0) | ✅ |
| 6 | Exported `.glb` loads in trimesh with correct geometry | ✅ |

#### Design Decisions
- **Cloth-only export:** The `.glb` contains only the simulation output mesh. Collision geometry (sphere/body) is excluded from the export — the `-v` flag provides live visualization with both meshes for debugging. This keeps exports clean for the frontend viewer in Sprint 3.
- **`storage/` over `outputs/`:** Matches the original architecture diagram. Simulation outputs are separate from source code, at the project root level.
- **Mannequin GLB deferred:** User has a mannequin `.glb` ready for Sprint 2 body mesh collision. Not needed for Sprint 1's analytical sphere.

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
