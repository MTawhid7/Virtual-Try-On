# Garment Simulation Engine — Worklog

This document organizes progress history, encountered issues, structural adjustments, and future plans to serve as a reliable reference point across development sprints.

---

### 📅 April 3, 2026: Sprint 2 Layer 3a — Collision Resolver Investigation & Mesh Pipeline

**Status:** ⚠️ In Progress — 10/12 integration tests passing (same count, different failing tests)  
**Focus:** Fix the 2 remaining failing integration tests in the body mesh collision layer. Validate and stabilize the body mesh asset pipeline.

#### Body Mesh Asset Pipeline

The body mesh was replaced. The previous asset (`male_body.glb`) was a manual Blender export that suffered from the "polygon soup" problem — Blender's glTF exporter duplicated every vertex at edge boundaries due to custom split normals, inflating a 5,690-face mesh to 16,500 disconnected vertices that `merge_vertices()` could not weld. This was identified as the root cause of instability in prior testing.

The new `mannequin.glb` (Sketchfab asset) was evaluated alongside a single-object re-export (`mannequin_2.glb`). Key findings:

- `mannequin.glb` has 3 scene objects: body (5,390 verts, 8,844 faces) + 2 eye meshes (2×380 faces). `trimesh.load(force='mesh')` already merges them. The existing `smart_process` pipeline handles this correctly.
- `mannequin_2.glb` (single object export) had a 6× worse avg edge length (0.124m vs 0.021m) — unusable as a physics asset. Deleted.
- The processed `mannequin_physics.glb` (5,689 verts, 9,604 faces, avg_edge 0.021m, 1.75m tall) is confirmed as the correct physics asset. It has 59 disconnected components — inherent open-boundary topology, not a processing failure. `fill_holes()` was tested and made things worse (59 → 62 components).
- Tests and scenes were updated to point directly to `mannequin_physics.glb`. `BodyCollider.from_glb()` was updated to detect the `_physics` suffix and skip `smart_process` (which would otherwise create a nonsensical `_physics_physics.glb` output).

#### Collision Resolver Investigation

The two failing tests are in tension with each other. Three candidate selection strategies were tested:

**Strategy 1 (original): Max-sd + 0.10m Euclidean guard**  
Tracks the shallowest penetrating triangle (maximum signed distance < thickness). Gives: energy_decay = 1.44 m/s (fails < 1.0), crumpling max_y = 2.05m (fails ≤ 1.85m). Root cause: false-hit triangles in the 0.063–0.10m euclidean range win the max-sd comparison when their normals happen to be close to 0.

**Strategy 2: Max-sd + cell_size × 2.0 Euclidean guard (0.063m)**  
Tightens the guard to scale with mesh resolution. Fixes crumpling (max_y ≤ 1.85m ✅) but causes energy explosion (35.9 m/s ❌). A wrong-normal triangle within the 0.063m zone wins max-sd and injects energy systematically every substep.

**Strategy 3 (current code): Min-euclidean + cell_size × 2.0 Euclidean guard**  
Selects the nearest triangle (minimum euclidean distance to closest point). Applies response only if nearest triangle has sd < thickness. Gives: energy_decay = 0.63 m/s ✅, crumpling max_y = 1.96m ❌ (still exceeds 1.85m by 0.11m). The upward crumpling is less severe than before but still fails. Cause: at mesh seam boundaries and open-boundary edges (59 disconnected components means seam edges are everywhere), the nearest triangle's interpolated normal is rotated relative to the true outward normal.

The core dilemma: no single candidate selection strategy fixes both tests simultaneously given the current mesh geometry and test thresholds.

#### Code Changes Made

| File | Change |
|------|--------|
| `collision/resolver.py` | Candidate selection: max-sd → min-euclidean; Euclidean guard: hardcoded 0.10m → `cell_size * 2.0` |
| `collision/body_collider.py` | Skip `smart_process` when path ends with `_physics` |
| `scenes/body_drape.py` | Asset: `male_body.glb` → `mannequin_physics.glb` |
| `tests/integration/test_layer3a_ext_body.py` | Asset: `male_body.glb` → `mannequin_physics.glb` |
| `CLAUDE.md` | Updated Critical Constraints (i32 overflow, max_contact_dist), Body Mesh section, Current State |
| `README.md` | Updated asset paths, status table, Quick Start |
| `docs/` | Deleted 2 old handoffs → replaced with `handoff_sprint2_layer3a_complete.md` |

#### Proposed Next Steps (Prioritized)

1. **Add backface-cull check to min-euclidean response** — After selecting nearest triangle, verify `dot(p - closest, best_normal) >= 0` (particle is on outward side). Reject response if negative — this filters seam-boundary triangles whose normals are rotated inward. This is the most promising fix for the crumpling test while preserving energy stability.

2. **Increase hash table size** — `StaticSpatialHash(table_size=262144)` reduces average bucket load from ~4.5 to ~1.1 entries, dramatically cutting false-hit candidate rate. Lower false-hit rate means fewer wrong-normal candidates in the search set.

3. **Calibrate test thresholds** — The crumpling margin of 0.05m (1.85m threshold) may be too tight for this mesh's topology. If the backface-cull fix still gives 1.87–1.89m, raising to 0.10m (1.90m) is defensible. The energy threshold (1.0 m/s) appears reasonable and should not be loosened.

4. **Full Sprint 1 regression** — `python -m pytest tests/ -v` has not been run since code changes. Confirm 74 Sprint 1 tests still pass.

5. **Visual smoke test** — Run `python -m simulation --scene body_drape` and view `storage/body_drape.glb` in a glTF viewer to confirm shoulder drape with no visible penetration.

---

### 📅 April 2, 2026 (Late PM): Sprint 2 Layer 3a-Extended — Body Mesh Collision

**Status:** ⚠️ In Progress — 10/12 integration tests passing  
**Focus:** Replace `SphereCollider` with `BodyCollider` using static spatial hash + point-triangle projection. Cloth grid dropped above the mannequin should drape over shoulders with zero penetration.

#### Completed Work

**New files:**
- `simulation/collision/point_triangle.py` — `@ti.func` geometry: `closest_point_on_triangle` (Voronoi region, all 7 cases), `closest_point_and_bary`, `signed_distance_to_triangle` (interpolated vertex normals — Vestra's "smoothed normal trick")
- `simulation/collision/spatial_hash.py` — `StaticSpatialHash`: Taichi fields for cell lookup + flat triangle index array. NumPy counting-sort build, `@ti.func` `hash_cell` + `cell_indices`.
- `simulation/collision/resolver.py` — `@ti.kernel resolve_body_collision`: queries 27-cell neighborhood, Euclidean distance guard, tracks shallowest penetration (max sd), position-based friction matching `SphereCollider`.
- `simulation/collision/body_collider.py` — `BodyCollider.from_glb()`: load GLB → rescale to 1.75m (body is in cm) → decimate to ~5K faces → remove degenerate triangles → recompute normals → auto cell_size (1.5× mean edge) → build hash. `resolve()` delegates to the kernel.
- `simulation/scenes/body_drape.py` — 30×30 cloth grid dropped at Y=1.8m onto body mesh. Same 6 validation checks as `sphere_drape.py`.
- `tests/unit/collision/test_point_triangle.py` — 20 tests: all 7 Voronoi regions, barycentric sum, signed distance sign, degenerate triangle. **20/20 passing.**
- `tests/unit/collision/test_spatial_hash.py` — 10 tests: single/multiple triangle build, no false negatives at vertices, distant query, hash collision handling. **10/10 passing.**
- `tests/integration/test_layer3a_ext_body.py` — 12 tests: collider load, decimation, vertex storage, no-NaN, no-penetration, drape shape, no-upward-crumpling, energy decay, edge length, interface swap. **10/12 passing.**

**Modified files:**
- `collision/__init__.py` — exports `BodyCollider`
- `scenes/__init__.py` — exports `run_body_drape`
- `simulation/__main__.py` — adds `body_drape` CLI scene
- `requirements.txt` — adds `fast-simplification>=0.1.7`
- `backend/data/bodies/male_body.glb` — copied from project root (41K vertices, in centimeters)

#### Issues Encountered & Root Causes

**Bug 1 — Taichi i32 overflow in hash build (FIXED)**  
`_hash_cell_np` in `spatial_hash.py` used Python unlimited integers. Taichi's `ti.i32` wraps on overflow (C-style). For body mesh coordinates (ix up to ~32), `32 × 73856093 = 2.36e9` overflows `ti.i32` max (2.15e9), giving a different bit pattern. Triangles went into wrong hash buckets — the kernel found wrong candidates for every overflowing cell. Effect: particle teleportation (134 m/s mean speed).  
*Fix:* `((ix * P1) & 0xFFFFFFFF) ^ ...` — mask to 32-bit before XOR to match Taichi's wrapping exactly. Must not use `np.int32()` constructor — raises `OverflowError` for values > `int32_max`.

**Bug 2 — Hash collision false positives (PARTIALLY FIXED)**  
Even with correct key arithmetic, a 65536-bucket table with ~300K entries has genuine hash bucket collisions. A shoulder particle (Y≈1.5m) would receive foot triangle candidates (Y≈0.05m). If the foot's vertex normals point downward (−Y), the signed distance `dot(p − closest, n)` from the shoulder particle is large-negative, triggering a collision response that teleports the particle 1.4m toward the foot.  
*Partial fix:* Added Euclidean distance guard (`|p − closest| > 0.1m → skip`) and changed from tracking minimum sd (deepest penetration) to maximum sd (shallowest — outermost surface). Improved mean speed from 134 m/s → 1.44 m/s.  
*Remaining:* 2 tests still fail (energy: 1.44 m/s > 1.0 threshold; upward crumpling: max Y = 2.05m > 1.85m). The 0.1m Euclidean threshold is still too loose for this mesh's cell_size (0.016m). Proposed next fix: tighten to `cell_size × 2 ≈ 0.035m`.

**Bug 3 — trimesh decimation API (FIXED)**  
`mesh.simplify_quadric_decimation(5000)` passes `5000` as `percent` (first positional arg, expected 0–1), not face count. Raises `"target_reduction must be between 0 and 1"`.  
*Fix:* Use `face_count=5000` as keyword argument.

#### Key Observations & Insights

- **Hash correctness requires i32 simulation.** Any Python function that builds the hash for Taichi to query must use `& 0xFFFFFFFF` masking on each multiplication term before XOR. Failure is silent — hash silently diverges for high coordinates (ix > ~29 with P1=73856093).
- **"Closest triangle" means Euclidean, not signed-distance.** Tracking minimum signed distance finds the most-penetrating triangle, which may be a false hash hit (large negative sd from a distant triangle facing away). Tracking maximum sd (shallowest contact) + Euclidean guard gives geometrically correct behavior.
- **Spatial hash table size matters.** With 5K decimated triangles × ~60 cells each ≈ 300K entries in 65536 buckets: ~4.5 entries per bucket → high collision rate. Consider 262144 (2^18) for next iteration.
- **Decimation + degenerate removal is necessary.** `simplify_quadric_decimation` produces zero-area triangles. These cause NaN in barycentric projection. `mesh.area_faces < 1e-10` filter removes them before upload.
- **Body mesh unit is centimeters.** GLB Y range 0.58–180.5. Scale = 1.75 / (180.5 − 0.58) ≈ 0.00972. Must shift feet to Y=0 after scaling.
- **`from __future__ import annotations` is forbidden in kernel files.** Applies to `point_triangle.py`, `spatial_hash.py`, `resolver.py`. Safe only in non-kernel orchestrator files like `body_collider.py`.

#### Passing Tests (10/12)
- `test_body_collider_loads`, `test_body_collider_decimated`, `test_body_collider_vertex_data_stored`
- `test_body_drape_no_nan`
- `test_body_drape_no_penetration`
- `test_body_drape_shape`
- `test_body_drape_edge_lengths_preserved`
- `test_engine_accepts_body_collider`, `test_engine_accepts_sphere_collider`, `test_both_colliders_have_resolve_method`

#### Failing Tests (2/12)
- `test_body_drape_energy_decay` — mean speed 1.44 m/s, threshold 1.0 m/s
- `test_body_drape_no_upward_crumpling` — max Y = 2.05m, threshold 1.85m

#### Next Steps
1. Tighten `max_contact_dist` in `resolver.py` from `0.10m` → `cell_size × 2 ≈ 0.035m`
2. If still failing: switch to minimum Euclidean distance candidate selection, apply response only if that nearest triangle has sd < thickness
3. Consider increasing `table_size` to 262144 to reduce hash collision rate
4. Run full `python -m pytest tests/ -v` regression (Sprint 1 + Sprint 2 tests)
5. Manual: `python -m simulation --scene body_drape` → inspect GLB in glTF viewer

---

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
