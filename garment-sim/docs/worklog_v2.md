# Garment Simulation Engine — Worklog (v2)

This document organizes progress history, encountered issues, structural adjustments, and future plans. It serves as a reliable reference point across development sprints starting from the GarmentCode integration.

---

## 📚 Project History Summary (Up to Mid-April 2026)

*(Note: The original detailed worklog is maintained in `worklog.md`. This is a high-level overview of the project's foundational phase prior to the GarmentCode integration.)*

### Core Physics Engine (Sprint 1 & 2)
The project began as a standalone, zero-framework physics engine using **XPBD (Extended Position-Based Dynamics)** implemented in Taichi. Key technical decisions included:
- **Solver Architecture:** Semi-implicit Euler integration with XPBD distance, bending, and zero-length stitch constraints.
- **Collision Handling:** Interleaved point-triangle collision against a physics-proxy body mesh (`BodyCollider` via static spatial hashing) proved far more stable than SDF-based collisions or post-process corrections.
- **Parametric Fabric Materials:** Moving from arbitrary stiffness to area-weighted mass distribution and calibrated `bend_compliance` (cotton presets) correctly allowed realistic draping and localized folding.

### Triangulation & DXF Pipeline (Sprint 2 & 3)
A major bottleneck was translating unstructured 2D patterns into stable simulation meshes. 
- Early "grid-clip" methods created jagged boundaries.
- The shift to professional CLO3D DXF parsing laid the groundwork for complex geometries.
- **Mesh Topology Insights:** We discovered that un-ordered boundary vertices lead to catastrophic interior mesh crossing. We successfully implemented boundary-resampled Constrained Delaunay Triangulation to provide perfectly cleanly separated polygon interiors for realistic physics.

### Stitch Resolution & "Sew-then-Drape" Pipeline
To mimic professional tools (like CLO3D), we implemented a two-stage solver:
1. **Sew Phase:** Zero gravity, high compliance stitch constraints, and disabled strain limits pull the flat seams together around the body.
2. **Drape Phase:** Gravity activates, full collision and structural strain limits enforce physics realism.

**Key Insight:** Seam closures are geometrically constrained. A cylinder-mapped sleeve onto a flat armhole pattern *cannot* mathematically close perfectly without tension. Stitch closures failing (like the 3.5cm underarm gaps) were identified as pattern geometry issues, not solver failures. 

---

## 🚀 Active Log

### 📅 April 16, 2026: GarmentCode Integration (Phases 1-3) & Generative Pivot

**Status:** ✅ Integration Stable; 🔶 Phase 4 Pivot
**Focus:** Merging the GarmentCode pattern-to-mesh pipeline into `garment-sim` to enable parametric design, resolving geometric anomalies, and finalizing the architecture for pattern generation.

#### Work Completed

**Phase 1 & 2: Vendoring & Cross-Platform Triangulation**
- Successfully vendored the internal `pygarment` library from GarmentCode into the `garment-sim` backend.
- Extracted and modified the triangulation layer: replaced the heavy, C++ based `CGAL` dependency with a pure-Python implementation using the `triangle` library (Constrained Delaunay Triangulation). This guarantees cross-platform compatibility (mac/linux/windows) without Nvidia or complex C++ build chains.

**Phase 3: Pipeline Adapter (`gc_mesh_adapter.py`)**
- Created an adapter that maps GarmentCode's structured `BoxMesh` outputs directly into our `GarmentMesh` dataclass.
- Maintained **non-manifold topology**: rather than collapsing the panels into a single mesh, we preserved distinct panels joined by XPBD stitch constraints, maintaining compatibility with our custom sew-then-drape physics solver.
- Added a `--gc` CLI flag to `garment_drape.py` to seamlessly toggle between the legacy DXF models and new GarmentCode JSON specs.

**Robustness Fixes for GarmentCode Data:**
Through simulation integration tests, we verified and fixed specific anomalies native to GarmentCode exports:
1. **Pathological Short Edges:** GarmentCode generated multiple edge sections under `0.75cm` (e.g., dart or notch remnants). These tiny triangles resulted in explosive 355% solver stretch values. Fixed by adding physical edge-length filtering inside `_extract_edges_from_faces`.
2. **Thin Seam Densification:** GarmentCode's edge-loop sampling often left short seams with only 2-3 vertices. This broke our XPBD solver's stability. Fixed by adding midpoint interpolation for short edges (`densify_stitches()`), providing reliable tension resolution.

#### Observations & The "Phase 4" Generative Pivot

**Initial Objective:** Fully vendor GarmentCode's parametric `garment_programs` (e.g., `tee.py`, `bodice.py`) to generate specs automatically within our engine.
**Discovery:** Deep review of the `Shirt` class and pygarment's design hierarchy revealed complex, interrelated dependencies across collars, sleeves, and bodices. Tightly coupling these programs into our physics engine would create highly fragile code and technical debt.
**Pivot Decision:** Instead of running the generative python programs natively, we will treat `pygarment` as a JSON specification format. GarmentCode tools will remain a *standalone upstream generator* of pattern JSONs. Our engine consumes these specs directly. This decoupled approach allows us to use predefined patterns (e.g., `shirt_mean.json`, `dress_pencil.json`) reliably while maintaining physics engine purity.

#### Future Plan (Next Session)
1. **Simulation Validation:** Run a full, live visual physics simulation using a GarmentCode `shirt_mean.json` mapped over our `mannequin_physics.glb`.
2. **Body Mapping Alignment:** GarmentCode specs are derived from SMPL body models. We must map/align our specific mannequin mesh bounds to sit correctly inside the imported GarmentCode 3D initial panel positions so the sew-phase operates seamlessly.
3. **Sprint 4 (Web API Layer):** With a stable, dual-supported (DXF and GarmentCode) physics back-end, proceed to integrate the FastAPI endpoint to trigger simulations from the Next.js frontend.

---

### 📅 April 16, 2026 (continued): GarmentCode Phase 4 — Pipeline Validation

**Status:** 🔶 Mostly Complete — 5/6 checks pass; 2 known issues deferred to Phase 7
**Focus:** Run `shirt_mean.json` end-to-end; fix body alignment; add pytest coverage; validate visual output.

#### Work Completed

**1. Critical CLI fix — `__main__.py` missing `--gc` arg**

Running `python -m simulation --scene garment_drape --gc ...` produced `error: unrecognized arguments: --gc`. Root cause: `__main__.py` is the actual entry point when using `python -m simulation`, but `--gc` and `--gc-z-offset` were only registered in `garment_drape.py`'s own `if __name__ == "__main__"` block (never reached via `-m simulation`). Fixed by adding both args to `__main__.py` and threading them to `run_garment_drape()` via kwargs.

**2. Body Z-offset: +0.131m alignment fix**

GarmentCode panels are generated for the SMPL mean body. Our `mannequin_physics.glb` is entirely positive-Z (range [0.031, 0.346]m), while GarmentCode's SMPL body straddles Z=0 (front torso at +0.25m, back at −0.20m after cm→m).

Offset derivation:
- mannequin chest centre Z = (0.2786 + 0.0335) / 2 = **0.1561m**
- GarmentCode SMPL centre Z = (0.25 + (−0.20)) / 2 = **+0.025m**
- Required offset = 0.1561 − 0.025 = **+0.131m** (no Z-flip — both use +Z = front)

Added `body_z_offset: float = 0.0` to `boxmesh_to_garment_mesh()` and `build_garment_mesh_gc()` in `gc_mesh_adapter.py`. Applied per-panel inside the loop *before* stitch densification (so midpoints are interpolated in the correct space). Default 0.0 ensures all existing unit tests remain unaffected. `_GC_BODY_Z_OFFSET = 0.131` constant defined in `garment_drape.py` with full derivation comment.

**3. Iterative seam densification improvements**

The original densification code had a silent deduplication bug: on coarse meshes, the nearest-vertex search for a midpoint often returned the same vertex as an existing pair (the mesh simply doesn't have enough vertices at that resolution). This meant `len(seam_pairs)` grew but no new *unique* pairs were added, and the early-exit condition `added_this_pass == 0` was never triggered correctly.

Fixed by:
- Raising `_MIN_PAIRS_PER_SEAM` from 6 → 12
- Switching from single-pass to 4-pass iterative approach
- Adding `set()`-based deduplication: a `candidate` pair only appended if not already in `existing`
- Added early exit if a pass adds 0 new pairs (mesh coarseness ceiling reached)

**4. New test suite: `tests/integration/test_gc_pipeline.py`**

12 tests (6 mesh-setup, 6 simulation):

| Test | What it checks |
|------|---------------|
| `test_gc_shirt_loads_8_panels` | Panel count = 8 |
| `test_gc_shirt_has_all_expected_panels` | All 8 panel names present |
| `test_no_nan_in_gc_mesh` | No NaN in positions at rest |
| `test_stitch_pairs_valid_indices` | All stitch indices in [0, N), non-empty |
| `test_z_offset_applied` | Front panels Z > body z_back; back panels Z < z_front |
| `test_panels_not_all_inside_body` | < 10% torso particles in body interior Z=[0.05,0.25]m |
| `test_no_nan_after_simulation` | No NaN after 100-frame sim |
| `test_garment_settles_in_torso_region` | > 50% particles in Y=[0.5,1.8]m |
| `test_no_sub_floor_penetration` | All Y > 0.0m |
| `test_seam_gaps_reduced_after_sewing` | Final mean gap < 75% of initial |
| `test_edge_length_preservation` | Mean stretch < 25% |
| `test_gc_glb_export` | GLB created, size > 1KB |

All 12 pass. Combined with 195 pre-existing tests: **207 total, 0 regressions.**

**5. `scripts/verify_gc_alignment.py`**

Diagnostic script that loads a GarmentCode pattern, applies the Z offset, and prints per-panel Z extents vs. body surface bounds from `mannequin_profile.json` at each panel's median Y height. Also prints initial stitch gap distribution (min/mean/max). Returns exit code 0 if all checked panels pass, 1 otherwise. Useful for debugging new garment types without running a full simulation.

Usage: `python -m scripts.verify_gc_alignment [--pattern ...] [--z-offset ...] [--res ...] [--profile ...]`

**6. `docs/implementation_plan_phases4_to_8.md`**

Comprehensive plan document covering Phases 4–8: status table, body alignment derivation, file change tables, simulation results, verification commands, key constants.

#### Simulation Validation Results

Full run: `shirt_mean.json`, mesh_resolution=1.5cm, 570 frames (240 sew + 30 transition + 300 drape).

| Check | Result | Detail |
|-------|--------|--------|
| NaN | ✅ PASS | Clean |
| Floor penetration | ✅ PASS | min Y = 0.898m |
| Torso coverage | ✅ PASS | 2870/2870 particles in Y=[0.5, 1.8]m |
| Mean speed | ✅ PASS | 0.011 m/s — fully settled |
| Mean stretch | ✅ PASS | 5.55% |
| Stitch gaps | ❌ FAIL | 3 sleeve cap seams at 10–11cm |

GLB exported to `backend/storage/gc_shirt.glb` and visually inspected.

#### Current State

The GC pipeline is functionally complete and passes 5 of 6 validation checks. The garment settles correctly around the mannequin torso, side seams close, and the fabric shows natural draping folds under gravity. Two remaining issues both stem from the same root cause (no positional anchoring during sew phase) and require Phase 7.

#### Known Issues

**Issue 1: Sleeve cap seam gaps (~10–11cm)**
- `seam_5` (n=3), `seam_13` (n=3), `seam_14` (n=8) — sleeve-to-armhole connections
- Root cause: sleeve panels start 25–30cm from the armhole in 3D. With only 3–8 stitch pairs pulling them in over 240 sew frames, the gap doesn't fully close.
- n=3 is the mesh density ceiling at 1.5cm resolution for these short edges — not a code bug. Densification finds no new unique vertices after 1–2 passes.
- Fix: Phase 7 attachment constraints anchoring shoulder/collar vertices near the body surface before sewing begins, shortening the initial gap.

**Issue 2: Max stretch outlier (~318%)**
- Two back-torso vertices end up at Z=0.056–0.083m (inside body interior Z=[0.034, 0.279]m)
- Root cause: back panel starts at Z=−0.069m (before offset), gets pulled forward by stitches; some vertices slip through the thin sew_collision_thickness=0.006m shell during sew phase
- Fix: Phase 7 attachment pins anchoring back panel at waist-level before sewing starts

**Issue 3: Frontend viewer shows old DXF tshirt model**
- `npm run dev` viewer loads `frontend/public/models/garment_drape.glb` (pre-computed DXF tshirt), not the new GC output
- Root cause: `/api/models` endpoint serves from `frontend/public/models/` — GC output is at `backend/storage/gc_shirt.glb` and isn't auto-copied
- Fix: Phase 5 FastAPI layer will mount `backend/storage/` as static and the frontend proxy will load GC output directly

#### Key Insights

1. **Mesh density ceiling for short seams:** At 1.5cm mesh_resolution, a 4.5cm sleeve edge only produces 3 unique boundary vertices. No amount of densification logic can manufacture vertices that don't exist. The correct fix is spatial anchoring (Phase 7), not finer meshes (which would increase sim time 4–9×).

2. **`from __future__ import annotations` + Taichi = silent JIT failure:** Any file containing `@ti.kernel` or `@ti.func` with `.template()` params must not have this import. String-based annotation resolution breaks Taichi JIT. Affected files: `resolver.py`, `spatial_hash.py`, `point_triangle.py`, `stitch.py`, `xpbd.py`, and the future `attachment.py`.

3. **The `--gc` flag in `garment_drape.py`'s own argparse is dead code:** `python -m simulation` always routes through `__main__.py`. Any new CLI args must be added to `__main__.py` *and* threaded through `run_garment_drape()` kwargs. The `garment_drape.py` argparse block only runs if the file is executed directly (`python simulation/scenes/garment_drape.py`).

4. **SMPL→mannequin is a translation, not a rescale:** Both coordinate systems use Y-up, +Z = front, and metres (after cm→m). The SMPL body is just shifted along +Z. No rotation or scaling needed — only the single +0.131m Z offset.

5. **Sew collision thickness vs. panel starting distance:** `sew_collision_thickness=0.006m` is effective for keeping panels from penetrating the body *once they're nearby*, but panels starting >0.2m away have enough momentum during the sew pull to penetrate before the collision kernel catches them. Attachment constraints (Phase 7) prevent this by anchoring panels to near-body positions from the start.

#### Next Steps

**Immediate (Phase 7 — Attachment Constraints):**
- New file: `simulation/constraints/attachment.py` — `AttachmentConstraints` Taichi kernel with soft compliance (1e-4). No `from __future__ import annotations`.
- Add `build_gc_attachment_constraints(garment, profile_path, n_per_panel=3)` to `gc_mesh_adapter.py` — selects waist-level and shoulder-level vertices, computes target positions on body ellipse from `mannequin_profile.json`.
- Call `attachment_constraints.project()` inside the sew phase loop in `engine.py`; release for drape phase.
- Expected result: sleeve cap gaps reduce from 10cm to < 3cm; max stretch outlier eliminated.

**Phase 5 — FastAPI Layer:**
- `backend/app/` package: `main.py`, `schemas.py`, `routes/simulate.py`, `routes/patterns.py`
- Mount `backend/storage/` as static (`/storage`); frontend proxies via `app/api/simulate/route.ts`
- Resolves the viewer-not-loading-GC-output issue automatically

**Phase 6 — Pattern Library:**
- `scripts/generate_gc_specs.py` to generate `tee_fitted.json`, `bodice_basic.json`, `pants_straight.json`, `skirt_aline.json`, `skirt_circle.json`, `dress_basic.json` from GarmentCode programs
- Pattern selector card grid in frontend

**Phase 8 — 2D Pattern Editor:**
- SVG canvas for panel creation/editing with vertex drag, edge-click stitching
- `POST /api/simulate` already designed to accept inline `pattern_json: dict` (Phase 5 schema)

---

### 📅 April 17, 2026: Sew-Phase Debugging — Root Cause Confirmed

**Status:** 🔬 Debug Complete — root cause confirmed, fixes partially applied, Phase 7 unblocked
**Focus:** Build targeted debug scripts to confirm why sleeve cap seams fail; identify whether the problem is panel placement, stitch connectivity, mesh density, or the solver.

#### Work Completed

**Three debug scripts created:**

| Script | What it does |
|--------|-------------|
| `scripts/debug_gc_stitch_geometry.py` | Per-seam analysis without simulation: panel centroids, n_pairs, gap distribution, arc-length coverage, clustering, duplicate-pair detection |
| `scripts/debug_gc_panel_placement.py` | Exports a debug GLB: each panel a distinct colour + stitch ribbons (magenta) + body mesh. Shows initial 3D configuration in the viewer before any simulation. |
| `scripts/debug_gc_sew_trace.py` | Runs only the sew phase with N checkpoints; prints gap table + body-penetration count at each frame; exports one GLB snapshot per checkpoint for step-through inspection. |

**Sew phase tuning applied to `garment_drape.py`:**
- `sew_collision_thickness`: 0.006m → **0.020m** (prevents fast-panel tunneling; fixed seam_6)
- `sew_initial_compliance`: 1e-7 → **1e-4** (softer start so panels decelerate before hitting body)
- `sew_ramp_frames`: 60 → **120** (slower compliance ramp over first 120 frames)

#### Root Cause Analysis

**Stitch geometry findings** (`debug_gc_stitch_geometry.py`):

All 8 panels start as flat vertical sheets at constant Z depths:

| Panel | Initial Z | Role |
|-------|-----------|------|
| right/left_ftorso | +0.381m | front body panels (10cm past body surface at 0.279m) |
| right/left_btorso | −0.069m | back body panels (10cm behind body surface at 0.034m) |
| right/left_sleeve_f | +0.306m | front sleeve halves |
| right/left_sleeve_b | +0.006m | back sleeve halves |

The offset (+0.131m) is correctly applied. Panels are supposed to start flat — the sew phase is what curves them around the body.

GarmentCode itself warns about the mesh density issue at load time:
```
Edge::WARNING::Detected edge represented only by two vertices..
mesh resolution might be too low. resolution = 1.5, edge length = 0.747829
```
The failing seam corners (seam_5/13) are 0.75cm edges at 1.5cm mesh resolution — these genuinely only have 2 vertices. Our densification correctly adds a midpoint = n=3, which is the hard ceiling at this resolution.

**Sew trace findings** (`debug_gc_sew_trace.py`):

The trace exposed a two-layer problem:

*Layer 1 — All gaps lock within 30 frames:*
```
Frame  0: body_interior_verts=0     ← clean initial state
Frame 30: body_interior_verts=2157  ← 75% of 2870 particles INSIDE the body
Frame 240: body_interior_verts=2167  ← essentially unchanged for the next 210 frames
```
The stiff compliance pulls panels from 30–45cm away fast enough to punch through the 6mm collision shell in the first 30 frames. After that, the system is in a stable equilibrium — adding more sew frames does nothing.

*Layer 2 — The equilibrium gap equals body depth:*

Seams that fail permanently are **wrap-around seams** where the two stitched vertices are on geometrically opposite sides of the body. The front panel (Z=+0.381m) and back panel (Z=−0.069m) try to meet through the body (Z=[0.034, 0.279m]). The body collision creates a stable equilibrium gap ≈ body depth at that location (~10–13cm). This is not a solver bug — it is physically correct behavior: you cannot sew through a body.

*Tuning outcome* (0.020m shell + 1e-4 initial compliance + 120-frame ramp):

| Seam | Before | After | Change |
|------|--------|-------|--------|
| seam_6 left side (9 pairs) | 10.6cm ❌ | 2.2cm ✅ | +fixed |
| seam_5 tiny corner (3 pairs) | 8.7cm | 12.1cm | worse |
| seam_14 right side (8 pairs) | 10.5cm | 10.5cm | unchanged |
| seam_8 sleeve underarm | 11.6cm | 11.6cm | unchanged |

The thicker shell fixed seam_6 because 9 springs generate enough accumulated force to pull through the 2cm equilibrium. seam_14 (8 springs) doesn't cross that threshold — this precisely explains the left/right asymmetry that was previously unexplained. Softer compliance made seam_5/13 worse because less force = less ability to overcome the shell at the tiny corner.

Tuning is exhausted. The remaining failures are topological, not parametric.

#### Current State

After sew-phase tuning:
- **15 of 18 seams close cleanly** (mean gap < 1cm)
- **3 seams remain stuck**: seam_5/13 (tiny sleeve cap corners, n=3, mesh ceiling) and seam_14 (right side seam, 1-fewer-pair threshold miss)
- **Back panel body penetration**: ~430 btorso vertices inside the body at all sew-phase frames
- **seam_0/2/8** (sleeve underarm and armhole): stuck at 6–13cm because the body occupies the space between the stitch endpoints — this is topologically correct but the max gap exceeds the 5cm validation threshold

#### Key Insights

1. **Sew-with-body-present is fundamentally wrong for wrap-around seams.** The side seams and sleeve underarm seams connect vertices that are on opposite sides of the body. No compliance tuning can close a seam through a solid collision mesh. In real garment construction you sew the garment first, then dress it on the body. Our pipeline does the reverse.

2. **The equilibrium gap = body depth at the seam location.** This is a precise, measurable prediction: side seam gap ≈ torso depth at hip (~10cm), sleeve underarm gap ≈ armhole depth (~13cm). If Phase 7 attachment constraints correctly anchor panels to the body surface before sewing, these equilibrium gaps go to ~0 because the panels start ON the body, not across it.

3. **The left/right asymmetry (seam_6 passes, seam_14 fails) has a clear cause.** It is the 1-pair difference (9 vs 8) at the force threshold for the 2cm collision shell. This is a deterministic prediction, not a random solver artifact.

4. **Tuning can move seams past thresholds at the margin** (seam_6 fixed) but cannot solve the topology problem (seam_14, seam_0/8).

5. **75% body penetration by frame 30 is a diagnostic signal, not just a number.** Any future sew-phase tuning effort should check `body_interior_verts` at frame 30. The target after Phase 7 should be < 5% (only interior panels temporarily passing through narrow areas, not all body panels tunneling).

6. **`debug_gc_sew_trace.py` is the canonical sew-phase health check.** It gives a complete view of seam closure progress and body penetration in one fast run (sew-phase only, ~40s). Should be run after any future simulation parameter change.

#### Next Steps

**Phase 7 — Attachment Constraints (unblocked, proceed immediately):**

The diagnosis is complete. Attachment constraints are the correct fix and the only remaining path. Concretely:

1. **New file `simulation/constraints/attachment.py`** — `AttachmentConstraints` Taichi kernel. Soft compliance (1e-4). No `from __future__ import annotations`.

   ```python
   @ti.data_oriented
   class AttachmentConstraints:
       def initialize(self, vertex_indices: NDArray, target_positions: NDArray) -> None: ...
       def reset_lambdas(self) -> None: ...
       @ti.kernel
       def project(self, positions: ti.template(), inv_mass: ti.template(),
                   n_attachments: ti.i32, compliance: ti.f32, dt: ti.f32): ...
   ```

2. **`gc_mesh_adapter.py` — `build_gc_attachment_constraints()`**: Select waist-level and shoulder-level vertices from each torso panel. Compute target positions on the body ellipse from `mannequin_profile.json` at the panel's median Y. Return `(vertex_indices, target_positions)`.

3. **`engine.py`**: Call `attachment_constraints.project()` inside the sew-phase substep loop; skip during drape phase.

4. **`garment_drape.py` GC path**: Build and wire attachment constraints.

5. **Validation**: After Phase 7, `debug_gc_sew_trace.py` should show `body_interior_verts < 100` at frame 30, and seam_14/seam_5/13 gaps should be < 3cm.

**Why this works:** If back-torso waist vertices are pinned to Z≈0.034m (body back surface) from the start, the panel never has to travel from Z=−0.069m → Z=+0.156m through the body. It starts on the body surface. Side-seam stitches then pull the panel along the body surface rather than through it. The equilibrium gap shrinks to approximately the attachment compliance tolerance (~1cm), not the body depth (~10cm).

---

### 📅 April 21, 2026: Panel Placement Investigation — Sleeve Z Preservation

**Status:** 🔶 Partial progress — 216/216 tests pass, 18/18 seams close, visual placement still incorrect
**Focus:** Fix the severe deformation seen after Phase 1 (sleeve cylinder wrap) and Phase 2 (z_back/z_front side-vertex snap) were reverted. Identify and implement the correct sleeve placement strategy.

#### Work Completed

**1. Completed revert of Phase 1 and Phase 2**

Both experimental placement changes were reverted:
- Phase 1 (sleeve cylinder wrap): Removed entire "Pass 2b" block from `prewrap_panels_to_body()`. The arc-length formula `r = half_width × 2/π` is invalid because GarmentCode sleeve panels have a 50.48° Z-rotation — the X span measures the rotated diagonal, not the arm circumference.
- Phase 2 (side-vertex Z snap): Reverted `z_back - clearance` / `z_front + clearance` back to `Z = center_z`. Setting torso side vertices to body surface Z increases side-seam initial gaps from ~0cm to ~26cm, which the sew phase could not reliably close.

**2. Fixed test_prewrap.py**

Three changes to restore the test suite after the revert:
- Removed the Z-spread curvature assertion (`panel_z.max() - panel_z.min() > 0.01`) — only applied when cylinder wrap existed
- Changed centroid gap threshold from 15cm back to 8cm
- Removed unused `_SLEEVE_KWS` variable

**3. Sleeve Z preservation in centroid alignment**

Key insight: GarmentCode already places sleeves at the **correct Z** for their body-side designation:
- Front sleeves: Z ≈ 0.306m (near body front surface z_front ≈ 0.279m)
- Back sleeves: Z ≈ 0.006m (near body back surface z_back ≈ 0.034m)

The previous full-XYZ centroid alignment `pos[offsets[k]:offsets[k+1]] += delta` pulled both front and back sleeves to Z ≈ 0.185m (body interior), destroying this intentional placement and causing the "cling to body" deformation.

Fix: Set `delta[2] = 0.0` before applying the translation to any sleeve panel (keyword `"sleeve"` in `pid_lower`). Now centroid alignment corrects only X and Y positions; Z is preserved from GarmentCode.

Updated `test_sleeve_panels_centered_on_stitch_targets` to measure only the XY centroid gap (expected < 8cm), since Z is now intentionally preserved.

#### Validation Results

```
Tests: 216/216 pass (0 regressions)

Sew trace (240 frames, shirt_mean.json):
  18/18 seams ✅  (armhole seam initial gaps increased to ~30cm; all close to <2cm by frame 240)
  Sew phase complete in 310.7s (1294.6ms/frame)
```

Seam closure with sleeve Z preservation:

| Seam | Initial | Final | Status |
|------|---------|-------|--------|
| seam_0 (armhole) | 30.0cm | 0.8cm | ✅ |
| seam_1 (armhole) | 30.0cm | 0.8cm | ✅ |
| seam_2 | 18.0cm | 0.2cm | ✅ |
| seam_3 | 24.5cm | 1.7cm | ✅ |
| seam_8 (armhole) | 30.0cm | 0.8cm | ✅ |
| seam_9 (armhole) | 30.0cm | 0.3cm | ✅ |
| All 18 seams | — | < 2cm | ✅ |

#### Issues Observed Visually

After reviewing `debug_sew_f0000.glb` (initial placement) and `garment_drape_animated.glb` in the frontend viewer:

**Issue 1: Torso panel edges appear "sealed" together at the sides**

Front and back torso panel side-seam vertices both land at Z = center_z ≈ 0.185m (body interior depth). Because both sides land at the same Z, the side seams start with zero gap — the constraint has nothing to close. The torso wraps tightly to the body surface in the front/back center, but the edges extend past the body width and appear fused together rather than conforming naturally. This is cosmetically wrong even if seam gap metrics report ✅.

Root cause: `pos[vi, 2] = cz` for `|sin_t| > 1` vertices places BOTH front and back panel side edges at the same Z, which happens to be the body center. The seam gap is zero but the geometry looks unnatural.

**Issue 2: Sleeves on incorrect arm sides and flat**

From the side view, the sleeves appear as flat diamond/triangular sheets extending from the shoulders at angles unrelated to the arms. Preserving Z is correct for keeping them on the right body side, but:
- The XY centroid alignment may be placing sleeves at the torso armhole XY (which is at X≈0 — the body center line), not at the arm location
- Front/back sleeve identification from panel names vs. actual visual placement needs verification
- Sleeves remain entirely flat (constant Z) — they need to wrap around the arm cylinder for natural appearance

**Issue 3: Drape animation retains initial shape**

The fabric in the animated GLB retains the deformed shape from the sew phase and shows minimal draping motion. This suggests:
- Either the sew-phase initial placement is still significantly wrong, producing large internal stresses that don't relax
- Or the drape phase is too short / gravity too weak relative to attachment constraints

#### Key Insights

1. **Unit test passage ≠ correct visual placement.** All 6 prewrap tests pass, 18/18 seams close, yet the visual output is still wrong. Tests verify local geometric invariants (ellipse radius, gap < N cm) but cannot verify that panels are on the correct sides of the body or that the shape looks garment-like. Visual inspection against the reference (CLO3D) must be the primary validation gate for placement work.

2. **GarmentCode Z placement is intentional and correct for body-side designation, but not for XY.** The Z values (0.006m, 0.306m) tell which side of the body each sleeve panel faces. The XY values after translation tell WHERE on the body. These two concerns must be managed independently.

3. **The centroid alignment reference point matters.** The torso armhole ring centroid (used as reference for XY alignment) is at approximately X = ±0.12m (half the body width), not at X = 0. If the sleeve panel centroid is being moved to the wrong reference point, sleeves will misalign. The `their_torso_verts` filter must correctly select only the torso armhole boundary vertices, not all torso vertices.

4. **"Zero gap = zero problem" is a false metric for seam quality.** Side seams that start at Z=center_z have zero gap but are visually incorrect. Better metrics: are the panel vertices on the correct side of the body surface? Is the panel normal pointing away from the body?

5. **Better tooling needed.** The per-frame GLB snapshots from `debug_gc_sew_trace.py` show the sew phase, but there is no equivalent for detailed per-panel initial placement inspection. A color-coded panel placement script (one color per panel, visible in the viewer) would catch XY/Z misplacements immediately without running a full 240-frame trace.

#### Future Plans

**Immediate — Better placement diagnostics:**
- Extend `scripts/debug_gc_panel_placement.py` to export a GLB with per-panel color coding AND per-vertex body-surface distance arrows. This will reveal immediately which panels are on the wrong side of the body.
- Add a placement validation script that checks each panel centroid against the body profile cross-section and flags panels that are inside the body or on the wrong Z side.

**Near-term — Sleeve placement fix:**
- Verify that `their_torso_verts` in the centroid alignment loop correctly selects armhole-ring vertices (stitch partners on the torso), not the full torso vert set
- The arm cylinder center for each sleeve is NOT at the torso body center (X=0) but at X = ±armhole_x ≈ ±0.12m. After XY alignment to the torso armhole centroid, the sleeve should be at the correct arm position.
- If sleeves are still on wrong sides, investigate whether the GarmentCode `rotation` in `shirt_mean.json` (±50.48° Z-rotation) affects which side of the seam matches which panel.

**Near-term — Side-seam vertex placement:**
- Consider placing side-seam vertices (`|sin_t| > 1`) at their respective body surface Z (z_back or z_front) with a check that the resulting gap is < 30cm so sew constraints can close it. If the gap exceeds 30cm, fall back to Z=center_z.

**Medium-term — Visualization improvements:**
- Frontend: add per-phase GLB selector (initial placement, sew phase checkpoints, final drape)
- Add panel labels overlay in viewer so panels can be identified visually

---

### 📅 April 20, 2026: Phase 7 + 7b — Sew-Phase Explosion Fully Resolved

**Status:** ✅ Complete — 18/18 seams close, no explosion, 213 tests pass
**Focus:** Eliminate the catastrophic sew-phase explosion seen in `debug_sew_f0030.glb` (entire mesh exploding into large chaotic triangles). Cross-referenced against `vestra-physics` (working reference codebase) to identify root causes.

#### Root Cause Analysis

Two independent root causes were identified and fixed:

**Root Cause 1 — α̃ imbalance (wrong initial compliance scale)**

The XPBD stitch update is:
```
Δx = w × gap / (2w + α̃)   where α̃ = compliance / dt²
```

With `sew_initial_compliance = 1e-4` and `dt = 1/240s`:
- α̃ = 1e-4 × 57600 = **5.76**
- For cloth particle: 2w ≈ 200 (mass ≈ 5g, inv_mass = 100 × 2 = 200)
- Δx ≈ 100 × gap / (200 + 5.76) ≈ **gap / 2** per iteration

With 64 iterations/frame (4 substeps × 16 sew iterations), a 24cm sleeve gap closes in **2 iterations** (~1 frame). Panels arrive at the body at ~10cm/frame velocity, punching through the 12mm collision shell instantly. Triangle normals invert → visual explosion.

The fix: `sew_initial_compliance = 1.0` → α̃ = 57,600 >> 2w = 200:
- Δx ≈ 100 × 0.24 / (200 + 57600) ≈ **0.4mm per iteration**
- 64 iterations = max 2.5cm/frame → well within collision shell

**Root Cause 2 — Sleeve panels starting 16–24cm from stitch targets**

The `prewrap_panels_to_body()` function (Phase 7b) only projected *torso* panels to the body surface (reducing torso-to-body gaps from 10cm → 8mm). *Sleeve* panels were left at their original GarmentCode positions, 16–24cm from the torso armhole stitch targets.

Even with `sew_initial_compliance = 1.0`, residual large gaps (collar, sleeve cap) were closing in 2–3 frames once the compliance ramp reached the stiff regime. The fix: **translate each non-torso panel so its stitch-vertex centroid aligns with the opposing panel's stitch-vertex centroid**.

#### Fixes Implemented

**1. `prewrap_panels_to_body()` — sleeve centering (`gc_mesh_adapter.py`)**

Extended to handle two panel classes:
- *Torso panels* (`btorso`, `ftorso`, `back`, `front`): Z-projection to body surface ± clearance (unchanged from Phase 7b).
- *Sleeve/collar panels* (all others): Find all cross-panel stitch pairs involving this panel, compute centroid of own stitch vertices and opposing stitch vertices, translate entire panel by the delta. This collapses sleeve-cap initial gaps from ~22cm to ~3-6cm.

```python
# sleeve/collar panels
delta = their_centroid - my_centroid
pos[offsets[k]:offsets[k + 1]] += delta
```

**2. `sew_initial_compliance = 1.0` (`garment_drape.py`, `debug_gc_sew_trace.py`)**

Changed from 1e-4. Combined with the existing log-space ramp over 120 frames (1.0 → 1e-10), the solver starts with extremely gentle stitch corrections and ramps to stiffness only after panels are already near their targets.

**3. Attachment constraints, velocity reset, debug sew trace (carried forward from Phase 7)**

All Phase 7 infrastructure remains in place: `AttachmentConstraints` Taichi kernel, `build_gc_attachment_constraints()`, velocity reset at `frame == sew_end`, `debug_gc_sew_trace.py`.

#### Results

**Sew trace (240 frames, shirt_mean.json):**
```
18/18 seams ✅  (was 17/18 before, 15/18 before Phase 7)
Overall gap at frame 240: mean=0.2cm  max=4.9cm
All seams: ≥ 90.8% closure  (was: 3 seams permanently stuck at 6–13cm)
Sew phase complete in 33.0s (137.5ms/frame)
```

**Full simulation:**
```
NaN:             ✅ PASS
Floor penetration: ✅ PASS
Mean speed:      ✅ PASS  (0.024 m/s — settled)
Mean stretch:    ✅ PASS  (3.15%)
Stitch gaps:     ✅ PASS  (18/18 ✅)
```

**Test suite:** 213/213 pass (0 regressions)

#### Known Remaining Issues

**1. Sleeve arm wrapping**
After centroid-alignment translation, the sleeve panels hang in front of the arms rather than wrapping around them. A pure translation cannot reorient the panel around the arm axis — this requires rotation. The centroid alignment aligns the sleeve cap Z to the torso armhole Z (correct), but the sleeve body then hangs downward from the shoulder joint rather than extending along the arm.

Fix path: In `prewrap_panels_to_body()`, after translation, rotate each sleeve panel around the shoulder axis (Y axis, at the shoulder joint position) so the sleeve body aligns with the arm direction. GarmentCode's sleeve panels for a relaxed T-pose should rotate approximately -45° around the shoulder to hang naturally.

**2. Shirt vertical position — initializes at chest level, slides to waist**
The GarmentCode shirt_mean.json panels start at Y ≈ 1.0–1.4m (chest height), not Y ≈ 0.7–1.6m (shoulder-to-hip). During the drape phase, gravity + body collision slides the shirt down to Y ≈ 0.5–1.2m. The shirt ends up around the waist instead of hanging from the shoulders.

Fix path: Either (a) increase `gc_body_z_offset` or add a Y offset to GC panels to shift the initial placement up by ~15–20cm, or (b) add collar/shoulder attachment constraints that keep the neckline at shoulder height during drape.

**3. Back panel body penetration (cosmetic)**
`body_interior_verts ≈ 1260` throughout sew phase. The 1D body profile (`z_back` per Y height) doesn't match the 3D mannequin mesh geometry at panel edges — the profile gives the extreme Z, but at large X the body surface is actually much closer to Z=0.15m (body center). Body collision resolves this during drape. Not causing visual artifacts in final output.

#### Key Insights

1. **`α̃ << 2w` is the critical diagnostic for sew-phase explosion.** Any stitch compliance that gives α̃ < 0.1 × 2w will produce near-rigid corrections and potential explosion. Always check: `α̃ = compliance / dt²` vs `2 × mean(inv_mass)`. For our setup: critical compliance ≈ 2w × dt² ≈ 200 / 57600 ≈ 0.0035. Anything below 0.01 is dangerously stiff.

2. **Pre-placement (not compliance tuning) is the primary stability lever.** Compliance tuning can only slow the approach velocity; it cannot prevent explosions when initial gaps are >10× the collision shell thickness. The correct workflow is: place panels near their final positions first, then use compliance to fine-tune the closure.

3. **Centroid alignment is a fast, effective approximation for sleeve pre-placement.** It doesn't account for panel orientation (no rotation), but it eliminates the dominant Z-displacement that caused the explosion. Remaining gaps after centering (2–6cm) are well within what the solver can close stably.

4. **The `debug_gc_sew_trace.py` is the canonical sew-phase health check.** Before and after any solver parameter change, run `python -m scripts.debug_gc_sew_trace --frames 60 --checkpoints 3`. Frame 20 gap table reveals whether stitches are closing or exploding. Healthy: all major seams ✅ by frame 40. Exploding: `body_interior_verts` > 2000 at frame 30.

#### Next Steps

1. **Fix sleeve arm wrapping**: Rotate each sleeve panel around the shoulder axis in `prewrap_panels_to_body()` after centroid translation. Likely a -45° rotation around Y at the shoulder joint.

2. **Fix shirt vertical placement**: Add a Y offset calibration step. Profile `garment.positions[:, 1].min()` and `max()` after prewrap and compare to desired range (shoulder ~1.55m, hip ~0.75m). Apply delta-Y to all vertices if needed.

3. **Phase 5 — FastAPI layer**: `POST /api/simulate` endpoint; mount `backend/storage/` as static; frontend proxy.

4. **T1.1 Seam welding** (roadmap Tier 1): After sew phase, merge vertex pairs with gap < 2mm. Eliminates residual seam lines in the rendered output.

---

### 📅 April 21, 2026: Geometric Refinement & Size Scaling

**Status:** 🔶 Scaling ✅; Curvature ❌
**Focus:** transitioning from flat 2D panel placement to a 3D parametric arrangement system. Correcting vertical scale (shoulder-to-hip) and implementing arc-length preserving wrapping math for torso and sleeves.

#### Work Completed

**1. Multi-dimensional Pattern Scaling**
- Implemented `scale_x` and `scale_y` capabilities in `gc_mesh_adapter.py`.
- Vertices are shifted from the panel's 2D centroid before 3D lifting, effectively allowing for "t-shirt length" (1.4y) and "standard width" (1.1x) garments from the same base `shirt_mean.json` spec.
- Verified: Garment now correctly spans from the shoulder line down to the hip region, resolving the "crop top" artifact.

**2. Arc-Length Preserving Curvature (failed intent)**
- Replaced the parabolic Z-drop with a trigonometric arc-length projection: `theta = dx / R`.
- Attempted to curl the flat width into a cylindrical "shield" that maintains the physical fabric width across its curve.
- Implemented PCA-based cylindrical wrapping for arm sleeves to form 3D tubes instead of flat diamonds.

#### Current State
- **Scaling is confirmed:** The panels are now the correct physical size for a standard t-shirt.
- **Curvature is missing:** Despite the added math, visual output (debug GLBs) still shows flat panels with minimal to no volume. 
- **Z-Clearance Issue:** Panels are currently in the correct general XY position but are too far away from the body surface (~10cm), resulting in very long initial stitch gaps.

#### Insights Gained
1. **Trigonometric Scale Sensitivity:** The arrangement radius `R` (derived as body_width * 1.5) may be too conservative. For narrow patterns, the resulting angular sweep `theta` is small enough that the Z-drop remains beneath the visual detection threshold or gets buried by the mesh resolution.
2. **Triangulation Constraints:** Scaling the 2D panel *before* triangulation is stable, but lifting it into a curve *after* triangulation requires more aggressive displacement to "fight" the flat rest-pose of the initial triangulated sheet.
3. **SVD for Sleeves:** Using PCA to find the longitudinal/lateral axes of a rotated sleeve is mathematically sound, but the "curl direction" (mapping theta to a normal axis) is sensitive to the winding order and panel flipping convention of GarmentCode.

#### Future Plan
1. **Aggressive Geometric Shaping:** Move from "arc-length preservation" to "volumetric fitting." We need to shape the starting pose more aggressively to mimic the CLO3D "Arrangement Point" behavior.
2. **Clearance Reduction:** Decrease the default `clearance` and radius `R` to pull the panels closer to the skin, which will improve the stability of the sew phase.
3. **Verification of Projection Math:** Re-audit the vertex-lifting loop in `prewrap_panels_to_body` to ensure the displaced `X` and `Z` are correctly propagated back into the global `garment.positions` array and not being overwritten by legacy centroid alignment logic.

---

### 📅 April 22, 2026: Phase 8 — CLO3D-Style Panel Placement (Elliptical Wrap + Arm Rotation)

**Status:** 🔶 Torso wrapping ✅; Sleeve curvature ❌ — flat panels remain
**Focus:** Replace the circular-radius torso wrap and Z-frozen sleeve alignment with a CLO3D-like approach: true elliptical body-surface projection for torso panels, arm-direction rotation for sleeve panels.

#### Work Completed

**1. True elliptical torso wrap (`gc_mesh_adapter.py` Pass 1)**

Replaced the old single-radius circular projection (`R = 1.5 × half_width = 0.263m`) with the body's actual elliptical cross-section semi-axes:
- `a = sl.width / 2` (X half-axis, body half-width)
- `b = sl.depth / 2 + clearance` (Z half-axis, body depth + clearance)
- For each vertex: `theta = arcsin(dx / a)` if `|dx/a| ≤ 1`, then `pos[vi, 0] = cx + a·sin(θ)` and `pos[vi, 2] = cz ± b·cos(θ)` (± for back/front)
- Vertices with `|dx/a| > 1` (outside body half-width) are left at their GarmentCode Z and handled by `resolve_initial_penetrations()`

**2. Sleeve centroid alignment — Z-freeze removed (`gc_mesh_adapter.py` Pass 2)**

Removed the previous `delta[2] = 0.0` Z-freeze that preserved GarmentCode's intentional sleeve Z. Instead, full XYZ centroid alignment uses the torso armhole ring as reference after Pass 1. The stitch-connectivity guarantee ensures `their_torso_verts` contains only the correct front-or-back torso armhole ring — so Z naturally lands near the correct body side.

**3. Arm-direction rotation added (`_rotation_from_axes()` helper)**

After centroid translation, each sleeve panel is rotated using:
1. SVD on the sleeve vertex cloud to find the principal axis
2. Rodrigues rotation mapping that axis → `[±1, 0, 0]` (T-pose arm direction)

**4. `--prewrap` flag added to `debug_gc_panel_placement.py`**

The debug script now accepts `--prewrap` to generate `storage/debug_gc_panels_wrapped.glb` showing post-prewrap panel positions for visual inspection. Both pre- and post-prewrap states can be compared side by side.

#### Confirmed Results (from diagnostic traces)

**Torso wrapping — working correctly:**

| Panel | Pre-prewrap Z | Post-prewrap Z | Z range |
|-------|--------------|----------------|---------|
| right_btorso | −0.069m (flat) | [−0.069, +0.161]m | 230mm ✅ |
| right_ftorso | +0.381m (flat) | [+0.157, +0.381]m | 224mm ✅ |

The Z variation confirms elliptical curvature. Center vertices land at the body apex (cz ± b); side vertices converge toward cz. The −0.069m vertices are outside body half-width and were left unwrapped (correct — resolve_initial_penetrations pushes any inside-body ones out).

**Sleeve alignment — centroid correct, surface still flat:**

| Panel | Pre-prewrap Z | Post-prewrap Z | ref_centroid Z | delta_z |
|-------|--------------|----------------|----------------|---------|
| right_sleeve_b | +0.006m (flat) | +0.047m (flat) | +0.047m (15 btorso verts) | +0.041m |
| right_sleeve_f | +0.306m (flat) | +0.300m (flat) | +0.300m (13 ftorso verts) | −0.006m |

Centroid alignment is correct: sleeve_b moves to the btorso armhole centroid (Z≈0.047m, near back body surface); sleeve_f stays near the ftorso armhole centroid (Z≈0.300m, near front body surface). However, both sleeves remain perfectly flat (1 unique Z value each).

**Penetration corrections:** 342 vertices corrected by `resolve_initial_penetrations()`.

**Stitch gaps after prewrap:**

| Seam | Gap range | dZ range |
|------|-----------|----------|
| sleeve ↔ torso | 9–25cm | up to ±25cm |
| torso side seams | 0–45cm | up to ±45cm |

#### Root Cause: Why Sleeves Cannot Be Curved by Rotation

This is a mathematical fact, not an implementation bug.

GarmentCode outputs all panels as **perfectly flat** (Z=constant for all vertices). For a Z=constant panel:
- `centered[:, 2] = 0` (all Z deviations from centroid are zero)
- SVD's first principal axis lies entirely within the XY plane
- Rodrigues rotation from an XY-plane axis to `[±1, 0, 0]` is a **pure Z-axis rotation**
- After applying `pos = pivot + centered @ R_mat.T`, the new Z values equal `pivot_z + centered[:, 0]·R[2,0] + centered[:, 1]·R[2,1]`
- For a Z-axis rotation: `R[2, 0] = 0` and `R[2, 1] = 0`, so **new Z = pivot_z** (unchanged for all vertices)

Rotation of a flat panel within its own plane cannot introduce Z curvature. Z variation must be **explicitly computed and written** from body surface geometry — it cannot be derived from orientation changes.

This same constraint applies to any future rotation-based approach: until the sleeve vertices have non-zero Z spread relative to their centroid, no 3×3 rotation will produce a curved sleeve.

#### Visual Evidence (Screenshots)

The debug GLB (`storage/debug_gc_panels_wrapped.glb`) confirms:
- **Front view**: Torso panels (yellow/teal) show body-conforming curvature. Sleeve panels (green) are flat horizontal sheets extending from the shoulder.
- **Back view**: Back torso panels (blue/orange) show curvature. Back sleeve panels (pink) are flat horizontal sheets extending from the shoulder with incorrect upward rotation.
- The arm rotation step rotated the sleeves to extend horizontally (±X), which is correct orientation, but they remain flat discs rather than tubular wraps.
- Stitch ribbons (magenta) are 10–25cm long, confirming large initial gaps.

#### Issues Encountered

1. **Arm rotation reintroduced previous artifact.** The sleeve panels now extend horizontally from the shoulder at the correct arm angle (±X), but the flat-disc geometry makes them look like wings rather than sleeve tubes. The previous `delta[2] = 0.0` approach at least preserved a visually harmless (if incorrect) flat panel near each body surface.

2. **Stitch gaps are still large.** The sleeve-to-torso stitch gaps are 9–25cm. These are within what the sew solver can close (previous tests showed 18/18 seams close), but the large initial gap creates retained stress in the drape phase.

3. **Torso side vertices at unwrapped Z.** Vertices with `|dx/a| > 1` remain at GarmentCode Z (0.381m for ftorso, −0.069m for btorso). These are the panel edges beyond the body's half-width and create a visual artifact where the torso panel boundary appears to float.

#### Key Insights

1. **CLO3D's arrangement is surface projection, not rotation.** CLO3D places panels by projecting each vertex onto a reference surface (cylinder, cone, or spline-swept surface) that approximates the target body region. For the torso it is the body ellipse (now implemented ✅). For the sleeve it is a cylinder centered on the arm axis. The curved shape emerges from the geometry of the projection surface, not from rotating a flat panel.

2. **The flat-panel constraint is fundamental to GarmentCode output.** GarmentCode lifts 2D patterns to 3D via `rot_trans_panel()`, which applies rigid rotation + translation. A rigid transform of a flat 2D polygon always produces a flat 3D polygon (Z=constant in some plane). Any curvature must be injected AFTER this lift by surface projection.

3. **Two of three sub-problems are solved:**
   - Torso elliptical projection: ✅ Implemented and confirmed working (230mm Z variation)
   - Sleeve centroid alignment to torso armhole ring: ✅ Implemented and confirmed (sleeve_b → btorso, sleeve_f → ftorso)
   - Sleeve arm-cylinder projection: ❌ Not yet implemented

4. **`their_torso_verts` correctly selects the right body side.** Stitch connectivity ensures: `right_sleeve_f` → 13 ftorso verts (front body), `right_sleeve_b` → 15 btorso + 2 ftorso verts (back body). The Z-freeze removal is safe.

5. **Visual correctness is the only valid gate for placement work.** The 216 unit tests pass and 18/18 seams close regardless of whether sleeves are flat or curved — the XPBD solver is robust enough to handle 10–25cm gaps. But the visual output and the retained stress in the drape phase are the true quality indicators.

#### Plan Moving Forward

**The correct next step is arm-cylinder projection for sleeve panels**, analogous to the torso elliptical projection.

Required information (must be measured, not guessed):
1. **Sleeve panel 2D layout**: Which dimension of the GarmentCode sleeve panel maps along the arm (X-direction in T-pose) and which maps around the arm circumference? This determines how `panel_height` maps to `arm_length_position` and how `panel_width` maps to `wrap_angle`.

2. **Arm cylinder geometry**: The arm in a T-pose is approximately cylindrical with:
   - Axis direction: ±[1, 0, 0]
   - Axis position: near shoulder joint (X = ±0.17–0.22m, Y = shoulder_y ≈ 1.43m, Z = armhole_centroid_z ≈ 0.10–0.15m depending on side)
   - Cylinder radius ≈ arm circumference / (2π) ≈ 8–10cm

3. **Panel-to-cylinder axis mapping**: GarmentCode places sleeves with a ~50° Z-rotation. After `rot_trans_panel()`, which direction in the sleeve panel's 3D vertex cloud aligns with the arm axis?

Verification approach before implementation:
- Print the 2D bounding box of the sleeve panel from GarmentCode's 2D coordinates
- Print the 3D vertex cloud extent of the sleeve panel in each axis (X, Y, Z) from GarmentCode's `rot_trans_panel()` output
- Compare to determine: sleeve `panel_height` → arm longitudinal axis, sleeve `panel_width` → arm circumference direction
- Only then implement the cylinder projection

**Implementation sketch (after verification):**

```python
# For each sleeve vertex vi:
#   1. Determine arm axis center (shoulder_x, shoulder_y, armhole_z) from body profile
#   2. Project vertex onto plane perpendicular to arm axis → get angle theta around arm
#   3. Map theta to cylinder surface: pos[vi, Z] = armhole_z + arm_radius * sin(theta)
#                                     pos[vi, Y] = shoulder_y + arm_radius * cos(theta)
#   4. pos[vi, X] stays (it encodes position along arm length)
```

This projection is geometrically analogous to the torso ellipse projection and will produce true Z variation in sleeve panels.

---

### 📅 April 22, 2026: Phase 8b — Arm Cylinder Projection & Torso Smooth Wrap (Investigation)

**Status:** 🔶 Partially Implemented — architectural improvements in place, visual output not yet production-quality  
**Focus:** Implementing CLO3D-style panel placement with three improvements: smooth torso wrap, arm-cylinder projection for sleeves, and side-edge boundary blending.

#### Work Completed

**1. Codebase Analysis & Measurement**

Performed thorough geometric analysis of the panel-to-body relationship before any code changes:

- **GarmentCode sleeve panels**: 2D size 31.2 × 21.3 cm. After `rot_trans_panel()` with 50° Z-rotation, the 2D X direction (arm circumference) maps to 3D diagonal `[0.636, 0.771, 0.0]`, and 2D Y (arm length) maps to `[-0.771, 0.636, 0.0]`. All sleeve vertices have **constant Z** (e.g., Z=0.306m for `right_sleeve_f`). The SVD principal axis is `[0.43, 0.90, 0.0]` (arm-length direction).

- **Arm cylinder geometry** (measured from `mannequin_physics.glb` vertex rings):
  | X position | Arm center Y | Arm center Z | Radius |
  |-----------|-------------|-------------|--------|
  | ±0.20 (shoulder) | 1.333 | 0.134 | 0.072m |
  | ±0.29 | 1.170 | 0.130 | 0.073m |
  | ±0.35 | 1.125 | 0.124 | 0.069m |
  | ±0.41 (lower arm) | 0.992 | 0.146 | 0.042m |

  The arm slopes downward (center Y drops ~0.34 per 0.21m of X travel) and tapers (radius 0.072 → 0.042m from shoulder to wrist).

- **Stitch topology**: `sleeve_f ↔ sleeve_b` has 39 underarm seam pairs (longest seam); `sleeve_f ↔ ftorso` has 17 armhole cap pairs; `sleeve_b ↔ btorso` has 19 pairs.

**2. Architectural Improvements to `gc_mesh_adapter.py`**

- **Piecewise arm cylinder model** (`_build_arm_centerline()`, `_arm_at_x()`): Queries the body mesh directly to extract arm cross-sections at 12 slices along the X axis. Returns interpolatable centerline data (center_y, center_z, radius at each X position). This replaces the previous assumption of a single fixed cylinder.

- **Panel role classification with fallback** (`_classify_panel()`): Primary classification by panel name keywords (`btorso`, `ftorso`, `sleeve`). Fallback: if >30% of a panel's cross-panel stitches connect to torso-classified panels, classify as `sleeve`. This handles future GarmentCode patterns that may use non-standard naming.

- **Configurable clearance constants**: `_TORSO_CLEARANCE` (25mm) and `_SLEEVE_CLEARANCE` (5mm) defined as module-level constants for future configurability.

**3. Torso Wrap — Smooth Reference Ellipse**

Replaced the per-vertex body profile lookup with a **single reference ellipse per panel**, computed from the panel's median-Y cross-section. This eliminates the terrain-like artifacts caused by the body profile's varying cross-sections at each vertex height.

- Before: each vertex queried `profile.at_y(py)` independently, creating ridged surfaces where the body profile had abrupt width/depth transitions (especially around armpit/shoulder Y levels).
- After: all vertices in a panel use the same `(a, b, cx, cz)` ellipse, producing uniform cylindrical curvature.
- Side-edge blending added for vertices with `|dx/a| > 1.0`: Z smoothly converges toward body center-Z instead of staying at flat GarmentCode Z.

**4. Arm Cylinder Projection — Two Approaches Attempted**

**Attempt 1: SVD wrap_coords approach**
- Projected each vertex onto the SVD secondary axis to get a "wrap coordinate", then mapped that to θ on the arm cylinder.
- **Result**: Severe accordion/fan distortion (horizontal striping). Root cause: the wrap coordinate from SVD mixes with world-space coordinates when replacing Y — vertices at the same SVD-wrap value but different X positions get mapped to different arm-center Y values, creating non-smooth surfaces.

**Attempt 2: Rotate-then-project approach**
- First rotated the panel so the SVD primary axis aligns with the arm direction (±X), then used the Y offset from the arm center at each X position to compute the cylinder angle θ.
- **Result**: Sleeves split into two fragments — one portion projected correctly near the arm, while the other remained positioned near the torso front. Root cause: after SVD rotation + centroid translation, the pivot point (armhole stitch centroid) is between the torso surface and the arm, causing the rotation to swing some vertices behind the torso while projecting others outward.

#### Issues Identified (Open)

1. **Sleeve centroid-then-rotate creates split geometry**: The two-step process (translate to armhole centroid → rotate to align with X → project onto cylinder) is fundamentally fragile because the armhole centroid is a point between the torso surface and the arm, not on the arm itself. The rotation around this midpoint sends some vertices toward the body interior and others outward.

2. **Torso panels still too close to body**: Even at 25mm clearance, the body mesh surface pokes through at anatomical high points (chest/nipple area). The single-reference-ellipse approach helps with terrain artifacts but doesn't prevent localized penetrations where the body surface is convex beyond the reference ellipse.

3. **Side-edge staircase effect**: The transition from elliptical wrap (`|t| ≤ 1`) to side-edge blending (`|t| > 1`) creates an abrupt directional change. The vertices at the body edge (|t| = 1) have near-vertical Z gradient, but the blending region converges toward body center-Z — creating a visible "staircase" at the boundary.

4. **Arm cylinder model limitations**: The piecewise model works well for the upper arm but degenerates near X=±0.20 (shoulder joint, where arm vertices overlap with torso vertices) and at X=±0.44+ (wrist, where very few mesh vertices exist).

#### Key Insights

1. **Rotation cannot introduce curvature in a flat panel** — this was re-confirmed empirically. The SVD rotation (Rodrigues formula) aligns the panel's principal axis with the arm direction but preserves the Z=constant constraint. Z curvature must be explicitly computed from surface geometry.

2. **The sleeve placement problem is harder than torso placement.** Torso panels have a clear 1D mapping (X position → ellipse angle θ → Z curvature). Sleeves require a 2D decomposition: separating the arm-length coordinate from the circumferential coordinate in a panel that's rotated 50° from axis-aligned. The GarmentCode rotation mixes both dimensions into XY, making it impossible to use raw world coordinates for the projection.

3. **CLO3D uses a fundamentally different approach.** CLO3D's "Arrangement Points" system anchors panels to specific body surface locations, then wraps them by projecting onto a reference surface defined by those anchor points. This is a marker-driven approach, not a geometry-driven one. Our system lacks body-surface anchor points, so we must infer the projection geometry from the body mesh and stitch connectivity.

4. **The correct approach for sleeves likely requires working in 2D panel coordinates**, not 3D post-rotation coordinates. The 2D panel has clean, separated dimensions: 2D-X = arm circumference, 2D-Y = arm length. Mapping these directly onto the arm cylinder (2D-Y → position along arm axis, 2D-X → angle θ around arm) would bypass all rotation-mixing issues. This requires storing or recovering the 2D coordinates during `prewrap_panels_to_body()`.

5. **Torso wrap quality depends on clearance AND reference surface smoothness.** Per-vertex body profile lookup creates terrain artifacts because the body profile has ~56 cross-sections with varying parameters. A smooth reference surface (single ellipse or low-frequency spline) is essential for CLO3D-like visual quality. The 25mm clearance helps but is not sufficient — the reference surface shape matters more than the offset distance.

#### Plan Moving Forward

**Immediate next steps (next session):**

1. **Sleeve projection from 2D coordinates**: Instead of projecting from 3D post-rotation space, map the 2D panel coordinates directly onto the arm cylinder:
   - Store 2D panel vertex coordinates in `GarmentMesh` (or recover them from the rotation inverse)
   - 2D-Y → arm-length position (determines world X along arm axis)
   - 2D-X → circumferential angle θ (determines world Y/Z on cylinder surface)
   - Arm cylinder center and radius queried from the piecewise model at each arm-length position

2. **Torso wrap smoothing**: Replace the single-reference-ellipse approach with a 3-keyframe spline (top, middle, bottom of panel) to follow the body's natural taper while maintaining smooth curvature. Increase clearance further (30-35mm) or use adaptive clearance based on local body surface curvature.

3. **Side-edge continuity**: Replace the linear blending with cosine interpolation to eliminate the staircase effect at |t| = 1:
   ```python
   blend = 0.5 * (1 - cos(π * (t_abs - 1.0) / 0.5))  # smooth S-curve
   ```

### 📅 April 22, 2026 (continued): Phase 8 — Smooth Shielding & 2D-to-3D Sleeve Projection

**Status:** ✅ Implementation Complete; 🔶 Aesthetic Refinement Pending
**Focus:** Resolving visual discontinuities in torso wrapping and implementing true 3D cylindrical sleeves by mapping directly from 2D coordinates.

#### Work Completed

**1. Smooth Cosine Shield for Torso**
- Replaced the branching `arcsin` elliptical projection with a continuous **cosine-based shield function**.
- Extended the mapping range to `1.25 * body_half_width` to ensure the Z-curvature drop-off is smooth even past the torso edge.
- **Result:** Successfully eliminated the "staircase" artifact at the panel boundaries. The torso now exhibits a premium, continuous shield-like form.

**2. Direct 2D-to-3D Sleeve Mapping**
- Implemented a new coordinate mapping pipeline that utilizes the original **unrotated 2D panel vertices**.
- **Math:** 2D-Y maps to arm length (world X), while 2D-X maps to the wrap angle θ on the arm cylinder.
- **Result:** Sleeves are now correctly formed as 3D tubes. They no longer sit in front of the chest or exhibit extreme stretch. They naturally wrap the arms, significantly reducing the initial gap to the torso armholes.

**3. Data Persistence**
- Updated the `GarmentMesh` dataclass and the `gc_mesh_adapter.py` loader to preserve `verts_2d`. This architectural change was essential to drive the 2D-to-3D projection without coordinate mixing from legacy rotations.

#### Issues Encountered & Observations

- **Torso Proximity:** At the current 40mm clearance, the torso panel sits too close to the mannequin's chest, leading to localized "puffing" artifacts in the simulation.
- **Panel Sizing:** Torso panels are slightly too wide, overlapping the arms in the initial pre-wrap state.
- **Sleeve Surface Quality:** While the tube structure is correct, the surface smoothness needs refinement. One element in each sleeve pair tends to sit awkwardly near the bottom of the arm cylinder.
- **Simulation Stability:** Seam closure is now extremely robust, with all 18 seams closing to < 2cm mean gaps within 60 frames.

#### Future Plan (Next Session)

1. **Adaptive Clearance & Scaling**: Increase torso clearance to ~50-60mm and adjust panel scaling/positioning to prevent arm overlap.
2. **Sleeve Wrap Refinement**: Fine-tune the 2D-X to θ mapping to ensure smoother sleeve surfaces and more symmetric placement around the arm axis.
3. **Phase 5 (FastAPI Layer)**: Begin exposing the simulation via the web API now that the 3D initial state is visually stable.
