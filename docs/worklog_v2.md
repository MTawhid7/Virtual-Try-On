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
