# garment-sim: Implementation Plan — Phases 4–8

## Status Summary

| Phase | Status | Goal |
|-------|--------|------|
| 1–3 | ✅ Complete | Vendor pygarment, replace CGAL, build GC adapter, wire `--gc` flag |
| 4 | 🔶 In Progress | Validate GC pipeline end-to-end on mannequin |
| 5 | ⬜ Pending | FastAPI layer + frontend simulation trigger |
| 6 | ⬜ Pending | Pattern library expansion (bodice, pants, skirts) |
| 7 | ⬜ Pending | Simulation quality: attachment constraints, body smoothing |
| 8 | ⬜ Pending | 2D interactive pattern editor |

---

## Context

**garment-sim** is a Taichi XPBD physics simulation engine that drapes garment panels around a 3D mannequin body using a sew-then-drape two-stage simulation. GarmentCode (SIGGRAPH Asia 2023, ETH Zurich) is a parametric sewing pattern design system that generates pattern JSONs from body measurements.

**End goal**: CLO3D-like workflow — select or create 2D garment panels, define stitches, pick fabric, simulate the garment sewing around the 3D body, view in browser, export GLB.

### Phases 1–3 recap

- `backend/pygarment/` — vendored subset of GarmentCode (garmentcode DSL, pattern serialization, BoxMesh mesh generation)
- `backend/simulation/mesh/gc_mesh_adapter.py` — converts `BoxMesh → GarmentMesh`; preserves non-manifold topology (panels stay separate; stitch_pairs become XPBD spring constraints)
- CGAL replaced with `triangle` library (cross-platform CDT)
- `--gc GC_SPEC` flag in `garment_drape.py` routes through `build_garment_mesh_gc()`
- 195 tests passing, 3 GC spec JSONs: `shirt_mean`, `hoody_mean`, `dress_pencil`

---

## Phase 4: Validate GC Pipeline (current)

**Goal**: Run `shirt_mean.json` end-to-end; fix body alignment; add pytest coverage.

### Body alignment fix (critical)

GarmentCode patterns are generated for the SMPL mean body. Our `mannequin_physics.glb` is a different mesh with a different Z range.

| | GarmentCode (after cm→m) | mannequin_physics.glb |
|--|--|--|
| Front torso Z | +0.25 m | front face = +0.279 m |
| Back torso Z | −0.20 m | back face = +0.034 m |
| **Centre Z** | **(0.25 − 0.20) / 2 = +0.025 m** | **(0.279 + 0.034) / 2 = +0.157 m** |
| **Required offset** | — | **+0.131 m** |

No Z-flip needed — both systems use +Z = front.

### Simulation validation results (shirt_mean.json, mesh_resolution=1.5cm)

| Check | Result | Detail |
|-------|--------|--------|
| NaN | ✅ PASS | Clean |
| Floor penetration | ✅ PASS | min Y = 0.898m |
| Torso coverage | ✅ PASS | 2870/2870 particles in Y=[0.5, 1.8]m |
| Mean speed | ✅ PASS | 0.011 m/s — fully settled |
| Mean stretch | ✅ PASS | 5.55% |
| Stitch gaps | ❌ FAIL | 3 sleeve cap seams at 10–11cm |

**Failing seams** are the sleeve-to-armhole connections (seam_5 n=3, seam_13 n=3, seam_14 n=8). Root cause: sleeve panels start 25–30cm from the armhole in 3D; with only 3–8 stitch pairs pulling them in over 240 sew frames, the gap doesn't fully close.

**Max stretch outlier (318%)**: Two back-torso vertices end up inside the body (Z=0.056–0.083m, inside body Z=[0.034, 0.279]m). The back panel starts at Z=−0.069m and is pulled forward by stitches; during the sew phase some vertices slip through the thin `sew_collision_thickness=0.006m` shell.

**Both issues require Phase 7 attachment constraints** to anchor panels near the body surface before sewing begins. This is the expected result for a first-pass simulation without pinning.

### Files changed

| File | Change |
|------|--------|
| `simulation/mesh/gc_mesh_adapter.py` | Added `body_z_offset: float = 0.0` to `boxmesh_to_garment_mesh()` and `build_garment_mesh_gc()`. Applied per-panel inside the loop so stitch midpoint densification uses corrected positions. |
| `simulation/scenes/garment_drape.py` | Added `_GC_BODY_Z_OFFSET = 0.131` constant (with derivation comment). Added `gc_body_z_offset` param to `run_garment_drape()`. Added per-panel Z/Y diagnostic prints after GC mesh is built. Added `--gc-z-offset METRES` CLI arg. |

### Files created

| File | Purpose |
|------|---------|
| `tests/integration/test_gc_pipeline.py` | 11 tests: 6 mesh-setup (fast, no sim) + 5 simulation tests. Covers: panel count, panel names, NaN-free positions, stitch index validity, Z-offset applied correctly, panels not inside body, no NaN after sim, torso settlement, no floor penetration, seam gap reduction, edge preservation, GLB export. |
| `scripts/verify_gc_alignment.py` | Interactive diagnostic: prints per-panel Z extents vs. body surface bounds from `mannequin_profile.json`, prints initial stitch gaps. Run with: `python -m scripts.verify_gc_alignment` |

### Verification commands

```bash
cd backend && source .venv/bin/activate

# 1. Alignment diagnostic (fast, no simulation)
python -m scripts.verify_gc_alignment

# 2. Full GC pipeline — static GLB
python -m simulation --scene garment_drape --gc data/patterns/garmentcode/shirt_mean.json

# 3. Animated GLB (morphtarget timeline of sew + drape)
python -m simulation --scene garment_drape --gc data/patterns/garmentcode/shirt_mean.json --animate

# 4. Run new GC-specific tests (adds 11 tests to the 195 existing)
python -m pytest tests/integration/test_gc_pipeline.py -v

# 5. Run the full test suite to confirm no regressions
python -m pytest tests/ -v
```

### Visual verification

After step 2 or 3, open the frontend viewer:
```bash
cd frontend && npm run dev
# Open http://localhost:3000
# The generated GLB is at backend/storage/garment_drape.glb
```

**What to look for:**
- Shirt panels wrap around the mannequin torso (not floating far from body, not inside it)
- Side seams closed (front and back panels meet at the sides)
- Fabric has natural folds/drape under gravity
- Sleeves positioned at shoulder height with correct angle

---

## Phase 5: FastAPI Integration Layer

**Goal**: `POST /api/simulate` → run simulation → return GLB URL; frontend connects live.

### Files to create

| File | Description |
|------|-------------|
| `app/__init__.py` | Package init |
| `app/main.py` | FastAPI app; mounts `/storage` as static; includes routers |
| `app/schemas.py` | `SimulateRequest` / `SimulateResponse` Pydantic models |
| `app/routes/simulate.py` | `POST /api/simulate` (synchronous; max 300 frames) |
| `app/routes/patterns.py` | `GET /api/patterns` — catalog from `data/patterns/` scan |
| `frontend/app/api/simulate/route.ts` | Next.js proxy → backend |
| `frontend/.env.local` | `BACKEND_URL=http://localhost:8000` |

### Key design decisions

- Synchronous execution for Phase 5 (no background job queue — deferred to later)
- GLBs written to `backend/storage/` and served as static files via FastAPI
- Frontend proxies to avoid CORS; `BACKEND_URL` env var for configurability
- `pattern_json: dict | None` in `SimulateRequest` enables inline pattern from editor (Phase 8)

### Uncommenting dependencies in `requirements.txt`

```
fastapi>=0.100
uvicorn[standard]>=0.24
pydantic>=2.0
```

### Verification

```bash
uvicorn app.main:app --port 8000 --reload

curl -X POST http://localhost:8000/api/simulate \
  -H "Content-Type: application/json" \
  -d '{"pattern_path":"garmentcode/shirt_mean.json","gc_mode":true}'
# → {"job_id":"...","status":"done","glb_url":"/storage/....glb"}
```

---

## Phase 6: Pattern Library Expansion

**Goal**: Generate bodice, pants, skirts, etc. from GarmentCode programs; add pattern selector in frontend.

### Files to create

| File | Description |
|------|-------------|
| `scripts/gc_body_params.py` | Maps `mannequin_profile.json` → GarmentCode body YAML dict |
| `scripts/generate_gc_specs.py` | Imports from `/Users/tawhid/Documents/GarmentCode/assets/garment_programs/`; generates 7+ spec JSONs to `data/patterns/garmentcode/` |
| `frontend/components/PatternSelector.tsx` | Card grid: pattern name + panel count badge + selection state |

### Target garment types

`tee_fitted`, `tee_loose`, `bodice_basic`, `pants_straight`, `skirt_aline`, `skirt_circle`, `dress_basic`

### Notes

- `generate_gc_specs.py` adds `/Users/tawhid/Documents/GarmentCode/` to `sys.path` — it does NOT modify garment-sim's vendored `pygarment/`
- Body params mapped from mannequin profile so generated patterns fit our specific mannequin proportions

---

## Phase 7: Simulation Quality

**Goal**: Attachment constraints to reduce panel drift during sew phase; self-collision for layered garments.

### Attachment constraints

**Problem**: During sew phase, no anchor points exist. Torso panels can drift in X/Y before stitches tighten, causing misalignment.

**Solution**: Soft positional pin constraints (compliance 1e-4) pulling selected waist/collar vertices toward target positions on the body ellipse. Released (not called) during drape phase.

| File | Change |
|------|--------|
| `simulation/constraints/attachment.py` | **NEW** — `AttachmentConstraints` Taichi kernel. No `from __future__ import annotations`. |
| `simulation/mesh/gc_mesh_adapter.py` | Add `build_gc_attachment_constraints(garment, profile_path)` |
| `simulation/core/engine.py` | Call `attachment_constraints.project()` inside sew phase loop |
| `simulation/scenes/garment_drape.py` | Build + attach for GC path |
| `tests/unit/constraints/test_attachment.py` | **NEW** — unit test |

### Self-collision flag

```bash
python -m simulation --scene garment_drape --gc hoody_mean.json --self-collision
```

Needed for: hoody (collar + body layers), dresses with skirt overlapping bodice.

---

## Phase 8: 2D Pattern Editor

**Goal**: Interactive frontend where users can create/edit panels and stitch definitions, trigger re-simulation, and see the 3D result live.

### Pages and components

| File | Description |
|------|-------------|
| `frontend/app/editor/page.tsx` | Split-pane: left = SVG 2D canvas, right = GarmentViewer |
| `frontend/components/editor/PanelCanvas.tsx` | SVG canvas — drag vertices, click edges to stitch |
| `frontend/lib/pattern_types.ts` | `Panel`, `Stitch`, `PatternState` TypeScript interfaces |
| `frontend/lib/panel_templates.ts` | Predefined shapes: rectangle, trapezoid skirt, tapered sleeve |

### Backend change

`POST /api/simulate` already accepts `pattern_json: dict | None` (added in Phase 5). If provided, written to a temp file and routed through `build_garment_mesh()` (native path).

### UX flow

1. Pick template panel shape
2. Drag vertices to customise
3. Select two edges → click Stitch
4. Click Simulate → POST pattern JSON → GLB loads in 3D viewer (~30s preview)
5. Export final GLB

---

## Technical Constraints (must not violate)

1. **No `from __future__ import annotations`** in files with `@ti.kernel` or `.template()` params: `resolver.py`, `spatial_hash.py`, `point_triangle.py`, `stitch.py`, `xpbd.py`, and any new kernel files.
2. **Do not replace Taichi XPBD with NVIDIA Warp** — the Taichi solver is the core differentiator.
3. **Preserve non-manifold topology** — panels stay separate; stitch pairs are XPBD spring constraints. Do not merge vertices across panels.
4. **Python 3.10–3.13** — Taichi 1.7.4 does not support 3.14.
5. **body_z_offset defaults to 0.0** in `gc_mesh_adapter.py` — existing unit tests use synthetic panels at known positions and must not be affected.

---

## Key Constants

```python
# garment_drape.py
_GC_BODY_Z_OFFSET = 0.131  # metres

# Derivation:
#   mannequin chest centre Z = (0.2786 + 0.0335) / 2 = 0.1561 m
#   GarmentCode SMPL centre Z = (0.25 + (-0.20)) / 2 = 0.025 m  [after cm→m]
#   offset = 0.1561 - 0.025 = 0.131 m
```

```python
# Simulation phase boundaries (garment_drape.py SimConfig)
total_frames = 570    # 240 sew + 30 transition + 300 drape
sew_frames = 240
transition_frames = 30
sew_gravity_fraction = 0.15   # 15% gravity during sew
sew_stitch_compliance = 1e-10  # very stiff — closes gaps fast
drape_stitch_compliance = 1e-8  # relaxed — natural drape
```
