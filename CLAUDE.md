# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
Your output will be evaluated by Codex and Gemini. Prioritize correctness, completeness, and unambiguous reasoning. Avoid assumptions and ensure all conclusions are justified.

## Commands

All commands run from `backend/` with the virtualenv activated:

```bash
cd backend
source .venv/bin/activate
```

**Setup:**
```bash
python3.13 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Run simulations:**
```bash
python -m simulation --scene freefall
python -m simulation --scene constrained_fall
python -m simulation --scene sphere_drape
python -m simulation --scene body_drape           # body mesh collision (Sprint 2)
python -m simulation --scene garment_drape        # pattern-based garment on body (Sprint 2 Layer 3b)
python -m simulation --scene sphere_drape -v          # live Taichi GGUI visualizer
python -m simulation --scene sphere_drape -o out.glb  # custom export path
# Default export: storage/{scene}.glb

# GarmentCode parametric patterns (Phase 4+)
python -m simulation --scene garment_drape --gc data/patterns/garmentcode/shirt_mean.json
python -m simulation --scene garment_drape --gc data/patterns/garmentcode/shirt_mean.json --animate
python -m simulation --scene garment_drape --gc data/patterns/garmentcode/hoody_mean.json
python -m simulation --scene garment_drape --gc data/patterns/garmentcode/dress_pencil.json
# --gc-z-offset METRES  override body Z offset (default: 0.131 for mannequin_physics.glb)
```

**GarmentCode diagnostics:**
```bash
python -m scripts.verify_gc_alignment                              # shirt_mean.json, default offset
python -m scripts.verify_gc_alignment --pattern data/patterns/garmentcode/hoody_mean.json
python -m scripts.verify_gc_alignment --z-offset 0.0              # check without offset
```

**Phase verification scripts (run from backend/):**
```bash
python -m scripts.visualize_phase1_triangulation   # panel triangulation GLBs
python -m scripts.visualize_phase2_stitch          # stitch constraint closure GLBs
python -m scripts.visualize_phase3_panel_builder   # two-panel garment mesh GLBs
python -m scripts.visualize_phase4_garment_drape   # full garment-on-body GLBs
# All outputs → storage/phase{N}_*.glb
```

**Pattern verification and manual stitching (run from backend/):**
```bash
python -m scripts.interactive_stitcher --pattern data/patterns/tshirt.json  # 2D manual stitch editor
python -m scripts.verify_dxf_import --pattern data/patterns/tshirt.json     # 3D diagnostic preview
# 3D Controls: 1=body, 2=panels, 3=boundary, 4=stitches, SHIFT+Click=Vertex ID
```

**Body analysis & panel preview (run from backend/):**
```bash
python -m scripts.analyze_body       # generate/update mannequin_profile.json
python -m scripts.visualize_body     # 3D visualizer with landmark rings (Taichi GGUI)
python -m scripts.preview_panels     # preview tank top panels wrapped on body (Taichi GGUI)
```

**Body mesh preprocessing** (run once if `mannequin_physics.glb` does not exist):
```bash
python -m scripts.process_body data/bodies/mannequin.glb          # auto-fix pipeline
python -m scripts.process_body data/bodies/mannequin.glb --force  # reprocess even if proxy exists
```

**Tests:**
```bash
python -m pytest tests/ -v                  # all tests
python -m pytest tests/unit/ -v            # unit tests only
python -m pytest tests/integration/ -v    # integration tests only
python -m pytest tests/unit/constraints/test_distance.py -v  # single file
```

## Architecture

The simulation module (`backend/simulation/`) is a **fully self-contained physics engine** with no web framework dependencies. The future FastAPI layer (`backend/app/`) will wrap it.

**Data flow:**
```
SimConfig → ParticleState (Taichi fields)
         → Integrator (predict positions via gravity)
         → XPBDSolver (project constraints: distance → bending → stitch)
             ↕ collider.resolve() interleaved inside solver loop
         → Integrator (update velocities from Δpos)
         → SimResult (positions, faces, normals, UVs)
         → write_glb() via trimesh
```

**Key modules:**
- `core/config.py` — `SimConfig` dataclass: physics params (dt, substeps, solver_iterations, collision_thickness, friction_coefficient, air_drag). Goldilocks defaults: 6 substeps × 12 iterations; `body_drape` uses 15 substeps × 2 iterations (more frequent collision resolution catches particles earlier in fall trajectory).
- `core/state.py` — `ParticleState`: Taichi SoA fields (positions, predicted, velocities, inv_mass). Pin particles by setting `inv_mass = 0`.
- `core/engine.py` — `SimulationEngine` orchestrator + `SimResult` (with `export_glb()`). The engine holds an optional `solver` (SolverStrategy Protocol) and `collider`. Loop order: predict → self-collide (once) → solver iterations (body collide interleaved) → update velocities → damping.
- `solver/integrator.py` — Semi-implicit Euler: predict step (gravity) + update step (velocity from Δpos + damping).
- `solver/xpbd.py` — `XPBDSolver` implementing `SolverStrategy` Protocol. Takes `stretch_compliance` and `bend_compliance` args (not from `SimConfig`). Lagrange multipliers reset each substep (no warm starting).
- `constraints/__init__.py` — `ConstraintSet` dataclass (holds distance, bending, stitch groups) and `build_constraints(positions, edges, faces, stitch_pairs, max_stitches)` factory. Pass `stitch_pairs` as `(S,2)` int32 array from `GarmentMesh.stitch_pairs`.
- `constraints/distance.py`, `constraints/bending.py` — Stateless Taichi kernels: `DistanceConstraints`, `BendingConstraints`. Each has a `.project()` kernel and `.reset_lambdas()` called at substep start.
- `constraints/stitch.py` — `StitchConstraints`: zero rest-length XPBD distance kernel. Compliance `1e-6` closes a 0.24m gap in ~80 iterations. **No `from __future__ import annotations`** — `.template()` params break Taichi JIT.
- `collision/sphere_collider.py` — Analytical sphere collider.
- `collision/body_collider.py` — `BodyCollider`: loads a GLB body mesh, builds a `StaticSpatialHash`, delegates resolution to `resolver.py`. Created via `BodyCollider.from_glb(path)`. Detects `_physics` suffix to skip preprocessing.
- `collision/spatial_hash.py` — `StaticSpatialHash`: builds O(1) cell-based candidate triangle lookup from vertex/face arrays.
- `collision/resolver.py` — `resolve_body_collision` Taichi kernel: 27-cell neighborhood search, min-euclidean candidate selection (nearest surface wins), push-out + position-based friction.
- `collision/point_triangle.py` — Taichi kernels: `closest_point_and_bary`, `signed_distance_to_triangle`.
- `scenes/` — CLI-runnable scene scripts that wire up `SimConfig`, mesh, constraints, solver, collider, and engine.
- `export/gltf_writer.py` — `write_glb()` using trimesh.
- `mesh/grid.py` — Uniform grid mesh generation with alternating diagonal triangulation (checkerboard) and shear (cross-diagonal) edges per quad for in-plane shear resistance. `compute_area_weighted_inv_masses(positions, faces, density)` — lumped-mass FEM.
- `mesh/triangulation.py` — `triangulate_panel(vertices_2d, resolution) → TriangulatedPanel`. Grid-clip approach: bounding-box grid → point-in-polygon → add polygon boundary vertices → earcut. Returns positions (XZ plane, Y=0), faces, edges, uvs, boundary_indices.
- `mesh/panel_builder.py` — `build_garment_mesh(pattern_path, resolution) → GarmentMesh`. Loads pattern JSON, triangulates panels, applies 3D placement (`rotation_x_deg` then `rotation_y_deg` then translate), merges into global arrays, resolves stitch vertex pairs via `_find_edge_particles`.
- `scripts/process_body.py` — `smart_process()`: evaluates a body GLB (via geometry/physics checks), auto-fixes scale, welds vertices, decimates to ≤5000 faces, recalculates normals. Outputs a `*_physics.glb` proxy cached alongside the source.

**SolverStrategy Protocol** (`solver/base.py`): `initialize(state, config)` + `step(state, dt)`. `XPBDSolver` implements this. Do not bypass it — it's the PD upgrade seam.

## Critical Constraints

**Taichi kernel rules:**
- Never add `from __future__ import annotations` to files containing `@ti.kernel` or `@ti.func` functions with `.template()` parameters — string-based annotation resolution breaks JIT compilation. Files `resolver.py`, `spatial_hash.py`, `point_triangle.py` must not have this import. `body_collider.py` may use it (no kernels).
- Keep Taichi kernels in standalone files; avoid binding them as `@staticmethod` inside classes.
- Constraint/collision kernels must be stateless (no captured state) — all inputs passed as parameters.
- **i32 overflow in spatial hash build:** `_hash_cell_np()` in `spatial_hash.py` must mask each multiplication with `& 0xFFFFFFFF` before XOR to simulate `ti.i32` wraparound. Python's unlimited integers diverge from Taichi's 32-bit arithmetic for coordinates where `ix × 73856093 > 2^31`. Do NOT use `np.int32()` constructor (raises `OverflowError`).

**Collision must be interleaved:** `collider.resolve()` is called *inside* the `solver_iterations` loop, not after. Post-process collision correction causes constraint-vs-collision fighting.

**Lagrange multipliers reset each substep** (no warm-starting). This is intentional — accumulation injected energy in prior work (Vestra).

**Area-weighted mass with checkerboard triangulation creates two vertex classes:** Interior vertices connect to either 8 triangles (even-parity) or 4 triangles (odd-parity), giving a 2:1 mass ratio between them. This is correct and expected — not a bug. Always pass `inv_masses` from `compute_area_weighted_inv_masses()` to `state.load_from_numpy()` in scenes that use a fabric preset; failing to do so reverts to 1 kg/particle.

**`max_contact_dist` scales with `cell_size`:** In `resolver.py`, the Euclidean guard is `cell_size * 2.0`. Do not hardcode a meter value — the right threshold is mesh-dependent. For `mannequin_physics.glb` (avg_edge 0.021m, cell_size 0.032m), this gives 0.063m.

**Physics proxy is the asset, not the source mesh:** Tests and scenes point to `mannequin_physics.glb` directly. `BodyCollider.from_glb()` skips `smart_process()` when the path ends in `_physics`. Do not point scenes to `mannequin.glb` — `smart_process` would re-evaluate it on every run.

**Python version:** Requires Python 3.10–3.13. Taichi 1.7.4 does not support Python 3.14.

## Body Mesh

- **Source:** `data/bodies/mannequin.glb` (5,390 verts, 8,844 faces, in centimeters, 3 scene objects)
- **Physics proxy:** `data/bodies/mannequin_physics.glb` (5,689 verts, 9,604 faces, 1.75m tall, 0.021m avg edge, Y=0 at feet)
- The proxy is generated by `smart_process(mannequin.glb)` — run once, cached. The 59 disconnected components are inherent mesh topology (open boundaries at neck, wrists, ankles) and are not a bug.
- `fill_holes()` is counterproductive on this mesh — it increases component count from 59 to 62.

**Measured surface positions (for garment panel placement):**
```
Overall Z extent: [0.031, 0.346]  — NOT symmetric around Z=0; entirely positive Z
Body center Z ≈ 0.185m

At Y≈0.85–1.40m (torso):
  Front surface Z_max ≈ 0.288m
  Back surface  Z_min ≈ 0.031m
  Body depth (front-back) ≈ 0.257m

At Y≈0.65–0.80m: extremely narrow X (legs/crotch) — avoid this range for shirts

X center ≈ 0.0 (body is X-symmetric)
X half-width at Y=1.0m: ±0.49m (including arms); torso only ≈ ±0.17m
Body side at |X|≈0.20: Z depth only 0.05–0.15m (much narrower than center)
```
Garment panels must start **outside** these bounds AND be **wide enough** for seam edges to extend past the body's sides. At |X|>0.22, the body Z-depth drops to <0.10m — stitch constraints can close this gap.

## Current State

**April 16, 2026: GarmentCode Phase 4 — Pipeline Validation**

Phase 4 implemented and validated. Full `shirt_mean.json` simulation runs end-to-end.

**Completed this session:**
- **`__main__.py` CLI fix:** `--gc` and `--gc-z-offset` args were missing from the central CLI router (`__main__.py`). Only `garment_drape.py`'s own `__main__` block had them. Fixed by adding both args and threading them through the `run_garment_drape()` kwargs block.
- **Body Z-offset (+0.131m):** GarmentCode SMPL body Z-center = +0.025m (after cm→m); `mannequin_physics.glb` center = +0.156m. Added `body_z_offset: float = 0.0` param to `boxmesh_to_garment_mesh()` and `build_garment_mesh_gc()`. Applied per-panel inside the loop before stitch densification. Default 0.0 keeps all existing unit tests unaffected. `_GC_BODY_Z_OFFSET = 0.131` constant in `garment_drape.py` (with derivation comment).
- **Iterative seam densification:** Raised `_MIN_PAIRS_PER_SEAM` from 6 → 12. Replaced single-pass densification with 4-pass iterative approach using `set()`-based deduplication to avoid duplicate nearest-vertex hits on coarse meshes.
- **mesh_resolution** lowered from 2.0 cm → 1.5 cm in GC path for more natural stitch pairs.
- **12 new integration tests** in `tests/integration/test_gc_pipeline.py` (6 mesh-setup + 6 simulation). All pass. Total: **207 tests, 0 failures.**
- **`scripts/verify_gc_alignment.py`**: diagnostic script printing per-panel Z extents vs body surface bounds, initial stitch gaps, overall pass/fail.
- **`docs/implementation_plan_phases4_to_8.md`**: comprehensive phase plan with simulation results table, body alignment derivation, key constants.

**Simulation validation results (shirt_mean.json, 1.5cm mesh):**
| Check | Result |
|-------|--------|
| NaN | ✅ PASS |
| Floor penetration | ✅ PASS (min Y = 0.898m) |
| Torso coverage | ✅ PASS (2870/2870 particles in Y=[0.5,1.8]m) |
| Mean speed | ✅ PASS (0.011 m/s — fully settled) |
| Mean stretch | ✅ PASS (5.55%) |
| Stitch gaps | ❌ FAIL (3 sleeve cap seams at 10–11cm) |

**Known issues (require Phase 7):**
1. **Sleeve cap seam gaps (~10–11cm):** `seam_5` n=3, `seam_13` n=3, `seam_14` n=8. Short sleeve edges at 1.5cm resolution genuinely produce only 3 unique vertex pairs — mesh density ceiling, not a code bug. Densification can't overcome it. Need Phase 7 attachment constraints to anchor shoulder/collar vertices and shorten the initial gap.
2. **Max stretch outlier (~318%):** Back torso panel starts at Z=−0.069m, pulled forward by stitches; some vertices slip through the thin sew_collision_thickness=0.006m shell. End positions Z=0.056–0.083m (inside body). Same fix: Phase 7 attachment pins.
3. **Frontend viewer shows old DXF tshirt:** `/api/models` endpoint serves `frontend/public/models/` pre-computed GLBs. GC output is at `backend/storage/gc_shirt.glb` (not auto-copied). Will resolve when Phase 5 FastAPI layer serves `backend/storage/` as static files.

**GarmentCode body alignment:**
```
mannequin_physics.glb: chest_z_front=0.2786m, chest_z_back=0.0335m
  → mannequin_center_z = (0.2786 + 0.0335) / 2 = 0.1561m
GarmentCode SMPL: front_torso_z=+0.25m, back_torso_z=−0.20m (after cm→m)
  → gc_center_z = (0.25 + (−0.20)) / 2 = +0.025m
Required offset: 0.1561 − 0.025 = 0.131m  (+Z direction, no flip needed)
```

**Next immediate tasks:**
  1. **Phase 7 — Attachment constraints:** Soft positional pin constraints (compliance 1e-4) anchoring waist/collar vertices during sew phase. Fixes both sleeve gap and back panel slip-through. New file: `simulation/constraints/attachment.py` (no `from __future__ import annotations`).
  2. **Phase 5 — FastAPI layer:** `POST /api/simulate` endpoint; serve `backend/storage/` as static files so frontend auto-loads the GC GLB.
  3. **Live visualization path:** `--visualize` flag via Taichi GGUI for real-time drape feedback.

**Performance note:** `body_drape` at 60×60 × 16 iterations + self-collision hash rebuild is intentionally slow — performance optimization is deferred. Do not optimize before Sprint 3.

**Body analysis tools:**
- `scripts/analyze_body.py` — generates `mannequin_profile.json` with cross-sections and landmarks
- `scripts/visualize_body.py` — Taichi GGUI 3D visualizer for verifying landmark positions
- `scripts/preview_panels.py` — Taichi GGUI preview of parametric panels on body
- `simulation/mesh/body_measurements.py` — loads profile JSON, provides interpolated cross-sections and `wrap_point()` for cylinder-wrap placement

## Pattern JSON and Panel Placement

`rotation_x_deg` (applied before `rotation_y_deg`) converts the local XZ panel to a vertical panel:
- `rotation_x_deg: -90` → local `(x, 0, z)` → `(x, z, 0)` — panel height (local Z) becomes world Y
- Then `rotation_y_deg` rotates in the horizontal plane (180° for back panel to face inward)

After `rotation_y_deg=180`, back panel polygon vertices are **mirrored** — edge [0,3] and [1,2] swap left/right sides. Stitch definitions for a symmetric front/back tank top:
```json
{"panel_a": "front", "edge_a": [0, 3], "panel_b": "back", "edge_b": [1, 2]}  // left seam
{"panel_a": "front", "edge_a": [1, 2], "panel_b": "back", "edge_b": [0, 3]}  // right seam
```
Position for back panel must account for the flip: use `[+half_width, Y_start, Z_back]` not `[-half_width, ...]`.

**Panel sizing rule:** Panels must be **wider than the body's torso** so that seam edges extend past the body's sides (|X|>0.20m). At the sides, front-to-back depth is only 0.05–0.15m, allowing stitch constraints to close the gap. Panels that are exactly as wide as the torso (~0.34m) will have seam edges stuck in front/behind the full body depth (0.26m), which body collision prevents from closing. Tank top panels: 0.52m width (seam edges at |X|=0.26, past the torso side).
