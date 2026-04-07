# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

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
```

**Phase verification scripts (run from backend/):**
```bash
python -m scripts.visualize_phase1_triangulation   # panel triangulation GLBs
python -m scripts.visualize_phase2_stitch          # stitch constraint closure GLBs
python -m scripts.visualize_phase3_panel_builder   # two-panel garment mesh GLBs
python -m scripts.visualize_phase4_garment_drape   # full garment-on-body GLBs
# All outputs → storage/phase{N}_*.glb
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

**Sprint 2 Layer 3b-Extended: Phases 1–4 complete. Body analysis validated. Panel shape under iteration.**

- **195/195 tests passing.**
- `mesh/triangulation.py`, `mesh/panel_builder.py`, `constraints/stitch.py` all implemented and unit-tested.
- `scenes/garment_drape.py` implemented and integration-tested (12 tests pass at reduced resolution/frames).
- Full-scene simulation: 540 particles, 42 stitch pairs, 480 frames → all 6 checks PASS.
- Live visualizer enhanced with stitch line rendering, pause/step controls, per-frame diagnostics.
- **Body Analysis v3**: Validated landmarks (neck, shoulder, armpit, chest, waist, hip) from `mannequin_profile.json`.
- **Panel Preview v3**: Parametric tank top shape wrapping onto body — functional but shape not yet professional quality.
- **Key insight**: CLO3D uses FLAT panel placement (not pre-wrapped). The panel SHAPE definition (Bézier curves from body measurements) is the bottleneck, not panel placement.

**Next immediate tasks:**
1. Implement Bézier-based panel outline generator (`pattern_generator.py`) using validated body measurements
2. Refine tank top shape to match CLO3D reference (armhole curves, neckline scoop, strap width)
3. Integrate refined panels into `garment_drape.py` with flat placement
4. Phase 5: `data/patterns/tshirt.json` (4 panels: front bodice, back bodice, 2 sleeves)
5. Sprint 3: FastAPI backend + Next.js frontend

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
