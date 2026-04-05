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
python -m simulation --scene sphere_drape -v          # live Taichi GGUI visualizer
python -m simulation --scene sphere_drape -o out.glb  # custom export path
# Default export: storage/{scene}.glb
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
- `core/engine.py` — `SimulationEngine` orchestrator + `SimResult` (with `export_glb()`). The engine holds an optional `solver` (SolverStrategy Protocol) and `collider`.
- `solver/integrator.py` — Semi-implicit Euler: predict step (gravity) + update step (velocity from Δpos + damping).
- `solver/xpbd.py` — `XPBDSolver` implementing `SolverStrategy` Protocol. Takes `stretch_compliance` and `bend_compliance` args (not from `SimConfig`). Lagrange multipliers reset each substep (no warm starting).
- `constraints/__init__.py` — `ConstraintSet` dataclass (holds distance + bending groups) and `build_constraints(positions, edges, faces)` factory that extracts topology and initializes constraints. Stitch constraints are stubbed (`# Sprint 2`).
- `constraints/distance.py`, `constraints/bending.py` — Stateless Taichi kernels: `DistanceConstraints`, `BendingConstraints`. Each has a `.project()` kernel and `.reset_lambdas()` called at substep start.
- `collision/sphere_collider.py` — Analytical sphere collider.
- `collision/body_collider.py` — `BodyCollider`: loads a GLB body mesh, builds a `StaticSpatialHash`, delegates resolution to `resolver.py`. Created via `BodyCollider.from_glb(path)`. Detects `_physics` suffix to skip preprocessing.
- `collision/spatial_hash.py` — `StaticSpatialHash`: builds O(1) cell-based candidate triangle lookup from vertex/face arrays.
- `collision/resolver.py` — `resolve_body_collision` Taichi kernel: 27-cell neighborhood search, min-euclidean candidate selection (nearest surface wins), push-out + position-based friction.
- `collision/point_triangle.py` — Taichi kernels: `closest_point_and_bary`, `signed_distance_to_triangle`.
- `scenes/` — CLI-runnable scene scripts that wire up `SimConfig`, mesh, constraints, solver, collider, and engine.
- `export/gltf_writer.py` — `write_glb()` using trimesh.
- `mesh/grid.py` — Uniform grid mesh generation with alternating diagonal triangulation (checkerboard) and shear (cross-diagonal) edges per quad for in-plane shear resistance. `compute_area_weighted_inv_masses(positions, faces, density)` — lumped-mass FEM: each triangle distributes `area/3` to each vertex, returns per-vertex `inv_mass = 1/(density × vertex_area)`. Sprint 2 adds earcut pattern triangulation.
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

## Current State

Sprint 1 complete. Sprint 2 complete through "Algorithm Upgrades" (Tracks A–D):
- **Layer 3a-Extended:** `BodyCollider` with `StaticSpatialHash`, `resolve_body_collision` kernel, `body_drape` scene, `scripts/process_body.py` pipeline.
- **Physics Realism:** Shear edges in `mesh/grid.py`, `materials/presets.py` with 5 fabric presets, `air_drag` in `SimConfig` + `Integrator`, substep rebalancing (6×12 → 15×2 for `body_drape`), two-zone contact friction.
- **Fabric Realism:** Area-weighted mass via `compute_area_weighted_inv_masses()`, cotton re-calibrated, `body_drape` grid upgraded to 60×60 × 8 iterations. Cloth forms visible folds and conforms to body surface.
- **Track A — Analytical bending:** Closed-form cotangent-weighted ∂θ/∂p gradients in `constraints/bending.py`, replacing 24 `atan2` FD evaluations per hinge. Sign convention: n1 = e × (p2-p0) → all n1 terms negated vs. Bergou paper.
- **Track B — Strain limiting:** `apply_strain_limit` kernel in `constraints/distance.py` clamps edges to [L₀×(1−max_compress), L₀×(1+max_stretch)] per substep. `max_stretch`/`max_compress` fields added to `FabricPreset` and `XPBDSolver`.
- **Track C — Self-collision:** `collision/self_collider.py` + `collision/self_resolver.py`. `ClothSelfCollider` rebuilds a dynamic hash once per substep via `update_hash()` (centroid-based, vectorised numpy argsort), then `resolve()` runs the kernel once per substep AFTER the XPBD iteration loop (not inside it). 1-ring exclusion via vertex equality in kernel. Penetration gate: `best_euclidean < thickness` — particle must be within the 4mm band, not just on the wrong signed-distance side (natural folds produce sd<0 at large euclidean distance and must not trigger). Normal-only correction; no tangential friction. `self_collision_thickness=0.004` added to `SimConfig`.
- **Track D — Constraint damping:** `apply_stretch_damping` / `apply_bend_damping` Taichi kernels in distance/bending constraints. Applied once per substep after `integrator.update()` via `solver.apply_damping(state)`. `stretch_damping` / `bend_damping` fields added to `FabricPreset` and `XPBDSolver`.

Sprint 2 remaining (Layer 3b-Extended): pattern JSON → earcut triangulation, stitch constraints, full garment pipeline. Files not yet created: `constraints/stitch.py`, `mesh/triangulation.py`, `mesh/panel_builder.py`, `mesh/placement.py`.

**Performance note:** `body_drape` at 60×60 × 8 iterations + self-collision hash rebuild is intentionally slow — performance optimization is deferred. Do not optimize before Sprint 3.
