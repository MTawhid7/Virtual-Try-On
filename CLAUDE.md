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
- `core/config.py` — `SimConfig` dataclass: physics params (dt, substeps, solver_iterations, collision_thickness, friction_coefficient). Goldilocks defaults: 6 substeps × 12 iterations.
- `core/state.py` — `ParticleState`: Taichi SoA fields (positions, predicted, velocities, inv_mass). Pin particles by setting `inv_mass = 0`.
- `core/engine.py` — `SimulationEngine` orchestrator + `SimResult` (with `export_glb()`). The engine holds an optional `solver` (SolverStrategy Protocol) and `collider`.
- `solver/integrator.py` — Semi-implicit Euler: predict step (gravity) + update step (velocity from Δpos + damping).
- `solver/xpbd.py` — `XPBDSolver` implementing `SolverStrategy` Protocol. Takes `stretch_compliance` and `bend_compliance` args (not from `SimConfig`). Lagrange multipliers reset each substep (no warm starting).
- `constraints/` — Stateless Taichi kernels: `DistanceConstraint`, `BendingConstraint`. Each has a `.project()` kernel.
- `collision/sphere_collider.py` — Analytical sphere collider.
- `collision/body_collider.py` — `BodyCollider`: loads a GLB body mesh, builds a `StaticSpatialHash`, delegates resolution to `resolver.py`. Created via `BodyCollider.from_glb(path)`. Detects `_physics` suffix to skip preprocessing.
- `collision/spatial_hash.py` — `StaticSpatialHash`: builds O(1) cell-based candidate triangle lookup from vertex/face arrays.
- `collision/resolver.py` — `resolve_body_collision` Taichi kernel: 27-cell neighborhood search, min-euclidean candidate selection (nearest surface wins), push-out + position-based friction.
- `collision/point_triangle.py` — Taichi kernels: `closest_point_and_bary`, `signed_distance_to_triangle`.
- `scenes/` — CLI-runnable scene scripts that wire up `SimConfig`, mesh, constraints, solver, collider, and engine.
- `export/gltf_writer.py` — `write_glb()` using trimesh.
- `mesh/grid.py` — Uniform grid mesh generation; Sprint 2 adds earcut pattern triangulation.
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

**`max_contact_dist` scales with `cell_size`:** In `resolver.py`, the Euclidean guard is `cell_size * 2.0`. Do not hardcode a meter value — the right threshold is mesh-dependent. For `mannequin_physics.glb` (avg_edge 0.021m, cell_size 0.032m), this gives 0.063m.

**Physics proxy is the asset, not the source mesh:** Tests and scenes point to `mannequin_physics.glb` directly. `BodyCollider.from_glb()` skips `smart_process()` when the path ends in `_physics`. Do not point scenes to `mannequin.glb` — `smart_process` would re-evaluate it on every run.

**Python version:** Requires Python 3.10–3.13. Taichi 1.7.4 does not support Python 3.14.

## Body Mesh

- **Source:** `data/bodies/mannequin.glb` (5,390 verts, 8,844 faces, in centimeters, 3 scene objects)
- **Physics proxy:** `data/bodies/mannequin_physics.glb` (5,689 verts, 9,604 faces, 1.75m tall, 0.021m avg edge, Y=0 at feet)
- The proxy is generated by `smart_process(mannequin.glb)` — run once, cached. The 59 disconnected components are inherent mesh topology (open boundaries at neck, wrists, ankles) and are not a bug.
- `fill_holes()` is counterproductive on this mesh — it increases component count from 59 to 62.

## Current State

Sprint 1 complete. Sprint 2 Layer 3a-Extended infrastructure complete: `BodyCollider` with `StaticSpatialHash`, `resolve_body_collision` kernel, `body_drape` scene, `scripts/process_body.py` pipeline. **2 integration tests still failing** — see `docs/handoff_sprint2_layer3a_complete.md` for full investigation details.

Sprint 2 remaining: fix the 2 failing collision tests, then pattern JSON → earcut triangulation, stitch constraints, full garment pipeline. `materials/` module stub exists but is empty.
