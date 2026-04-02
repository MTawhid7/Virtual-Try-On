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
python -m simulation --scene sphere_drape -v          # live Taichi GGUI visualizer
python -m simulation --scene sphere_drape -o out.glb  # custom export path
# Default export: storage/{scene}.glb
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
- `core/config.py` — `SimConfig` dataclass: all physics params (dt, substeps, solver_iterations, compliance values). Goldilocks defaults: 6 substeps × 12 iterations.
- `core/state.py` — `ParticleState`: Taichi SoA fields (positions, prev_positions, velocities, inv_mass). Pin particles by setting `inv_mass = 0`.
- `core/engine.py` — `SimulationEngine` orchestrator + `SimResult` (with `export_glb()`). The engine holds an optional `solver` (SolverStrategy Protocol) and `collider`.
- `solver/integrator.py` — Semi-implicit Euler: predict step (gravity) + update step (velocity from Δpos + damping).
- `solver/xpbd.py` — `XPBDSolver` implementing `SolverStrategy` Protocol. Lagrange multipliers reset each substep (no warm starting).
- `constraints/` — Stateless Taichi kernels: `DistanceConstraint`, `BendingConstraint`. Each has a `.project()` kernel.
- `collision/sphere_collider.py` — Analytical sphere collider; Sprint 2 will add `BodyCollider` (spatial hash + point-triangle projection).
- `scenes/` — CLI-runnable scene scripts that wire up `SimConfig`, mesh, constraints, solver, collider, and engine.
- `export/gltf_writer.py` — `write_glb()` using trimesh.
- `mesh/grid.py` — Uniform grid mesh generation; Sprint 2 adds earcut pattern triangulation.

**SolverStrategy Protocol** (`solver/base.py`): `initialize(state, config)` + `step(state, dt)`. `XPBDSolver` implements this. Do not bypass it — it's the PD upgrade seam.

## Critical Constraints

**Taichi kernel rules:**
- Never add `from __future__ import annotations` to files containing `@ti.kernel` functions with `.template()` parameters — string-based annotation resolution breaks JIT compilation.
- Keep Taichi kernels in standalone files; avoid binding them as `@staticmethod` inside classes.
- Constraint/collision kernels must be stateless (no captured state) — all inputs passed as parameters.

**Collision must be interleaved:** `collider.resolve()` is called *inside* the `solver_iterations` loop, not after. Post-process collision correction causes constraint-vs-collision fighting.

**Lagrange multipliers reset each substep** (no warm-starting). This is intentional — accumulation injected energy in prior work (Vestra).

**Python version:** Requires Python 3.10–3.13. Taichi 1.7.4 does not support Python 3.14.

## Current State

Sprint 1 complete. Sprint 2 next: body mesh collision (`BodyCollider`), pattern JSON → earcut triangulation, stitch constraints, full garment pipeline (`--pattern tshirt --fabric cotton`).
