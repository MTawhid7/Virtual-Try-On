# Garment Simulation Engine

A modular garment simulation platform that takes 2D fabric pattern pieces, automatically stitches them together, and drapes them onto a 3D body model using physics simulation.

## Architecture

**Solver:** XPBD (Extended Position-Based Dynamics) in Taichi kernels, with a `SolverStrategy` Protocol abstraction enabling a future Projective Dynamics upgrade.

**Collision:** Mesh-proxy + static spatial hash + point-triangle projection with smoothed normals — proven stable in prior work. No SDF. No IPC.

**Output:** Bake-and-serve model — simulation runs server-side, producing a static `.glb` file rendered by the frontend.

```
backend/
├── simulation/          ← Physics engine (standalone, no web deps)
│   ├── core/            Engine orchestration, config, particle state
│   ├── solver/          XPBD solver + semi-implicit Euler integrator
│   ├── constraints/     Distance, bending, stitch constraints
│   ├── collision/       Sphere + body mesh colliders (spatial hash + point-triangle)
│   ├── materials/       Fabric presets (compliance-based)
│   ├── mesh/            Grid generation, pattern triangulation
│   └── export/          glTF/GLB output
├── app/                 ← FastAPI web layer (Sprint 3)
├── data/bodies/         Body GLB meshes (mannequin.glb → mannequin_physics.glb proxy)
├── storage/             Simulation output .glb files
├── tests/
│   ├── unit/            ← Component-level math & logic tests
│   └── integration/     ← Layer-by-layer physics validation
```

## Quick Start

### Prerequisites

- **Python 3.10–3.13** (Taichi 1.7.4 does not yet support Python 3.14)
- pip
- On macOS: `brew install python@3.13` if only Python 3.14 is available

### Setup

```bash
cd backend
python3.13 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### Run

```bash
# Layer 1: Freefall test (gravity only, no constraints)
python -m simulation --scene freefall

# Layer 2: Constrained fall (distance + bending, pinned corners)
python -m simulation --scene constrained_fall

# Layer 3a: Analytical sphere collision
python -m simulation --scene sphere_drape
python -m simulation --scene sphere_drape -v   # live Taichi GGUI visualizer

# Sprint 2 Layer 3a-Extended: cloth draped over body mesh
# Requires data/bodies/mannequin_physics.glb (already in repo)
python -m simulation --scene body_drape
# → storage/body_drape.glb

# Export to custom path
python -m simulation --scene sphere_drape -o my_output.glb
```

> **Tip:** Append `-v` to any scene to watch it render in real-time. All exports go to `storage/{scene}.glb` by default.

### Test

All tests are located in the `backend/tests/` directory. Run then from the `backend/` root:

```bash
# Run all tests
python -m pytest tests/ -v

# Run only unit tests
python -m pytest tests/unit/ -v

# Run only layer-by-layer integration tests
python -m pytest tests/integration/ -v
```

## Development Phases

| Phase | Status | Description |
|-------|--------|-------------|
| **Sprint 1, Layer 1** | ✅ | Particle system — gravity, integration, grid mesh |
| **Sprint 1, Layer 2** | ✅ | Distance + bending constraints, XPBD solver |
| **Sprint 1, Layer 3a** | ✅ | Sphere collision (analytical & visualizer implementations) |
| **Sprint 1, Layer 3b** | ✅ | glTF export — `write_glb()` via trimesh, `SimResult.export_glb()`, CLI `--output` flag |
| **Sprint 2, Layer 3a-Ext** | ⚠️ | Body mesh collision — `BodyCollider` (spatial hash + point-triangle). 10/12 tests pass; candidate selection strategy under investigation (see `docs/handoff_sprint2_layer3a_complete.md`) |
| **Sprint 2, Layer 3b-Ext** | ⬜ | Pattern JSON → earcut triangulation, stitch constraints, full garment pipeline |
| **Sprint 3** | ⬜ | FastAPI backend, Next.js + R3F frontend |
| **Sprint 4** | ⬜ | Integration, polish, end-to-end testing |

## Technology Stack

| Component | Technology |
|-----------|------------|
| Simulation kernels | [Taichi Lang](https://taichi-lang.org/) |
| Numerics | NumPy |
| Mesh I/O | trimesh |
| Triangulation | mapbox-earcut |
| Mesh decimation | fast-simplification |
| API (Sprint 3) | FastAPI |
| Frontend (Sprint 3) | Next.js + React Three Fiber |

## Key Design Decisions

- **XPBD over PD for Phase 1:** Unconditionally stable, faster to first demo, proven in prior work (Vestra). PD upgrade path preserved via `SolverStrategy` Protocol.
- **Mesh-proxy collision over SDF:** Voxel SDF caused gradient artifacts and upward crumpling. Point-triangle projection with smoothed normals is geometrically accurate and stable.
- **Interleaved collision:** Collision resolved inside the solver iteration loop (not post-process), preventing the constraint-vs-collision fighting observed in prior work.
- **No self-collision in Phase 1:** Deferred — both prior engines (Vestra/Vistio) showed this is a separate, complex challenge.
- **Compliance-based materials:** Fabrics defined by XPBD compliance values, not Young's modulus. Tuned by visual result. Physical parameter mapping deferred to PD upgrade.

## License

Private — all rights reserved.
