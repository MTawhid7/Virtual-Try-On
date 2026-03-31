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
│   ├── collision/       Spatial hash + point-triangle body collision
│   ├── materials/       Fabric presets (compliance-based)
│   ├── mesh/            Grid generation, pattern triangulation
│   └── export/          glTF/GLB output
├── app/                 ← FastAPI web layer (Sprint 3)
├── data/                Body models, pattern JSONs
└── tests/
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
```

### Test

```bash
python -m pytest tests/ -v
```

## Development Phases

| Phase | Status | Description |
|-------|--------|-------------|
| **Sprint 1, Layer 1** | ✅ | Particle system — gravity, integration, grid mesh |
| **Sprint 1, Layer 2** | ⬜ | Distance + bending constraints |
| **Sprint 1, Layer 3a** | ⬜ | Sphere collision (analytical) |
| **Sprint 1, Layer 3b** | ⬜ | glTF export |
| **Sprint 2** | ⬜ | Body mesh collision, stitch, patterns, garment pipeline |
| **Sprint 3** | ⬜ | FastAPI backend, Next.js + R3F frontend |
| **Sprint 4** | ⬜ | Integration, polish, end-to-end testing |

## Technology Stack

| Component | Technology |
|-----------|------------|
| Simulation kernels | [Taichi Lang](https://taichi-lang.org/) |
| Numerics | NumPy |
| Mesh I/O | trimesh |
| Triangulation | mapbox-earcut |
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
