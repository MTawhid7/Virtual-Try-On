# Project Garment Simulation — AI Developer Guide

Welcome to the Garment Simulation Engine. This file provides architectural context, engineering principles, and a clear system map so any AI developer can quickly orient themselves within the codebase and understand the design philosophy.

## 🎯 Project Objective
To engineer a production-ready, modular garment simulation platform. It takes 2D pattern piece data, dynamically stitches them, and simulates draping them across arbitrary 3D body models leveraging **XPBD (Extended Position-Based Dynamics)** physically backed by **Taichi kernels** for native parallel hardware execution.

## 📁 System Architecture
The application runs as a detached "bake-and-serve" Python system scaling up to eventual server deployments via FastAPI to WebGL environments. 

### Directory Structure
```
garment-sim/
├── docs/                # System architectures, execution plans, and worklogs
├── backend/
│   ├── simulation/      # The isolated physical engine module (No web-framework deps!)
│   │   ├── core/        # Orchestrator, system configs, & data shapes (ParticleState, SimResult, engine.py)
│   │   ├── solver/      # Integrator + Base XPBD handler (Abstract SolverStrategy scaling to future PD!)
│   │   ├── constraints/ # Individual math equations: Distance, Bending, etc.
│   │   ├── collision/   # Spatial hashing mechanisms & Colliders (Sphere/Body)
│   │   ├── scenes/      # Executable CLI targets orchestrating configurations
│   │   ├── mesh/        # Grid mesh generations & triangulation functions
│   │   └── __init__.py
│   ├── outputs/         # Visual cache holding .obj / .gltf physics dumps
│   └── tests/           # Robustly partitioned `unit` (logic) and `integration` (layer bounds) scripts
└── app/                 # (Sprint 3) Scheduled to handle HTTP / FastAPI mappings
```

## 🧠 Core Engineering Principles
If you are extending the system, please strictly follow these rules to maintain stability and performance:
1. **Mathematical Isolation:** Components under `constraints/` and `collision/` must remain perfectly standalone stateless executions powered strictly by variables passed into their `@ti.kernel` parameters. This preserves testing determinism.
2. **SolverStrategy Protocol:** Do not tightly bind XPBD properties inside the orchestration classes. The `SimulationEngine` runs via an abstract protocol so that future migrations to Projective Dynamics (PD) do not rewrite the stack.
3. **Interleaved Constraints Calculation:** To avoid fighting errors, always calculate colliding limits/bothers *inside* the solving iterators alongside standard constraints per-frame not as a post-process correction step.
4. **Taichi Compilation Strictness:** Taichi utilizes intense JIT analysis on `@ti.kernel` functions. **Never** include `from __future__ import annotations` inside any files containing active kernel parameters executing `.template()` tracking, as string mapping breaks kernel compilation scopes instantly. Keep modules functionally uncoupled where possible avoiding convoluted class `@staticmethod` kernel bindings.
5. **Decoupled Validations:** We require strict mathematical unit testing natively executing independently without the core `__main__.py` loop before we accept pull logic into full scale integration files simulating across structural geometries.

## 🚀 Current Technical State
**Stage:** Sprint 1, Layer 3a Completed 
We are utilizing full XPBD iteration techniques mapping native gravitation drops alongside active distance limits and analytical sphere collisions cleanly out of the Taichi GPU system.

**Abilities Available:**
- Can spawn freeform grids alongside collision bodies mapped across distance and isometric bending constraints physically resolving inside standard Python environments perfectly decoupled from dependencies.
- Complete live visual integration pipeline invoking cross-platform Taichi `window` GUI debugging parameters seamlessly triggered via `-v` flags.

**Next Steps & Implementation Horizons:**
- **Layer 3b:** Developing static `.glb` sequences bypassing rigid `.obj` outputs scaling toward true sequential outputs.
- **Sprint 2:** Removing Analytical Sphere functions to implement scalable generic Spatial Hashing targeting arbitrary incoming standard mesh bodies. Incorporate robust velocity boundary clamps directly into the interaction logic preventing "bounce" discrepancies.
