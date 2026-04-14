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

# Sprint 2 Layer 3b-Extended: pattern-based garment draped on body
python -m simulation --scene garment_drape
# → storage/garment_drape.glb

# Animated GLB export (morph-target drape sequence for frontend player)
python -m simulation --scene garment_drape --animate
# → storage/garment_drape_animated.glb (+ static garment_drape.glb alongside)

# Export GLBs to frontend viewer
python -m scripts.export_for_viewer
# → frontend/public/models/*.glb  (then open http://localhost:3000/viewer)

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
| **Sprint 2, Layer 3a-Ext** | ✅ | Body mesh collision — `BodyCollider` (spatial hash + point-triangle). Tuned culling thresholds, fixed mesh alignment, and integrated live Taichi GUI visualizer support. |
| **Sprint 2, Physics Realism** | ✅ | Shear edges, fabric presets (`materials/presets.py`), resting contact friction, air drag, substep rebalancing (6×12 → 15×2), compliance re-calibration for new substep_dt. |
| **Sprint 2, Fabric Realism** | ✅ | Area-weighted particle mass (lumped-mass FEM, density from `FabricPreset`), softer cotton bending (`bend_compliance 8e-4→7.4e-3`), reduced damping/friction, grid 40×40→60×60, solver iterations 2→8. Cloth now forms visible folds and conforms to body surface. |
| **Sprint 2, Algorithm Upgrades** | ✅ | Track A: analytical bending gradients (cotangent-weighted ∂θ/∂p, closes-form Bergou formula). Track B: hard strain limiting per substep (Provot/Müller clamping). Track C: cloth self-collision (dynamic centroid hash, 1-ring exclusion, euclidean penetration gate). Track D: constraint-based velocity damping (stretch + bend modes). |
| **Sprint 2, Draping Realism** | ✅ | Fixed collision velocity injection, reordered simulation loop, and tuned materials. Cloth settles at ~0.15 m/s. |
| **Sprint 2, Sew-then-Drape** | ✅ | Implemented a 30 FPS pipeline with boundary resampling (min 7mm edge), dense stitching (Steiner points), and a two-stage assembly loop. Seam welding and pattern validation added. |
| **Sprint 3, Viewer** | ✅ | Next.js 16 + React Three Fiber viewer with studio lighting, OrbitControls, cloth/body material assignment, model selector, wireframe toggle, background toggle, cloth color swatches. |
| **Sprint 3, Animated GLB** | 🔶 | Raw glTF 2.0 morph-target animated export (`write_glb_animated()`). Frontend `AnimationMixer` player with play/pause, timeline scrubber, SEW/DRAPE phase badge, speed selector. Geometry fixes applied (face winding, stitch clustering); re-simulation pending. |
| **Sprint 3, Geometry Debug** | 🔶 | Root-cause analysis scripts + fixes validated. Right sleeve surface irregularity resolved. Remaining: shoulder-sleeve gap at armhole junction and body-conformity stiffness. Max seam gap 3.57cm, max stretch 84.6% (was 169.6%). |
| **Sprint 3, Backend API** | ⬜ | FastAPI layer to serve simulations via HTTP (pattern selector + fabric picker → run simulation). |
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
- **Self-collision: euclidean gate over signed-distance gate.** `best_euclidean < thickness` (not `best_sd < -threshold`) is the correct penetration test. Signed distance alone is fooled by natural cloth folds — a particle 24mm below a fold's face normal has sd = −24mm which exceeds any small threshold, yet it is not penetrating. Euclidean distance to the surface must be within the thickness band for a genuine penetration. Additionally, self-collision runs once per substep after the XPBD iteration loop (not inside it): running it 8× per substep re-applies corrections to stale positions, amplifying each correction 8×.
- **Self-collision: normal-only correction.** No tangential/friction component in the push-out response. Friction displacement fights distance constraints and causes cascade amplification when accumulated across multiple iterations.
- **Compliance-based materials:** Fabrics defined by XPBD compliance values, not Young's modulus. Tuned by visual result. Physical parameter mapping deferred to PD upgrade.
- **Substep count over iteration count:** 15 substeps × 2 iterations outperforms 6 substeps × 12 for cloth-body collision — more frequent collision resolution catches particles earlier in their fall trajectory, reducing lateral drift.
- **Two-zone contact friction:** Position-based friction is applied both on active penetration (`sd < thickness`) and in the resting contact zone (`euclidean < thickness × 5`). The resting zone prevents cloth from sliding off curved surfaces under gravity without any push-out correction. (Bridson 2002)
- **Collision velocity injection:** In XPBD, `velocity = (positions - predicted) / dt`. Collision kernels modify `positions[i]`, which injects velocity unless `predicted[i]` is also updated. The fix is `predicted[i] += collision_correction_delta` — this cancels only the collision contribution to velocity while preserving the constraint-solving velocity. Critically, the resting contact block must NOT update `predicted[i]`; doing so would zero all velocity for particles within the contact zone (40mm), freezing the cloth.
- **Air drag:** Exponential velocity decay (`v *= exp(-drag × dt)`) applied before gravity each substep suppresses high-frequency oscillations and aids settling. Default `0.0` (disabled) for all scenes except `body_drape`.
- **Area-weighted particle mass:** Per-vertex mass = `density × (sum of adjacent triangle areas / 3)` (lumped-mass FEM). Uniform `inv_mass=1.0` made a 40×40 cotton cloth weigh 1,600 kg instead of 0.43 kg, making gravity-to-constraint ratios 3,700× wrong. Correct mass is critical for natural fold formation — the compliance-based softness only has its intended effect when inertia is physically calibrated.

## License

Private — all rights reserved.
