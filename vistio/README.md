# Vistio

**GPU-accelerated garment physics simulation engine.**

Built in Rust with a modular crate architecture, Vistio targets production-grade drape quality using Projective Dynamics, pluggable constitutive models, IPC-class contact handling, and (planned) `wgpu` compute shaders.

> *Vistio* — from Italian *"vestito"* (dress/garment).

---

## Status

> 🤖 **AI Agents & Contributors**: Please begin your exploration by reading `docs/AI_ARCHITECTURE.md` for a comprehensive overview of crate boundaries, core data loops, and architectural invariants.

✅ **Tier 0 — Foundation & Interface Contract** — Complete
✅ **Tier 1 — Projective Dynamics Solver** — Complete
✅ **Tier 2 — Co-Rotational FEM & Visual Simulation** — Complete
✅ **Tier 3 — Discrete Shells & Anisotropic Materials** — Complete
🔲 **Tier 4 — IPC Barrier Contact & Augmented Lagrangian** — In Progress (Unstable)

| Gate | Result |
| --- | --- |
| Build | ✅ 14 crates |
| Tests | ✅ 218 pass |
| Clippy | ✅ 0 errors |
| CLI | ✅ 4 subcommands |

### What's Implemented

- **Core types** — Strongly-typed IDs, error handling, physical constants
- **Linear algebra** — 3×2 deformation gradients, polar decomposition, CSR sparse matrices
- **Mesh system** — SoA TriangleMesh, procedural generators (quad grid, UV sphere), topology queries, vertex normals, alternating checkerboard triangulation
- **Materials** — ConstitutiveModel trait, FabricProperties (KES-mapped), 5 built-in presets, `CoRotationalModel` (tension-field theory), `OrthotropicLinearModel` (anisotropic)
- **Solver** — Projective Dynamics robust local-global solver, prefactored Cholesky (`faer`), ARAP co-rotational elements, integrated dihedral bending system matrix, Rayleigh damping, area-weighted lumped mass. Augmented Lagrangian (AL) outer loop for rigorous contact enforcement.
- **Contact** — Incremental Potential Contact (IPC) pipeline: $C^2$ log-barrier energy, generic distance primitives, Continuous Collision Detection (CCD) for tunneling prevention, $O(N \log N)$ Bounding Volume Hierarchy (BVH) broad phase. Guaranteeably intersection-free simulation.
- **Viewer** — Bevy PBR real-time 3D viewer with pan/orbit camera, dynamic vertex normals, double-sided materials, shadow casting
- **GPU abstraction** — GpuBackend trait, CpuFallback (axpy, dot, fill), ComputeBuffer
- **Telemetry** — EventBus (mpsc), 7 event types, pluggable sinks
- **Debug** — InspectionHook trait, state snapshots with bincode serialization
- **Benchmarks** — 2 procedural scenarios (hanging sheet, sphere drape), metrics collection, CSV export
- **Rendering** — Renderer trait, HeadlessRenderer
- **CLI** — `simulate`, `benchmark`, `inspect`, `visualize` subcommands

### What's Not Yet Implemented

- **GPU compute** — wgpu backend with WGSL shaders (Tier 5)
- **Adaptive remeshing** — Dynamic mesh refinement (Tier 6)

### Known Limitations

- **CPU Performance on Dense Collisions:** While the simulation represents state-of-the-art mathematical robustness (IPC/Augmented Lagrangian), resolving macroscopic mesh-on-mesh self-overlap (e.g., highly complex folding like the `self_fold` or future 3D garments) natively on a single-threaded CPU causes substantial framerate drops. Dense barrier operations will be actively transitioned to WGSL GPU compute (Tier 5).

---

## Architecture

```text
vistio/
├── Cargo.toml           # Workspace root
└── crates/
    ├── vistio-types/     # IDs, errors, constants
    ├── vistio-math/      # Mat3x2, polar decomp, CSR sparse
    ├── vistio-mesh/      # SoA TriangleMesh, generators, topology
    ├── vistio-material/  # ConstitutiveModel, 5 fabric presets
    ├── vistio-solver/    # SolverStrategy, SimulationState, PD stub
    ├── vistio-contact/   # BroadPhase/NarrowPhase/ContactResponse
    ├── vistio-gpu/       # GpuBackend trait, CpuFallback
    ├── vistio-telemetry/ # EventBus, event types, sinks
    ├── vistio-debug/     # InspectionHook, StateSnapshot
    ├── vistio-bench/     # 3 benchmark scenarios, metrics
    ├── vistio-render/    # Renderer trait, HeadlessRenderer
    ├── vistio-io/        # SimulationInput/Output, validator
    └── vistio-cli/       # CLI binary (clap)
```

| Layer | Crates | Purpose |
| --- | --- | --- |
| **Foundation** | `types`, `math` | Shared types, linear algebra |
| **Core** | `mesh`, `material`, `solver`, `contact`, `gpu` | Simulation engine |
| **Infrastructure** | `bench`, `debug`, `telemetry` | Testing & monitoring |
| **Interface** | `io`, `render`, `cli` | I/O, rendering, CLI |

---

## Getting Started

### Prerequisites

- Rust 1.83+ (stable)
- Cargo

### Build

```bash
cargo build --workspace
```

### Test

```bash
# Run all 111 tests
cargo test --workspace

# Run a single crate's tests
cargo test -p vistio-solver

# Run a single test with output
cargo test -- pd_stub_gravity_motion --nocapture
```

### Lint

```bash
cargo clippy --workspace -- -D warnings
```

### CLI

```bash
# Run all 3 benchmark scenarios
cargo run --bin vistio -- benchmark --scenario all

# Run a single scenario with CSV output
cargo run --bin vistio -- benchmark --scenario hanging_sheet --output results.csv

# Validate a solver config
cargo run --bin vistio -- validate simulation.toml

# Inspect a state snapshot
cargo run --bin vistio -- inspect snapshot.bin
```

---

## Benchmark Scenarios

| Scenario | Description | Mesh | Timesteps |
| --- | --- | --- | --- |
| `hanging_sheet` | 1m² cloth pinned at top edge | 20×20 (441 verts) | 120 (2s) |
| `sphere_drape` | 1.5m² cloth falling onto sphere | 20×20 (441 verts) | 180 (3s) |
| `self_fold` | Cloth dropped diagonally onto ground, folding onto itself | 0.2m × 2.0m ribbon | 300 (5s) |
| `cusick_drape` | ISO 9073-9 constraint: 30cm circular swatch falling onto a 0.3m pedestal | Clipped Quad Grid | 300 (5s) |

> **Note:** Scenarios leverage the Tier 4 Augmented Lagrangian IPC collision pipeline, guaranteeing mathematically intersection-free results against colliders.

---

## Material Presets

| Preset | Density (g/m²) | Use Case |
| --- | --- | --- |
| `chiffon` | 40 | Sheer, lightweight drape |
| `silk_charmeuse` | 80 | Flowing formal wear |
| `jersey_knit` | 160 | Stretchy casual wear |
| `cotton_twill` | 200 | Structured garments |
| `denim_14oz` | 400 | Heavy, stiff jeans |

---

## Testing Guidelines

All structural tests live in `crates/<name>/tests/<name>_tests.rs`. Internal unit tests testing private functions are organized into dedicated modules in `crates/<name>/src/tests/`. No inline `#[cfg(test)]` blocks are permitted.

| Category | Example |
| --- | --- |
| Construction | `state_from_mesh`, `buffer_zeros` |
| Invariants | `validate_catches_oob_index` |
| Behavior | `pd_stub_gravity_motion` |
| Edge cases | `polar_degenerate_does_not_panic` |
| Serialization | `config_serialization` |
| Internal Collision | `bvh_broad_phase_nearby_finds_pairs` |

---

## Technology Stack

| Component | Technology |
| --- | --- |
| Language | Rust (stable 1.83+) |
| Math | `glam` (SIMD-accelerated) |
| GPU (planned) | `wgpu` + WGSL |
| Sparse LA (planned) | `faer`, CHOLMOD FFI |
| Debug (planned) | Rerun, Tracy |
| Config | TOML + serde |
| CLI | clap 4.x |

---

## Roadmap

| Tier | Focus | Status |
| --- | --- | --- |
| **Tier 0** | Foundation, traits, pipeline | ✅ Complete |
| **Tier 1** | Real PD solver, spatial hash, `faer` | ✅ Complete |
| **Tier 2** | Co-Rotational FEM, Bevy viewer, collision pipeline | ✅ Complete |
| **Tier 3** | Discrete shell bending, anisotropic materials | ✅ Complete |
| **Tier 4** | IPC barriers, CCD, robust self-collision | ✅ Complete |
| **Tier 5** | GPU acceleration (`wgpu`) | 🔲 Planned |
| **Tier 6** | Adaptive remeshing | 🔲 Planned |

## License

MIT
