# Vistio - AI Context & Architecture Blueprint

## 1. Purpose

This document serves as the central architectural reference for AI agents and human engineers contributing to the Vistio garment simulation engine. It is designed to provide rapid context, map crate boundaries, define the core data flow, and document recommended patterns to minimize discovery overhead and unnecessary file scanning.

We suggest utilizing this document as the primary "source of truth" to direct your searches, locate entry points, and verify architectural invariants when approaching a new task.

## 2. Core Execution Flow (The Main Loop)

The primary simulation loop is coordinated in integration modules like `vistio-viewer` (for Bevy realtime) and `vistio-bench` (for CLI analysis). A typical physics frame execution is suggested to follow these phases:

1. **Prediction**: `SimulationState` predicts new hypothetical positions without constraints ($x^* = pos + v \Delta t + g \Delta t^2$).
2. **Augmented Lagrangian (AL) Outer Loop**: Coordinates the contact forces.
3. **Local-Global PD Solve**: `ProjectiveDynamicsSolver` runs an iterative loop:
   - *Local Step*: Computes projected valid states from constraints (e.g., ARAP/membrane logic, Discrete Shells bending, pinned positions).
   - *Global Step*: Solves the prefactored sparse Cholesky matrix containing internal energy components (handled natively via `faer`).
4. **Collision / IPC Enforcement**:
   - Identify candidate pairs via `BvhBroadPhase` ($O(N \log N)$).
   - Compute barrier gradients and distance metrics for vertex-triangle and edge-edge pairings.
   - Ensure the solver adheres to minimum distance thresholds dynamically.
5. **Integration & Finalization**: Update velocity $v = (x_{new} - x_{old}) / \Delta t$ and commit final positions to the `SimulationState`.

## 3. Crate Architecture & Responsibilities

The workspace is logically decoupled into 14 modular crates. It is highly recommended to respect these strict boundaries to avoid circular dependencies or massive compiler invalidation.

- **`vistio-bench`**: Integration harness executing offline canonical scenarios (`hanging_sheet`, `sphere_drape`, `self_fold`) and emitting metrics.
- **`vistio-cli`**: Command Line Interface parsing arguments routing to various subcommands.
- **`vistio-contact`**: Specialized physics logic for Bounding Volume Hierarchies, IPC log-barrier functions, CCD, and distance primitives.
- **`vistio-debug`**: Snapshotting, state dumps, and diagnostic inspection hooks.
- **`vistio-gpu`**: Traits, stubs, and buffers meant to facilitate eventual GPU (`wgpu`) data transfers.
- **`vistio-io`**: Serialization tools and TOML configuration validators.
- **`vistio-material`**: Constitutive models (e.g., Co-rotational tension-field, Orthotropic definitions) that translate distinct fabric data into energy parameters.
- **`vistio-math`**: Linear algebra extensions, polar decomposition, and solver math (heavily extending `glam`).
- **`vistio-mesh`**: Structure of Arrays (SoA) `TriangleMesh` management, vertex neighborhood topology queries, and procedural cloth generators.
- **`vistio-render`**: Headless simulation traits preventing standard physics logic from depending directly on UI backends.
- **`vistio-solver`**: The heavy lifting engine (`ProjectiveDynamicsSolver`, `SimulationState`, matrix assembly, and the constraint solver loops).
- **`vistio-telemetry`**: Multi-threaded, lock-free `mpsc` metric and event publishing.
- **`vistio-types`**: Fundamental type definitions, standard aliases, and universal IDs utilized throughout all crates.
- **`vistio-viewer`**: Real-time interactive 3D simulation frontend bridging the physics core into a Bevy app context.

## 4. Key Data Structures (Where to go first)

When attempting to understand or extend functionality, these locations are the recommended entry points:

- **State Management**: `crates/vistio-solver/src/state.rs` (`SimulationState`)
- **Core Physics Solver**: `crates/vistio-solver/src/pd_solver.rs` (`ProjectiveDynamicsSolver`)
- **Collision Responses**: `crates/vistio-contact/src/...` (`CollisionPipeline`, `SphereCollider`, `GroundPlane`)
- **Main Run Loops**: `crates/vistio-viewer/src/lib.rs` (`simulate_cloth` system) and `crates/vistio-bench/src/runner.rs` (`Runner`)

## 5. Recommended Patterns & Invariants

While flexibility is encouraged, the historical development of Vistio provides evidence for these recommended patterns:

- **Mass Consistency**: Ensure that mass values computed in the `ProjectiveDynamicsSolver` matrix ($M/h^2$) identically mirror the scaled/lumped parameters in `SimulationState`. Misaligned momentum calculations often break barriers.
- **Logarithmic Barriers Over Explicit Interventions**: Historically, explicit position projections caused explosive jitter. The recommended paradigm for contact resolution is to tune Augmented Lagrangian IPC parameters (e.g., `d_hat`, `kappa`, `epsilon`) rather than injecting manual offsets (`dx`) or modifying velocities dynamically outside the implicit loop.
- **Continuous Collision Detection (CCD)**: CCD acts as the primary safety net to prevent high-velocity implicit tunneling. Avoid bypassing CCD indiscriminately, though skip mechanisms for pre-penetrating nodes are occasionally employed.
- **Fixed Timesteps**: The implicit math relies heavily on deterministic, fixed timesteps (e.g., `1/60s`). `vistio-viewer` utilizes an accumulator pattern to safely decouple frame rendering drops from physical integrations.

## 6. Guidelines for AI Agents

- **Avoid Global Grep / Brute Force Scanning**: We recommend leveraging this document's crate mapping to focus your file investigation explicitly on the target domains.
- **Test Organization**: Vistio strictly organizes tests into `crates/<name>/tests/` (for integration scenarios) and `crates/<name>/src/tests/` (for unit-targeting internal logic). Please adhere to these locations.
- **Human Collaboration**: After completing a major task "Tier" or solving structural anomalies, you are encouraged to document your insights deeply in `docs/WORKLOG.md`. Concurrently, update this `AI_ARCHITECTURE.md` file if data flows or crate structures pivot entirely. Use tool callbacks (`notify_user`) for major blocking decisions.
- **Investigating Failures**: Root logs or standalone compiler pipelines are frequently dumped to `tools/logs/`. Refer to `test_errors.txt` inside that directory if test feedback requires raw compiler debugging.

## 7. Known Ambiguities & Technical Debt

- The exact interaction zone between IPC internal barrier repulsion bounds and explicit post-solve safety nets (like hard floor resolutions) historically introduces transient resting oscillations. Tuning heuristic damping models is a known open-ended optimization space.
