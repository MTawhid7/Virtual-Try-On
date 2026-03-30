# Vestra Physics Engine

![Version](https://img.shields.io/badge/Version-3.0.0-blue)
![Status](https://img.shields.io/badge/Status-Stable-green)
![Tech](https://img.shields.io/badge/Tech-Rust_%7C_WASM_%7C_SIMD128_%7C_Rayon_%7C_React_Three_Fiber-orange)

**Vestra** is a high-performance, real-time Virtual Try-On (VTO) engine built for the modern web.

It utilizes a **Hybrid Compute Architecture** to bridge the gap between physical realism and browser performance. The core physics solver is written in **Rust**, compiled to **WebAssembly (WASM)** with **SIMD128** acceleration, and communicates with a **React Three Fiber** frontend via a **Zero-Copy** memory model.

Unlike traditional JavaScript-based physics engines, Vestra is designed specifically for cloth simulation, handling heavy fabrics (cotton, denim) on articulated characters without the jitter, tunneling, or "floating" artifacts common in web-based solutions.

---

## 📚 Table of Contents

- [Architecture](#-architecture)
- [Key Features](#-key-features)
- [Project Structure](#-project-structure)
- [Getting Started](#-getting-started)
- [Configuration](#%EF%B8%8F-configuration)
- [Development Workflow](#-development-workflow)
- [Roadmap](#-roadmap)

---

## 🏗 Architecture

Vestra follows **Clean Architecture** principles to ensure separation of concerns, testability, and scalability.

### 1. The Core (Domain Layer)

Located in `src/core`.

- **Entities:** Pure data logic (`Garment`, `SimulationState`).
- **Interfaces:** Contracts for external dependencies.
- **Logic:** Universal constants and sizing standards.

### 2. The Application (Use Cases Layer)

Located in `src/application`.

- **Pipelines:** Orchestrates complex workflows (`AssetPreparation`).
- **Orchestration:** Coordinates data flow between UI, Physics, and Renderer.

### 3. The Infrastructure (Adapter Layer)

Located in `src/infrastructure`.

- **WASM Adapter:** Managing the Rust backend, memory marshalling, and thread safety.
- **Worker Throttling:** Intelligent step regulation to prevent "Death Spiral" crashes on low-end devices.
- **Geometry Engine:** Mesh analysis and processing.

### 4. The Presentation (UI Layer)

Located in `src/presentation`.

- **React Components:** Pure UI overlays.
- **Zustand Stores:** State management for the simulation loop.
- **Canvas:** React Three Fiber scene graph.

### 5. The Physics Backend (Rust)

Located in `physics/`.

- **Data-Oriented Design:** SoA layout for SIMD efficiency.
- **XPBD Solver:** Extended Position Based Dynamics with **Chebyshev Acceleration**.
- **Parallelism:** Multithreaded collision resolution (Rayon) and Broad/Narrow phase partitioning.
- **WASM SIMD:** 128-bit vector instructions for math-heavy code.
- **Zero-Allocation:** Pre-allocated buffers prevent GC spikes.

---

## 🚀 Key Features

### 🧶 Physics & Simulation

- **XPBD Solver:** Stable simulation of stiff constraints.
- **Parallel Self-Collision:** High-performance, graph-colored collision resolution using Rayon (multithreading).
- **Anisotropic Bending:** Realistic fabric buckling (warp/weft differentiation).
- **Aerodynamics:** Lift and drag forces based on wind vectors.
- **WASM SIMD:** Manual 128-bit vector intrinsics for constraints and integration.
- **Chebyshev Acceleration:** Faster convergence for stiff materials.
- **Speculative Contacts:** Prevents tunneling during fast motion.

### ⚡ Performance & Stability

- **Zero-Copy Rendering:** Shared memory buffer between WASM and WebGL.
- **Physics Worker:** Dedicated WebWorker decoupling physics (30Hz+) from rendering (60Hz).
- **Adaptive Throttling:** Automatically skips physics steps if the worker is overloaded, preventing browser freezes.
- **Interpolation Safety:** Prevents visual snapping when steps are skipped.

### 📐 Asset & Intelligence

- **Smart Welding:** Merges UV seams while respecting normals.
- **Auto-Orientation:** "Voter" algorithm corrects upside-down or backward mannequins.
- **Anatomical Anchoring:** Snaps garments to body features (Neck/Shoulders).

---

## 📂 Project Structure

```text
root/
├── physics/                    # Rust Source Code (Crate)
│   ├── src/
│   │   ├── engine/             # State, Config, Loop
│   │   ├── systems/            # Constraints, Forces, Dynamics
│   │   ├── collision/          # Spatial Hash, Narrow Phase, Collider
│   │   └── utils/              # Math, Profiling (Disabled in Prod)
│   ├── tests/                  # Integration Tests
│   └── Cargo.toml
│
├── src/                        # TypeScript Frontend
│   ├── core/                   # Domain Entities
│   ├── application/            # Use Cases
│   ├── infrastructure/         # Adapters (WASM, Worker)
│   └── presentation/           # React UI & Scenes
│
└── public/                     # Static Assets
```

---

## ⚡ Getting Started

### Prerequisites

- **Node.js** (v18+)
- **Rust** (Stable)
- **wasm-pack** (`cargo install wasm-pack`)

### Installation

1. **Clone the repository:**

   ```bash
   git clone https://github.com/MTawhid7/vestra-physics.git
   cd vestra-physics
   ```

2. **Install JS dependencies:**

   ```bash
   npm install
   ```

3. **Compile the Physics Engine:**
   *Recommended Build Command (enables parallel threads & SIMD):*

   ```bash
   npm run build:physics
   ```

4. **Run the Development Server:**

   ```bash
   npm run dev
   ```

---

## ⚙️ Configuration

Modify `physics/src/engine/config.rs` to tune the physics behavior.

```rust
pub struct PhysicsConfig {
    pub substeps: usize,          // Default: 8 (Higher = More stable)
    pub solver_iterations: usize, // Default: 8 (Higher = Stiffer)
    pub gravity: Vec3,            // Default: 0, -9.81, 0
    pub wind: Vec3,               // Vector for aerodynamics
    pub distance_compliance: f32, // Stiffness (1e-7 = Rigid, 1e-4 = Stretchy)
    // ...
}
```

---

## 🛠 Development Workflow

1. **Frontend:** Edit React/TS files -> Vite HMR updates instantly.
2. **Physics:** Edit Rust files -> Run `npm run build:physics` -> Refresh page.
3. **Tests:**

   ```bash
   cd physics
   cargo test              # Run all tests
   cargo test --test systems # Run specific suite
   ```

---

## 🔮 Roadmap

- [x] **Parallel Batch Solving:** Multithreaded collision resolution.
- [x] **Stability Fixes:** "Death Spiral" prevention and invisible mesh fix.
- [x] **Rust Testing:** Comprehensive unit test suite.
- [x] **Manual SIMD Optimization:** +simd128 and glam-simd enabled.
- [ ] **WebGPU Solver:** Port core constraints to Compute Shaders.
- [ ] **Temporal Coherence:** Incremental spatial hashing.
- [ ] **Mobile Algorithmic Pruning:** Sleep system for dormant particles.
- [ ] **Layering Support:** Complex multi-garment interactions.

---

## 📄 License

This project is licensed under the MIT License.
