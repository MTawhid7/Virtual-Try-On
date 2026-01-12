# 3D Garment Visualization Engine (V4)

![Status](https://img.shields.io/badge/Status-Phase_1_Complete-green) ![Tech](https://img.shields.io/badge/Tech-Rust_WASM_%7C_React_Three_Fiber-orange) ![Physics](https://img.shields.io/badge/Physics-XPBD_%2B_SDF-blue)

An industrial-grade, web-based Virtual Try-On (VTO) engine. Version 4 represents a complete architectural rewrite, moving the core physics solver from TypeScript to **Rust (WebAssembly)** to achieve high-performance, mobile-compatible simulation.

---

## 🎯 Project Objective

To build a **Universal Fitting Engine** capable of:

1. **Arbitrary Fit:** Simulating any garment (shirt, dress, coat) on any body shape (SMPL, scanned mesh).
2. **Zero Vibration:** Eliminating the "jitter" common in web-based physics using robust collision handling.
3. **Mobile Performance:** Running at 60 FPS on mobile devices via WASM and Shared Memory.

---

## 🏗️ Architecture V4: The Hybrid Compute Model

We utilize a **Data-Oriented Design** where heavy computation is offloaded to Rust, while rendering remains in the flexible React ecosystem.

### 1. The Core (Rust + WASM)

* **Role:** Physics Solver, Collision Detection, Constraint Management.
* **Memory Model:** **Zero-Copy Shared Memory**. Rust owns the data; JavaScript reads it directly via `Float32Array` views. No serialization overhead.
* **Collision:** **Signed Distance Fields (SDF)**. Replaces mesh-based raycasting with volumetric lookups for $O(1)$ collision checks and perfect smoothness. Currently implements a **T-Shape (Spine + Shoulders)** analytic SDF.
* **Constraints:** XPBD (Distance, Bending, Tethers) with sub-stepping.

### 2. The Bridge (TypeScript)

* **Role:** Orchestration and Rendering.
* **Rendering:** **React Three Fiber (R3F)** renders the mesh using the shared memory buffer directly.
* **Skinning:** Maps the High-Res visual mesh (~15k verts) to the Low-Res physics proxy (~1k verts) using Barycentric interpolation.
* **Welding:** Automatically merges split vertices in GLB files to ensure topological integrity.

---

## 📂 Project Structure

```text
root/
├── physics-core/          # [NEW] The Rust Crate (WASM Solver)
│   ├── src/
│   │   ├── collider/      # SDF Logic (Capsule, T-Shape)
│   │   ├── constraints/   # Distance, Bending, Tether
│   │   ├── data.rs        # Physics Data (SoA Layout)
│   │   └── lib.rs         # WASM Entry Point
│   └── Cargo.toml
├── src/
│   ├── v4/                # [ACTIVE] The V4 Implementation
│   │   ├── adapter/       # Bridge between React and WASM
│   │   ├── components/    # R3F Components (Garment, Visualizer)
│   │   └── utils/         # Skinning & Geometry Tools
│   ├── archives/          # [LEGACY] Previous iterations
│   └── App.tsx            # Entry point
└── vite.config.ts         # Configured with vite-plugin-wasm
```

---

## 📜 Evolution & Lessons Learned

### ❌ V1 - V3 (Legacy)

* **Failures:** Rubber-like stretching, jittering rigid bodies, and performance bottlenecks on the JS main thread.
* **Status:** Archived.

### 🚀 V4: Hybrid Rust/WASM (Current)

* **Success:**
  * **Stable Draping:** Shirt hangs naturally on shoulders without sliding off.
  * **No Explosions:** Solved via robust "Tether" constraints and correct geometry welding.
  * **High Performance:** Zero-copy memory allows driving high-res visuals with low-res physics at 60fps.
  * **Visual Fidelity:** Skinning system hides the low-poly physics mesh completely.

---

## 📦 Installation & Setup

### Prerequisites

1. **Node.js** (v16+)
2. **Rust & Cargo** (Latest Stable)
3. **wasm-pack** (`cargo install wasm-pack`)

### Running the Project

1. **Install Dependencies:**

    ```bash
    npm install
    ```

2. **Build the Rust Core:**

    ```bash
    cd physics-core
    wasm-pack build --target web
    cd ..
    ```

3. **Run the Dev Server:**

    ```bash
    npm run dev
    ```

---

## 🔮 Roadmap

### ✅ Phase 1: Foundation (Completed)

* [x] Port XPBD solver to Rust.
* [x] Implement Zero-Copy memory bridge.
* [x] Implement Analytic SDF Collision (T-Shape).
* [x] Implement Visual Skinning (High-Res -> Low-Res).
* [x] Solve topology issues (Welding & Tethers).

### 🚧 Phase 2: The Fitting Pipeline (Next)

* [ ] **SMPL Integration:** Replace T-Shape SDF with actual Mannequin SDF.
* [ ] **Auto-Scaling:** Algorithm to match shirt size to body size.
* [ ] **Inflation:** Pre-simulation step to resolve initial intersections.

### 🔮 Phase 3: Advanced Physics

* [ ] **Fabric Library:** Real-world material constants (Cotton, Silk, Denim).
* [ ] **Aerodynamics:** Lift and Drag based on triangle normals.
* [ ] **WebGPU:** Compute Shaders for massive particle counts (>10k).
