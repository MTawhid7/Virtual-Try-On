# 3D Garment Visualization Engine (V4)

![Status](https://img.shields.io/badge/Status-Phase_1_Complete-green) ![Tech](https://img.shields.io/badge/Tech-Rust_WASM_%7C_React_Three_Fiber-orange) ![Physics](https://img.shields.io/badge/Physics-XPBD_%2B_SDF-blue)

An industrial-grade, web-based Virtual Try-On (VTO) engine. Version 4 represents a complete architectural rewrite, moving the core physics solver from TypeScript to **Rust (WebAssembly)** to achieve high-performance, mobile-compatible simulation with **Zero-Copy** memory synchronization.

---

## 🎯 Project Objective

To build a **Universal Fitting Engine** capable of:

1. **Arbitrary Fit:** Simulating any garment (shirt, dress, coat) on any body shape.
2. **Zero Vibration:** Eliminating the "jitter" common in web-based physics using robust collision handling.
3. **Real-Time Interaction:** Allowing users to grab, pull, and stretch fabric naturally.

---

## 🏗️ Architecture V4: The Hybrid Compute Model

We utilize a **Data-Oriented Design** where heavy computation is offloaded to Rust, while rendering remains in the flexible React ecosystem.

### 1. The Core (Rust + WASM)

* **Role:** Physics Solver, Collision Detection, Constraint Management.
* **Memory Model:** **Zero-Copy Shared Memory**. Rust owns the data; JavaScript reads it directly via `Float32Array` views. No serialization overhead.
* **Solver:** XPBD (Extended Position Based Dynamics) with sub-stepping (20 steps/frame).
* **Collision:**
  * **Body:** Analytic SDF (Signed Distance Field) approximating a T-Shape (Spine + Shoulders).
  * **Self:** Spatial Hashing ($O(N)$) to prevent fabric interpenetration.

### 2. The Bridge (TypeScript)

* **Role:** Orchestration and Rendering.
* **Asset Loading:** Parses GLB files and performs **Vertex Welding** to ensure topological integrity.
* **Rendering:** **React Three Fiber (R3F)** renders the mesh using the shared memory buffer directly.
* **Skinning:** Maps the High-Res visual mesh (~15k verts) to the Low-Res physics proxy (~1k verts) using Barycentric interpolation.

---

## 🎮 Controls & Interaction

* **Orbit:** Left Click + Drag (Background)
* **Pan:** Right Click + Drag
* **Zoom:** Scroll
* **Interact:** **Click + Drag on the Shirt** to pull the fabric. The engine uses "Area Grabbing" to simulate grasping a patch of cloth.

---

## 📂 Project Structure

```text
root/
├── physics-core/          # [RUST] The Physics Engine (WASM)
│   ├── src/
│   │   ├── collider/      # SDF (T-Shape) & Spatial Hash
│   │   ├── constraints/   # Distance, Bending, Tether, Mouse
│   │   ├── data.rs        # Physics Data (SoA Layout)
│   │   └── lib.rs         # WASM Entry Point
│   └── Cargo.toml
├── src/
│   ├── v4/                # [TYPESCRIPT] The V4 Implementation
│   │   ├── adapter/       # Bridge between React and WASM
│   │   ├── components/    # R3F Components (Garment, Visualizer)
│   │   └── utils/         # Skinning & Geometry Tools
│   ├── archives/          # [LEGACY] Previous iterations (V1-V3)
│   └── App.tsx            # Entry point
└── vite.config.ts         # Configured with vite-plugin-wasm
```

---

## 🛠️ Technology Stack

| Component | Technology | Rationale |
| :--- | :--- | :--- |
| **Solver** | **Rust (WASM)** | Near-native performance, SIMD-ready, deterministic memory. |
| **Frontend** | **React Three Fiber** | Declarative 3D scene management. |
| **Collision** | **SDF + Spatial Hash** | Volumetric collision prevents tunneling; Spatial Hashing adds volume. |
| **Build Tool** | **Vite + wasm-pack** | Hot Module Replacement (HMR) for both TS and Rust. |

---

## 📦 Installation & Setup

### Prerequisites

1. **Node.js** (v16+)
2. **Rust & Cargo** (Latest Stable)

    ```bash
    curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
    ```

3. **wasm-pack**

    ```bash
    cargo install wasm-pack
    ```

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

* [x] **Rust Port:** XPBD solver (Distance, Bending, Tethers) running in WASM.
* [x] **Zero-Copy Bridge:** Shared memory pipeline established.
* [x] **Stability:** Solved mesh disintegration via Vertex Welding and Tether Constraints.
* [x] **Collision:** Implemented Analytic T-Shape SDF and Spatial Hashing.
* [x] **Interaction:** Implemented Area-based Mouse Grabbing.
* [x] **Visuals:** High-fidelity Skinning (Visual Mesh driven by Physics Proxy).

### 🚧 Phase 2: The Fitting Pipeline (Next)

* [ ] **Real SDF:** Replace T-Shape with voxelized Mannequin SDF.
* [ ] **Auto-Scaling:** Algorithm to match shirt size to body size.
* [ ] **Inflation:** Pre-simulation step to resolve initial intersections.

### 🔮 Phase 3: Advanced Physics

* [ ] **Fabric Library:** Real-world material constants (Cotton, Silk, Denim).
* [ ] **Aerodynamics:** Lift and Drag based on triangle normals.
* [ ] **WebGPU:** Compute Shaders for massive particle counts (>10k).
